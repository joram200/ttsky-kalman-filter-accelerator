// =============================================================================
// ulp_check.cpp — Verilator ULP tolerance harness for tt_um_joram200
//
// Drives the Verilator-compiled model (Vtt_um_joram200) via bit-banged SPI
// (CPOL=0, CPHA=0, MSB-first, 8-bit frames). Runs one Kalman measurement-update
// and compares every output element against a C++ F64 golden reference using the
// ULP distance metric (integer bit-pattern reinterpret).
//
// Build and run:
//   make -f test/Makefile.ulp
//   ./obj_dir/Vtt_um_joram200
//
// Exit code 0 = all outputs within ULP_THRESHOLD; non-zero = failure.
// =============================================================================

#include "Vtt_um_joram200.h"
#include "verilated.h"

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// Configurable threshold
// ---------------------------------------------------------------------------
static constexpr uint64_t ULP_THRESHOLD = 4;

// ---------------------------------------------------------------------------
// ULP distance
// ---------------------------------------------------------------------------
static uint64_t ulp_distance(double a, double b) {
    int64_t ai, bi;
    std::memcpy(&ai, &a, 8);
    std::memcpy(&bi, &b, 8);
    int64_t diff = ai - bi;
    return static_cast<uint64_t>(diff < 0 ? -diff : diff);
}

// ---------------------------------------------------------------------------
// Golden reference — mirrors the RTL algorithm
// (Newton-Raphson 3-iteration reciprocal for S_inv)
// ---------------------------------------------------------------------------
static void kalman_ref(double z, const double x_in[3], const double P_in[9],
                       double r, double x_out[3], double P_out[9]) {
    double y   = z - x_in[0];
    double S   = P_in[0] + r;

    // NR seed: 0x7FDE600000000000 - bits(S)
    uint64_t s_bits;
    std::memcpy(&s_bits, &S, 8);
    uint64_t seed_bits = 0x7FDE600000000000ULL - s_bits;
    double x_nr;
    std::memcpy(&x_nr, &seed_bits, 8);
    for (int i = 0; i < 3; i++)
        x_nr = x_nr * (2.0 - S * x_nr);
    double S_inv = x_nr;

    double K[3];
    for (int i = 0; i < 3; i++)
        K[i] = P_in[i * 3] * S_inv;

    for (int i = 0; i < 3; i++)
        x_out[i] = x_in[i] + K[i] * y;

    // IKH = I - K*H, H = [1,0,0]
    double IKH[9] = {
        1.0 - K[0], 0.0, 0.0,
        -K[1],      1.0, 0.0,
        -K[2],      0.0, 1.0
    };

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            double acc = 0.0;
            for (int k = 0; k < 3; k++)
                acc += IKH[i * 3 + k] * P_in[k * 3 + j];
            P_out[i * 3 + j] = acc;
        }
}

// ---------------------------------------------------------------------------
// Verilator model wrapper
// ---------------------------------------------------------------------------
static Vtt_um_joram200* dut = nullptr;
static uint64_t sim_time = 0;

static void clk_tick(int n = 1) {
    for (int i = 0; i < n; i++) {
        dut->clk = 0; dut->eval(); sim_time++;
        dut->clk = 1; dut->eval(); sim_time++;
    }
}

// ---------------------------------------------------------------------------
// Bit-bang SPI helpers
// uio_in[0] = SCLK, uio_in[1] = MOSI, uio_in[3] = CS_N
// uio_out[2] = MISO
// ---------------------------------------------------------------------------
static constexpr int SPI_HALF = 4;  // system clocks per SCLK half-period

static void set_sclk(int v) { dut->uio_in = (dut->uio_in & ~0x01) | (v & 1); }
static void set_mosi(int v) { dut->uio_in = (dut->uio_in & ~0x02) | ((v & 1) << 1); }
static void set_cs_n(int v) { dut->uio_in = (dut->uio_in & ~0x08) | ((v & 1) << 3); }
static int  get_miso()      { return (dut->uio_out >> 2) & 1; }

static uint8_t spi_xfer_byte(uint8_t tx) {
    uint8_t rx = 0;
    for (int b = 7; b >= 0; b--) {
        set_mosi((tx >> b) & 1);
        clk_tick(SPI_HALF);
        set_sclk(1);
        clk_tick(SPI_HALF);
        rx = (uint8_t)((rx << 1) | get_miso());
        set_sclk(0);
    }
    return rx;
}

static void spi_write_reg(uint8_t addr, uint64_t data) {
    set_cs_n(0); clk_tick(SPI_HALF);
    spi_xfer_byte(0x80u | (addr & 0x7Fu));
    for (int b = 7; b >= 0; b--)
        spi_xfer_byte((uint8_t)((data >> (b * 8)) & 0xFF));
    clk_tick(SPI_HALF); set_cs_n(1); clk_tick(SPI_HALF * 2);
}

static uint64_t spi_read_reg(uint8_t addr) {
    set_cs_n(0); clk_tick(SPI_HALF);
    spi_xfer_byte(0x00u | (addr & 0x7Fu));
    uint64_t rx = 0;
    for (int b = 7; b >= 0; b--)
        rx |= ((uint64_t)spi_xfer_byte(0) << (b * 8));
    clk_tick(SPI_HALF); set_cs_n(1); clk_tick(SPI_HALF * 2);
    return rx;
}

static void spi_write_burst(uint8_t start_addr, const uint64_t* data, int n) {
    set_cs_n(0); clk_tick(SPI_HALF);
    spi_xfer_byte(0x80u | (start_addr & 0x7Fu));
    for (int r = 0; r < n; r++)
        for (int b = 7; b >= 0; b--)
            spi_xfer_byte((uint8_t)((data[r] >> (b * 8)) & 0xFF));
    clk_tick(SPI_HALF); set_cs_n(1); clk_tick(SPI_HALF * 2);
}

static void spi_read_burst(uint8_t start_addr, uint64_t* data, int n) {
    set_cs_n(0); clk_tick(SPI_HALF);
    spi_xfer_byte(0x00u | (start_addr & 0x7Fu));
    for (int r = 0; r < n; r++) {
        data[r] = 0;
        for (int b = 7; b >= 0; b--)
            data[r] |= ((uint64_t)spi_xfer_byte(0) << (b * 8));
    }
    clk_tick(SPI_HALF); set_cs_n(1); clk_tick(SPI_HALF * 2);
}

static bool poll_done(int max_ticks = 100000) {
    for (int i = 0; i < max_ticks / 20; i++) {
        clk_tick(10);
        uint64_t stat = spi_read_reg(1);
        if (stat & 1) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// double <-> uint64 helpers
// ---------------------------------------------------------------------------
static inline uint64_t dbl_to_u64(double x) {
    uint64_t b; std::memcpy(&b, &x, 8); return b;
}
static inline double u64_to_dbl(uint64_t b) {
    double x; std::memcpy(&x, &b, 8); return x;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtt_um_joram200;

    // Init: CS_N high, SCLK low, MOSI low
    dut->uio_in = 0x08;
    dut->ui_in  = 0;
    dut->ena    = 1;
    dut->rst_n  = 0;
    clk_tick(20);
    dut->rst_n  = 1;
    clk_tick(10);

    // Test scenario: z=1.5, x_in=[0,0,0], P_in=I3, r=5.0
    double z_val    = 1.5;
    double x_in[3]  = {0.0, 0.0, 0.0};
    double P_in[9]  = {1.0, 0.0, 0.0,
                       0.0, 1.0, 0.0,
                       0.0, 0.0, 1.0};
    double r_val    = 5.0;

    // Compute golden reference
    double x_ref[3], P_ref[9];
    kalman_ref(z_val, x_in, P_in, r_val, x_ref, P_ref);

    // Write z, x_in[0:2] as burst (regs 2-5)
    uint64_t z_x_buf[4] = {
        dbl_to_u64(z_val),
        dbl_to_u64(x_in[0]),
        dbl_to_u64(x_in[1]),
        dbl_to_u64(x_in[2])
    };
    spi_write_burst(2, z_x_buf, 4);

    // Write P_in[0:8] (regs 9-17)
    uint64_t P_buf[9];
    for (int i = 0; i < 9; i++) P_buf[i] = dbl_to_u64(P_in[i]);
    spi_write_burst(9, P_buf, 9);

    // Fire: CTRL.start = 1
    spi_write_reg(0, 1ULL);

    // Poll done
    if (!poll_done()) {
        fprintf(stderr, "ERROR: timeout waiting for done signal\n");
        delete dut;
        return 1;
    }

    // Read x_out[0:2] and P_out[0:8]
    uint64_t x_hw_bits[3], P_hw_bits[9];
    spi_read_burst(6,  x_hw_bits, 3);
    spi_read_burst(18, P_hw_bits, 9);

    // ULP comparison
    uint64_t max_ulp = 0;
    bool all_pass = true;
    int fail_count = 0;

    printf("=== ULP Check: x_out[0:2] ===\n");
    for (int i = 0; i < 3; i++) {
        double hw_val = u64_to_dbl(x_hw_bits[i]);
        uint64_t d = ulp_distance(x_ref[i], hw_val);
        max_ulp = std::max(max_ulp, d);
        const char* tag = (d <= ULP_THRESHOLD) ? "PASS" : "FAIL";
        if (d > ULP_THRESHOLD) { all_pass = false; fail_count++; }
        printf("  x_out[%d]: ref=%.17g hw=%.17g ULP=%llu %s\n",
               i, x_ref[i], hw_val, (unsigned long long)d, tag);
    }

    printf("=== ULP Check: P_out[0:8] ===\n");
    for (int i = 0; i < 9; i++) {
        double hw_val = u64_to_dbl(P_hw_bits[i]);
        uint64_t d = ulp_distance(P_ref[i], hw_val);
        max_ulp = std::max(max_ulp, d);
        const char* tag = (d <= ULP_THRESHOLD) ? "PASS" : "FAIL";
        if (d > ULP_THRESHOLD) { all_pass = false; fail_count++; }
        printf("  P_out[%d]: ref=%.17g hw=%.17g ULP=%llu %s\n",
               i, P_ref[i], hw_val, (unsigned long long)d, tag);
    }

    printf("\n=== Summary ===\n");
    printf("Max ULP error: %llu (threshold: %llu)\n",
           (unsigned long long)max_ulp,
           (unsigned long long)ULP_THRESHOLD);
    printf("Result: %s (%d/%d outputs failed)\n",
           all_pass ? "PASS" : "FAIL", fail_count, 12);

    dut->final();
    delete dut;
    return all_pass ? 0 : 1;
}
