// =============================================================================
// tb_top.sv — SPI master BFM testbench for tt_um_joram200
// Task 5 Option A: All constants in F32 (stored in lower 32 bits of 64-bit SPI word).
//
// Module hierarchy:
//   spi_master_bfm   — SPI master with named tasks
//   result_checker   — F32 ULP comparator (4-ULP threshold)
//   program_block    — One Kalman measurement-update scenario
//   tb_top           — Top-level testbench
//
// SPI protocol: CPOL=0, CPHA=0, MSB-first, 8-bit frames.
// Command byte: [7]=R/W (1=write, 0=read), [6:0]=register index.
// Data: 8 bytes per register, MSB-first.
// F32 value occupies lower 32 bits: host sends 4 zero bytes then 4 F32 bytes.
// DUT returns 4 zero bytes then 4 F32 bytes.
// burst_buf[i] = {32'h0, f32_value} for all data registers.
//
// Register map:
//   0   CTRL       W    [0]=start, [1]=sw_rst
//   1   STAT       R    [0]=done,  [1]=busy
//   2   z          W    measurement F32
//   3-5  x_in[0:2] W   prior state
//   6-8  x_out[0:2] R  corrected state
//   9-17 P_in[0:8] W   prior covariance (row-major)
//  18-26 P_out[0:8] R  updated covariance
//  27   R_REG      R/W  measurement noise R (default 5.0 F32)
//
// iverilog 12 compatibility notes:
//   - No unpacked array subroutine ports
//   - Burst data passed via module-level burst_buf array
//   - No variable declarations inside initial blocks
// =============================================================================
`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// spi_master_bfm — SPI master bus functional model
// burst_buf[] holds 64-bit SPI words; F32 data is in lower 32 bits.
// -----------------------------------------------------------------------------
module spi_master_bfm #(
    parameter real SPI_CLK_NS = 80.0
)(
    input  logic        clk_sys,
    output logic        sclk,
    output logic        mosi,
    output logic        cs_n,
    input  logic        miso
);
    logic [63:0] burst_buf [0:8];

    integer _bb;
    initial begin
        sclk = 1'b0;
        mosi = 1'b0;
        cs_n = 1'b1;
        for (_bb = 0; _bb < 9; _bb = _bb + 1)
            burst_buf[_bb] = 64'h0;
    end

    task automatic spi_xfer_byte (
        input  logic [7:0] tx,
        output logic [7:0] rx
    );
        integer b;
        rx = 8'h0;
        for (b = 7; b >= 0; b = b - 1) begin
            mosi = tx[b];
            #(SPI_CLK_NS / 2.0);
            sclk = 1'b1;
            #(SPI_CLK_NS / 2.0);
            rx[b] = miso;
            sclk = 1'b0;
        end
    endtask

    task automatic spi_write (
        input logic [6:0]  addr,
        input logic [63:0] data
    );
        logic [7:0] dummy;
        integer byte_idx;
        cs_n = 1'b0;
        #(SPI_CLK_NS);
        spi_xfer_byte(8'h80 | {1'b0, addr}, dummy);
        for (byte_idx = 7; byte_idx >= 0; byte_idx = byte_idx - 1)
            spi_xfer_byte(data[byte_idx*8 +: 8], dummy);
        #(SPI_CLK_NS);
        cs_n = 1'b1;
        #(SPI_CLK_NS * 2.0);
    endtask

    task automatic spi_read (
        input  logic [6:0]  addr,
        output logic [63:0] data
    );
        logic [7:0] dummy, rx_b;
        integer byte_idx;
        data = 64'h0;
        cs_n = 1'b0;
        #(SPI_CLK_NS);
        spi_xfer_byte(8'h00 | {1'b0, addr}, dummy);
        for (byte_idx = 7; byte_idx >= 0; byte_idx = byte_idx - 1) begin
            spi_xfer_byte(8'h00, rx_b);
            data[byte_idx*8 +: 8] = rx_b;
        end
        #(SPI_CLK_NS);
        cs_n = 1'b1;
        #(SPI_CLK_NS * 2.0);
    endtask

    task automatic spi_write_burst (
        input logic [6:0] start_addr,
        input int         n
    );
        logic [7:0] dummy;
        integer reg_i, byte_idx;
        cs_n = 1'b0;
        #(SPI_CLK_NS);
        spi_xfer_byte(8'h80 | {1'b0, start_addr}, dummy);
        for (reg_i = 0; reg_i < n; reg_i = reg_i + 1) begin
            for (byte_idx = 7; byte_idx >= 0; byte_idx = byte_idx - 1)
                spi_xfer_byte(burst_buf[reg_i][byte_idx*8 +: 8], dummy);
        end
        #(SPI_CLK_NS);
        cs_n = 1'b1;
        #(SPI_CLK_NS * 2.0);
    endtask

    task automatic spi_read_burst (
        input logic [6:0] start_addr,
        input int         n
    );
        logic [7:0] dummy, rx_b;
        integer reg_i, byte_idx;
        cs_n = 1'b0;
        #(SPI_CLK_NS);
        spi_xfer_byte(8'h00 | {1'b0, start_addr}, dummy);
        for (reg_i = 0; reg_i < n; reg_i = reg_i + 1) begin
            burst_buf[reg_i] = 64'h0;
            for (byte_idx = 7; byte_idx >= 0; byte_idx = byte_idx - 1) begin
                spi_xfer_byte(8'h00, rx_b);
                burst_buf[reg_i][byte_idx*8 +: 8] = rx_b;
            end
        end
        #(SPI_CLK_NS);
        cs_n = 1'b1;
        #(SPI_CLK_NS * 2.0);
    endtask

    task automatic poll_done (
        input int timeout_ns
    );
        logic [63:0] stat;
        int elapsed;
        elapsed = 0;
        do begin
            #200;
            elapsed = elapsed + 200;
            spi_read(7'd1, stat);
        end while (!stat[0] && elapsed < timeout_ns);
        if (!stat[0]) $fatal(1, "TIMEOUT: done never asserted");
    endtask

endmodule


// -----------------------------------------------------------------------------
// result_checker — compare F32 hardware outputs against pre-computed golden ref
// F32 values are zero-extended to 64 bits: burst_buf = {32'h0, f32_value}.
// ULP distance computed on the 64-bit representation; since upper 32 bits are
// both zero, this equals the F32 ULP distance for positive values.
//
// Scenario: z=1.5, x_in=[0,0,0], P_in=I3, r=5.0 (F32)
//   S=6.0, K=[1/6,0,0], x_out=[0.25,0,0], P_out=diag(5/6,1,1)
// -----------------------------------------------------------------------------
module result_checker #(
    parameter int ULP_THRESHOLD = 4
)();
    // Golden reference — F32 bit patterns, zero-extended to 64 bits
    logic [63:0] REF_XOUT [0:2];
    logic [63:0] REF_POUT [0:8];

    logic [63:0] hw_x [0:2];
    logic [63:0] hw_p [0:8];

    integer _i;
    initial begin
        // x_out[0] = 0.25 in F32 = 0x3E800000
        REF_XOUT[0] = 64'h000000003E800000;
        REF_XOUT[1] = 64'h0000000000000000;
        REF_XOUT[2] = 64'h0000000000000000;

        // P_out[0,0] = 5/6 in F32 = 0x3F555555 (nearest even rounding)
        REF_POUT[0] = 64'h000000003F555555;
        REF_POUT[1] = 64'h0000000000000000;
        REF_POUT[2] = 64'h0000000000000000;
        REF_POUT[3] = 64'h0000000000000000;
        // P_out[1,1] = 1.0 in F32 = 0x3F800000
        REF_POUT[4] = 64'h000000003F800000;
        REF_POUT[5] = 64'h0000000000000000;
        REF_POUT[6] = 64'h0000000000000000;
        REF_POUT[7] = 64'h0000000000000000;
        // P_out[2,2] = 1.0 in F32 = 0x3F800000
        REF_POUT[8] = 64'h000000003F800000;

        for (_i = 0; _i < 3; _i = _i + 1) hw_x[_i] = 64'h0;
        for (_i = 0; _i < 9; _i = _i + 1) hw_p[_i] = 64'h0;
    end

    // ULP distance (signed-integer reinterpret on 64-bit word)
    function automatic longint unsigned ulp_dist (
        input logic [63:0] a, b
    );
        longint signed ai, bi;
        ai = $signed(a);
        bi = $signed(b);
        ulp_dist = (ai > bi) ? longint'(ai - bi) : longint'(bi - ai);
    endfunction

    task automatic check ();
        int pass;
        longint unsigned d;
        integer i;
        pass = 1;
        for (i = 0; i < 3; i = i + 1) begin
            d = ulp_dist(hw_x[i], REF_XOUT[i]);
            if (d > ULP_THRESHOLD) begin
                $error("x_out[%0d]: hw=%016h ref=%016h ULP=%0d FAIL", i, hw_x[i], REF_XOUT[i], d);
                pass = 0;
            end else begin
                $display("x_out[%0d]: ULP=%0d PASS", i, d);
            end
        end
        for (i = 0; i < 9; i = i + 1) begin
            d = ulp_dist(hw_p[i], REF_POUT[i]);
            if (d > ULP_THRESHOLD) begin
                $error("P_out[%0d]: hw=%016h ref=%016h ULP=%0d FAIL", i, hw_p[i], REF_POUT[i], d);
                pass = 0;
            end else begin
                $display("P_out[%0d]: ULP=%0d PASS", i, d);
            end
        end
        if (pass)
            $display("RESULT_CHECKER: ALL OUTPUTS WITHIN %0d ULP — PASS", ULP_THRESHOLD);
        else
            $fatal(1, "RESULT_CHECKER: ONE OR MORE OUTPUTS EXCEEDED ULP THRESHOLD");
    endtask

endmodule


// -----------------------------------------------------------------------------
// program_block — F32 test scenario data
// Scenario: z=1.5, x_in=[0,0,0], P_in=I3, r=5.0
// All values stored as {32'h0, F32_value} in 64-bit burst_buf entries.
// -----------------------------------------------------------------------------
module program_block (
    input  logic clk
);
    logic [63:0] Z_VAL;
    logic [63:0] X_IN [0:2];
    logic [63:0] P_IN [0:8];

    integer _i;
    initial begin
        // z = 1.5 in F32 = 0x3FC00000
        Z_VAL  = 64'h000000003FC00000;

        X_IN[0] = 64'h0000000000000000; // 0.0
        X_IN[1] = 64'h0000000000000000;
        X_IN[2] = 64'h0000000000000000;

        // Identity matrix (row-major), F32 1.0 = 0x3F800000
        for (_i = 0; _i < 9; _i = _i + 1) P_IN[_i] = 64'h0;
        P_IN[0] = 64'h000000003F800000; // 1.0 [0,0]
        P_IN[4] = 64'h000000003F800000; // 1.0 [1,1]
        P_IN[8] = 64'h000000003F800000; // 1.0 [2,2]
    end
    // R_REG default (5.0 F32 = 0x40A00000) is loaded at reset — no explicit write needed
endmodule


// -----------------------------------------------------------------------------
// tb_top — top-level testbench
// -----------------------------------------------------------------------------
module tb_top;

    logic clk, rst_n;
    initial clk = 1'b0;
    always #10 clk = ~clk;  // 50 MHz (20 ns period)

    logic [7:0] ui_in, uo_out, uio_in, uio_out, uio_oe;
    logic       ena;

    logic sclk_bfm, mosi_bfm, cs_n_bfm, miso_bfm;
    assign uio_in[0] = sclk_bfm;
    assign uio_in[1] = mosi_bfm;
    assign uio_in[3] = cs_n_bfm;
    assign miso_bfm  = uio_out[2];
    assign uio_in[7:4] = 4'h0;
    assign uio_in[2]   = 1'b0;
    assign ui_in       = 8'h0;
    assign ena         = 1'b1;

    tt_um_joram200 dut (
        .ui_in   (ui_in),
        .uo_out  (uo_out),
        .uio_in  (uio_in),
        .uio_out (uio_out),
        .uio_oe  (uio_oe),
        .ena     (ena),
        .clk     (clk),
        .rst_n   (rst_n)
    );

    spi_master_bfm #(.SPI_CLK_NS(80.0)) u_bfm (
        .clk_sys (clk),
        .sclk    (sclk_bfm),
        .mosi    (mosi_bfm),
        .cs_n    (cs_n_bfm),
        .miso    (miso_bfm)
    );

    result_checker #(.ULP_THRESHOLD(4)) u_checker ();
    program_block u_prog (.clk(clk));

    initial begin
        $dumpfile("tb_top.fst");
        $dumpvars(0, tb_top);
    end

    initial begin
        integer i;

        rst_n = 1'b0;
        repeat(20) @(posedge clk);
        rst_n = 1'b1;
        repeat(10) @(posedge clk);

        $display("=== SPI Kalman update test (F32 Option A) ===");

        // Write z (reg 2) and x_in[0:2] (regs 3-5) in one burst of 4
        u_bfm.burst_buf[0] = u_prog.Z_VAL;
        u_bfm.burst_buf[1] = u_prog.X_IN[0];
        u_bfm.burst_buf[2] = u_prog.X_IN[1];
        u_bfm.burst_buf[3] = u_prog.X_IN[2];
        u_bfm.spi_write_burst(7'd2, 4);

        // Write P_in[0:8] (regs 9-17) in one burst of 9
        for (i = 0; i < 9; i = i + 1)
            u_bfm.burst_buf[i] = u_prog.P_IN[i];
        u_bfm.spi_write_burst(7'd9, 9);

        // Fire: write CTRL.start = 1
        u_bfm.spi_write(7'd0, 64'h0000000000000001);

        // Poll done (500 µs timeout)
        $display("Waiting for done...");
        u_bfm.poll_done(500000);
        $display("Done asserted.");

        // Read x_out[0:2] (regs 6-8)
        u_bfm.spi_read_burst(7'd6, 3);
        for (i = 0; i < 3; i = i + 1)
            u_checker.hw_x[i] = u_bfm.burst_buf[i];

        // Read P_out[0:8] (regs 18-26)
        u_bfm.spi_read_burst(7'd18, 9);
        for (i = 0; i < 9; i = i + 1)
            u_checker.hw_p[i] = u_bfm.burst_buf[i];

        // ULP comparison against golden reference
        u_checker.check();

        $display("=== Test complete ===");
        $finish;
    end

    // Timeout watchdog
    initial begin
        #10_000_000;
        $fatal(1, "WATCHDOG: simulation exceeded 10 ms limit");
    end

endmodule
