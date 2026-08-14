# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0
#
# test.py — Cocotb SPI BFM testbench for tt_um_joram200
#
# Drives the Kalman filter hardware accelerator via bit-banged SPI (CPOL=0, CPHA=0,
# MSB-first, 8-bit frames). Runs one Kalman measurement-update and compares all
# outputs against a Python F64 golden reference within a 4-ULP tolerance.
#
# SPI pin mapping (uio_in / uio_out):
#   uio_in[0]  = SCLK
#   uio_in[1]  = MOSI
#   uio_out[2] = MISO
#   uio_in[3]  = CS_N  (active low)

import struct
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SPI_HALF_CLK = 5   # system-clock cycles per SCLK half-period

# Register indices
REG_CTRL   = 0
REG_STAT   = 1
REG_Z      = 2
REG_XIN0   = 3
REG_XIN1   = 4
REG_XIN2   = 5
REG_XOUT0  = 6
REG_XOUT1  = 7
REG_XOUT2  = 8
REG_PIN0   = 9   # P_in[0..8] are regs 9..17
REG_POUT0  = 18  # P_out[0..8] are regs 18..26
REG_RREG   = 27

ULP_THRESHOLD = 4

# ---------------------------------------------------------------------------
# F64 helpers
# ---------------------------------------------------------------------------
def f64_pack(x: float) -> int:
    """Convert Python float to 64-bit integer bit pattern (IEEE-754 F64)."""
    return struct.unpack('Q', struct.pack('d', x))[0]

def f64_unpack(bits: int) -> float:
    """Convert 64-bit integer bit pattern to Python float."""
    return struct.unpack('d', struct.pack('Q', bits & 0xFFFFFFFFFFFFFFFF))[0]

def ulp_distance(a: float, b: float) -> int:
    """ULP distance between two F64 values (integer reinterpret metric)."""
    ai = struct.unpack('q', struct.pack('d', a))[0]
    bi = struct.unpack('q', struct.pack('d', b))[0]
    return abs(ai - bi)

# ---------------------------------------------------------------------------
# Kalman update golden reference (matches RTL algorithm exactly)
# ---------------------------------------------------------------------------
def kalman_ref(z: float, x_in: list, P_in: list, r: float = 5.0):
    """
    Scalar Kalman measurement update.
    H = [1, 0, 0], R = r, 3-iteration NR reciprocal for S_inv.
    Returns (x_out, P_out) as lists of Python floats.
    """
    # Innovation
    y = z - x_in[0]
    # Innovation covariance S = P_in[0,0] + R
    S = P_in[0] + r
    # Newton-Raphson reciprocal: seed = 0x7FDE600000000000 - bits(S)
    s_bits = f64_pack(S)
    seed_bits = (0x7FDE600000000000 - s_bits) & 0xFFFFFFFFFFFFFFFF
    x_nr = f64_unpack(seed_bits)
    for _ in range(3):
        x_nr = x_nr * (2.0 - S * x_nr)
    S_inv = x_nr
    # Kalman gain K = P_in[:,0] * S_inv
    K = [P_in[i * 3] * S_inv for i in range(3)]
    # State update x_out = x_in + K * y
    x_out = [x_in[i] + K[i] * y for i in range(3)]
    # IKH = I - K*H  (H = [1,0,0])
    IKH = [
        1.0 - K[0], 0.0,  0.0,
        -K[1],       1.0,  0.0,
        -K[2],       0.0,  1.0,
    ]
    # P_out = IKH * P_in
    P_out = []
    for i in range(3):
        for j in range(3):
            acc = 0.0
            for k in range(3):
                acc += IKH[i * 3 + k] * P_in[k * 3 + j]
            P_out.append(acc)
    return x_out, P_out

# ---------------------------------------------------------------------------
# SPI bit-bang helpers
# ---------------------------------------------------------------------------
def _sclk(dut, v):
    dut.uio_in.value = (int(dut.uio_in.value) & ~0x01) | (int(v) & 0x01)

def _mosi(dut, v):
    dut.uio_in.value = (int(dut.uio_in.value) & ~0x02) | ((int(v) & 0x01) << 1)

def _cs_n(dut, v):
    dut.uio_in.value = (int(dut.uio_in.value) & ~0x08) | ((int(v) & 0x01) << 3)

def _miso(dut) -> int:
    return (int(dut.uio_out.value) >> 2) & 0x01


async def _spi_xfer_byte(dut, tx_byte: int) -> int:
    """Transfer one byte MSB-first; returns received byte."""
    rx_byte = 0
    for bit in range(7, -1, -1):
        _mosi(dut, (tx_byte >> bit) & 1)
        await ClockCycles(dut.clk, SPI_HALF_CLK)
        _sclk(dut, 1)
        await ClockCycles(dut.clk, SPI_HALF_CLK)
        rx_byte = (rx_byte << 1) | _miso(dut)
        _sclk(dut, 0)
    return rx_byte


async def spi_write(dut, addr: int, value: int):
    """Write a single 64-bit register."""
    _cs_n(dut, 0)
    await ClockCycles(dut.clk, SPI_HALF_CLK)
    # Command byte: R/W=1 (write), addr[6:0]
    await _spi_xfer_byte(dut, 0x80 | (addr & 0x7F))
    # 8 data bytes MSB-first
    for byte_idx in range(7, -1, -1):
        byte_val = (value >> (byte_idx * 8)) & 0xFF
        await _spi_xfer_byte(dut, byte_val)
    await ClockCycles(dut.clk, SPI_HALF_CLK)
    _cs_n(dut, 1)
    await ClockCycles(dut.clk, SPI_HALF_CLK)


async def spi_read(dut, addr: int) -> int:
    """Read a single 64-bit register; returns 64-bit integer."""
    _cs_n(dut, 0)
    await ClockCycles(dut.clk, SPI_HALF_CLK)
    # Command byte: R/W=0 (read), addr[6:0]
    await _spi_xfer_byte(dut, 0x00 | (addr & 0x7F))
    rx = 0
    for _ in range(8):
        b = await _spi_xfer_byte(dut, 0x00)
        rx = (rx << 8) | b
    await ClockCycles(dut.clk, SPI_HALF_CLK)
    _cs_n(dut, 1)
    await ClockCycles(dut.clk, SPI_HALF_CLK)
    return rx


async def spi_write_burst(dut, start_addr: int, values: list):
    """Write a burst of 64-bit registers starting at start_addr."""
    _cs_n(dut, 0)
    await ClockCycles(dut.clk, SPI_HALF_CLK)
    await _spi_xfer_byte(dut, 0x80 | (start_addr & 0x7F))
    for val in values:
        for byte_idx in range(7, -1, -1):
            byte_val = (val >> (byte_idx * 8)) & 0xFF
            await _spi_xfer_byte(dut, byte_val)
    await ClockCycles(dut.clk, SPI_HALF_CLK)
    _cs_n(dut, 1)
    await ClockCycles(dut.clk, SPI_HALF_CLK)


async def spi_read_burst(dut, start_addr: int, count: int) -> list:
    """Read burst of 'count' 64-bit registers starting at start_addr."""
    _cs_n(dut, 0)
    await ClockCycles(dut.clk, SPI_HALF_CLK)
    await _spi_xfer_byte(dut, 0x00 | (start_addr & 0x7F))
    results = []
    for _ in range(count):
        rx = 0
        for _ in range(8):
            b = await _spi_xfer_byte(dut, 0x00)
            rx = (rx << 8) | b
        results.append(rx)
    await ClockCycles(dut.clk, SPI_HALF_CLK)
    _cs_n(dut, 1)
    await ClockCycles(dut.clk, SPI_HALF_CLK)
    return results


async def poll_done(dut, timeout_cycles: int = 2000):
    """Poll STAT until done is set or timeout."""
    for _ in range(timeout_cycles // 20):
        stat = await spi_read(dut, REG_STAT)
        if stat & 0x01:  # done bit
            return True
        await ClockCycles(dut.clk, 5)
    return False

# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_kalman_spi(dut):
    dut._log.info("tt_um_joram200: SPI Kalman update test starting")

    # 50 MHz system clock
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())

    # Initial pin state: CS_N high (deasserted), SCLK low, MOSI low
    dut.ena.value   = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0x08   # CS_N = 1
    dut.rst_n.value  = 0
    await ClockCycles(dut.clk, 20)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    # ------------------------------------------------------------------
    # Define one Kalman update scenario
    # ------------------------------------------------------------------
    z_float  = 1.5
    x_float  = [0.0, 0.0, 0.0]
    P_float  = [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
    r_float  = 5.0  # default R_REG

    z_bits   = f64_pack(z_float)
    x_bits   = [f64_pack(v) for v in x_float]
    P_bits   = [f64_pack(v) for v in P_float]

    # Compute golden reference
    x_ref, P_ref = kalman_ref(z_float, x_float, P_float, r_float)
    dut._log.info(f"Golden x_out: {x_ref}")
    dut._log.info(f"Golden P_out: {P_ref}")

    # ------------------------------------------------------------------
    # Write inputs via SPI burst: z (reg 2), x_in[0:2] (regs 3-5)
    # ------------------------------------------------------------------
    await spi_write_burst(dut, REG_Z, [z_bits] + x_bits)

    # Write P_in[0:8] (regs 9-17)
    await spi_write_burst(dut, REG_PIN0, P_bits)

    # Write CTRL[start=1]: value 0x0000000000000001
    await spi_write(dut, REG_CTRL, 0x0000000000000001)

    # ------------------------------------------------------------------
    # Poll for done
    # ------------------------------------------------------------------
    dut._log.info("Waiting for computation to complete...")
    ok = await poll_done(dut, timeout_cycles=4000)
    assert ok, "Timeout waiting for Kalman update done signal"
    dut._log.info("Done signal received")

    # ------------------------------------------------------------------
    # Read outputs
    # ------------------------------------------------------------------
    x_out_bits = await spi_read_burst(dut, REG_XOUT0, 3)
    P_out_bits = await spi_read_burst(dut, REG_POUT0, 9)

    x_out_hw = [f64_unpack(b) for b in x_out_bits]
    P_out_hw = [f64_unpack(b) for b in P_out_bits]

    dut._log.info(f"HW x_out: {x_out_hw}")
    dut._log.info(f"HW P_out: {P_out_hw}")

    # ------------------------------------------------------------------
    # ULP comparison
    # ------------------------------------------------------------------
    max_ulp = 0
    all_pass = True

    for i in range(3):
        d = ulp_distance(x_ref[i], x_out_hw[i])
        max_ulp = max(max_ulp, d)
        if d > ULP_THRESHOLD:
            dut._log.error(f"x_out[{i}]: ref={x_ref[i]:.17g} hw={x_out_hw[i]:.17g} ULP={d}")
            all_pass = False
        else:
            dut._log.info(f"x_out[{i}]: ULP={d} PASS")

    for i in range(9):
        d = ulp_distance(P_ref[i], P_out_hw[i])
        max_ulp = max(max_ulp, d)
        if d > ULP_THRESHOLD:
            dut._log.error(f"P_out[{i}]: ref={P_ref[i]:.17g} hw={P_out_hw[i]:.17g} ULP={d}")
            all_pass = False
        else:
            dut._log.info(f"P_out[{i}]: ULP={d} PASS")

    dut._log.info(f"Max ULP error across all 12 outputs: {max_ulp}")
    assert all_pass, f"ULP check failed (max error = {max_ulp}, threshold = {ULP_THRESHOLD})"
    dut._log.info("All outputs within ULP threshold — PASS")
