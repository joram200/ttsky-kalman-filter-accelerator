# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0
#
# test.py — Cocotb parallel-GPIO testbench for tt_um_joram200
#
# INT10 Q4.6 fixed-point, 1D (scalar) Kalman update.
#
# Interface (parallel GPIO register file):
#   ui_in[7:0]  = wr_data[7:0]  write data byte
#   uio_in[2:0] = addr[2:0]     register address
#   uio_in[3]   = byte_sel      0=MSB byte, 1=LSB byte
#   uio_in[4]   = wr_en         rising-edge triggered write enable
#   uio_in[5]   = sw_rst_in     software reset (level)
#   uio_in[6]   = start_in      rising-edge one-shot start
#   uo_out[7:0] = rd_data[7:0]  read data byte (combinational)
#
# Register map:
#   1  STAT   R    [0]=done_latch, [1]=busy
#   2  z      W    measurement Q4.6
#   3  x_in   W    prior state Q4.6
#   4  x_out  R    corrected state Q4.6
#   5  P_in   W    prior covariance Q4.6
#   6  P_out  R    updated covariance Q4.6
#   7  R_REG  R/W  noise R (default 5.0 = 0x140)
#
# Scenario: z=1.5, x_in=0, P_in=1.0, R=5.0 (default)
#   Expected: x_out=0x000F (15), P_out=0x0036 (54)

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

ULP_THRESHOLD = 4

# ---------------------------------------------------------------------------
# Q4.6 golden reference (integer arithmetic, matches RTL exactly)
# ---------------------------------------------------------------------------
def kalman_ref_q46(z_q46: int, x_in_q46: int, P_in_q46: int, R_q46: int):
    """
    Scalar Kalman update in Q4.6 integer arithmetic.
    All values are 10-bit signed integers (Q4.6: value_real = val / 64).
    Returns (x_out, P_out) as 10-bit signed integers.
    """
    def sign_ext10(v):
        v &= 0x3FF
        if v & 0x200:
            v -= 0x400
        return v

    def mul_q46(a, b):
        """Signed Q4.6 × Q4.6 → Q4.6 via >> 6."""
        return (a * b) >> 6

    y       = z_q46 - x_in_q46          # innovation (signed)
    S       = P_in_q46 + R_q46          # innovation covariance
    S_inv   = 4096 // S if S > 0 else 0 # floor(4096/S) = 1/S in Q4.6
    K       = mul_q46(P_in_q46, S_inv)  # Kalman gain
    ky      = mul_q46(K, y)             # K * y
    kp      = mul_q46(K, P_in_q46)     # K * P
    x_out   = x_in_q46 + ky
    P_out   = P_in_q46 - kp
    return sign_ext10(x_out), sign_ext10(P_out)


# ---------------------------------------------------------------------------
# GPIO helpers — minimal cycle count for fast GL simulation
# ---------------------------------------------------------------------------
async def gpio_write_byte(dut, addr: int, byte_sel: int, data: int):
    """Write one byte (MSB or LSB) to addressed register via GPIO."""
    dut.ui_in.value  = data & 0xFF
    dut.uio_in.value = (addr & 0x7) | ((byte_sel & 0x1) << 3)  # wr_en=0
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = (addr & 0x7) | ((byte_sel & 0x1) << 3) | (1 << 4)  # wr_en=1
    await ClockCycles(dut.clk, 1)   # sync FF stage 1
    await ClockCycles(dut.clk, 1)   # sync FF stage 2: write executes
    dut.uio_in.value = (addr & 0x7) | ((byte_sel & 0x1) << 3)  # wr_en=0
    await ClockCycles(dut.clk, 1)


async def gpio_write(dut, addr: int, value: int):
    """Write 16-bit value (MSB byte then LSB byte) to register."""
    await gpio_write_byte(dut, addr, 0, (value >> 8) & 0xFF)
    await gpio_write_byte(dut, addr, 1, value & 0xFF)


async def gpio_read_byte(dut, addr: int, byte_sel: int) -> int:
    """Read one byte (MSB or LSB) from addressed register (combinational)."""
    dut.uio_in.value = (addr & 0x7) | ((byte_sel & 0x1) << 3)
    await ClockCycles(dut.clk, 1)
    return int(dut.uo_out.value)


async def gpio_read(dut, addr: int) -> int:
    """Read 16-bit value from register."""
    msb = await gpio_read_byte(dut, addr, 0)
    lsb = await gpio_read_byte(dut, addr, 1)
    return (msb << 8) | lsb


async def gpio_start(dut):
    """Assert rising edge on start_in (uio_in[6])."""
    dut.uio_in.value = int(dut.uio_in.value) & ~(1 << 6)
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = int(dut.uio_in.value) | (1 << 6)
    await ClockCycles(dut.clk, 1)   # sync stage 1
    await ClockCycles(dut.clk, 1)   # sync stage 2: start fires
    dut.uio_in.value = int(dut.uio_in.value) & ~(1 << 6)
    await ClockCycles(dut.clk, 1)


async def poll_done(dut, timeout_cycles: int = 500) -> bool:
    """Poll STAT register until done_latch (bit 0) asserts or timeout."""
    for _ in range(timeout_cycles):
        stat = await gpio_read(dut, 1)  # STAT register
        if stat & 0x01:
            return True
        await ClockCycles(dut.clk, 2)
    return False


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_kalman_spi(dut):
    dut._log.info("tt_um_joram200: parallel GPIO Kalman update test (INT10 Q4.6)")

    clock = Clock(dut.clk, 20, units="ns")   # 50 MHz
    cocotb.start_soon(clock.start())

    # Initial state
    dut.ena.value     = 1
    dut.ui_in.value   = 0
    dut.uio_in.value  = 0
    dut.rst_n.value   = 0
    await ClockCycles(dut.clk, 20)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Scenario: z=1.5, x_in=0, P_in=1.0, R=5.0 (default R_REG=0x140)
    Z_VAL  = 0x0060   # 1.5 × 64 = 96  in Q4.6
    X_IN   = 0x0000   # 0.0 × 64 = 0
    P_IN   = 0x0040   # 1.0 × 64 = 64

    dut._log.info(f"Writing z=0x{Z_VAL:04X}, x_in=0x{X_IN:04X}, P_in=0x{P_IN:04X}")

    await gpio_write(dut, 2, Z_VAL)   # reg 2 = z
    await gpio_write(dut, 3, X_IN)    # reg 3 = x_in
    await gpio_write(dut, 5, P_IN)    # reg 5 = P_in

    dut._log.info("Asserting start")
    await gpio_start(dut)

    dut._log.info("Polling for done...")
    ok = await poll_done(dut, timeout_cycles=500)
    assert ok, "Timeout: done_latch never asserted within 500 polls"
    dut._log.info("Done asserted")

    x_hw_raw = await gpio_read(dut, 4)   # reg 4 = x_out
    P_hw_raw = await gpio_read(dut, 6)   # reg 6 = P_out

    # Sign-extend 10-bit values from 16-bit read
    def sign_ext10(v):
        v &= 0x3FF
        return v - 0x400 if v & 0x200 else v

    x_hw = sign_ext10(x_hw_raw)
    P_hw = sign_ext10(P_hw_raw)
    dut._log.info(f"HW results: x_out=0x{x_hw_raw:04X} ({x_hw}), P_out=0x{P_hw_raw:04X} ({P_hw})")

    # Golden reference (integer Q4.6 arithmetic)
    R_DEFAULT = 0x140   # 5.0 × 64 = 320
    x_ref, P_ref = kalman_ref_q46(
        sign_ext10(Z_VAL), sign_ext10(X_IN), sign_ext10(P_IN), R_DEFAULT
    )
    dut._log.info(f"Reference: x_out={x_ref} (0x{x_ref & 0x3FF:04X}), P_out={P_ref} (0x{P_ref & 0x3FF:04X})")

    # ULP comparison (integer distance)
    x_ulp = abs(x_hw - x_ref)
    P_ulp = abs(P_hw - P_ref)

    dut._log.info(f"x_out ULP={x_ulp}, P_out ULP={P_ulp}")

    assert x_ulp <= ULP_THRESHOLD, \
        f"x_out: hw={x_hw} ref={x_ref} ULP={x_ulp} > {ULP_THRESHOLD}"
    assert P_ulp <= ULP_THRESHOLD, \
        f"P_out: hw={P_hw} ref={P_ref} ULP={P_ulp} > {ULP_THRESHOLD}"

    dut._log.info(f"All outputs within {ULP_THRESHOLD} ULP — PASS")
