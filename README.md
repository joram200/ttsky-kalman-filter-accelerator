![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# Kalman Filter Hardware Accelerator

**INT10 Q4.6 fixed-point scalar Kalman filter measurement-update step, implemented as a Tiny Tapeout 1×1 tile on sky130 HD.**

- [Read the full project documentation](docs/info.md)

## What is Tiny Tapeout?

Tiny Tapeout is an educational project that makes it easier and cheaper than ever to get your digital designs manufactured on a real chip. To learn more and get started, visit https://tinytapeout.com.

## Project overview

This design accelerates the **measurement-update ("correct") step** of a discrete-time 1D scalar Kalman filter using **10-bit signed Q4.6 fixed-point arithmetic** (scale factor 64, range ±7.984, resolution 1/64 ≈ 0.0156). It targets a single 1×1 Tiny Tapeout tile using the sky130 HD standard-cell library.

The accelerator communicates with a host processor via a **parallel GPIO register file** — no SPI or AXI bus required. All 8 data input pins carry a write-data byte; 7 bidirectional pins carry the address, byte select, write enable, start trigger, and software reset.

### Source files

| File | Description |
|---|---|
| `src/top.sv` | Tiny Tapeout top-level wrapper (`tt_um_joram200`) |
| `src/interface.sv` | Parallel GPIO register file (`par_reg`) |
| `src/compute_core.sv` | Multiplier, divider, and Kalman FSM (`kalman_update`) |

### Compute core modules

| Module | Function | Latency |
|---|---|---|
| `int10_mul_seq` | Signed 10×10 → 10 shift-and-add (Baugh-Wooley), Q4.6 | 11 cycles |
| `int10_div` | Restoring divider: floor(4096 / S_Q46), saturates at 0x1FF | 14 cycles |
| `kalman_update` | 8-state FSM; 1 shared multiplier + 1 divider | ~50 cycles |

### Pin assignments

| Pin | Direction | Function |
|---|---|---|
| `ui[7:0]` | Input | WR_DATA[7:0] — write data byte |
| `uo[7:0]` | Output | RD_DATA[7:0] — read data byte (combinational) |
| `uio[2:0]` | Input | ADDR[2:0] — register address |
| `uio[3]` | Input | BYTE_SEL — 0 = MSB byte, 1 = LSB byte |
| `uio[4]` | Input | WR_EN — write enable (rising-edge triggered) |
| `uio[5]` | Input | SW_RST — software reset (active-high level) |
| `uio[6]` | Input | START — start trigger (rising-edge one-shot) |
| `uio[7]` | Input | (unused) |

### Register map

| Addr | Name | Dir | Description |
|---|---|---|---|
| 0 | — | — | Reserved |
| 1 | STAT | R | `[0]` = done\_latch (clears on next start), `[1]` = busy |
| 2 | z | W | Scalar measurement (Q4.6) |
| 3 | x\_in | W | Prior state estimate (Q4.6) |
| 4 | x\_out | R | Corrected state estimate (Q4.6) |
| 5 | P\_in | W | Prior error covariance (Q4.6) |
| 6 | P\_out | R | Updated error covariance (Q4.6) |
| 7 | R\_REG | R/W | Measurement noise covariance R (Q4.6, default 5.0 = 0x140) |

Registers are 10-bit wide; each is read/written as two bytes (MSB byte first, bits [9:8] in bits [1:0] of the data byte).

## How to test

See [docs/info.md](docs/info.md) for a full description of the write/read protocol, a worked numerical example, and testbench details.

## Resources

- [FAQ](https://tinytapeout.com/faq/)
- [Digital design lessons](https://tinytapeout.com/digital_design/)
- [Learn how semiconductors work](https://tinytapeout.com/siliwiz/)
- [Join the community](https://tinytapeout.com/discord)
- [Build your design locally](https://www.tinytapeout.com/guides/local-hardening/)
