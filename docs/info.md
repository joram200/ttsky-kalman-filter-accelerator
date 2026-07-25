<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

This project implements the **Kalman filter measurement-update ("correct") step** as a fixed-function hardware accelerator, offloading the most compute-intensive portion of a discrete-time Kalman filter cycle from a soft-core RISC-V processor.

### Problem domain

The Kalman filter tracks a 3-state system (position, velocity, acceleration) with a scalar position-only measurement. The time-update (predict) step is kept in software; only the measurement-update is accelerated here. The observation model is hardwired as **H = [1, 0, 0]** and the measurement noise covariance **R** is programmable at runtime.

### Architecture overview

The accelerator is controlled via an **AXI4-Lite slave register file** (`interface.sv`) wrapped by a top-level integration module (`top.sv`). The computational core (`compute_core.sv`) contains four sub-modules:

| Module | Function | Latency |
|---|---|---|
| `f64_mul` | IEEE-754 double-precision multiplier (3-stage pipeline) | 2 cycles |
| `f64_add` | IEEE-754 double-precision adder (4-stage pipeline) | 3 cycles |
| `gemm_systolic` | 3×3 weight-stationary systolic GEMM, 24-state FSM | 15 cycles |
| `kalman_update` | Top-level Kalman FSM (51 states) | ~80 cycles |

### Register map (8-byte stride)

| Offset | Name | Dir | Description |
|---|---|---|---|
| `0x00` | CTRL | R/W | `[0]` = start pulse, `[1]` = software reset |
| `0x08` | STAT | RO | `[0]` = done (1-cycle pulse), `[1]` = busy |
| `0x10` | z | WO | Scalar measurement (F64) |
| `0x18–0x28` | x\_in[0:2] | WO | Prior state vector (F64 × 3) |
| `0x30–0x40` | x\_out[0:2] | RO | Corrected state vector (F64 × 3) |
| `0x58–0x98` | P\_in[0:8] | WO | Prior covariance matrix (F64 × 9, row-major) |
| `0xA0–0xE0` | P\_out[0:8] | RO | Updated covariance matrix (F64 × 9, row-major) |
| `0xE8` | R\_REG | R/W | Measurement noise covariance R (F64, default 5.0) |

### Computation sequence (51-state FSM)

1. **INNOV**: Compute innovation y̅ = z − x\_in[0]
2. **S\_COMP**: Compute innovation covariance S = P\_in[0,0] + R
3. **NR0–NR2**: Three Newton-Raphson iterations to compute S⁻¹ (using DSP-friendly seed `0x7FDE_6000_0000_0000 − S`)
4. **K\_COMP**: Compute Kalman gain K = P\_in[:,0] × S⁻¹ (scalar multiply × 3)
5. **X\_CORR**: Correct state x\_out = x\_in + K × y̅ (multiply-accumulate × 3)
6. **P\_UPD**: Update covariance P\_out = (I − KH) × P\_in via the 3×3 systolic GEMM array
7. **DONE\_S**: Assert done for one cycle, return to IDLE

All arithmetic is **IEEE-754 double-precision (64-bit)**. The pipelined multiplier and adder run as always-on datapaths; the FSM steers operand muxes and captures outputs at the correct pipeline delay.

### Performance (benchmarked on RVfpgaEL2 SoC at 13 MHz)

| Metric | Software (Eigen C++) | HW Accelerator | Speedup |
|---|---|---|---|
| Latency per update | 9.55 ms | 87.8 µs | **108.8×** |
| Throughput | 104.7 samples/sec | 11,386 samples/sec | **108.7×** |
| Energy per update | ~1,910 µJ | ~37.1 µJ | **~51× lower** |
| Heap usage | 49,760 B | 0 B | — |

The dominant cost is AXI transaction latency (34 MMIO round-trips × ~2.6 µs = 87.8 µs/update); the compute core itself completes in ~1.3% of the total update time.

### Synthesis (Artix-7 xc7a100t, 100 MHz OOC)

| Resource | Used | Available | Utilization |
|---|---|---|---|
| Slice LUTs | 21,314 | 63,400 | 33.62% |
| Slice Registers | 14,151 | 126,800 | 11.16% |
| DSP48E1 | 153 | 240 | 63.75% |
| Block RAM | 0 | 135 | 0% |
| Total on-chip power | 0.509 W | — | — |

Timing: **WNS = +0.326 ns** at 100 MHz (all constraints met, 0 failing endpoints).

## How to test

The accelerator communicates entirely through its AXI4-Lite register interface. A full measurement-update cycle proceeds as follows:

### One-time setup

1. Write the measurement noise covariance R (IEEE-754 F64) to offset `0xE8`. The default value is R = 5.0 (`0x4014_0000_0000_0000`). Skip this step if using the default.

### Per-iteration procedure

1. **Write inputs** (in any order before triggering start):
   - Write scalar measurement z (F64) to `0x10`
   - Write prior state x\_in[0], x\_in[1], x\_in[2] (F64) to `0x18`, `0x20`, `0x28`
   - Write prior covariance P\_in[0]…P\_in[8] (F64, row-major) to `0x58`, `0x60`, …, `0x98`
2. **Start**: Write `0x0000_0000_0000_0001` to CTRL (`0x00`). This generates a one-cycle start pulse internally.
3. **Poll**: Read STAT (`0x08`) until bit[1] (busy) goes low, indicating the 51-state FSM has completed and outputs are stable.
4. **Read outputs**:
   - Read corrected state x\_out[0], x\_out[1], x\_out[2] (F64) from `0x30`, `0x38`, `0x40`
   - Read updated covariance P\_out[0]…P\_out[8] (F64, row-major) from `0xA0`, `0xA8`, …, `0xE0`
5. Pass x\_out and P\_out as x\_in and P\_in for the next iteration (after the software time-update step).

### Simulation testbench

The included testbench (`tb_top.sv`) drives 15 Kalman measurement-update iterations using:
- Measurement z = 0.5 (constant)
- Initial covariance P₀ = 100·I
- R = 5.0
- Pre-computed reference P matrices (from an Eigen C++ software run on RVfpgaEL2) for correctness checking

The testbench verifies all 12 outputs (3 x\_out + 9 P\_out) per iteration (180 total checks) against software-reference values with a ≤ 4 ULP tolerance, and prints `SIMULATION RESULT: PASS` if all checks pass.

To run simulation, compile all three RTL files (`compute_core.sv`, `interface.sv`, `top.sv`) and `tb_top.sv` with a SystemVerilog-capable simulator (e.g., Vivado xsim, ModelSim, or Verilator with appropriate wrappers).

## External hardware

None. This is a pure digital logic accelerator with no external components required. In the original SoC integration, the accelerator was memory-mapped at a fixed base address and accessed via MMIO from a VeerWolf EL2 RISC-V soft core, but the accelerator itself requires only a clock, reset, and AXI4-Lite bus connection.
