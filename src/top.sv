`timescale 1ns/1ps
// =============================================================================
// top.sv — Tiny Tapeout standard top-level wrapper for tt_um_joram200
// INT10 Q4.6 fixed-point, 1D (scalar) state.
//
// Instantiates:
//   u_par  par_reg       (interface.sv) — parallel GPIO register file
//   u_core kalman_update (compute_core.sv) — 1D scalar Kalman update FSM
//
// Pin assignments:
//   ui_in[7:0]  → wr_data[7:0]   write data byte
//   uio_in[2:0] → addr[2:0]      register address
//   uio_in[3]   → byte_sel       0=MSB byte, 1=LSB byte
//   uio_in[4]   → wr_en          write enable (rising-edge triggered)
//   uio_in[5]   → sw_rst_in      software reset (level)
//   uio_in[6]   → start_in       start trigger (rising-edge one-shot)
//   uio_in[7]   → (unused)
//   uio_oe      = 8'h00           (all bidir pins configured as input)
//   uio_out     = 8'h00
//   uo_out[7:0] → rd_data[7:0]   read data byte (addressed register, byte selected)
// =============================================================================
module tt_um_joram200 (
    input  logic [7:0] ui_in,
    output logic [7:0] uo_out,
    input  logic [7:0] uio_in,
    output logic [7:0] uio_out,
    output logic [7:0] uio_oe,
    input  logic       ena,
    input  logic       clk,
    input  logic       rst_n
);

    // All bidir pins are inputs (no MISO output)
    assign uio_oe  = 8'h00;
    assign uio_out = 8'h00;

    // Core control / status wires
    logic        core_start, sw_rst_w;
    logic        core_done,  core_busy;

    // Register file wires — INT10 Q4.6, 1D scalar state
    logic [9:0] z_w;
    logic [9:0] r_val_w;
    logic [9:0] x_in_w;
    logic [9:0] P_in_w;
    logic [9:0] x_out_w;
    logic [9:0] P_out_w;

    // Read data byte from register file
    logic [7:0]  rd_data_w;

    // Parallel GPIO register file
    par_reg u_par (
        .clk        (clk),
        .rst_n      (rst_n),
        .wr_data    (ui_in),
        .addr       (uio_in[2:0]),
        .byte_sel   (uio_in[3]),
        .wr_en      (uio_in[4]),
        .sw_rst_in  (uio_in[5]),
        .start_in   (uio_in[6]),
        .core_start (core_start),
        .sw_rst     (sw_rst_w),
        .z_reg      (z_w),
        .x_in_reg   (x_in_w),
        .P_in_reg   (P_in_w),
        .r_val      (r_val_w),
        .done       (core_done),
        .busy       (core_busy),
        .x_out      (x_out_w),
        .P_out      (P_out_w),
        .rd_data    (rd_data_w)
    );

    // Kalman update core; soft-reset via sw_rst from register file
    kalman_update u_core (
        .clk       (clk),
        .rst_n     (rst_n & ~sw_rst_w),
        .start     (core_start),
        .z         (z_w),
        .x_in      (x_in_w),
        .P_in      (P_in_w),
        .r_val     (r_val_w),
        .x_out     (x_out_w),
        .P_out     (P_out_w),
        .done      (core_done),
        .busy      (core_busy)
    );

    // Read data byte drives all output pins
    assign uo_out = rd_data_w;

    // Suppress unused port warnings
    logic _ena_unused;
    assign _ena_unused = ena;
    logic _uio7_unused;
    assign _uio7_unused = uio_in[7];

endmodule
