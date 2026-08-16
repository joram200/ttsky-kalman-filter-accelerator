`timescale 1ns/1ps
// =============================================================================
// top.sv — Tiny Tapeout standard top-level wrapper for tt_um_joram200
// Task 5 Option A: internal data widths changed to F32 (32-bit per element).
//
// Instantiates:
//   u_spi  spi_slave     (interface.sv) — SPI register file, 28 registers
//   u_core kalman_update (compute_core.sv) — scalar Kalman update FSM
//
// Pin assignments:
//   uio_in[0]  → SCLK (SPI clock)
//   uio_in[1]  → MOSI (SPI data in)
//   uio_out[2] → MISO (SPI data out)
//   uio_in[3]  → CS_N (SPI chip select, active low)
//   uio_oe     = 8'b0000_0100  (only bit 2 is output)
//   uo_out[0]  = busy
//   uo_out[1]  = done
//   uo_out[7:2] = 6'b0
//   ui_in       unused
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

    // SPI signals
    logic sclk, mosi, miso, cs_n;
    assign sclk  = uio_in[0];
    assign mosi  = uio_in[1];
    assign cs_n  = uio_in[3];

    // uio direction: only bit 2 (MISO) is driven by ASIC
    assign uio_oe  = 8'b0000_0100;
    assign uio_out = {5'b0, miso, 2'b0};

    // Core control / status wires
    logic        core_start, sw_rst_w;
    logic        core_done,  core_busy;

    // Register file wires — F32 widths
    logic [31:0]  z_w;
    logic [31:0]  r_val_w;
    logic [95:0]  x_out_w;  // 3×32
    logic [287:0] P_out_w;  // 9×32

    // Write-event wires (SPI interface → kalman_update)
    logic        x_wr_en_w;
    logic [1:0]  x_wr_idx_w;
    logic [31:0] x_wr_val_w;
    logic        p_wr_en_w;
    logic [3:0]  p_wr_idx_w;
    logic [31:0] p_wr_val_w;

    // SPI slave
    spi_slave u_spi (
        .clk        (clk),
        .rst_n      (rst_n),
        .sclk       (sclk),
        .mosi       (mosi),
        .cs_n       (cs_n),
        .miso       (miso),
        .core_start (core_start),
        .sw_rst     (sw_rst_w),
        .z_reg      (z_w),
        .x_wr_en    (x_wr_en_w),
        .x_wr_idx   (x_wr_idx_w),
        .x_wr_val   (x_wr_val_w),
        .p_wr_en    (p_wr_en_w),
        .p_wr_idx   (p_wr_idx_w),
        .p_wr_val   (p_wr_val_w),
        .r_val      (r_val_w),
        .done       (core_done),
        .busy       (core_busy),
        .x_out      (x_out_w),
        .P_out      (P_out_w)
    );

    // Kalman update core; soft-reset via sw_rst from SPI slave
    kalman_update u_core (
        .clk       (clk),
        .rst_n     (rst_n & ~sw_rst_w),
        .start     (core_start),
        .z         (z_w),
        .x_wr_en   (x_wr_en_w),
        .x_wr_idx  (x_wr_idx_w),
        .x_wr_val  (x_wr_val_w),
        .p_wr_en   (p_wr_en_w),
        .p_wr_idx  (p_wr_idx_w),
        .p_wr_val  (p_wr_val_w),
        .r_val     (r_val_w),
        .x_out     (x_out_w),
        .P_out     (P_out_w),
        .done      (core_done),
        .busy      (core_busy)
    );

    // Status outputs
    assign uo_out = {6'b0, core_done, core_busy};

    // Suppress unused port warning
    logic _ena_unused;
    assign _ena_unused = ena;
    logic [7:0] _ui_unused;
    assign _ui_unused = ui_in;

endmodule
