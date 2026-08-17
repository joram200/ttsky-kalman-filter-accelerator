`timescale 1ns/1ps
// =============================================================================
// interface.sv — Parallel GPIO register file for tt_um_joram200
// INT10 Q4.6 fixed-point version, 1D (scalar) state.
//
// Write protocol (byte-serial, MSB first):
//   1. Place register address on addr[2:0], 0 on byte_sel, MSB byte on wr_data.
//   2. Pulse wr_en high for ≥1 clk cycle → wr_data[1:0] stored in bits [9:8].
//   3. Place LSB byte on wr_data, 1 on byte_sel.
//   4. Pulse wr_en again → LSB stored, 10-bit write complete.
//
// Read protocol (combinational):
//   Set addr[2:0] and byte_sel; rd_data is valid the same cycle.
//
// Register map (8 registers, indices 0–7):
//   0   (reserved)
//   1   STAT      R    [0]=done_latch (clears on next start), [1]=busy
//   2   z         W    scalar measurement Q4.6
//   3   x_in      W    prior scalar state Q4.6
//   4   x_out     R    corrected scalar state Q4.6
//   5   P_in      W    prior scalar covariance Q4.6
//   6   P_out     R    updated scalar covariance Q4.6
//   7   R_REG     R/W  measurement noise R; reset default = 5.0 (0x140 Q4.6)
//
// Dedicated control pins (not register-mapped):
//   start_in  → rising-edge one-shot core_start; also clears done_latch
//   sw_rst_in → level, passed to kalman_update as combined reset
// =============================================================================
module par_reg (
    input  logic        clk,
    input  logic        rst_n,
    // Parallel data/control inputs
    input  logic [7:0]  wr_data,    // write data byte
    input  logic [2:0]  addr,       // register address
    input  logic        byte_sel,   // 0=MSB byte, 1=LSB byte
    input  logic        wr_en,      // write enable (rising-edge triggered)
    input  logic        start_in,   // start (rising-edge → one-shot core_start)
    input  logic        sw_rst_in,  // software reset (level)
    // Core control outputs
    output logic        core_start,
    output logic        sw_rst,
    // Register file outputs
    output logic [9:0]  z_reg,
    output logic [9:0]  x_in_reg,
    output logic [9:0]  P_in_reg,
    output logic [9:0]  r_val,
    // Core status inputs
    input  logic        done,
    input  logic        busy,
    input  logic [9:0]  x_out,
    input  logic [9:0]  P_out,
    // Byte-wide read data output
    output logic [7:0]  rd_data
);

    localparam logic [9:0] R_DEFAULT = 10'h140; // 5.0 in Q4.6

    // -------------------------------------------------------------------------
    // Two-stage synchronisers for edge-triggered control inputs
    // -------------------------------------------------------------------------
    logic wr_en_r1,  wr_en_r2;
    logic start_r1,  start_r2;
    logic sw_rst_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_en_r1  <= 1'b0; wr_en_r2  <= 1'b0;
            start_r1  <= 1'b0; start_r2  <= 1'b0;
            sw_rst_r  <= 1'b0;
        end else begin
            wr_en_r1  <= wr_en;    wr_en_r2  <= wr_en_r1;
            start_r1  <= start_in; start_r2  <= start_r1;
            sw_rst_r  <= sw_rst_in;
        end
    end

    wire wr_rise    = wr_en_r1  && !wr_en_r2;
    wire start_rise = start_r1  && !start_r2;

    assign sw_rst = sw_rst_r;

    // -------------------------------------------------------------------------
    // Register file
    // -------------------------------------------------------------------------
    logic [9:0]  x_in_r;
    logic [9:0]  P_in_r;
    logic        done_latch;

    assign x_in_reg = x_in_r;
    assign P_in_reg = P_in_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            core_start <= 1'b0;
            done_latch <= 1'b0;
            z_reg      <= 10'h0;
            x_in_r     <= 10'h0;
            P_in_r     <= 10'h0;
            r_val      <= R_DEFAULT;
        end else begin
            core_start <= start_rise;
            if (done)        done_latch <= 1'b1;
            if (start_rise)  done_latch <= 1'b0;

            if (wr_rise) begin
                if (!byte_sel) begin
                    case (addr)
                        3'd2: z_reg[9:8]   <= wr_data[1:0];
                        3'd3: x_in_r[9:8]  <= wr_data[1:0];
                        3'd5: P_in_r[9:8]  <= wr_data[1:0];
                        3'd7: r_val[9:8]   <= wr_data[1:0];
                        default: ;
                    endcase
                end else begin
                    case (addr)
                        3'd2: z_reg[7:0]   <= wr_data;
                        3'd3: x_in_r[7:0]  <= wr_data;
                        3'd5: P_in_r[7:0]  <= wr_data;
                        3'd7: r_val[7:0]   <= wr_data;
                        default: ;
                    endcase
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Combinational read mux (no sequential state — no MISO shift register)
    // -------------------------------------------------------------------------
    logic [15:0] rd_word;
    always_comb begin
        case (addr)
            3'd1:    rd_word = {14'b0, busy, done_latch};
            3'd4:    rd_word = {6'b0, x_out};
            3'd6:    rd_word = {6'b0, P_out};
            3'd7:    rd_word = {6'b0, r_val};
            default: rd_word = 16'h0;
        endcase
    end

    assign rd_data = byte_sel ? rd_word[7:0] : rd_word[15:8];

endmodule
