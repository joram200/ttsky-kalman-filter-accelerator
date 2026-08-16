`timescale 1ns/1ps
// =============================================================================
// interface.sv — SPI slave register file for tt_um_joram200
// Task 5 Option A: core data widths are F32 (32-bit per element).
//
// Protocol: CPOL=0, CPHA=0, MSB-first, 8-bit frames.
// Transaction format: 1 command byte then N×8 data bytes while CS_N is low.
// Command byte: bit[7]=R/W (1=write, 0=read), bits[6:0]=start register address.
// Each register = 8 data bytes (64-bit SPI word). Address auto-increments
// by 1 per completed 64-bit word within a CS_N-low burst.
//
// F32 data occupies the LOWER 32 bits of each 64-bit SPI word.
// Host sends: [63:32]=zeros, [31:0]=F32 value (MSB-first byte order).
// DUT returns: [63:32]=zeros, [31:0]=F32 value.
//
// Register map (28 registers, indices 0–27):
//   0   CTRL      W      [0]=start one-shot pulse, [1]=sw_rst (level)
//   1   STAT      R      [0]=done_latch (clears on STAT read), [1]=busy
//   2   z         W      scalar measurement F32
//   3   x_in[0]   W      prior state element 0 F32
//   4   x_in[1]   W      prior state element 1 F32
//   5   x_in[2]   W      prior state element 2 F32
//   6   x_out[0]  R      corrected state element 0 F32
//   7   x_out[1]  R      corrected state element 1 F32
//   8   x_out[2]  R      corrected state element 2 F32
//   9   P_in[0]   W      prior covariance [0,0] F32
//  10   P_in[1]   W      [0,1]
//  11   P_in[2]   W      [0,2]
//  12   P_in[3]   W      [1,0]
//  13   P_in[4]   W      [1,1]
//  14   P_in[5]   W      [1,2]
//  15   P_in[6]   W      [2,0]
//  16   P_in[7]   W      [2,1]
//  17   P_in[8]   W      [2,2]
//  18   P_out[0]  R      updated covariance [0,0] F32
//  19   P_out[1]  R      [0,1]
//  20   P_out[2]  R      [0,2]
//  21   P_out[3]  R      [1,0]
//  22   P_out[4]  R      [1,1]
//  23   P_out[5]  R      [1,2]
//  24   P_out[6]  R      [2,0]
//  25   P_out[7]  R      [2,1]
//  26   P_out[8]  R      [2,2]
//  27   R_REG     R/W    measurement noise R; reset default = 5.0 F32
// =============================================================================
module spi_slave (
    input  logic        clk,
    input  logic        rst_n,
    // SPI pins (CPOL=0, CPHA=0)
    input  logic        sclk,
    input  logic        mosi,
    input  logic        cs_n,
    output logic        miso,
    // Core control
    output logic        core_start,   // 1-cycle pulse when CTRL[0] written as 1
    output logic        sw_rst,       // level: mirrors CTRL[1]
    // Kalman inputs (written by host) — F32 widths
    output logic [31:0]  z_reg,
    // Write-event ports: emitted when host writes x_in (addrs 3-5) or P_in (addrs 9-17)
    output logic        x_wr_en,
    output logic [1:0]  x_wr_idx,
    output logic [31:0] x_wr_val,
    output logic        p_wr_en,
    output logic [3:0]  p_wr_idx,
    output logic [31:0] p_wr_val,
    output logic [31:0]  r_val,      // R_REG; reset default = 5.0 F32
    // Kalman status / outputs (read by host) — F32 widths
    input  logic        done,
    input  logic        busy,
    input  logic [95:0]  x_out,      // 3×32
    input  logic [287:0] P_out       // 9×32
);

    // -------------------------------------------------------------------------
    // Two-stage synchroniser for SPI pins → clk domain
    // -------------------------------------------------------------------------
    logic sclk_r1, sclk_r2;
    logic cs_n_r1, cs_n_r2;
    logic mosi_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_r1 <= 1'b0; sclk_r2 <= 1'b0;
            cs_n_r1 <= 1'b1; cs_n_r2 <= 1'b1;
            mosi_r  <= 1'b0;
        end else begin
            sclk_r1 <= sclk;   sclk_r2 <= sclk_r1;
            cs_n_r1 <= cs_n;   cs_n_r2 <= cs_n_r1;
            mosi_r  <= mosi;
        end
    end

    wire sclk_rise = ( sclk_r1 && !sclk_r2);
    wire sclk_fall = (!sclk_r1 &&  sclk_r2);
    wire cs_fall   = (!cs_n_r1 &&  cs_n_r2);
    wire cs_rise   = ( cs_n_r1 && !cs_n_r2);
    wire cs_active = !cs_n_r1;

    // -------------------------------------------------------------------------
    // Transaction state machine
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        ST_IDLE = 2'd0,
        ST_CMD  = 2'd1,
        ST_DATA = 2'd2
    } spi_st_t;

    spi_st_t st;

    logic [2:0] bit_cnt;   // bit position within current byte
    logic [2:0] byte_cnt;  // byte index within current 64-bit word
    logic       rw;        // 1=write, 0=read
    logic [6:0] cur_addr;  // current register address

    // Incoming shift register
    logic [7:0]  rx_byte;
    logic [55:0] rx_acc;   // accumulates bytes 0..6

    // MISO shift register (64-bit for SPI word)
    logic [63:0] tx_shift;

    // done latch
    logic done_latch;

    // Default R value (5.0 in F32)
    localparam logic [31:0] R_DEFAULT = 32'h40A0_0000; // 5.0 F32

    // Unpack flat input ports to internal arrays for x_out and P_out
    logic [31:0] x_out_arr [0:2];
    logic [31:0] P_out_arr [0:8];

    assign x_out_arr[0] = x_out[31:0];
    assign x_out_arr[1] = x_out[63:32];
    assign x_out_arr[2] = x_out[95:64];

    assign P_out_arr[0] = P_out[31:0];
    assign P_out_arr[1] = P_out[63:32];
    assign P_out_arr[2] = P_out[95:64];
    assign P_out_arr[3] = P_out[127:96];
    assign P_out_arr[4] = P_out[159:128];
    assign P_out_arr[5] = P_out[191:160];
    assign P_out_arr[6] = P_out[223:192];
    assign P_out_arr[7] = P_out[255:224];
    assign P_out_arr[8] = P_out[287:256];

    // -------------------------------------------------------------------------
    // Register read function — returns 64-bit SPI word.
    // F32 values are zero-extended: {32'h0, f32_value}.
    // -------------------------------------------------------------------------
    function automatic logic [63:0] reg_read_fn (input logic [6:0] addr);
        case (addr)
            7'd1:  reg_read_fn = {62'b0, busy, done_latch};
            7'd6:  reg_read_fn = {32'h0, x_out_arr[0]};
            7'd7:  reg_read_fn = {32'h0, x_out_arr[1]};
            7'd8:  reg_read_fn = {32'h0, x_out_arr[2]};
            7'd18: reg_read_fn = {32'h0, P_out_arr[0]};
            7'd19: reg_read_fn = {32'h0, P_out_arr[1]};
            7'd20: reg_read_fn = {32'h0, P_out_arr[2]};
            7'd21: reg_read_fn = {32'h0, P_out_arr[3]};
            7'd22: reg_read_fn = {32'h0, P_out_arr[4]};
            7'd23: reg_read_fn = {32'h0, P_out_arr[5]};
            7'd24: reg_read_fn = {32'h0, P_out_arr[6]};
            7'd25: reg_read_fn = {32'h0, P_out_arr[7]};
            7'd26: reg_read_fn = {32'h0, P_out_arr[8]};
            7'd27: reg_read_fn = {32'h0, r_val};
            default: reg_read_fn = 64'h0;
        endcase
    endfunction

    // Pre-compute constant bit/range-selects as wires (iverilog 12 "sorry" fix).
    wire [63:0] tx_shift_shifted = {tx_shift[62:0], 1'b0};
    wire        miso_hold        = tx_shift[63];

    // rx_byte slices
    wire [6:0] rx_byte_lo   = rx_byte[6:0];
    wire       rx_byte_b6   = rx_byte[6];
    wire [5:0] rx_byte_b5_0 = rx_byte[5:0];
    wire       rx_byte_b0   = rx_byte[0];

    // rx_acc slices
    wire [47:0] rx_acc_lo   = rx_acc[47:0];   // for accumulation
    // Lower 32-bit word extraction: bytes 4..7 of the 64-bit SPI word.
    // rx_acc[23:0] = bytes 4..6 (3 bytes), then rx_byte_lo (7 bits) + mosi_r (1 bit) = byte 7.
    wire [23:0] rx_acc_lo24 = rx_acc[23:0];   // for F32 lower-word extraction

    // -------------------------------------------------------------------------
    // Main sequential block
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= ST_IDLE;
            bit_cnt    <= 3'd0;
            byte_cnt   <= 3'd0;
            rw         <= 1'b0;
            cur_addr   <= 7'd0;
            rx_byte    <= 8'h0;
            rx_acc     <= 56'h0;
            tx_shift   <= 64'h0;
            miso       <= 1'b0;
            done_latch <= 1'b0;
            core_start <= 1'b0;
            sw_rst     <= 1'b0;
            z_reg      <= 32'h0;
            x_wr_en    <= 1'b0;
            x_wr_idx   <= 2'd0;
            x_wr_val   <= 32'h0;
            p_wr_en    <= 1'b0;
            p_wr_idx   <= 4'd0;
            p_wr_val   <= 32'h0;
            r_val      <= R_DEFAULT;
        end else begin
            core_start <= 1'b0;
            x_wr_en    <= 1'b0;
            p_wr_en    <= 1'b0;

            if (done) done_latch <= 1'b1;

            if (cs_fall) begin
                st       <= ST_CMD;
                bit_cnt  <= 3'd0;
                byte_cnt <= 3'd0;
                rx_byte  <= 8'h0;
                rx_acc   <= 56'h0;
            end

            if (cs_rise) begin
                st <= ST_IDLE;
            end

            if (sclk_rise && cs_active) begin
                case (st)

                    ST_CMD: begin
                        rx_byte <= {rx_byte_lo, mosi_r};
                        if (bit_cnt == 3'd7) begin
                            rw       <= rx_byte_b6;
                            cur_addr <= {rx_byte_b5_0, mosi_r};
                            st       <= ST_DATA;
                            bit_cnt  <= 3'd0;
                            byte_cnt <= 3'd0;
                            rx_byte  <= 8'h0;
                            rx_acc   <= 56'h0;
                            if (!rx_byte_b6) begin
                                tx_shift <= reg_read_fn({rx_byte_b5_0, mosi_r});
                                if ({rx_byte_b5_0, mosi_r} == 7'd1) done_latch <= 1'b0;
                            end
                        end else begin
                            bit_cnt <= bit_cnt + 3'd1;
                        end
                    end

                    ST_DATA: begin
                        rx_byte <= {rx_byte_lo, mosi_r};
                        if (bit_cnt == 3'd7) begin
                            bit_cnt <= 3'd0;
                            if (byte_cnt == 3'd7) begin
                                // ---- 64-bit word complete ----
                                // F32 value = lower 32 bits of SPI word
                                //   = {rx_acc_lo24[23:0], rx_byte_lo[6:0], mosi_r[0]}
                                byte_cnt <= 3'd0;
                                rx_acc   <= 56'h0;
                                rx_byte  <= 8'h0;
                                if (rw) begin
                                    case (cur_addr)
                                        7'd0: begin
                                            // CTRL: bit[0]=start, bit[1]=sw_rst
                                            core_start <= mosi_r;
                                            sw_rst     <= rx_byte_b0;
                                        end
                                        7'd2:  z_reg <= {rx_acc_lo24, rx_byte_lo, mosi_r};
                                        7'd3: begin
                                            x_wr_en  <= 1'b1; x_wr_idx <= 2'd0;
                                            x_wr_val <= {rx_acc_lo24, rx_byte_lo, mosi_r};
                                        end
                                        7'd4: begin
                                            x_wr_en  <= 1'b1; x_wr_idx <= 2'd1;
                                            x_wr_val <= {rx_acc_lo24, rx_byte_lo, mosi_r};
                                        end
                                        7'd5: begin
                                            x_wr_en  <= 1'b1; x_wr_idx <= 2'd2;
                                            x_wr_val <= {rx_acc_lo24, rx_byte_lo, mosi_r};
                                        end
                                        7'd9: begin
                                            p_wr_en  <= 1'b1; p_wr_idx <= 4'd0;
                                            p_wr_val <= {rx_acc_lo24, rx_byte_lo, mosi_r};
                                        end
                                        7'd10: begin
                                            p_wr_en  <= 1'b1; p_wr_idx <= 4'd1;
                                            p_wr_val <= {rx_acc_lo24, rx_byte_lo, mosi_r};
                                        end
                                        7'd11: begin
                                            p_wr_en  <= 1'b1; p_wr_idx <= 4'd2;
                                            p_wr_val <= {rx_acc_lo24, rx_byte_lo, mosi_r};
                                        end
                                        7'd12: begin
                                            p_wr_en  <= 1'b1; p_wr_idx <= 4'd3;
                                            p_wr_val <= {rx_acc_lo24, rx_byte_lo, mosi_r};
                                        end
                                        7'd13: begin
                                            p_wr_en  <= 1'b1; p_wr_idx <= 4'd4;
                                            p_wr_val <= {rx_acc_lo24, rx_byte_lo, mosi_r};
                                        end
                                        7'd14: begin
                                            p_wr_en  <= 1'b1; p_wr_idx <= 4'd5;
                                            p_wr_val <= {rx_acc_lo24, rx_byte_lo, mosi_r};
                                        end
                                        7'd15: begin
                                            p_wr_en  <= 1'b1; p_wr_idx <= 4'd6;
                                            p_wr_val <= {rx_acc_lo24, rx_byte_lo, mosi_r};
                                        end
                                        7'd16: begin
                                            p_wr_en  <= 1'b1; p_wr_idx <= 4'd7;
                                            p_wr_val <= {rx_acc_lo24, rx_byte_lo, mosi_r};
                                        end
                                        7'd17: begin
                                            p_wr_en  <= 1'b1; p_wr_idx <= 4'd8;
                                            p_wr_val <= {rx_acc_lo24, rx_byte_lo, mosi_r};
                                        end
                                        7'd27: r_val <= {rx_acc_lo24, rx_byte_lo, mosi_r};
                                        default: ;
                                    endcase
                                end
                                cur_addr <= cur_addr + 7'd1;
                                if (!rw) begin
                                    tx_shift <= reg_read_fn(cur_addr + 7'd1);
                                    if ((cur_addr + 7'd1) == 7'd1) done_latch <= 1'b0;
                                end
                            end else begin
                                rx_acc   <= {rx_acc_lo, rx_byte_lo, mosi_r};
                                byte_cnt <= byte_cnt + 3'd1;
                                rx_byte  <= 8'h0;
                            end
                        end else begin
                            bit_cnt <= bit_cnt + 3'd1;
                        end
                    end

                    default: ;
                endcase
            end

            if (sclk_fall && cs_active) begin
                miso     <= miso_hold;
                tx_shift <= tx_shift_shifted;
            end
        end
    end

endmodule
