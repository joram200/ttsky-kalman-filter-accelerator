`timescale 1ns/1ps
// =============================================================================
// interface.sv — SPI slave register file for tt_um_joram200
// INT16 Q8.8 fixed-point version, 2D state (x ∈ ℝ², P ∈ ℝ²ˣ²).
//
// Protocol: CPOL=0, CPHA=0, MSB-first, 8-bit frames.
// Transaction: 1 command byte + 8 data bytes (64-bit SPI word).
// Command byte: bit[7]=R/W (1=write, 0=read), bits[6:0]=register address.
// INT16 value occupies the LOWER 16 bits of each 64-bit SPI word.
// Host sends: [63:16]=zeros, [15:0]=Q8.8 value (MSB-first byte order).
//
// Register map (17 registers, indices 0–16):
//   0   CTRL      W      [0]=start one-shot, [1]=sw_rst (level)
//   1   STAT      R      [0]=done_latch (clears on read), [1]=busy
//   2   z         W      scalar measurement Q8.8
//   3   x_in[0]   W      prior state element 0 Q8.8
//   4   x_in[1]   W      prior state element 1 Q8.8
//   5   x_out[0]  R      corrected state element 0 Q8.8
//   6   x_out[1]  R      corrected state element 1 Q8.8
//   7   P_in[0]   W      prior covariance [0,0] Q8.8
//   8   P_in[1]   W      prior covariance [0,1] Q8.8
//   9   P_in[2]   W      prior covariance [1,0] Q8.8
//  10   P_in[3]   W      prior covariance [1,1] Q8.8
//  11   P_out[0]  R      updated covariance [0,0] Q8.8
//  12   P_out[1]  R      updated covariance [0,1] Q8.8
//  13   P_out[2]  R      updated covariance [1,0] Q8.8
//  14   P_out[3]  R      updated covariance [1,1] Q8.8
//  15   R_REG     R/W   measurement noise R; reset default = 5.0 (0x0500)
// =============================================================================
module spi_slave (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        sclk,
    input  logic        mosi,
    input  logic        cs_n,
    output logic        miso,
    output logic        core_start,
    output logic        sw_rst,
    output logic [15:0] z_reg,
    output logic [31:0] x_in_reg,    // 2×16
    output logic [63:0] P_in_reg,    // 4×16
    output logic [15:0] r_val,
    input  logic        done,
    input  logic        busy,
    input  logic [31:0] x_out,       // 2×16
    input  logic [63:0] P_out        // 4×16
);

    // -------------------------------------------------------------------------
    // Two-stage synchroniser
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
            sclk_r1 <= sclk;  sclk_r2 <= sclk_r1;
            cs_n_r1 <= cs_n;  cs_n_r2 <= cs_n_r1;
            mosi_r  <= mosi;
        end
    end

    wire sclk_rise = ( sclk_r1 && !sclk_r2);
    wire sclk_fall = (!sclk_r1 &&  sclk_r2);
    wire cs_fall   = (!cs_n_r1 &&  cs_n_r2);
    wire cs_rise   = ( cs_n_r1 && !cs_n_r2);
    wire cs_active = !cs_n_r1;

    // -------------------------------------------------------------------------
    // SPI state machine
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        ST_IDLE = 2'd0,
        ST_CMD  = 2'd1,
        ST_DATA = 2'd2
    } spi_st_t;

    spi_st_t st;
    logic [2:0] bit_cnt;
    logic [2:0] byte_cnt;
    logic       rw;
    logic [6:0] cur_addr;
    logic [7:0]  rx_byte;
    logic [55:0] rx_acc;
    logic [63:0] tx_shift;
    logic        done_latch;

    localparam logic [15:0] R_DEFAULT = 16'h0500; // 5.0 in Q8.8

    // Unpack x_out and P_out for read function
    logic [15:0] x_out_arr [0:1];
    logic [15:0] P_out_arr [0:3];

    assign x_out_arr[0] = x_out[15:0];
    assign x_out_arr[1] = x_out[31:16];

    assign P_out_arr[0] = P_out[15:0];
    assign P_out_arr[1] = P_out[31:16];
    assign P_out_arr[2] = P_out[47:32];
    assign P_out_arr[3] = P_out[63:48];

    // Stored prior state/covariance (written by host before start)
    logic [15:0] x_in_arr [0:1];
    logic [15:0] P_in_arr [0:3];

    assign x_in_reg = {x_in_arr[1], x_in_arr[0]};
    assign P_in_reg = {P_in_arr[3], P_in_arr[2], P_in_arr[1], P_in_arr[0]};

    // -------------------------------------------------------------------------
    // Register read: returns {48'h0, int16_value} in 64-bit SPI word
    // -------------------------------------------------------------------------
    function automatic logic [63:0] reg_read_fn (input logic [6:0] addr);
        case (addr)
            7'd1:  reg_read_fn = {62'b0, busy, done_latch};
            7'd5:  reg_read_fn = {48'h0, x_out_arr[0]};
            7'd6:  reg_read_fn = {48'h0, x_out_arr[1]};
            7'd11: reg_read_fn = {48'h0, P_out_arr[0]};
            7'd12: reg_read_fn = {48'h0, P_out_arr[1]};
            7'd13: reg_read_fn = {48'h0, P_out_arr[2]};
            7'd14: reg_read_fn = {48'h0, P_out_arr[3]};
            7'd15: reg_read_fn = {48'h0, r_val};
            default: reg_read_fn = 64'h0;
        endcase
    endfunction

    // Pre-compute constant bit/range-selects (iverilog 12 "sorry" fix)
    wire [63:0] tx_shift_shifted = {tx_shift[62:0], 1'b0};
    wire        miso_hold        = tx_shift[63];

    wire [6:0] rx_byte_lo   = rx_byte[6:0];
    wire       rx_byte_b6   = rx_byte[6];
    wire [5:0] rx_byte_b5_0 = rx_byte[5:0];
    wire       rx_byte_b0   = rx_byte[0];

    wire [47:0] rx_acc_lo   = rx_acc[47:0];

    // INT16 extraction: lower 16 bits of 64-bit SPI word = bytes 6 and 7
    // byte 6 = rx_acc[7:0], byte 7 = {rx_byte[6:0], mosi_r}
    wire [7:0] rx_acc_b6 = rx_acc[7:0];

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
            z_reg      <= 16'h0;
            x_in_arr[0] <= 16'h0; x_in_arr[1] <= 16'h0;
            P_in_arr[0] <= 16'h0; P_in_arr[1] <= 16'h0;
            P_in_arr[2] <= 16'h0; P_in_arr[3] <= 16'h0;
            r_val       <= R_DEFAULT;
        end else begin
            core_start <= 1'b0;

            if (done) done_latch <= 1'b1;

            if (cs_fall) begin
                st       <= ST_CMD;
                bit_cnt  <= 3'd0;
                byte_cnt <= 3'd0;
                rx_byte  <= 8'h0;
                rx_acc   <= 56'h0;
            end

            if (cs_rise) st <= ST_IDLE;

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
                                // 64-bit word complete
                                // INT16 = lower 16 bits = bytes 6 and 7
                                byte_cnt <= 3'd0;
                                rx_acc   <= 56'h0;
                                rx_byte  <= 8'h0;
                                if (rw) begin
                                    case (cur_addr)
                                        7'd0: begin
                                            core_start <= mosi_r;
                                            sw_rst     <= rx_byte_b0;
                                        end
                                        7'd2:  z_reg        <= {rx_acc_b6, rx_byte_lo, mosi_r};
                                        7'd3:  x_in_arr[0]  <= {rx_acc_b6, rx_byte_lo, mosi_r};
                                        7'd4:  x_in_arr[1]  <= {rx_acc_b6, rx_byte_lo, mosi_r};
                                        7'd7:  P_in_arr[0]  <= {rx_acc_b6, rx_byte_lo, mosi_r};
                                        7'd8:  P_in_arr[1]  <= {rx_acc_b6, rx_byte_lo, mosi_r};
                                        7'd9:  P_in_arr[2]  <= {rx_acc_b6, rx_byte_lo, mosi_r};
                                        7'd10: P_in_arr[3]  <= {rx_acc_b6, rx_byte_lo, mosi_r};
                                        7'd15: r_val        <= {rx_acc_b6, rx_byte_lo, mosi_r};
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
