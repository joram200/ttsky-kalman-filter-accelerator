`timescale 1ns/1ps
// =============================================================================
// interface.sv — SPI slave register file for tt_um_joram200
//
// Protocol: CPOL=0, CPHA=0, MSB-first, 8-bit frames.
// Transaction format: 1 command byte then N×8 data bytes while CS_N is low.
// Command byte: bit[7]=R/W (1=write, 0=read), bits[6:0]=start register address.
// Each F64 register = 8 data bytes. Address auto-increments by 1 per completed
// 64-bit word within a CS_N-low burst.
//
// Register map (28 registers, indices 0–27):
//   0   CTRL      W      [0]=start one-shot pulse, [1]=sw_rst (level)
//   1   STAT      R      [0]=done_latch (clears on STAT read), [1]=busy
//   2   z         W      scalar measurement F64
//   3   x_in[0]   W      prior state element 0 F64
//   4   x_in[1]   W      prior state element 1 F64
//   5   x_in[2]   W      prior state element 2 F64
//   6   x_out[0]  R      corrected state element 0 F64
//   7   x_out[1]  R      corrected state element 1 F64
//   8   x_out[2]  R      corrected state element 2 F64
//   9   P_in[0]   W      prior covariance [0,0] F64
//  10   P_in[1]   W      [0,1]
//  11   P_in[2]   W      [0,2]
//  12   P_in[3]   W      [1,0]
//  13   P_in[4]   W      [1,1]
//  14   P_in[5]   W      [1,2]
//  15   P_in[6]   W      [2,0]
//  16   P_in[7]   W      [2,1]
//  17   P_in[8]   W      [2,2]
//  18   P_out[0]  R      updated covariance [0,0] F64
//  19   P_out[1]  R      [0,1]
//  20   P_out[2]  R      [0,2]
//  21   P_out[3]  R      [1,0]
//  22   P_out[4]  R      [1,1]
//  23   P_out[5]  R      [1,2]
//  24   P_out[6]  R      [2,0]
//  25   P_out[7]  R      [2,1]
//  26   P_out[8]  R      [2,2]
//  27   R_REG     R/W    measurement noise R; reset default = 5.0 F64
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
    // Kalman inputs (written by host)
    output logic [63:0] z_reg,
    output logic [63:0] x_in_reg [0:2],
    output logic [63:0] P_in_reg [0:8],
    output logic [63:0] r_val,        // R_REG; reset default = 5.0 F64
    // Kalman status / outputs (read by host)
    input  logic        done,
    input  logic        busy,
    input  logic [63:0] x_out  [0:2],
    input  logic [63:0] P_out  [0:8]
);

    // -------------------------------------------------------------------------
    // Two-stage synchroniser for SPI pins → clk domain
    // -------------------------------------------------------------------------
    logic sclk_r1, sclk_r2;
    logic cs_n_r1, cs_n_r2;
    logic mosi_r;   // one-stage is sufficient after the two-stage

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_r1 <= 1'b0; sclk_r2 <= 1'b0;
            cs_n_r1 <= 1'b1; cs_n_r2 <= 1'b1;
            mosi_r  <= 1'b0;
        end else begin
            sclk_r1 <= sclk;   sclk_r2 <= sclk_r1;
            cs_n_r1 <= cs_n;   cs_n_r2 <= cs_n_r1;
            mosi_r  <= mosi;   // sample MOSI one cycle before SCLK rise is detected
        end
    end

    // Detected one clk cycle after the actual edge (sclk_r2 = two cycles old)
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

    logic [2:0] bit_cnt;   // bit position within current byte (0=first bit received)
    logic [2:0] byte_cnt;  // byte index within current 64-bit word (0=first byte)
    logic       rw;        // 1=write, 0=read (latched from command byte)
    logic [6:0] cur_addr;  // current register address (auto-increments per word)

    // Incoming shift register: assembled MSB-first over 8 clocks per byte,
    // then packed 8 bytes left-to-right into rx_word.
    logic [7:0]  rx_byte;   // shift register for one byte
    logic [55:0] rx_acc;    // accumulates bytes 0..6; byte 7 is rx_byte

    // MISO shift register
    logic [63:0] tx_shift;

    // done latch
    logic done_latch;

    // Default R value
    localparam logic [63:0] R_DEFAULT = 64'h4014_0000_0000_0000; // 5.0 F64

    // -------------------------------------------------------------------------
    // Register read function
    // -------------------------------------------------------------------------
    function automatic logic [63:0] reg_read_fn (input logic [6:0] addr);
        case (addr)
            7'd1:  reg_read_fn = {62'b0, busy, done_latch};
            7'd6:  reg_read_fn = x_out[0];
            7'd7:  reg_read_fn = x_out[1];
            7'd8:  reg_read_fn = x_out[2];
            7'd18: reg_read_fn = P_out[0];
            7'd19: reg_read_fn = P_out[1];
            7'd20: reg_read_fn = P_out[2];
            7'd21: reg_read_fn = P_out[3];
            7'd22: reg_read_fn = P_out[4];
            7'd23: reg_read_fn = P_out[5];
            7'd24: reg_read_fn = P_out[6];
            7'd25: reg_read_fn = P_out[7];
            7'd26: reg_read_fn = P_out[8];
            7'd27: reg_read_fn = r_val;
            default: reg_read_fn = 64'h0;
        endcase
    endfunction

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
            z_reg         <= 64'h0;
            x_in_reg[0]   <= 64'h0;
            x_in_reg[1]   <= 64'h0;
            x_in_reg[2]   <= 64'h0;
            P_in_reg[0]   <= 64'h0;
            P_in_reg[1]   <= 64'h0;
            P_in_reg[2]   <= 64'h0;
            P_in_reg[3]   <= 64'h0;
            P_in_reg[4]   <= 64'h0;
            P_in_reg[5]   <= 64'h0;
            P_in_reg[6]   <= 64'h0;
            P_in_reg[7]   <= 64'h0;
            P_in_reg[8]   <= 64'h0;
            r_val         <= R_DEFAULT;
        end else begin
            // One-cycle pulse default
            core_start <= 1'b0;

            // Capture done pulse from compute core
            if (done) done_latch <= 1'b1;

            // CS_N falling edge: reset transaction state
            if (cs_fall) begin
                st       <= ST_CMD;
                bit_cnt  <= 3'd0;
                byte_cnt <= 3'd0;
                rx_byte  <= 8'h0;
                rx_acc   <= 56'h0;
            end

            // CS_N rising edge: end transaction
            if (cs_rise) begin
                st <= ST_IDLE;
            end

            // SCLK rising edge: shift in MOSI
            if (sclk_rise && cs_active) begin
                case (st)

                    // --------------------------------------------------------
                    // ST_CMD: receive 8-bit command byte
                    // --------------------------------------------------------
                    ST_CMD: begin
                        rx_byte <= {rx_byte[6:0], mosi_r};
                        if (bit_cnt == 3'd7) begin
                            // Full command byte assembled: {rx_byte[6:0], mosi_r}
                            // bit[7] = MSB = rw (rx_byte[6] after 7 shifts)
                            // bits[6:0] = addr ({rx_byte[5:0], mosi_r})
                            rw       <= rx_byte[6];
                            cur_addr <= {rx_byte[5:0], mosi_r};
                            st       <= ST_DATA;
                            bit_cnt  <= 3'd0;
                            byte_cnt <= 3'd0;
                            rx_byte  <= 8'h0;
                            rx_acc   <= 56'h0;
                            // For reads: pre-load the MISO shift register
                            if (!rx_byte[6]) begin
                                tx_shift <= reg_read_fn({rx_byte[5:0], mosi_r});
                                // Clear done_latch if reading STAT
                                if ({rx_byte[5:0], mosi_r} == 7'd1) done_latch <= 1'b0;
                            end
                        end else begin
                            bit_cnt <= bit_cnt + 3'd1;
                        end
                    end

                    // --------------------------------------------------------
                    // ST_DATA: receive/transmit 64-bit register words
                    // --------------------------------------------------------
                    ST_DATA: begin
                        rx_byte <= {rx_byte[6:0], mosi_r};
                        if (bit_cnt == 3'd7) begin
                            bit_cnt <= 3'd0;
                            if (byte_cnt == 3'd7) begin
                                // ---- 64-bit word complete ----
                                byte_cnt <= 3'd0;
                                rx_acc   <= 56'h0;
                                rx_byte  <= 8'h0;
                                if (rw) begin
                                    // Full word: {rx_acc[55:0], rx_byte[6:0], mosi_r}
                                    // (rx_acc and rx_byte hold OLD values; NBAs haven't fired)
                                    case (cur_addr)
                                        7'd0: begin
                                            // CTRL: bit[0]=start, bit[1]=sw_rst
                                            // Last bit received = mosi_r = word bit[0]
                                            // Second-to-last = rx_byte[0] = word bit[1]
                                            core_start <= mosi_r;
                                            sw_rst     <= rx_byte[0];
                                        end
                                        7'd2:  z_reg       <= {rx_acc, rx_byte[6:0], mosi_r};
                                        7'd3:  x_in_reg[0] <= {rx_acc, rx_byte[6:0], mosi_r};
                                        7'd4:  x_in_reg[1] <= {rx_acc, rx_byte[6:0], mosi_r};
                                        7'd5:  x_in_reg[2] <= {rx_acc, rx_byte[6:0], mosi_r};
                                        7'd9:  P_in_reg[0] <= {rx_acc, rx_byte[6:0], mosi_r};
                                        7'd10: P_in_reg[1] <= {rx_acc, rx_byte[6:0], mosi_r};
                                        7'd11: P_in_reg[2] <= {rx_acc, rx_byte[6:0], mosi_r};
                                        7'd12: P_in_reg[3] <= {rx_acc, rx_byte[6:0], mosi_r};
                                        7'd13: P_in_reg[4] <= {rx_acc, rx_byte[6:0], mosi_r};
                                        7'd14: P_in_reg[5] <= {rx_acc, rx_byte[6:0], mosi_r};
                                        7'd15: P_in_reg[6] <= {rx_acc, rx_byte[6:0], mosi_r};
                                        7'd16: P_in_reg[7] <= {rx_acc, rx_byte[6:0], mosi_r};
                                        7'd17: P_in_reg[8] <= {rx_acc, rx_byte[6:0], mosi_r};
                                        7'd27: r_val       <= {rx_acc, rx_byte[6:0], mosi_r};
                                        default: ;
                                    endcase
                                end
                                // Advance burst address
                                cur_addr <= cur_addr + 7'd1;
                                // For read bursts: pre-load next register
                                if (!rw) begin
                                    tx_shift <= reg_read_fn(cur_addr + 7'd1);
                                    if ((cur_addr + 7'd1) == 7'd1) done_latch <= 1'b0;
                                end
                            end else begin
                                // Pack completed byte into rx_acc (bytes 0..6)
                                rx_acc   <= {rx_acc[47:0], rx_byte[6:0], mosi_r};
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

            // SCLK falling edge: drive MISO from tx_shift (CPHA=0 convention)
            if (sclk_fall && cs_active) begin
                miso     <= tx_shift[63];
                tx_shift <= {tx_shift[62:0], 1'b0};
            end
        end
    end

endmodule
