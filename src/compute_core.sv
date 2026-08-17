`timescale 1ns/1ps
// =============================================================================
// compute_core.sv — INT10 Q4.6 fixed-point scalar Kalman filter (1D state)
//
// Q4.6 format: value_real = register_value / 64  (signed 10-bit integer)
//   Multiply: (a_Q46 × b_Q46) >> 6  → Q4.6 result
//   1/S:      4096 / S_Q46           → S_inv in Q4.6
//
// Modules:
//   int10_mul_seq  — 11-cycle signed 10×10→10 shift-and-add (Baugh-Wooley)
//   int10_div      — 14-cycle restoring divider: floor(4096 / S_Q46)
//   kalman_update  — FSM; 1D (scalar) state; 1 shared multiplier + 1 divider
//
// Scalar update (H=1, F=1, Q=0):
//   y_tilde = z − x_in
//   S       = P_in + R
//   S_inv   = floor(4096 / S)           [div]
//   K       = P_in × S_inv >> 6         [mul]
//   ky_tmp  = K × y_tilde >> 6          [mul]
//   x_out   = x_in + ky_tmp             [add, combinational]
//   kp_tmp  = K × P_in >> 6             [mul]
//   P_out   = P_in − kp_tmp             [sub, combinational]
//
// FSM: IDLE → S_COMP → DIV → K_COMP → KY_COMP → X_ADD → P_UPDATE → DONE_S
// =============================================================================

// -----------------------------------------------------------------------------
// int10_mul_seq — Signed 10×10 → 10 shift-and-add multiplier (11 cycles)
//   Baugh-Wooley: bits 0..8 add partial product; bit 9 subtracts.
//   result = signed(a) × signed(b) >> 6  (Q4.6 × Q4.6 → Q4.6)
// -----------------------------------------------------------------------------
module int10_mul_seq (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [9:0]  a,
    input  logic [9:0]  b,
    output wire  [9:0]  result,
    output logic        done
);
    logic [19:0] accum;
    logic [9:0]  a_r;
    logic [19:0] b_ext;
    logic [3:0]  bit_cnt;
    logic        busy;

    // result is combinational: valid while accum holds the finished product
    assign result = accum[15:6];

    logic [19:0] new_accum;
    always_comb begin
        if (!a_r[0])
            new_accum = accum;
        else if (bit_cnt < 4'd9)
            new_accum = accum + b_ext;
        else
            new_accum = accum - b_ext;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy    <= 1'b0; done    <= 1'b0;
            accum   <= 20'h0; bit_cnt <= 4'd0;
            a_r     <= 10'h0; b_ext   <= 20'h0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                a_r     <= a;
                b_ext   <= {{10{b[9]}}, b};
                accum   <= 20'h0;
                bit_cnt <= 4'd0;
                busy    <= 1'b1;
            end else if (busy) begin
                if (bit_cnt < 4'd10) begin
                    accum   <= new_accum;
                    b_ext   <= {b_ext[18:0], 1'b0};
                    a_r     <= {1'b0, a_r[9:1]};
                    bit_cnt <= bit_cnt + 4'd1;
                end else begin
                    done    <= 1'b1;
                    busy    <= 1'b0;
                    bit_cnt <= 4'd0;
                end
            end
        end
    end
endmodule

// -----------------------------------------------------------------------------
// int10_div — Sequential restoring divider: floor(4096 / S_Q46)
//   Returns S_inv in Q4.6: 1/S_real = S_inv_Q46 / 64.
//   14 cycles. Saturates to 10'h1FF if S_Q46 <= 8 (quotient would exceed 511).
// -----------------------------------------------------------------------------
module int10_div (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [9:0]  S_Q46,
    output wire  [9:0]  inv_Q46,
    output logic        done
);
    logic [12:0] partial_rem;
    logic [11:0] quotient;
    logic [3:0]  bit_cnt;
    logic        busy;

    wire [9:0]  div_s       = (S_Q46 == 10'h0) ? 10'h001 : S_Q46;
    wire        dbit        = (bit_cnt == 4'd0) ? 1'b1 : 1'b0;
    wire [12:0] shifted_rem = {partial_rem[11:0], dbit};
    wire [12:0] trial       = shifted_rem - {3'b0, div_s};

    // Saturate when S_Q46 <= 8 (floor(4096/8)=512 > 511 max Q4.6 positive)
    // or when quotient overflows 10 bits (|quotient[11:9]).
    assign inv_Q46 = (S_Q46 <= 10'd8)   ? 10'h1FF :
                     (|quotient[11:9])   ? 10'h1FF : quotient[9:0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0; done <= 1'b0;
            partial_rem <= 13'h0;
            quotient <= 12'h0; bit_cnt <= 4'd0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                partial_rem <= 13'h0;
                quotient    <= 12'h0;
                bit_cnt     <= 4'd0;
                busy        <= 1'b1;
            end else if (busy) begin
                if (bit_cnt < 4'd13) begin
                    // Shift-register quotient: MSB first, one bit per cycle.
                    partial_rem <= (!trial[12]) ? trial : shifted_rem;
                    if (bit_cnt > 4'd0)
                        quotient <= {quotient[10:0], !trial[12]};
                    bit_cnt <= bit_cnt + 4'd1;
                end else begin
                    done    <= 1'b1;
                    busy    <= 1'b0;
                    bit_cnt <= 4'd0;
                end
            end
        end
    end
endmodule

// -----------------------------------------------------------------------------
// kalman_update — INT10 Q4.6 scalar Kalman filter update (1D, H=1)
//
// FSM: IDLE → S_COMP → DIV → K_COMP → KY_COMP → X_ADD → P_UPDATE → DONE_S
// S_COMP and X_ADD are combinational (1 cycle each).
// DIV, K_COMP, KY_COMP, P_UPDATE each use the shared multiplier or divider.
// -----------------------------------------------------------------------------
module kalman_update (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [9:0]  z,
    input  logic [9:0]  x_in,
    input  logic [9:0]  P_in,
    input  logic [9:0]  r_val,
    output logic [9:0]  x_out,
    output logic [9:0]  P_out,
    output logic        done,
    output logic        busy
);
    typedef enum logic [2:0] {
        IDLE     = 3'd0,
        S_COMP   = 3'd1,
        DIV      = 3'd2,
        K_COMP   = 3'd3,
        KY_COMP  = 3'd4,
        X_ADD    = 3'd5,
        P_UPDATE = 3'd6,
        DONE_S   = 3'd7
    } state_t;

    state_t state, state_nxt;

    // -------------------------------------------------------------------------
    // Intermediate registers
    // -------------------------------------------------------------------------
    logic [9:0] K_reg;

    // Combinational intermediates — stable because par_reg holds inputs steady
    wire [9:0] y_tilde_comb = z - x_in;
    wire [9:0] S_comb       = P_in + r_val;

    // -------------------------------------------------------------------------
    // Shared multiplier + divider
    // -------------------------------------------------------------------------
    logic [9:0] mul_a, mul_b, mul_result;
    logic        mul_start_w, mul_done_w, mul_active;

    logic [9:0] div_result;
    logic        div_start_w, div_done_w, div_active;

    int10_mul_seq u_mul (
        .clk(clk), .rst_n(rst_n), .start(mul_start_w),
        .a(mul_a), .b(mul_b), .result(mul_result), .done(mul_done_w)
    );
    int10_div u_div (
        .clk(clk), .rst_n(rst_n), .start(div_start_w),
        .S_Q46(S_comb), .inv_Q46(div_result), .done(div_done_w)
    );

    // mul_start_w — fires when not active and state needs a multiply
    always_comb begin
        mul_start_w = 1'b0;
        if (!mul_active) begin
            case (state)
                K_COMP:   mul_start_w = 1'b1;
                KY_COMP:  mul_start_w = 1'b1;
                P_UPDATE: mul_start_w = 1'b1;
                default:  ;
            endcase
        end
    end

    // div_start_w — single pulse on entering DIV
    assign div_start_w = (state == DIV) && !div_active;

    // mul_a / mul_b mux
    always_comb begin
        mul_a = 10'h0; mul_b = 10'h0;
        case (state)
            K_COMP:   begin mul_a = P_in;  mul_b = div_result;   end  // K = P*S_inv
            KY_COMP:  begin mul_a = K_reg; mul_b = y_tilde_comb; end  // ky = K*y
            P_UPDATE: begin mul_a = K_reg; mul_b = P_in;         end  // kp = K*P
            default: ;
        endcase
    end

    // FSM next-state
    always_comb begin
        state_nxt = state;
        case (state)
            IDLE:     if (start)      state_nxt = S_COMP;
            S_COMP:                   state_nxt = DIV;
            DIV:      if (div_done_w) state_nxt = K_COMP;
            K_COMP:   if (mul_done_w) state_nxt = KY_COMP;
            KY_COMP:  if (mul_done_w) state_nxt = X_ADD;
            X_ADD:                    state_nxt = P_UPDATE;
            P_UPDATE: if (mul_done_w) state_nxt = DONE_S;
            DONE_S:                   state_nxt = IDLE;
            default:                  state_nxt = IDLE;
        endcase
    end

    // FSM datapath
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            mul_active <= 1'b0; div_active <= 1'b0;
            K_reg      <= 10'h0;
            x_out      <= 10'h0; P_out   <= 10'h0;
        end else begin
            state <= state_nxt;

            if (mul_start_w)     mul_active <= 1'b1;
            else if (mul_done_w) mul_active <= 1'b0;

            if (div_start_w)     div_active <= 1'b1;
            else if (div_done_w) div_active <= 1'b0;

            case (state)
                S_COMP:   ;  // no datapath; transitions to DIV next cycle
                K_COMP:   if (mul_done_w) K_reg  <= mul_result;
                X_ADD:    x_out <= x_in + mul_result;
                P_UPDATE: if (mul_done_w) P_out  <= P_in - mul_result;
                default: ;
            endcase
        end
    end

    assign done = (state == DONE_S);
    assign busy = (state != IDLE) && (state != DONE_S);

endmodule
