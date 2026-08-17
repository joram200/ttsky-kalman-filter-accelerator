`timescale 1ns/1ps
// =============================================================================
// compute_core.sv — INT16 Q8.8 fixed-point scalar Kalman filter (1D state)
//
// Q8.8 format: value_real = register_value / 256  (signed 16-bit integer)
//   Multiply: (a_Q88 × b_Q88) >> 8  → Q8.8 result
//   1/S:      65536 / S_Q88          → S_inv in Q8.8
//
// Modules:
//   int16_mul_seq  — 17-cycle signed 16×16→16 shift-and-add (Baugh-Wooley)
//   int16_div      — 17-cycle restoring divider: floor(65536 / S_Q88)
//   kalman_update  — FSM; 1D (scalar) state; 1 shared multiplier + 1 divider
//
// Scalar update (H=1, F=1, Q=0):
//   y_tilde = z − x_in
//   S       = P_in + R
//   S_inv   = floor(65536 / S)          [div]
//   K       = P_in × S_inv >> 8         [mul]
//   ky_tmp  = K × y_tilde >> 8          [mul]
//   x_out   = x_in + ky_tmp             [add, combinational]
//   kp_tmp  = K × P_in >> 8             [mul]
//   P_out   = P_in − kp_tmp             [sub, combinational]
//
// FSM: IDLE → S_COMP → DIV → K_COMP → KY_COMP → X_ADD → P_UPDATE → DONE_S
// =============================================================================

// -----------------------------------------------------------------------------
// int16_mul_seq — Signed 16×16 → 16 shift-and-add multiplier (17 cycles)
//   Baugh-Wooley: bits 0..14 add partial product; bit 15 subtracts.
//   result = signed(a) × signed(b) >> 8  (Q8.8 × Q8.8 → Q8.8)
// -----------------------------------------------------------------------------
module int16_mul_seq (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [15:0] a,
    input  logic [15:0] b,
    output logic [15:0] result,
    output logic        done
);
    logic [31:0] accum;
    logic [15:0] a_r;
    logic [31:0] b_ext;
    logic [4:0]  bit_cnt;
    logic        busy;

    logic [31:0] new_accum;
    always_comb begin
        if (!a_r[0])
            new_accum = accum;
        else if (bit_cnt < 5'd15)
            new_accum = accum + b_ext;
        else
            new_accum = accum - b_ext;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy    <= 1'b0; done    <= 1'b0; result  <= 16'h0;
            accum   <= 32'h0; bit_cnt <= 5'd0;
            a_r     <= 16'h0; b_ext   <= 32'h0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                a_r     <= a;
                b_ext   <= {{16{b[15]}}, b};
                accum   <= 32'h0;
                bit_cnt <= 5'd0;
                busy    <= 1'b1;
            end else if (busy) begin
                if (bit_cnt < 5'd16) begin
                    accum   <= new_accum;
                    b_ext   <= {b_ext[30:0], 1'b0};
                    a_r     <= {1'b0, a_r[15:1]};
                    bit_cnt <= bit_cnt + 5'd1;
                end else begin
                    result  <= accum[23:8];
                    done    <= 1'b1;
                    busy    <= 1'b0;
                    bit_cnt <= 5'd0;
                end
            end
        end
    end
endmodule

// -----------------------------------------------------------------------------
// int16_div — Sequential restoring divider: floor(65536 / S_Q88)
//   Returns S_inv in Q8.8: 1/S_real = S_inv_Q88 / 256.
//   17 cycles. Saturates to 0xFFFF if S_Q88 == 0.
// -----------------------------------------------------------------------------
module int16_div (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [15:0] S_Q88,
    output logic [15:0] inv_Q88,
    output logic        done
);
    logic [16:0] partial_rem;
    logic [15:0] divisor_r;
    logic [15:0] quotient;
    logic [4:0]  bit_cnt;
    logic        busy;

    logic        dbit;
    logic [16:0] shifted_rem;
    logic [16:0] trial;

    assign dbit        = (bit_cnt == 5'd0) ? 1'b1 : 1'b0;
    assign shifted_rem = {partial_rem[15:0], dbit};
    assign trial       = shifted_rem - {1'b0, divisor_r};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0; done <= 1'b0; inv_Q88 <= 16'h0;
            partial_rem <= 17'h0; divisor_r <= 16'h0;
            quotient <= 16'h0; bit_cnt <= 5'd0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                divisor_r   <= (S_Q88 == 16'h0) ? 16'h0001 : S_Q88;
                partial_rem <= 17'h0;
                quotient    <= 16'h0;
                bit_cnt     <= 5'd0;
                busy        <= 1'b1;
            end else if (busy) begin
                if (bit_cnt < 5'd17) begin
                    if (!trial[16]) begin
                        partial_rem <= trial;
                        if (bit_cnt > 5'd0) begin
                            case (bit_cnt)
                                5'd1:  quotient[15] <= 1'b1;
                                5'd2:  quotient[14] <= 1'b1;
                                5'd3:  quotient[13] <= 1'b1;
                                5'd4:  quotient[12] <= 1'b1;
                                5'd5:  quotient[11] <= 1'b1;
                                5'd6:  quotient[10] <= 1'b1;
                                5'd7:  quotient[9]  <= 1'b1;
                                5'd8:  quotient[8]  <= 1'b1;
                                5'd9:  quotient[7]  <= 1'b1;
                                5'd10: quotient[6]  <= 1'b1;
                                5'd11: quotient[5]  <= 1'b1;
                                5'd12: quotient[4]  <= 1'b1;
                                5'd13: quotient[3]  <= 1'b1;
                                5'd14: quotient[2]  <= 1'b1;
                                5'd15: quotient[1]  <= 1'b1;
                                5'd16: quotient[0]  <= 1'b1;
                                default: ;
                            endcase
                        end
                    end else begin
                        partial_rem <= shifted_rem;
                    end
                    bit_cnt <= bit_cnt + 5'd1;
                end else begin
                    inv_Q88 <= (S_Q88 == 16'h0) ? 16'hFFFF : quotient;
                    done    <= 1'b1;
                    busy    <= 1'b0;
                    bit_cnt <= 5'd0;
                end
            end
        end
    end
endmodule

// -----------------------------------------------------------------------------
// kalman_update — INT16 Q8.8 scalar Kalman filter update (1D, H=1)
//
// FSM: IDLE → S_COMP → DIV → K_COMP → KY_COMP → X_ADD → P_UPDATE → DONE_S
// S_COMP and X_ADD are combinational (1 cycle each).
// DIV, K_COMP, KY_COMP, P_UPDATE each use the shared multiplier or divider.
// -----------------------------------------------------------------------------
module kalman_update (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [15:0] z,
    input  logic [15:0] x_in,
    input  logic [15:0] P_in,
    input  logic [15:0] r_val,
    output logic [15:0] x_out,
    output logic [15:0] P_out,
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
    logic [15:0] y_tilde;
    logic [15:0] S_reg;
    logic [15:0] S_inv;
    logic [15:0] K_reg;
    logic [15:0] ky_tmp;

    // -------------------------------------------------------------------------
    // Shared multiplier + divider
    // -------------------------------------------------------------------------
    logic [15:0] mul_a, mul_b, mul_result;
    logic        mul_start_w, mul_done_w, mul_active;

    logic [15:0] div_result;
    logic        div_start_w, div_done_w, div_active;

    int16_mul_seq u_mul (
        .clk(clk), .rst_n(rst_n), .start(mul_start_w),
        .a(mul_a), .b(mul_b), .result(mul_result), .done(mul_done_w)
    );
    int16_div u_div (
        .clk(clk), .rst_n(rst_n), .start(div_start_w),
        .S_Q88(S_reg), .inv_Q88(div_result), .done(div_done_w)
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
        mul_a = 16'h0; mul_b = 16'h0;
        case (state)
            K_COMP:   begin mul_a = P_in;  mul_b = S_inv;   end  // K = P*S_inv
            KY_COMP:  begin mul_a = K_reg; mul_b = y_tilde; end  // ky = K*y
            P_UPDATE: begin mul_a = K_reg; mul_b = P_in;    end  // kp = K*P
            default: ;
        endcase
    end

    // FSM next-state (no sub_cnt needed — scalar, one operation per state)
    always_comb begin
        state_nxt = state;
        case (state)
            IDLE:     if (start)     state_nxt = S_COMP;
            S_COMP:                  state_nxt = DIV;
            DIV:      if (div_done_w) state_nxt = K_COMP;
            K_COMP:   if (mul_done_w) state_nxt = KY_COMP;
            KY_COMP:  if (mul_done_w) state_nxt = X_ADD;
            X_ADD:                   state_nxt = P_UPDATE;
            P_UPDATE: if (mul_done_w) state_nxt = DONE_S;
            DONE_S:                  state_nxt = IDLE;
            default:                 state_nxt = IDLE;
        endcase
    end

    // FSM datapath
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            mul_active <= 1'b0; div_active <= 1'b0;
            y_tilde    <= 16'h0; S_reg   <= 16'h0; S_inv  <= 16'h0;
            K_reg      <= 16'h0; ky_tmp  <= 16'h0;
            x_out      <= 16'h0; P_out   <= 16'h0;
        end else begin
            state <= state_nxt;

            if (mul_start_w)     mul_active <= 1'b1;
            else if (mul_done_w) mul_active <= 1'b0;

            if (div_start_w)     div_active <= 1'b1;
            else if (div_done_w) div_active <= 1'b0;

            case (state)
                S_COMP: begin
                    y_tilde <= z - x_in;
                    S_reg   <= P_in + r_val;
                end
                DIV:      if (div_done_w) S_inv  <= div_result;
                K_COMP:   if (mul_done_w) K_reg  <= mul_result;
                KY_COMP:  if (mul_done_w) ky_tmp <= mul_result;
                X_ADD:    x_out <= x_in + ky_tmp;
                P_UPDATE: if (mul_done_w) P_out  <= P_in - mul_result;
                default: ;
            endcase
        end
    end

    assign done = (state == DONE_S);
    assign busy = (state != IDLE) && (state != DONE_S);

endmodule
