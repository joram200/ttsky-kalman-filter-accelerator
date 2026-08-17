`timescale 1ns/1ps
// =============================================================================
// compute_core.sv — INT16 Q8.8 fixed-point Kalman filter accelerator
//
// Q8.8 format: value_real = register_value / 256  (signed 16-bit integer)
//   Multiply: (a_Q88 × b_Q88) >> 8  → Q8.8 result
//   1/S:      65536 / S_Q88          → S_inv in Q8.8
//
// Modules:
//   int16_mul_seq  — 17-cycle signed 16×16→16 shift-and-add (Baugh-Wooley)
//   int16_div      — 17-cycle restoring divider: floor(65536 / S_Q88)
//   kalman_update  — FSM; rank-1 P update; 1 shared multiplier + 1 divider
//
// FSM states: IDLE → S_COMP → DIV → K_COMP → KY_COMP → X_ADD → P_UPDATE → DONE_S
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
    logic [31:0] b_ext;   // b sign-extended to 32 bits, shifted left each cycle
    logic [4:0]  bit_cnt;
    logic        busy;

    // Partial product: add for bits 0..14, subtract for bit 15
    logic [31:0] new_accum;
    always_comb begin
        if (!a_r[0])
            new_accum = accum;
        else if (bit_cnt < 5'd15)
            new_accum = accum + b_ext;
        else
            new_accum = accum - b_ext;   // Baugh-Wooley sign-bit correction
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
                b_ext   <= {{16{b[15]}}, b};   // sign-extend b
                accum   <= 32'h0;
                bit_cnt <= 5'd0;
                busy    <= 1'b1;
            end else if (busy) begin
                if (bit_cnt < 5'd16) begin
                    accum   <= new_accum;
                    b_ext   <= {b_ext[30:0], 1'b0};   // shift b left
                    a_r     <= {1'b0, a_r[15:1]};      // shift a right
                    bit_cnt <= bit_cnt + 5'd1;
                end else begin
                    result  <= accum[23:8];   // Q16.16 → Q8.8
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
//   17 cycles (16 quotient bits + 1 output cycle).
//   Saturates to 0xFFFF if S_Q88 == 0.
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

    // Dividend = 17'h10000; bit 16 = 1 on first iteration, 0 thereafter
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
                // 17 iterations: processes all 17 bits of dividend 65536
                // bit_cnt=0: processes bit 16 (=1); quotient bit 16 discarded (= 0 for S>1)
                // bit_cnt=1..16: processes bits 15..0 (=0); sets quotient[15..0]
                if (bit_cnt < 5'd17) begin
                    if (!trial[16]) begin   // trial >= 0: quotient bit = 1
                        partial_rem <= trial;
                        if (bit_cnt > 5'd0) begin  // skip bit_cnt=0 (quotient bit 16)
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
                    end else begin          // trial < 0: quotient bit stays 0
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
// kalman_update — INT16 Q8.8 Kalman filter update (H=[1,0,0], rank-1 P update)
//
//   P_new[i,j] = P[i,j] − K[i] × P[0,j]    i,j ∈ {0,1,2}
//
// Arithmetic: 1 shared int16_mul_seq + 1 int16_div.
// Add/subtract operations are combinational (1 cycle).
//
// FSM: IDLE → S_COMP → DIV → K_COMP → KY_COMP → X_ADD → P_UPDATE → DONE_S
// -----------------------------------------------------------------------------
module kalman_update (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         start,
    input  logic [15:0]  z,
    input  logic [47:0]  x_in,    // 3×16
    input  logic [143:0] P_in,    // 9×16
    input  logic [15:0]  r_val,
    output logic [47:0]  x_out,   // 3×16
    output logic [143:0] P_out,   // 9×16
    output logic         done,
    output logic         busy
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
    logic [1:0] sub_cnt, sub_nxt;

    // -------------------------------------------------------------------------
    // Unpack flat inputs
    // -------------------------------------------------------------------------
    logic [15:0] x_in_arr [0:2];
    logic [15:0] p_in_arr [0:8];
    genvar ki;
    generate
        for (ki = 0; ki < 3; ki++) begin : g_unpack_x
            assign x_in_arr[ki] = x_in[ki*16 +: 16];
        end
        for (ki = 0; ki < 9; ki++) begin : g_unpack_p
            assign p_in_arr[ki] = P_in[ki*16 +: 16];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Pack outputs
    // -------------------------------------------------------------------------
    logic [15:0] x_out_arr [0:2];
    logic [15:0] acc [0:8];
    generate
        for (ki = 0; ki < 3; ki++) begin : g_pack_x
            assign x_out[ki*16 +: 16] = x_out_arr[ki];
        end
        for (ki = 0; ki < 9; ki++) begin : g_pack_p
            assign P_out[ki*16 +: 16] = acc[ki];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Intermediate registers
    // -------------------------------------------------------------------------
    logic [15:0] y_tilde;
    logic [15:0] S_reg;
    logic [15:0] S_inv;
    logic [15:0] K_reg [0:2];
    logic [15:0] ky_arr [0:2];

    // P_UPDATE counters
    logic [1:0] i_cnt, j_cnt;
    logic       pu_last;
    assign pu_last = (i_cnt == 2'd2) && (j_cnt == 2'd2);

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

    // -------------------------------------------------------------------------
    // mul_start_w — fires when not active and state needs a multiply
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    // mul_a / mul_b mux
    // -------------------------------------------------------------------------
    always_comb begin
        mul_a = 16'h0; mul_b = 16'h0;
        case (state)
            K_COMP: begin
                mul_b = S_inv;
                case (sub_cnt)
                    2'd0: mul_a = p_in_arr[0];
                    2'd1: mul_a = p_in_arr[3];
                    2'd2: mul_a = p_in_arr[6];
                    default: mul_a = 16'h0;
                endcase
            end
            KY_COMP: begin
                mul_b = y_tilde;
                case (sub_cnt)
                    2'd0: mul_a = K_reg[0];
                    2'd1: mul_a = K_reg[1];
                    2'd2: mul_a = K_reg[2];
                    default: mul_a = 16'h0;
                endcase
            end
            P_UPDATE: begin
                case (i_cnt)
                    2'd0: mul_a = K_reg[0];
                    2'd1: mul_a = K_reg[1];
                    2'd2: mul_a = K_reg[2];
                    default: mul_a = 16'h0;
                endcase
                case (j_cnt)
                    2'd0: mul_b = p_in_arr[0];
                    2'd1: mul_b = p_in_arr[1];
                    2'd2: mul_b = p_in_arr[2];
                    default: mul_b = 16'h0;
                endcase
            end
            default: ;
        endcase
    end

    // -------------------------------------------------------------------------
    // FSM next-state
    // -------------------------------------------------------------------------
    always_comb begin
        state_nxt = state;
        sub_nxt   = sub_cnt + 2'd1;

        case (state)
            IDLE:
                if (start) begin state_nxt = S_COMP; sub_nxt = 2'd0; end
                else sub_nxt = 2'd0;

            S_COMP: begin
                // 1-cycle combinational: y and S registered, then go to DIV
                state_nxt = DIV; sub_nxt = 2'd0;
            end

            DIV: begin
                sub_nxt = 2'd0;
                if (div_done_w) begin state_nxt = K_COMP; sub_nxt = 2'd0; end
            end

            K_COMP: begin
                if (!mul_done_w) sub_nxt = sub_cnt;
                else if (sub_cnt == 2'd2) begin state_nxt = KY_COMP; sub_nxt = 2'd0; end
            end

            KY_COMP: begin
                if (!mul_done_w) sub_nxt = sub_cnt;
                else if (sub_cnt == 2'd2) begin state_nxt = X_ADD; sub_nxt = 2'd0; end
            end

            X_ADD: begin
                // 1-cycle: compute x_out combinationally
                state_nxt = P_UPDATE; sub_nxt = 2'd0;
            end

            P_UPDATE: begin
                sub_nxt = 2'd0;
                if (mul_done_w && pu_last) state_nxt = DONE_S;
            end

            DONE_S: begin state_nxt = IDLE; sub_nxt = 2'd0; end
            default: begin state_nxt = IDLE; sub_nxt = 2'd0; end
        endcase
    end

    // -------------------------------------------------------------------------
    // FSM datapath
    // -------------------------------------------------------------------------
    integer ii;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; sub_cnt <= 2'd0;
            mul_active <= 1'b0; div_active <= 1'b0;
            y_tilde <= 16'h0; S_reg <= 16'h0; S_inv <= 16'h0;
            for (ii = 0; ii < 3; ii++) begin
                K_reg[ii]     <= 16'h0;
                ky_arr[ii]    <= 16'h0;
                x_out_arr[ii] <= 16'h0;
            end
            i_cnt <= 2'd0; j_cnt <= 2'd0;
            for (ii = 0; ii < 9; ii++) acc[ii] <= 16'h0;
        end else begin
            state   <= state_nxt;
            sub_cnt <= sub_nxt;

            if (mul_start_w)  mul_active <= 1'b1;
            else if (mul_done_w) mul_active <= 1'b0;

            if (div_start_w)  div_active <= 1'b1;
            else if (div_done_w) div_active <= 1'b0;

            case (state)

                S_COMP: begin
                    y_tilde <= z - x_in_arr[0];
                    S_reg   <= p_in_arr[0] + r_val;
                end

                DIV: begin
                    if (div_done_w) S_inv <= div_result;
                end

                K_COMP: begin
                    if (mul_done_w) begin
                        case (sub_cnt)
                            2'd0: K_reg[0] <= mul_result;
                            2'd1: K_reg[1] <= mul_result;
                            2'd2: K_reg[2] <= mul_result;
                            default: ;
                        endcase
                    end
                end

                KY_COMP: begin
                    if (mul_done_w) begin
                        case (sub_cnt)
                            2'd0: ky_arr[0] <= mul_result;
                            2'd1: ky_arr[1] <= mul_result;
                            2'd2: ky_arr[2] <= mul_result;
                            default: ;
                        endcase
                    end
                end

                X_ADD: begin
                    x_out_arr[0] <= x_in_arr[0] + ky_arr[0];
                    x_out_arr[1] <= x_in_arr[1] + ky_arr[1];
                    x_out_arr[2] <= x_in_arr[2] + ky_arr[2];
                    i_cnt <= 2'd0; j_cnt <= 2'd0;
                end

                // Rank-1 P update: acc[i*3+j] = P[i,j] − K[i]×P[0,j]
                // mul computes K[i]*P[0,j]; subtract is combinational on done
                P_UPDATE: begin
                    if (mul_done_w) begin
                        case ({i_cnt, j_cnt})
                            4'b0000: acc[0] <= p_in_arr[0] - mul_result;
                            4'b0001: acc[1] <= p_in_arr[1] - mul_result;
                            4'b0010: acc[2] <= p_in_arr[2] - mul_result;
                            4'b0100: acc[3] <= p_in_arr[3] - mul_result;
                            4'b0101: acc[4] <= p_in_arr[4] - mul_result;
                            4'b0110: acc[5] <= p_in_arr[5] - mul_result;
                            4'b1000: acc[6] <= p_in_arr[6] - mul_result;
                            4'b1001: acc[7] <= p_in_arr[7] - mul_result;
                            4'b1010: acc[8] <= p_in_arr[8] - mul_result;
                            default: ;
                        endcase
                        if (!pu_last) begin
                            if (j_cnt == 2'd2) begin
                                j_cnt <= 2'd0;
                                i_cnt <= i_cnt + 2'd1;
                            end else begin
                                j_cnt <= j_cnt + 2'd1;
                            end
                        end
                    end
                end

                default: ;
            endcase
        end
    end

    assign done = (state == DONE_S);
    assign busy = (state != IDLE) && (state != DONE_S);

endmodule
