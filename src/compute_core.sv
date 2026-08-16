`timescale 1ns/1ps
// =============================================================================
// compute_core.sv — F32 + Time-multiplexed compute core for tt_um_joram200
// Task 6 (iteration 5+6): Rank-1 P update + NR 4→3 + f32_add_seq + state retention
//
// vs iter4 (bit-serial f32_mul_seq):
//   - GEMM_INIT + GEMM_COMPUTE (27-MAC) replaced by P_UPDATE (9-MAC rank-1)
//     P_new[i,j] = P[i,j] - K[i]*P[0,j]  (H=[1,0,0] simplification)
//     Eliminates: ikha_elem 9:1 mux, pb_elem 9:1→3:1, acc_elem 9:1 mux,
//                 IKH_arr, one_minus_k0_r, k_cnt, GEMM_INIT state
//   - NR reduced 4→2 iterations (seed gives 8-bit acc; NR1 completes 32-bit)
//   - f32_add (combinational barrel-shifter) → f32_add_seq (shift-by-1,
//     variable latency 3..49 cycles); eliminates 24-bit barrel shifter and
//     25-bit combinational adder.
//   - Both mul and add now use start/done handshake; FSM stalls on both.
//   - two_m_sx_r register isolates NR dependency (add result for mul_b).
//
// Module hierarchy:
//   f32_mul_seq   — bit-serial 26-cycle multiplier (unchanged)
//   f32_add_seq   — shift-by-1 adder, variable latency
//   kalman_update — FSM with rank-1 P_UPDATE replacing GEMM
//
// FSM states: IDLE→S_COMP→NR0→NR1→NR2→K_COMP→KY_COMP→X_ADD→P_UPDATE→DONE_S
//
// iter6 adds state retention: x_reg[3] + p_reg[9] stored on-chip.
//   DONE_S captures x_out_arr→x_reg, acc→p_reg (for next update).
//   Write-event ports (x_wr_en/idx/val, p_wr_en/idx/val) allow the SPI
//   interface to initialise x_reg and p_reg before the first update.
//   x_in and P_in input ports are removed; core reads from x_reg and p_reg.
// =============================================================================

// -----------------------------------------------------------------------------
// f32_mul_seq — Sequential IEEE-754 F32 multiplier (26-cycle, unchanged)
// -----------------------------------------------------------------------------
module f32_mul_seq (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] result,
    output logic        done
);
    logic        r_sign_r;
    logic signed [9:0] exp_sum_r;
    logic        a_nan_r, b_nan_r, a_inf_r, b_inf_r, a_zero_r, b_zero_r;

    logic [47:0] accum;
    logic [47:0] b_shifted;
    logic [23:0] a_shifted;
    logic [4:0]  bit_cnt;
    logic        busy;

    logic [47:0] new_accum;
    assign new_accum = a_shifted[0] ? (accum + b_shifted) : accum;

    logic [22:0]       norm_man;
    logic signed [9:0] norm_exp;
    logic [31:0]       final_result;

    always_comb begin
        norm_man = accum[47] ? accum[46:24] : accum[45:23];
        norm_exp = accum[47] ? (exp_sum_r + 10'd1) : exp_sum_r;

        if (a_nan_r || b_nan_r)
            final_result = 32'h7FC0_0000;
        else if ((a_inf_r && b_zero_r) || (b_inf_r && a_zero_r))
            final_result = 32'h7FC0_0000;
        else if (a_inf_r || b_inf_r)
            final_result = {r_sign_r, 8'hFF, 23'h0};
        else if (a_zero_r || b_zero_r)
            final_result = {r_sign_r, 31'b0};
        else if ($signed(norm_exp) >= $signed(10'd255))
            final_result = {r_sign_r, 8'hFF, 23'h0};
        else if ($signed(norm_exp) <= $signed(10'd0))
            final_result = {r_sign_r, 31'b0};
        else
            final_result = {r_sign_r, norm_exp[7:0], norm_man};
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0; done <= 1'b0; result <= 32'h0; bit_cnt <= 5'd0;
            accum <= 48'h0; b_shifted <= 48'h0; a_shifted <= 24'h0;
            r_sign_r <= 1'b0; exp_sum_r <= 10'h0;
            a_nan_r <= 1'b0; b_nan_r <= 1'b0; a_inf_r <= 1'b0;
            b_inf_r <= 1'b0; a_zero_r <= 1'b0; b_zero_r <= 1'b0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                r_sign_r  <= a[31] ^ b[31];
                exp_sum_r <= $signed({2'b00, a[30:23]}) + $signed({2'b00, b[30:23]})
                             - $signed(10'd127);
                a_nan_r  <= (a[30:23]==8'hFF) && (a[22:0]!=23'h0);
                b_nan_r  <= (b[30:23]==8'hFF) && (b[22:0]!=23'h0);
                a_inf_r  <= (a[30:23]==8'hFF) && (a[22:0]==23'h0);
                b_inf_r  <= (b[30:23]==8'hFF) && (b[22:0]==23'h0);
                a_zero_r <= (a[30:23]==8'h0)  && (a[22:0]==23'h0);
                b_zero_r <= (b[30:23]==8'h0)  && (b[22:0]==23'h0);
                a_shifted <= (a[30:23]==8'h0) ? {1'b0,a[22:0]} : {1'b1,a[22:0]};
                b_shifted <= {24'b0, ((b[30:23]==8'h0) ? {1'b0,b[22:0]} : {1'b1,b[22:0]})};
                accum <= 48'h0; bit_cnt <= 5'd0; busy <= 1'b1;
            end else if (busy) begin
                if (bit_cnt < 5'd24) begin
                    accum     <= new_accum;
                    b_shifted <= {b_shifted[46:0], 1'b0};
                    a_shifted <= {1'b0, a_shifted[23:1]};
                    bit_cnt   <= bit_cnt + 5'd1;
                end else begin
                    result <= final_result; done <= 1'b1; busy <= 1'b0; bit_cnt <= 5'd0;
                end
            end
        end
    end
endmodule

// -----------------------------------------------------------------------------
// f32_add_seq — Sequential IEEE-754 F32 adder (variable latency, shift-by-1)
//
// Latency (cycles from start to done visible):
//   Special case (nan/inf/zero): 1 cycle
//   Normal, shift_amt=N, norm_shifts=M: 1(idle→align) + N(align) + 1(add) + M(norm)
//   Minimum: 2 cycles (shift=0, no norm); Maximum: ~49 cycles (shift=23, norm=23)
//
// Handshake: start pulses 1 cycle; done pulses 1 cycle; result valid when done=1.
// -----------------------------------------------------------------------------
module f32_add_seq (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] result,
    output logic        done
);

    // Internal states
    typedef enum logic [1:0] {
        FADD_IDLE  = 2'd0,
        FADD_ALIGN = 2'd1,
        FADD_ADD   = 2'd2,
        FADD_NORM  = 2'd3
    } fadd_st_t;
    fadd_st_t add_st;

    // Registered at start
    logic        r_sign_r;        // result sign = big operand sign
    logic        eff_sub_r;       // effective subtraction flag
    logic [7:0]  big_exp_r;       // exponent of larger operand
    logic [23:0] big_man_r;       // mantissa of larger (with implicit 1)
    logic [23:0] sml_man_r;       // mantissa of smaller — right-shifted each ALIGN cycle
    logic [4:0]  align_cnt_r;     // alignment shift counter (counts down to 0)
    logic        is_special_r;    // any special-case applies
    logic [31:0] spec_result_r;   // precomputed special-case result

    // After FADD_ADD
    logic [24:0] sum_raw_r;       // full addition result (25 bits)
    logic [8:0]  res_exp_r;       // current exponent during normalization

    // -------------------------------------------------------------------------
    // Combinational inputs decode (used at start)
    // -------------------------------------------------------------------------
    logic        a_nan_c, b_nan_c, a_inf_c, b_inf_c, a_zero_c, b_zero_c;
    logic        is_spec_c;
    logic [31:0] spec_res_c;
    logic        do_swap_c;
    logic [7:0]  big_exp_c;
    logic [23:0] big_man_c, sml_man_c;
    logic [7:0]  exp_diff_c;
    logic [4:0]  shift_amt_c;

    always_comb begin
        a_nan_c  = (a[30:23]==8'hFF) && (a[22:0]!=23'h0);
        b_nan_c  = (b[30:23]==8'hFF) && (b[22:0]!=23'h0);
        a_inf_c  = (a[30:23]==8'hFF) && (a[22:0]==23'h0);
        b_inf_c  = (b[30:23]==8'hFF) && (b[22:0]==23'h0);
        a_zero_c = (a[30:23]==8'h0)  && (a[22:0]==23'h0);
        b_zero_c = (b[30:23]==8'h0)  && (b[22:0]==23'h0);
        is_spec_c = a_nan_c || b_nan_c || a_inf_c || b_inf_c || a_zero_c || b_zero_c;

        // Special-case result
        if (a_nan_c || b_nan_c)
            spec_res_c = 32'h7FC0_0000;
        else if (a_inf_c && b_inf_c && (a[31] != b[31]))
            spec_res_c = 32'h7FC0_0000;
        else if (a_inf_c || b_inf_c)
            spec_res_c = a_inf_c ? a : b;
        else if (a_zero_c && b_zero_c)
            spec_res_c = 32'h0;
        else if (a_zero_c)
            spec_res_c = b;
        else
            spec_res_c = a;  // b_zero_c

        // Big/small ordering
        if (a[30:23] > b[30:23])       do_swap_c = 1'b0;
        else if (b[30:23] > a[30:23])  do_swap_c = 1'b1;
        else                           do_swap_c = (b[22:0] > a[22:0]);

        if (do_swap_c) begin
            big_exp_c = b[30:23];
            big_man_c = (b[30:23]==8'h0) ? {1'b0,b[22:0]} : {1'b1,b[22:0]};
            sml_man_c = (a[30:23]==8'h0) ? {1'b0,a[22:0]} : {1'b1,a[22:0]};
            exp_diff_c = b[30:23] - a[30:23];
        end else begin
            big_exp_c = a[30:23];
            big_man_c = (a[30:23]==8'h0) ? {1'b0,a[22:0]} : {1'b1,a[22:0]};
            sml_man_c = (b[30:23]==8'h0) ? {1'b0,b[22:0]} : {1'b1,b[22:0]};
            exp_diff_c = a[30:23] - b[30:23];
        end
        shift_amt_c = (exp_diff_c > 8'd23) ? 5'd23 : exp_diff_c[4:0];
    end

    // -------------------------------------------------------------------------
    // Combinational: the 25-bit add/sub result (used in FADD_ADD state)
    // -------------------------------------------------------------------------
    logic [24:0] fadd_sum;
    always_comb begin
        if (!eff_sub_r)
            fadd_sum = {1'b0, big_man_r} + {1'b0, sml_man_r};
        else
            fadd_sum = {1'b0, big_man_r} - {1'b0, sml_man_r};
    end

    // -------------------------------------------------------------------------
    // Sequential state machine
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            add_st        <= FADD_IDLE;
            done          <= 1'b0;
            result        <= 32'h0;
            r_sign_r      <= 1'b0;
            eff_sub_r     <= 1'b0;
            big_exp_r     <= 8'h0;
            big_man_r     <= 24'h0;
            sml_man_r     <= 24'h0;
            align_cnt_r   <= 5'h0;
            sum_raw_r     <= 25'h0;
            res_exp_r     <= 9'h0;
            is_special_r  <= 1'b0;
            spec_result_r <= 32'h0;
        end else begin
            done <= 1'b0;  // default: deasserted

            case (add_st)

                FADD_IDLE: begin
                    if (start) begin
                        if (is_spec_c) begin
                            // Immediate output for special cases (1-cycle latency)
                            result <= spec_res_c;
                            done   <= 1'b1;
                            // add_st stays FADD_IDLE
                        end else begin
                            // Capture for normal operation
                            r_sign_r    <= do_swap_c ? b[31] : a[31];
                            eff_sub_r   <= a[31] ^ b[31];  // always XOR of signs
                            big_exp_r   <= big_exp_c;
                            big_man_r   <= big_man_c;
                            sml_man_r   <= sml_man_c;
                            align_cnt_r <= shift_amt_c;
                            add_st      <= FADD_ALIGN;
                        end
                        // Also register spec info in case needed later
                        is_special_r  <= is_spec_c;
                        spec_result_r <= spec_res_c;
                    end
                end

                FADD_ALIGN: begin
                    if (align_cnt_r == 5'h0) begin
                        // Alignment complete — proceed to addition
                        add_st <= FADD_ADD;
                    end else begin
                        sml_man_r   <= {1'b0, sml_man_r[23:1]};  // right-shift 1 bit
                        align_cnt_r <= align_cnt_r - 5'd1;
                    end
                end

                FADD_ADD: begin
                    // fadd_sum computed combinationally from big_man_r/sml_man_r/eff_sub_r
                    if (fadd_sum == 25'h0) begin
                        result <= 32'h0;
                        done   <= 1'b1;
                        add_st <= FADD_IDLE;
                    end else if (fadd_sum[24]) begin
                        // Carry out: right-shift by 1, exp+1
                        if (big_exp_r == 8'hFE) begin
                            result <= {r_sign_r, 8'hFF, 23'h0};  // overflow → inf
                        end else begin
                            result <= {r_sign_r, big_exp_r + 8'd1, fadd_sum[23:1]};
                        end
                        done   <= 1'b1;
                        add_st <= FADD_IDLE;
                    end else if (fadd_sum[23]) begin
                        // Already normalized
                        result <= {r_sign_r, big_exp_r, fadd_sum[22:0]};
                        done   <= 1'b1;
                        add_st <= FADD_IDLE;
                    end else begin
                        // Need left-shifts to normalize
                        sum_raw_r <= fadd_sum;
                        res_exp_r <= {1'b0, big_exp_r};
                        add_st    <= FADD_NORM;
                    end
                end

                FADD_NORM: begin
                    if (sum_raw_r[23] || res_exp_r == 9'h0) begin
                        // Normalized or exponent underflow → output
                        result <= (res_exp_r == 9'h0) ? {r_sign_r, 31'b0}
                                                      : {r_sign_r, res_exp_r[7:0], sum_raw_r[22:0]};
                        done   <= 1'b1;
                        add_st <= FADD_IDLE;
                    end else begin
                        sum_raw_r <= {sum_raw_r[23:0], 1'b0};  // left-shift
                        res_exp_r <= res_exp_r - 9'd1;
                    end
                end

                default: add_st <= FADD_IDLE;
            endcase
        end
    end

endmodule

// -----------------------------------------------------------------------------
// kalman_update — Kalman filter update (F32, iter5)
// H = [1, 0, 0] (scalar measurement)
//
// P covariance update uses rank-1 formula:
//   P_new[i,j] = P[i,j] - K[i] * P[0,j]   for i,j ∈ {0,1,2}
// This replaces the full 27-MAC GEMM with 9 independent MACs.
//
// Arithmetic: 1 shared f32_mul_seq + 1 shared f32_add_seq.
//   Both have start/done handshake; FSM stalls until each completes.
//
// FSM states (9):
//   IDLE → S_COMP → NR0 → NR1 → K_COMP → KY_COMP → X_ADD → P_UPDATE → DONE_S
// -----------------------------------------------------------------------------
module kalman_update (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [31:0]  z,
    // Write-event ports: SPI interface writes x and P before each update
    input  logic        x_wr_en,
    input  logic [1:0]  x_wr_idx,   // 0..2
    input  logic [31:0] x_wr_val,
    input  logic        p_wr_en,
    input  logic [3:0]  p_wr_idx,   // 0..8
    input  logic [31:0] p_wr_val,
    input  logic [31:0]  r_val,
    output logic [95:0]  x_out,
    output logic [287:0] P_out,
    output logic        done,
    output logic        busy
);

    localparam logic [31:0] F32_TWO = 32'h4000_0000;

    typedef enum logic [3:0] {
        IDLE     = 4'd0,
        S_COMP   = 4'd1,
        NR0      = 4'd2,
        NR1      = 4'd3,
        NR2      = 4'd9,
        K_COMP   = 4'd4,
        KY_COMP  = 4'd5,
        X_ADD    = 4'd6,
        P_UPDATE = 4'd7,
        DONE_S   = 4'd8
    } state_t;

    state_t state, state_nxt;
    logic [1:0] sub_cnt, sub_nxt;

    // -------------------------------------------------------------------------
    // On-chip state registers (retain x and P across updates)
    // Written by write-event ports from the SPI interface, and captured
    // from x_out_arr / acc at DONE_S.
    // -------------------------------------------------------------------------
    logic [31:0] x_reg [0:2];
    logic [31:0] p_reg [0:8];

    // -------------------------------------------------------------------------
    // Output packing
    // -------------------------------------------------------------------------
    logic [31:0] x_out_arr [0:2];
    genvar ku;
    generate
        for (ku = 0; ku < 3; ku++) begin : pack_xout
            assign x_out[ku*32 +: 32] = x_out_arr[ku];
        end
    endgenerate

    logic [31:0] acc [0:8];
    generate
        for (ku = 0; ku < 9; ku++) begin : pack_pout
            assign P_out[ku*32 +: 32] = acc[ku];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Intermediate registers
    // -------------------------------------------------------------------------
    logic [31:0] y_tilde;         // z - x_in[0]
    logic [31:0] S_reg;           // P_in[0] + r_val
    logic [31:0] S_inv;           // 1/S via Newton-Raphson
    logic [31:0] nr_x;            // NR iterate
    logic [31:0] sx_reg;          // S * nr_x (NR intermediate)
    logic [31:0] two_m_sx_r;      // 2 - sx_reg  (NR sub1 add result, fed to sub2 mul)
    logic [31:0] K_reg [0:2];     // Kalman gain K[i] = P[i*3] * S_inv
    logic [31:0] ky_arr [0:2];    // K[i] * y_tilde

    // P_UPDATE counters: j fastest, then i
    logic [1:0] i_cnt, j_cnt;
    logic       pu_last;
    assign pu_last = (i_cnt == 2'd2) && (j_cnt == 2'd2);

    // -------------------------------------------------------------------------
    // 9-to-1 mux: p_reg[i*3+j]  (add_a in P_UPDATE)
    // -------------------------------------------------------------------------
    logic [31:0] p_in_elem;
    always_comb begin
        case ({i_cnt, j_cnt})
            4'b0000: p_in_elem = p_reg[0];
            4'b0001: p_in_elem = p_reg[1];
            4'b0010: p_in_elem = p_reg[2];
            4'b0100: p_in_elem = p_reg[3];
            4'b0101: p_in_elem = p_reg[4];
            4'b0110: p_in_elem = p_reg[5];
            4'b1000: p_in_elem = p_reg[6];
            4'b1001: p_in_elem = p_reg[7];
            4'b1010: p_in_elem = p_reg[8];
            default: p_in_elem = 32'h0;
        endcase
    end

    // -------------------------------------------------------------------------
    // Shared sequential multiplier and adder
    // -------------------------------------------------------------------------
    logic [31:0] mul_a, mul_b, mul_result;
    logic        mul_start_w, mul_done_w, mul_active;

    logic [31:0] add_a, add_b, add_result;
    logic        add_start_w, add_done_w, add_active;

    f32_mul_seq u_shared_mul (
        .clk(clk), .rst_n(rst_n), .start(mul_start_w),
        .a(mul_a), .b(mul_b), .result(mul_result), .done(mul_done_w)
    );
    f32_add_seq u_shared_add (
        .clk(clk), .rst_n(rst_n), .start(add_start_w),
        .a(add_a), .b(add_b), .result(add_result), .done(add_done_w)
    );

    // Newton-Raphson initial seed
    function automatic logic [31:0] nr_seed (input logic [31:0] s);
        nr_seed = 32'h7EF1_27EA - s;
    endfunction

    // -------------------------------------------------------------------------
    // mul_start_w
    //   NR sub 0 and sub 2: multiply S*nr_x, then nr_x*(2-sx)
    //   K_COMP, KY_COMP: one mul per sub_cnt
    //   P_UPDATE: start each MAC when neither mul nor add is running
    // -------------------------------------------------------------------------
    always_comb begin
        mul_start_w = 1'b0;
        if (!mul_active) begin
            case (state)
                NR0, NR1, NR2:
                    if (sub_cnt == 2'd0 || sub_cnt == 2'd2) mul_start_w = 1'b1;
                K_COMP:  mul_start_w = 1'b1;
                KY_COMP: mul_start_w = 1'b1;
                P_UPDATE:
                    // Don't start a new mul while add is running or if both are idle
                    // after the last MAC (handled by termination in next-state)
                    if (!add_active) mul_start_w = 1'b1;
                default: ;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // add_start_w
    //   S_COMP: each sub-cycle is one add
    //   NR sub 1: add(F32_TWO, -sx_reg) = 2-sx
    //   X_ADD: each sub-cycle is one add
    //   P_UPDATE: add fires immediately after mul_done_w
    // -------------------------------------------------------------------------
    always_comb begin
        add_start_w = 1'b0;
        if (!add_active) begin
            case (state)
                S_COMP:
                    if (sub_cnt == 2'd0 || sub_cnt == 2'd1) add_start_w = 1'b1;
                NR0, NR1, NR2:
                    if (sub_cnt == 2'd1) add_start_w = 1'b1;
                X_ADD:
                    if (sub_cnt == 2'd0 || sub_cnt == 2'd1 || sub_cnt == 2'd2)
                        add_start_w = 1'b1;
                P_UPDATE:
                    // Triggered by mul_done_w (not just !add_active alone)
                    if (mul_done_w) add_start_w = 1'b1;
                default: ;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // mul_a / mul_b input mux
    // -------------------------------------------------------------------------
    always_comb begin
        mul_a = 32'h0;
        mul_b = 32'h0;
        case (state)
            NR0, NR1, NR2: begin
                if (sub_cnt == 2'd0) begin
                    mul_a = S_reg; mul_b = nr_x;
                end else begin
                    // sub_cnt==2: nr_x * (2-sx), using registered two_m_sx_r
                    mul_a = nr_x; mul_b = two_m_sx_r;
                end
            end
            K_COMP: begin
                mul_b = S_inv;
                case (sub_cnt)
                    2'd0: mul_a = p_reg[0];
                    2'd1: mul_a = p_reg[3];
                    2'd2: mul_a = p_reg[6];
                    default: mul_a = 32'h0;
                endcase
            end
            KY_COMP: begin
                mul_b = y_tilde;
                case (sub_cnt)
                    2'd0: mul_a = K_reg[0];
                    2'd1: mul_a = K_reg[1];
                    2'd2: mul_a = K_reg[2];
                    default: mul_a = 32'h0;
                endcase
            end
            P_UPDATE: begin
                // K_reg[i] * p_reg[j]  (first row of P)
                case (i_cnt)
                    2'd0: mul_a = K_reg[0];
                    2'd1: mul_a = K_reg[1];
                    2'd2: mul_a = K_reg[2];
                    default: mul_a = 32'h0;
                endcase
                case (j_cnt)
                    2'd0: mul_b = p_reg[0];
                    2'd1: mul_b = p_reg[1];
                    2'd2: mul_b = p_reg[2];
                    default: mul_b = 32'h0;
                endcase
            end
            default: ;
        endcase
    end

    // -------------------------------------------------------------------------
    // add_a / add_b input mux
    // -------------------------------------------------------------------------
    always_comb begin
        add_a = 32'h0;
        add_b = 32'h0;
        case (state)
            S_COMP: begin
                if (sub_cnt == 2'd0) begin
                    add_a = z;
                    add_b = {~x_reg[0][31], x_reg[0][30:0]};  // z - x[0]
                end else begin
                    add_a = p_reg[0];
                    add_b = r_val;  // S = P[0,0] + R
                end
            end
            NR0, NR1, NR2: begin
                // sub_cnt==1: 2.0 - sx_reg
                add_a = F32_TWO;
                add_b = {~sx_reg[31], sx_reg[30:0]};
            end
            X_ADD: begin
                case (sub_cnt)
                    2'd0: begin add_a = x_reg[0]; add_b = ky_arr[0]; end
                    2'd1: begin add_a = x_reg[1]; add_b = ky_arr[1]; end
                    2'd2: begin add_a = x_reg[2]; add_b = ky_arr[2]; end
                    default: ;
                endcase
            end
            P_UPDATE: begin
                // P[i,j] + (- K[i]*P[0,j])
                add_a = p_in_elem;
                add_b = {~mul_result[31], mul_result[30:0]};  // negate mul result
            end
            default: ;
        endcase
    end

    // -------------------------------------------------------------------------
    // FSM next-state + sub_nxt
    // Stall rules:
    //   States using add: advance sub_cnt only on add_done_w
    //   NR sub 0, sub 2: stall until mul_done_w
    //   NR sub 1: stall until add_done_w
    //   K_COMP, KY_COMP: stall each sub until mul_done_w
    //   P_UPDATE: advance i/j on add_done_w; exit on pu_last && add_done_w
    // -------------------------------------------------------------------------
    always_comb begin
        state_nxt = state;
        sub_nxt   = sub_cnt + 2'd1;  // default: advance

        case (state)
            IDLE:
                if (start) begin state_nxt = S_COMP; sub_nxt = 2'd0; end
                else sub_nxt = 2'd0;

            S_COMP: begin
                // Each sub-cycle stalls on add
                if (!add_done_w) sub_nxt = sub_cnt;
                else if (sub_cnt == 2'd1) begin state_nxt = NR0; sub_nxt = 2'd0; end
            end

            NR0, NR1, NR2: begin
                if (sub_cnt == 2'd0) begin
                    // First mul stall
                    if (!mul_done_w) sub_nxt = 2'd0;
                    // else sub_nxt = 1 (default)
                end else if (sub_cnt == 2'd1) begin
                    // Add stall (2-sx computation)
                    if (!add_done_w) sub_nxt = 2'd1;
                    // else sub_nxt = 2 (default)
                end else begin
                    // sub_cnt == 2: second mul stall
                    if (!mul_done_w) sub_nxt = 2'd2;
                    else begin
                        sub_nxt = 2'd0;
                        case (state)
                            NR0: state_nxt = NR1;
                            NR1: state_nxt = NR2;
                            NR2: state_nxt = K_COMP;
                            default: state_nxt = K_COMP;
                        endcase
                    end
                end
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
                // 3 sub-cycles (sub 3 removed — one_minus_k0_r no longer needed)
                if (!add_done_w) sub_nxt = sub_cnt;
                else if (sub_cnt == 2'd2) begin state_nxt = P_UPDATE; sub_nxt = 2'd0; end
            end

            P_UPDATE: begin
                sub_nxt = 2'd0;
                if (add_done_w && pu_last) state_nxt = DONE_S;
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
            state     <= IDLE;
            sub_cnt   <= 2'd0;
            mul_active <= 1'b0;
            add_active <= 1'b0;
            y_tilde   <= 32'h0;
            S_reg     <= 32'h0;
            S_inv     <= 32'h0;
            nr_x      <= 32'h0;
            sx_reg    <= 32'h0;
            two_m_sx_r <= 32'h0;
            for (ii = 0; ii < 3; ii++) begin
                K_reg[ii]     <= 32'h0;
                ky_arr[ii]    <= 32'h0;
                x_out_arr[ii] <= 32'h0;
                x_reg[ii]     <= 32'h0;
            end
            i_cnt <= 2'd0; j_cnt <= 2'd0;
            for (ii = 0; ii < 9; ii++) begin
                acc[ii]   <= 32'h0;
                p_reg[ii] <= 32'h0;
            end
        end else begin
            // Write-event: SPI interface writes x or P into on-chip registers
            if (x_wr_en) begin
                case (x_wr_idx)
                    2'd0: x_reg[0] <= x_wr_val;
                    2'd1: x_reg[1] <= x_wr_val;
                    2'd2: x_reg[2] <= x_wr_val;
                    default: ;
                endcase
            end
            if (p_wr_en) begin
                case (p_wr_idx)
                    4'd0: p_reg[0] <= p_wr_val;
                    4'd1: p_reg[1] <= p_wr_val;
                    4'd2: p_reg[2] <= p_wr_val;
                    4'd3: p_reg[3] <= p_wr_val;
                    4'd4: p_reg[4] <= p_wr_val;
                    4'd5: p_reg[5] <= p_wr_val;
                    4'd6: p_reg[6] <= p_wr_val;
                    4'd7: p_reg[7] <= p_wr_val;
                    4'd8: p_reg[8] <= p_wr_val;
                    default: ;
                endcase
            end
            state   <= state_nxt;
            sub_cnt <= sub_nxt;

            // Mul/add active tracking
            if (mul_start_w)  mul_active <= 1'b1;
            else if (mul_done_w) mul_active <= 1'b0;

            if (add_start_w)  add_active <= 1'b1;
            else if (add_done_w) add_active <= 1'b0;

            case (state)

                // S_COMP: 2 add sub-cycles
                // sub 0: y_tilde = z - x_in[0]
                // sub 1: S_reg = P[0,0] + R; nr_x = NR seed
                S_COMP: begin
                    if (add_done_w) begin
                        if (sub_cnt == 2'd0) begin
                            y_tilde <= add_result;
                        end else begin
                            S_reg <= add_result;
                            nr_x  <= nr_seed(add_result);
                        end
                    end
                end

                // NR0, NR1, NR2: sub 0 → sx=S*nr_x; sub 1 → two_m_sx=2-sx (add);
                //                sub 2 → nr_x = nr_x*(2-sx)
                NR0, NR1, NR2: begin
                    if (mul_done_w && sub_cnt == 2'd0) sx_reg <= mul_result;
                    if (add_done_w && sub_cnt == 2'd1) two_m_sx_r <= add_result;
                    if (mul_done_w && sub_cnt == 2'd2) begin
                        S_inv <= mul_result;
                        // NR0 and NR1 update nr_x for next iteration; NR2 is final
                        if (state == NR0 || state == NR1) nr_x <= mul_result;
                    end
                end

                // K_COMP: K[i] = P[i*3] * S_inv
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

                // KY_COMP: ky[i] = K[i] * y_tilde
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

                // X_ADD: 3 sub-cycles; x_out[i] = x_in[i] + ky[i]
                //   (sub 3 removed — one_minus_k0_r no longer needed for rank-1)
                X_ADD: begin
                    if (add_done_w) begin
                        case (sub_cnt)
                            2'd0: x_out_arr[0] <= add_result;
                            2'd1: x_out_arr[1] <= add_result;
                            2'd2: begin
                                x_out_arr[2] <= add_result;
                                // Init P_UPDATE counters on transition
                                i_cnt <= 2'd0;
                                j_cnt <= 2'd0;
                            end
                            default: ;
                        endcase
                    end
                end

                // P_UPDATE: 9 MACs, rank-1 formula
                //   mul: K[i] * P[0,j]  →  add: P[i,j] + neg(mul)  →  acc[i*3+j]
                //   i/j advance on add_done_w
                P_UPDATE: begin
                    if (add_done_w) begin
                        // Write result
                        case ({i_cnt, j_cnt})
                            4'b0000: acc[0] <= add_result;
                            4'b0001: acc[1] <= add_result;
                            4'b0010: acc[2] <= add_result;
                            4'b0100: acc[3] <= add_result;
                            4'b0101: acc[4] <= add_result;
                            4'b0110: acc[5] <= add_result;
                            4'b1000: acc[6] <= add_result;
                            4'b1001: acc[7] <= add_result;
                            4'b1010: acc[8] <= add_result;
                            default: ;
                        endcase
                        // Advance counters (j fastest)
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

                // DONE_S: capture outputs into on-chip state registers for next update
                DONE_S: begin
                    for (ii = 0; ii < 3; ii++) x_reg[ii] <= x_out_arr[ii];
                    for (ii = 0; ii < 9; ii++) p_reg[ii] <= acc[ii];
                end
                default: ;
            endcase
        end
    end

    assign done = (state == DONE_S);
    assign busy = (state != IDLE) && (state != DONE_S);

endmodule
