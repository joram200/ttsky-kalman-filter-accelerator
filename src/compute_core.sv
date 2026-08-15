`timescale 1ns/1ps
// =============================================================================
// compute_core.sv — F32 + Time-multiplexed compute core for tt_um_joram200
// Task 6 (iteration 2): F32 arithmetic, 1 shared mul+add per module,
//   combinational IKH/P forwarding, eliminated redundant register banks.
//
// Area reductions vs task6 iter1 (1849%):
//   kalman: 7 parallel f32_add → 1 shared; IKH/P_out_arr/gs_A/gs_B eliminated
//   gemm:   A_reg/B_reg/C_arr eliminated; C driven directly from acc
// =============================================================================

// -----------------------------------------------------------------------------
// f32_mul — Combinational IEEE-754 single-precision multiplier
// -----------------------------------------------------------------------------
module f32_mul (
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] result
);

    logic        a_sign, b_sign, r_sign;
    logic [7:0]  a_exp,  b_exp;
    logic [23:0] a_man,  b_man;

    assign a_sign = a[31];
    assign b_sign = b[31];
    assign a_exp  = a[30:23];
    assign b_exp  = b[30:23];
    assign a_man  = (a_exp == 8'h0) ? {1'b0, a[22:0]} : {1'b1, a[22:0]};
    assign b_man  = (b_exp == 8'h0) ? {1'b0, b[22:0]} : {1'b1, b[22:0]};

    logic a_nan, b_nan, a_inf, b_inf, a_zero, b_zero;
    assign a_nan  = (a_exp == 8'hFF) && (a[22:0] != 23'h0);
    assign b_nan  = (b_exp == 8'hFF) && (b[22:0] != 23'h0);
    assign a_inf  = (a_exp == 8'hFF) && (a[22:0] == 23'h0);
    assign b_inf  = (b_exp == 8'hFF) && (b[22:0] == 23'h0);
    assign a_zero = (a_exp == 8'h0)  && (a[22:0] == 23'h0);
    assign b_zero = (b_exp == 8'h0)  && (b[22:0] == 23'h0);

    logic [47:0] product;
    assign product = {24'b0, a_man} * {24'b0, b_man};

    logic signed [9:0] exp_sum;
    assign exp_sum = $signed({2'b00, a_exp}) + $signed({2'b00, b_exp})
                     - $signed(10'd127);

    logic [22:0] norm_man;
    logic signed [9:0] norm_exp;
    assign norm_man = product[47] ? product[46:24] : product[45:23];
    assign norm_exp = product[47] ? (exp_sum + 10'd1) : exp_sum;

    assign r_sign = a_sign ^ b_sign;

    assign result =
        (a_nan || b_nan)                              ? 32'h7FC0_0000 :
        ((a_inf && b_zero) || (b_inf && a_zero))      ? 32'h7FC0_0000 :
        (a_inf || b_inf)                              ? {r_sign, 8'hFF, 23'h0} :
        (a_zero || b_zero)                            ? {r_sign, 31'b0}        :
        ($signed(norm_exp) >= $signed(10'd255))       ? {r_sign, 8'hFF, 23'h0} :
        ($signed(norm_exp) <= $signed(10'd0))         ? {r_sign, 31'b0}        :
                                                        {r_sign, norm_exp[7:0], norm_man};

endmodule

// -----------------------------------------------------------------------------
// f32_add — Combinational IEEE-754 single-precision adder/subtractor
// -----------------------------------------------------------------------------
module f32_add (
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] result
);

    logic        a_sign, b_sign;
    logic [7:0]  a_exp,  b_exp;
    logic [22:0] a_frac, b_frac;

    assign a_sign = a[31];
    assign b_sign = b[31];
    assign a_exp  = a[30:23];
    assign b_exp  = b[30:23];
    assign a_frac = a[22:0];
    assign b_frac = b[22:0];

    logic a_nan, b_nan, a_inf, b_inf, a_zero, b_zero;
    assign a_nan  = (a_exp == 8'hFF) && (a_frac != 23'h0);
    assign b_nan  = (b_exp == 8'hFF) && (b_frac != 23'h0);
    assign a_inf  = (a_exp == 8'hFF) && (a_frac == 23'h0);
    assign b_inf  = (b_exp == 8'hFF) && (b_frac == 23'h0);
    assign a_zero = (a_exp == 8'h0)  && (a_frac == 23'h0);
    assign b_zero = (b_exp == 8'h0)  && (b_frac == 23'h0);

    logic        big_sign, sml_sign;
    logic [7:0]  big_exp,  sml_exp;
    logic [23:0] big_man,  sml_man;
    logic        do_swap;

    always_comb begin
        if (a_exp > b_exp) begin
            do_swap = 1'b0;
        end else if (b_exp > a_exp) begin
            do_swap = 1'b1;
        end else begin
            do_swap = (b_frac > a_frac);
        end

        if (do_swap) begin
            big_sign = b_sign; big_exp = b_exp;
            big_man  = (b_exp == 8'h0) ? {1'b0, b_frac} : {1'b1, b_frac};
            sml_sign = a_sign; sml_exp = a_exp;
            sml_man  = (a_exp == 8'h0) ? {1'b0, a_frac} : {1'b1, a_frac};
        end else begin
            big_sign = a_sign; big_exp = a_exp;
            big_man  = (a_exp == 8'h0) ? {1'b0, a_frac} : {1'b1, a_frac};
            sml_sign = b_sign; sml_exp = b_exp;
            sml_man  = (b_exp == 8'h0) ? {1'b0, b_frac} : {1'b1, b_frac};
        end
    end

    logic [4:0]  shift_amt;
    logic [23:0] sml_aligned;

    assign shift_amt   = ((big_exp - sml_exp) > 8'd31) ? 5'd31 : big_exp[4:0] - sml_exp[4:0];
    assign sml_aligned = sml_man >> shift_amt;

    logic [24:0] sum_raw;
    logic        eff_sub;
    logic        r_sign;

    always_comb begin
        eff_sub = big_sign ^ sml_sign;
        r_sign  = big_sign;
        if (!eff_sub) begin
            sum_raw = {1'b0, big_man} + {1'b0, sml_aligned};
        end else begin
            sum_raw = {1'b0, big_man} - {1'b0, sml_aligned};
        end
    end

    logic [8:0]  res_exp;
    logic [22:0] res_man;
    logic [23:0] shifted;

    logic [4:0] lz_count;
    always_comb begin : lz_encoder
        lz_count = 5'd24;
        for (int i = 0; i <= 23; i++) begin
            if (sum_raw[i]) lz_count = 5'(23 - i);
        end
    end

    assign shifted  = sum_raw[23:0] << lz_count;
    assign res_exp  = sum_raw[24] ? ({1'b0, big_exp} + 9'd1)           :
                      sum_raw[23] ? {1'b0, big_exp}                      :
                                    ({1'b0, big_exp} - {4'b0, lz_count});
    assign res_man  = sum_raw[24] ? sum_raw[23:1]  :
                      sum_raw[23] ? sum_raw[22:0]  :
                                    shifted[22:0];

    assign result =
        (a_nan || b_nan)                          ? 32'h7FC0_0000           :
        (a_inf && b_inf && (a_sign != b_sign))    ? 32'h7FC0_0000           :
        (a_inf || b_inf)                          ? (a_inf ? a : b)         :
        (a_zero && b_zero)                        ? 32'h0                   :
        a_zero                                    ? b                       :
        b_zero                                    ? a                       :
        (sum_raw == 25'h0)                        ? 32'h0                   :
        (res_exp[8] || (res_exp == 9'h0))         ? {r_sign, 31'b0}         :
        (res_exp >= 9'h0FF)                       ? {r_sign, 8'hFF, 23'h0}  :
                                                    {r_sign, res_exp[7:0], res_man};

endmodule

// -----------------------------------------------------------------------------
// gemm_systolic — 3×3 matrix multiply C=A×B, time-multiplexed (F32, iter2)
// 1 shared f32_mul + 1 shared f32_add; COMPUTE runs 27 cycles.
// Counter order: k (outer, 0→2), i (middle, 0→2), j (inner, 0→2).
// Each COMPUTE cycle: acc[i*3+j] += A_in[i*3+k] × B_in[k*3+j]
//
// Eliminated vs iter1: A_reg, B_reg, C_arr (read inputs directly; C from acc).
// A and B inputs must remain stable from gs_start through gs_done (guaranteed
// by kalman_update: gs_A_flat/gs_B_flat are combinational wires from
// stable K_reg/P_reg during WAIT_P).
//
// Ports A, B, C are flat packed (9×32=288 bits) for Yosys compatibility.
// -----------------------------------------------------------------------------
module gemm_systolic (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [287:0] A,
    input  logic [287:0] B,
    output logic [287:0] C,
    output logic        done,
    output logic        busy
);

    typedef enum logic [1:0] {
        IDLE    = 2'd0,
        LOAD    = 2'd1,
        COMPUTE = 2'd2,
        FINISH  = 2'd3
    } gstate_t;

    gstate_t gstate, gstate_nxt;

    // Unpack flat inputs to wire arrays (no registered copies)
    logic [31:0] A_in [0:8];
    logic [31:0] B_in [0:8];
    genvar unp;
    generate
        for (unp = 0; unp < 9; unp++) begin : unpack_in
            assign A_in[unp] = A[unp*32 +: 32];
            assign B_in[unp] = B[unp*32 +: 32];
        end
    endgenerate

    // acc is the working accumulator; C driven directly from acc (no C_arr)
    logic [31:0] acc [0:8];
    generate
        for (unp = 0; unp < 9; unp++) begin : pack_out
            assign C[unp*32 +: 32] = acc[unp];
        end
    endgenerate

    // Counters: k=outer(0..2), i=middle(0..2), j=inner(0..2)
    logic [1:0] k_cnt, i_cnt, j_cnt;

    // Index wires — 4-bit to avoid overflow (max = 2*3+2 = 8)
    logic [3:0] a_idx, b_idx, acc_idx;
    assign a_idx   = {2'b0, i_cnt} * 4'd3 + {2'b0, k_cnt};
    assign b_idx   = {2'b0, k_cnt} * 4'd3 + {2'b0, j_cnt};
    assign acc_idx = {2'b0, i_cnt} * 4'd3 + {2'b0, j_cnt};

    // 1 shared multiplier + 1 shared adder; reads directly from input wires
    logic [31:0] gmul_out, gadd_out;
    f32_mul u_gmul (.a(A_in[a_idx]), .b(B_in[b_idx]), .result(gmul_out));
    f32_add u_gadd (.a(acc[acc_idx]), .b(gmul_out),   .result(gadd_out));

    // All-done flag for COMPUTE state
    wire g_compute_done = (k_cnt == 2'd2) && (i_cnt == 2'd2) && (j_cnt == 2'd2);

    // FSM next-state
    always_comb begin
        gstate_nxt = gstate;
        case (gstate)
            IDLE:    if (start)          gstate_nxt = LOAD;
            LOAD:                        gstate_nxt = COMPUTE;
            COMPUTE: if (g_compute_done) gstate_nxt = FINISH;
            FINISH:                      gstate_nxt = IDLE;
            default:                     gstate_nxt = IDLE;
        endcase
    end

    integer gi;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gstate <= IDLE;
            k_cnt  <= 2'd0; i_cnt <= 2'd0; j_cnt <= 2'd0;
            for (gi = 0; gi < 9; gi++) acc[gi] <= 32'h0;
        end else begin
            gstate <= gstate_nxt;

            case (gstate)
                LOAD: begin
                    // Zero accumulators; inputs read directly in COMPUTE
                    for (gi = 0; gi < 9; gi++) acc[gi] <= 32'h0;
                    k_cnt <= 2'd0; i_cnt <= 2'd0; j_cnt <= 2'd0;
                end

                COMPUTE: begin
                    acc[acc_idx] <= gadd_out;

                    // Advance counters: j fastest, then i, then k
                    if (j_cnt == 2'd2) begin
                        j_cnt <= 2'd0;
                        if (i_cnt == 2'd2) begin
                            i_cnt <= 2'd0;
                            if (k_cnt != 2'd2)
                                k_cnt <= k_cnt + 2'd1;
                        end else begin
                            i_cnt <= i_cnt + 2'd1;
                        end
                    end else begin
                        j_cnt <= j_cnt + 2'd1;
                    end
                end

                // FINISH: acc holds final result; C is driven combinationally
                // from acc so no copy needed. Stays stable until next LOAD.
                FINISH: ;

                default: ;
            endcase
        end
    end

    assign done = (gstate == FINISH);
    assign busy = (gstate != IDLE) && (gstate != FINISH);

endmodule

// -----------------------------------------------------------------------------
// kalman_update — Kalman filter update kernel, time-multiplexed (F32, iter2)
// H = [1, 0, 0] (scalar measurement)
//
// vs iter1: 7 parallel f32_add → 1 shared u_shared_add.
//   IKH, P_out_arr, gs_A, gs_B registers eliminated:
//     IKH     → one_minus_k0_r (single reg) + combinational wires
//     P_out   → driven directly from gs_C_flat (gemm output)
//     gs_A_flat → combinational from K_reg + one_minus_k0_r
//     gs_B_flat → combinational from P_reg
//
// FSM (sub-cycles in parentheses):
//   IDLE→INNOV(1)→S_COMP(2)→NR0(2)→NR1(2)→NR2(2)→NR3(2)
//   →K_COMP(3)→KY_COMP(3)→X_ADD(4)→P_UPD(1)→WAIT_P→DONE_S→IDLE
//
// NR sub=1: shared_add(F32_TWO,-sx_reg) chains into shared_mul(nr_x,add_out)
//           within one clock cycle (both combinational, latched at posedge).
//
// Ports x_in, P_in, x_out, P_out are flat packed for Yosys.
// -----------------------------------------------------------------------------
module kalman_update (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [31:0]  z,
    input  logic [95:0]  x_in,
    input  logic [287:0] P_in,
    input  logic [31:0]  r_val,
    output logic [95:0]  x_out,
    output logic [287:0] P_out,
    output logic        done,
    output logic        busy
);

    localparam logic [31:0] F32_TWO = 32'h4000_0000;
    localparam logic [31:0] F32_ONE = 32'h3F80_0000;

    // FSM states
    typedef enum logic [3:0] {
        IDLE    = 4'd0,
        INNOV   = 4'd1,   // latch z, x_in, P_in
        S_COMP  = 4'd2,   // 2 sub-cycles: y_tilde then S_reg+nr_x seed
        NR0     = 4'd3,   // NR iteration 0 (2 sub-cycles)
        NR1     = 4'd4,   // NR iteration 1
        NR2     = 4'd5,   // NR iteration 2
        NR3     = 4'd11,  // NR iteration 3 (final)
        K_COMP  = 4'd6,   // K[i] = P_in[i*3] * S_inv (3 sub-cycles)
        KY_COMP = 4'd7,   // ky[i] = K[i] * y_tilde (3 sub-cycles)
        X_ADD   = 4'd12,  // 4 sub-cycles: x_out[0..2] then one_minus_k0_r
        P_UPD   = 4'd8,   // Fire gemm_systolic (gs_A_flat/gs_B_flat ready)
        WAIT_P  = 4'd9,   // Wait for gemm done
        DONE_S  = 4'd10
    } state_t;

    state_t state, state_nxt;
    logic [1:0] sub_cnt, sub_nxt;

    // Unpack flat input ports
    logic [31:0] x_in_arr [0:2];
    logic [31:0] P_in_arr [0:8];

    genvar ku;
    generate
        for (ku = 0; ku < 3; ku++) begin : unpack_xin
            assign x_in_arr[ku] = x_in[ku*32 +: 32];
        end
        for (ku = 0; ku < 9; ku++) begin : unpack_pin
            assign P_in_arr[ku] = P_in[ku*32 +: 32];
        end
    endgenerate

    // x_out packed from x_out_arr; P_out driven directly from gemm C output
    logic [31:0] x_out_arr [0:2];
    generate
        for (ku = 0; ku < 3; ku++) begin : pack_xout
            assign x_out[ku*32 +: 32] = x_out_arr[ku];
        end
    endgenerate

    // Registered intermediate values
    logic [31:0] z_reg, y_tilde, S_reg, S_inv;
    logic [31:0] x_reg  [0:2];
    logic [31:0] P_reg  [0:8];
    logic [31:0] K_reg  [0:2];
    logic [31:0] sx_reg;         // intermediate S*nr_x for NR
    logic [31:0] ky_arr [0:2];   // K[i]*y_tilde
    logic [31:0] nr_x;
    logic [31:0] one_minus_k0_r; // 1 - K[0], computed in X_ADD sub=3

    // gemm_systolic signals
    logic        gs_start, gs_done, gs_busy;
    logic [287:0] gs_A_flat, gs_B_flat, gs_C_flat;

    // gs_A_flat: IKH matrix, combinationally from K_reg and one_minus_k0_r
    // IKH = (I - K*H) where H=[1,0,0]:
    //   row0: [1-K[0], 0,      0     ]
    //   row1: [-K[1],  1,      0     ]
    //   row2: [-K[2],  0,      1     ]
    // Stored column-major (element [i,j] = row i, col j → index i*3+j):
    assign gs_A_flat[0*32 +: 32] = one_minus_k0_r;
    assign gs_A_flat[1*32 +: 32] = 32'h0;
    assign gs_A_flat[2*32 +: 32] = 32'h0;
    assign gs_A_flat[3*32 +: 32] = {~K_reg[1][31], K_reg[1][30:0]};  // -K[1]
    assign gs_A_flat[4*32 +: 32] = F32_ONE;
    assign gs_A_flat[5*32 +: 32] = 32'h0;
    assign gs_A_flat[6*32 +: 32] = {~K_reg[2][31], K_reg[2][30:0]};  // -K[2]
    assign gs_A_flat[7*32 +: 32] = 32'h0;
    assign gs_A_flat[8*32 +: 32] = F32_ONE;

    // gs_B_flat: P_reg directly (stable during WAIT_P)
    genvar gk;
    generate
        for (gk = 0; gk < 9; gk++) begin : gs_b_flat
            assign gs_B_flat[gk*32 +: 32] = P_reg[gk];
        end
    endgenerate

    // P_out driven directly from gemm C output (stable after FINISH until next LOAD)
    assign P_out = gs_C_flat;

    gemm_systolic u_gemm (
        .clk   (clk),
        .rst_n (rst_n),
        .start (gs_start),
        .A     (gs_A_flat),
        .B     (gs_B_flat),
        .C     (gs_C_flat),
        .done  (gs_done),
        .busy  (gs_busy)
    );

    // -------------------------------------------------------------------------
    // Shared multiplier: 1 f32_mul, muxed inputs
    // -------------------------------------------------------------------------
    logic [31:0] shared_a, shared_b, shared_r;
    f32_mul u_shared_mul (.a(shared_a), .b(shared_b), .result(shared_r));

    // -------------------------------------------------------------------------
    // Shared adder: 1 f32_add, muxed inputs
    // In NR sub=1: output feeds shared_mul b-input (combinational chain).
    // -------------------------------------------------------------------------
    logic [31:0] shared_add_a, shared_add_b, shared_add_r;
    f32_add u_shared_add (.a(shared_add_a), .b(shared_add_b), .result(shared_add_r));

    // Newton-Raphson seed: bit-trick 1/S approximation (F32)
    function automatic logic [31:0] nr_seed (input logic [31:0] s);
        nr_seed = 32'h7EF1_27EA - s;
    endfunction

    // -------------------------------------------------------------------------
    // Shared adder mux
    // -------------------------------------------------------------------------
    always_comb begin
        shared_add_a = 32'h0;
        shared_add_b = 32'h0;
        case (state)
            S_COMP: begin
                if (sub_cnt == 2'd0) begin
                    // sub 0: y_tilde = z_reg - x_reg[0]
                    shared_add_a = z_reg;
                    shared_add_b = {~x_reg[0][31], x_reg[0][30:0]};
                end else begin
                    // sub 1: S_comb = P_reg[0] + r_val
                    shared_add_a = P_reg[0];
                    shared_add_b = r_val;
                end
            end
            NR0, NR1, NR2, NR3: begin
                if (sub_cnt == 2'd1) begin
                    // sub 1: two_minus_sx = F32_TWO - sx_reg
                    // (feeds shared_mul b-input in same cycle)
                    shared_add_a = F32_TWO;
                    shared_add_b = {~sx_reg[31], sx_reg[30:0]};
                end
                // sub 0: adder idle
            end
            X_ADD: begin
                case (sub_cnt)
                    2'd0: begin
                        shared_add_a = x_reg[0];
                        shared_add_b = ky_arr[0];
                    end
                    2'd1: begin
                        shared_add_a = x_reg[1];
                        shared_add_b = ky_arr[1];
                    end
                    2'd2: begin
                        shared_add_a = x_reg[2];
                        shared_add_b = ky_arr[2];
                    end
                    2'd3: begin
                        // one_minus_k0 = F32_ONE - K_reg[0]
                        shared_add_a = F32_ONE;
                        shared_add_b = {~K_reg[0][31], K_reg[0][30:0]};
                    end
                endcase
            end
            default: ;
        endcase
    end

    // -------------------------------------------------------------------------
    // Shared multiplier mux
    // NR sub=1: b = shared_add_r (two_minus_sx, combinational chain)
    // -------------------------------------------------------------------------
    always_comb begin
        shared_a = 32'h0;
        shared_b = 32'h0;
        case (state)
            NR0, NR1, NR2, NR3: begin
                if (sub_cnt == 2'd0) begin
                    shared_a = S_reg;   // sub 0: S_reg * nr_x → sx_reg
                    shared_b = nr_x;
                end else begin
                    shared_a = nr_x;          // sub 1: nr_x * (2-sx) → new nr_x
                    shared_b = shared_add_r;  // two_minus_sx (combinational)
                end
            end
            K_COMP: begin
                shared_b = S_inv;
                case (sub_cnt)
                    2'd0: shared_a = P_reg[0];
                    2'd1: shared_a = P_reg[3];
                    2'd2: shared_a = P_reg[6];
                    default: shared_a = 32'h0;
                endcase
            end
            KY_COMP: begin
                shared_b = y_tilde;
                case (sub_cnt)
                    2'd0: shared_a = K_reg[0];
                    2'd1: shared_a = K_reg[1];
                    2'd2: shared_a = K_reg[2];
                    default: shared_a = 32'h0;
                endcase
            end
            default: begin
                shared_a = 32'h0;
                shared_b = 32'h0;
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // FSM next-state + sub_nxt
    // -------------------------------------------------------------------------
    always_comb begin
        state_nxt = state;
        sub_nxt   = sub_cnt + 2'd1;  // default: advance sub counter

        case (state)
            IDLE:    begin
                if (start) begin state_nxt = INNOV;  sub_nxt = 2'd0; end
                else             sub_nxt = 2'd0;
            end
            INNOV:   begin state_nxt = S_COMP;  sub_nxt = 2'd0; end
            S_COMP:  if (sub_cnt == 2'd1) begin state_nxt = NR0;     sub_nxt = 2'd0; end
            NR0:     if (sub_cnt == 2'd1) begin state_nxt = NR1;     sub_nxt = 2'd0; end
            NR1:     if (sub_cnt == 2'd1) begin state_nxt = NR2;     sub_nxt = 2'd0; end
            NR2:     if (sub_cnt == 2'd1) begin state_nxt = NR3;     sub_nxt = 2'd0; end
            NR3:     if (sub_cnt == 2'd1) begin state_nxt = K_COMP;  sub_nxt = 2'd0; end
            K_COMP:  if (sub_cnt == 2'd2) begin state_nxt = KY_COMP; sub_nxt = 2'd0; end
            KY_COMP: if (sub_cnt == 2'd2) begin state_nxt = X_ADD;   sub_nxt = 2'd0; end
            X_ADD:   if (sub_cnt == 2'd3) begin state_nxt = P_UPD;   sub_nxt = 2'd0; end
            P_UPD:   begin state_nxt = WAIT_P;  sub_nxt = 2'd0; end
            WAIT_P:  begin
                if (gs_done) begin state_nxt = DONE_S; sub_nxt = 2'd0; end
                else              sub_nxt = 2'd0;
            end
            DONE_S:  begin state_nxt = IDLE;    sub_nxt = 2'd0; end
            default: begin state_nxt = IDLE;    sub_nxt = 2'd0; end
        endcase
    end

    // -------------------------------------------------------------------------
    // FSM datapath
    // -------------------------------------------------------------------------
    integer ii;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= IDLE;
            sub_cnt  <= 2'd0;
            gs_start <= 1'b0;
            for (ii = 0; ii < 9; ii++) P_reg[ii] <= 32'h0;
            for (ii = 0; ii < 3; ii++) begin
                x_reg[ii]     <= 32'h0;
                K_reg[ii]     <= 32'h0;
                ky_arr[ii]    <= 32'h0;
                x_out_arr[ii] <= 32'h0;
            end
            z_reg          <= 32'h0;
            S_reg          <= 32'h0;
            S_inv          <= 32'h0;
            nr_x           <= 32'h0;
            sx_reg         <= 32'h0;
            y_tilde        <= 32'h0;
            one_minus_k0_r <= 32'h0;
        end else begin
            state   <= state_nxt;
            sub_cnt <= sub_nxt;
            gs_start <= 1'b0;

            case (state)
                INNOV: begin
                    z_reg <= z;
                    for (ii = 0; ii < 3; ii++) x_reg[ii] <= x_in_arr[ii];
                    for (ii = 0; ii < 9; ii++) P_reg[ii] <= P_in_arr[ii];
                end

                // S_COMP: 2 sub-cycles using shared adder
                // sub 0: y_tilde = z_reg - x_reg[0]
                // sub 1: S_reg = P_reg[0] + r_val; nr_x = nr_seed(S_reg)
                S_COMP: begin
                    if (sub_cnt == 2'd0) begin
                        y_tilde <= shared_add_r;
                    end else begin  // sub=1
                        S_reg <= shared_add_r;
                        nr_x  <= nr_seed(shared_add_r);
                    end
                end

                // NR0..NR3: 2 sub-cycles each
                // sub 0: sx_reg = S_reg * nr_x  (mul)
                // sub 1: nr_x = nr_x * (2 - sx_reg)  (add chains into mul)
                NR0, NR1, NR2: begin
                    if (sub_cnt == 2'd0) begin
                        sx_reg <= shared_r;
                    end else begin
                        nr_x  <= shared_r;
                        S_inv <= shared_r;
                    end
                end

                NR3: begin
                    if (sub_cnt == 2'd0) begin
                        sx_reg <= shared_r;
                    end else begin
                        S_inv <= shared_r;
                    end
                end

                // K_COMP: 3 sub-cycles
                K_COMP: begin
                    case (sub_cnt)
                        2'd0: K_reg[0] <= shared_r;
                        2'd1: K_reg[1] <= shared_r;
                        2'd2: K_reg[2] <= shared_r;
                        default: ;
                    endcase
                end

                // KY_COMP: 3 sub-cycles
                KY_COMP: begin
                    case (sub_cnt)
                        2'd0: ky_arr[0] <= shared_r;
                        2'd1: ky_arr[1] <= shared_r;
                        2'd2: ky_arr[2] <= shared_r;
                        default: ;
                    endcase
                end

                // X_ADD: 4 sub-cycles using shared adder
                // sub 0-2: x_out_arr[i] = x_reg[i] + ky_arr[i]
                // sub 3:   one_minus_k0_r = 1.0 - K_reg[0]
                //          (gs_A_flat[0] is driven from one_minus_k0_r combinationally)
                X_ADD: begin
                    case (sub_cnt)
                        2'd0: x_out_arr[0] <= shared_add_r;
                        2'd1: x_out_arr[1] <= shared_add_r;
                        2'd2: x_out_arr[2] <= shared_add_r;
                        2'd3: one_minus_k0_r <= shared_add_r;
                    endcase
                end

                // P_UPD: gs_A_flat and gs_B_flat are already driven combinationally.
                // Just assert gs_start (gemm will latch A and B in its LOAD state).
                P_UPD: begin
                    gs_start <= 1'b1;
                end

                WAIT_P: ;  // wait for gs_done

                // DONE_S: P_out is already driven from gs_C_flat (acc is stable).
                DONE_S: ;

                default: ;
            endcase
        end
    end

    assign done = (state == DONE_S);
    assign busy = (state != IDLE) && (state != DONE_S);

endmodule
