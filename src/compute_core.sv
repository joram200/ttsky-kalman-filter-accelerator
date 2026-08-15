`timescale 1ns/1ps
// =============================================================================
// compute_core.sv — Time-multiplexed F64 compute core for tt_um_joram200
// Option B: exact IEEE-754 F64 arithmetic, 1 shared f64_mul per module.
// Module hierarchy (dependency order):
//   f64_mul       — Combinational IEEE-754 F64 multiplier (unchanged)
//   f64_add       — Combinational IEEE-754 F64 adder/subtractor (unchanged)
//   gemm_systolic — 3×3 GEMM, 1 shared f64_mul+add, 27-cycle COMPUTE state
//   kalman_update — Kalman update, 1 shared f64_mul, sub-cycle FSM expansion
//
// Area reduction vs baseline:
//   gemm_systolic: 9 f64_mul → 1 shared (COMPUTE runs 27 cycles)
//   kalman_update direct: 9 f64_mul → 1 shared (sub-cycle sequencing)
//   Total f64_mul: 18 → 2 (one per module)
//   Total f64_add: 17 → 8 (7 kept in kalman_update + 1 in gemm)
// =============================================================================

// -----------------------------------------------------------------------------
// f64_mul — Combinational IEEE-754 double-precision multiplier (unchanged)
// -----------------------------------------------------------------------------
module f64_mul (
    input  logic [63:0] a,
    input  logic [63:0] b,
    output logic [63:0] result
);

    logic        a_sign, b_sign, r_sign;
    logic [10:0] a_exp,  b_exp;
    logic [52:0] a_man,  b_man;

    assign a_sign = a[63];
    assign b_sign = b[63];
    assign a_exp  = a[62:52];
    assign b_exp  = b[62:52];
    assign a_man  = (a_exp == 11'h0) ? {1'b0, a[51:0]} : {1'b1, a[51:0]};
    assign b_man  = (b_exp == 11'h0) ? {1'b0, b[51:0]} : {1'b1, b[51:0]};

    logic a_nan, b_nan, a_inf, b_inf, a_zero, b_zero;
    assign a_nan  = (a_exp == 11'h7FF) && (a[51:0] != 52'h0);
    assign b_nan  = (b_exp == 11'h7FF) && (b[51:0] != 52'h0);
    assign a_inf  = (a_exp == 11'h7FF) && (a[51:0] == 52'h0);
    assign b_inf  = (b_exp == 11'h7FF) && (b[51:0] == 52'h0);
    assign a_zero = (a_exp == 11'h0)   && (a[51:0] == 52'h0);
    assign b_zero = (b_exp == 11'h0)   && (b[51:0] == 52'h0);

    logic [105:0] product;
    assign product = {53'b0, a_man} * {53'b0, b_man};

    logic signed [12:0] exp_sum;
    assign exp_sum = $signed({2'b00, a_exp}) + $signed({2'b00, b_exp})
                     - $signed(13'd1023);

    logic [51:0] norm_man;
    logic signed [12:0] norm_exp;
    assign norm_man = product[105] ? product[104:53] : product[103:52];
    assign norm_exp = product[105] ? (exp_sum + 13'd1) : exp_sum;

    assign r_sign = a_sign ^ b_sign;

    assign result =
        (a_nan || b_nan)                              ? 64'h7FF8_0000_0000_0000 :
        ((a_inf && b_zero) || (b_inf && a_zero))      ? 64'h7FF8_0000_0000_0000 :
        (a_inf || b_inf)                              ? {r_sign, 11'h7FF, 52'h0} :
        (a_zero || b_zero)                            ? {r_sign, 63'b0}          :
        ($signed(norm_exp) >= $signed(13'd2047))      ? {r_sign, 11'h7FF, 52'h0} :
        ($signed(norm_exp) <= $signed(13'd0))         ? {r_sign, 63'b0}          :
                                                        {r_sign, norm_exp[10:0], norm_man};

endmodule

// -----------------------------------------------------------------------------
// f64_add — Combinational IEEE-754 double-precision adder/subtractor (unchanged)
// -----------------------------------------------------------------------------
module f64_add (
    input  logic [63:0] a,
    input  logic [63:0] b,
    output logic [63:0] result
);

    logic        a_sign, b_sign;
    logic [10:0] a_exp,  b_exp;
    logic [51:0] a_frac, b_frac;

    assign a_sign = a[63];
    assign b_sign = b[63];
    assign a_exp  = a[62:52];
    assign b_exp  = b[62:52];
    assign a_frac = a[51:0];
    assign b_frac = b[51:0];

    logic a_nan, b_nan, a_inf, b_inf, a_zero, b_zero;
    assign a_nan  = (a_exp == 11'h7FF) && (a_frac != 52'h0);
    assign b_nan  = (b_exp == 11'h7FF) && (b_frac != 52'h0);
    assign a_inf  = (a_exp == 11'h7FF) && (a_frac == 52'h0);
    assign b_inf  = (b_exp == 11'h7FF) && (b_frac == 52'h0);
    assign a_zero = (a_exp == 11'h0)   && (a_frac == 52'h0);
    assign b_zero = (b_exp == 11'h0)   && (b_frac == 52'h0);

    logic        big_sign, sml_sign;
    logic [10:0] big_exp,  sml_exp;
    logic [52:0] big_man,  sml_man;
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
            big_man  = (b_exp == 11'h0) ? {1'b0, b_frac} : {1'b1, b_frac};
            sml_sign = a_sign; sml_exp = a_exp;
            sml_man  = (a_exp == 11'h0) ? {1'b0, a_frac} : {1'b1, a_frac};
        end else begin
            big_sign = a_sign; big_exp = a_exp;
            big_man  = (a_exp == 11'h0) ? {1'b0, a_frac} : {1'b1, a_frac};
            sml_sign = b_sign; sml_exp = b_exp;
            sml_man  = (b_exp == 11'h0) ? {1'b0, b_frac} : {1'b1, b_frac};
        end
    end

    logic [5:0]  shift_amt;
    logic [52:0] sml_aligned;

    assign shift_amt   = ((big_exp - sml_exp) > 11'd63) ? 6'd63 : big_exp[5:0] - sml_exp[5:0];
    assign sml_aligned = sml_man >> shift_amt;

    logic [53:0] sum_raw;
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

    logic [11:0]  res_exp;
    logic [51:0]  res_man;
    logic [52:0]  shifted;

    logic [5:0] lz_count;
    always_comb begin : lz_encoder
        lz_count = 6'd53;
        for (int i = 0; i <= 52; i++) begin
            if (sum_raw[i]) lz_count = 6'(52 - i);
        end
    end

    assign shifted  = sum_raw[52:0] << lz_count;
    assign res_exp  = sum_raw[53] ? ({1'b0, big_exp} + 12'd1)            :
                      sum_raw[52] ? {1'b0, big_exp}                        :
                                    ({1'b0, big_exp} - {6'b0, lz_count});
    assign res_man  = sum_raw[53] ? sum_raw[52:1]  :
                      sum_raw[52] ? sum_raw[51:0]  :
                                    shifted[51:0];

    assign result =
        (a_nan || b_nan)                          ? 64'h7FF8_0000_0000_0000 :
        (a_inf && b_inf && (a_sign != b_sign))    ? 64'h7FF8_0000_0000_0000 :
        (a_inf || b_inf)                          ? (a_inf ? a : b)         :
        (a_zero && b_zero)                        ? 64'h0                   :
        a_zero                                    ? b                       :
        b_zero                                    ? a                       :
        (sum_raw == 54'h0)                        ? 64'h0                   :
        (res_exp[11] || (res_exp == 12'h0))       ? {r_sign, 63'b0}         :
        (res_exp >= 12'h7FF)                      ? {r_sign, 11'h7FF, 52'h0}:
                                                    {r_sign, res_exp[10:0], res_man};

endmodule

// -----------------------------------------------------------------------------
// gemm_systolic — 3×3 matrix multiply C=A×B, time-multiplexed (Option B)
// 1 shared f64_mul + 1 shared f64_add; COMPUTE runs 27 cycles.
// Counter order: k (outer, 0→2), i (middle, 0→2), j (inner, 0→2).
// Each COMPUTE cycle: acc[i*3+j] += A_reg[i*3+k] × B_reg[k*3+j]
// FSM: IDLE → LOAD → COMPUTE(27 cy) → FINISH → IDLE
// Ports A, B, C are flat packed vectors (9×64=576 bits) for Yosys compatibility.
// -----------------------------------------------------------------------------
module gemm_systolic (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [575:0] A,
    input  logic [575:0] B,
    output logic [575:0] C,
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

    // Unpack flat inputs to internal unpacked arrays
    logic [63:0] A_in [0:8];
    logic [63:0] B_in [0:8];
    genvar unp;
    generate
        for (unp = 0; unp < 9; unp++) begin : unpack_in
            assign A_in[unp] = A[unp*64 +: 64];
            assign B_in[unp] = B[unp*64 +: 64];
        end
    endgenerate

    logic [63:0] C_arr [0:8];
    generate
        for (unp = 0; unp < 9; unp++) begin : pack_out
            assign C[unp*64 +: 64] = C_arr[unp];
        end
    endgenerate

    logic [63:0] A_reg [0:8];
    logic [63:0] B_reg [0:8];
    logic [63:0] acc   [0:8];

    // Counters: k=outer(0..2), i=middle(0..2), j=inner(0..2)
    logic [1:0] k_cnt, i_cnt, j_cnt;

    // Index wires — 4-bit to avoid overflow (max = 2*3+2 = 8)
    logic [3:0] a_idx, b_idx, acc_idx;
    assign a_idx   = {2'b0, i_cnt} * 4'd3 + {2'b0, k_cnt};
    assign b_idx   = {2'b0, k_cnt} * 4'd3 + {2'b0, j_cnt};
    assign acc_idx = {2'b0, i_cnt} * 4'd3 + {2'b0, j_cnt};

    // 1 shared multiplier + 1 shared adder
    logic [63:0] gmul_out, gadd_out;
    f64_mul u_gmul (.a(A_reg[a_idx]), .b(B_reg[b_idx]), .result(gmul_out));
    f64_add u_gadd (.a(acc[acc_idx]), .b(gmul_out),     .result(gadd_out));

    // All-done flag for COMPUTE state
    wire g_compute_done = (k_cnt == 2'd2) && (i_cnt == 2'd2) && (j_cnt == 2'd2);

    // FSM next-state
    always_comb begin
        gstate_nxt = gstate;
        case (gstate)
            IDLE:    if (start)         gstate_nxt = LOAD;
            LOAD:                       gstate_nxt = COMPUTE;
            COMPUTE: if (g_compute_done) gstate_nxt = FINISH;
            FINISH:                     gstate_nxt = IDLE;
            default:                    gstate_nxt = IDLE;
        endcase
    end

    integer gi;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gstate <= IDLE;
            k_cnt  <= 2'd0; i_cnt <= 2'd0; j_cnt <= 2'd0;
            for (gi = 0; gi < 9; gi++) begin
                A_reg[gi] <= 64'h0;
                B_reg[gi] <= 64'h0;
                acc[gi]   <= 64'h0;
                C_arr[gi] <= 64'h0;
            end
        end else begin
            gstate <= gstate_nxt;

            case (gstate)
                LOAD: begin
                    for (gi = 0; gi < 9; gi++) begin
                        A_reg[gi] <= A_in[gi];
                        B_reg[gi] <= B_in[gi];
                        acc[gi]   <= 64'h0;
                    end
                    k_cnt <= 2'd0; i_cnt <= 2'd0; j_cnt <= 2'd0;
                end

                COMPUTE: begin
                    // Accumulate partial product into the correct output element
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

                FINISH: begin
                    for (gi = 0; gi < 9; gi++) C_arr[gi] <= acc[gi];
                end

                default: ;
            endcase
        end
    end

    assign done = (gstate == FINISH);
    assign busy = (gstate != IDLE) && (gstate != FINISH);

endmodule

// -----------------------------------------------------------------------------
// kalman_update — Kalman filter update kernel, time-multiplexed (Option B)
// H = [1, 0, 0] (scalar measurement)
//
// Area reduction: 9 direct f64_mul → 1 shared u_shared_mul; sub-cycle FSM.
// 7 f64_add instances retained: u_nr_sub, u_xo0/1/2, u_1mk0, u_ytilde, u_scomb.
//
// FSM (sub-cycles in parentheses):
//   IDLE→INNOV(1)→S_COMP(1)→NR0(2)→NR1(2)→NR2(2)→NR3(2)
//   →K_COMP(3)→KY_COMP(3)→X_ADD(1)→P_UPD(1)→WAIT_P→DONE_S→IDLE
//
// Ports x_in, P_in, x_out, P_out are flat packed vectors for Yosys.
// -----------------------------------------------------------------------------
module kalman_update (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [63:0] z,
    input  logic [191:0] x_in,
    input  logic [575:0] P_in,
    input  logic [63:0] r_val,
    output logic [191:0] x_out,
    output logic [575:0] P_out,
    output logic        done,
    output logic        busy
);

    localparam logic [63:0] F64_TWO = 64'h4000_0000_0000_0000;
    localparam logic [63:0] F64_ONE = 64'h3FF0_0000_0000_0000;

    // FSM states
    typedef enum logic [3:0] {
        IDLE    = 4'd0,
        INNOV   = 4'd1,   // latch z, x_in, P_in
        S_COMP  = 4'd2,   // latch nr_x seed, y_tilde, S_reg
        NR0     = 4'd3,   // NR iteration 0 (2 sub-cycles)
        NR1     = 4'd4,   // NR iteration 1
        NR2     = 4'd5,   // NR iteration 2
        NR3     = 4'd11,  // NR iteration 3 (final)
        K_COMP  = 4'd6,   // K[i] = P_in[i*3] * S_inv (3 sub-cycles)
        KY_COMP = 4'd7,   // ky[i] = K[i] * y_tilde (3 sub-cycles, was X_CORR)
        X_ADD   = 4'd12,  // x_out[i] = x_reg[i] + ky[i], build IKH (1 cycle)
        P_UPD   = 4'd8,   // Fire gemm_systolic(IKH, P_in)
        WAIT_P  = 4'd9,   // Wait for gemm done
        DONE_S  = 4'd10
    } state_t;

    state_t state, state_nxt;

    // Sub-cycle counter (max 2 for NR and KY_COMP; max 2 for K_COMP)
    logic [1:0] sub_cnt, sub_nxt;

    // Unpack flat input ports
    logic [63:0] x_in_arr [0:2];
    logic [63:0] P_in_arr [0:8];

    genvar ku;
    generate
        for (ku = 0; ku < 3; ku++) begin : unpack_xin
            assign x_in_arr[ku] = x_in[ku*64 +: 64];
        end
        for (ku = 0; ku < 9; ku++) begin : unpack_pin
            assign P_in_arr[ku] = P_in[ku*64 +: 64];
        end
    endgenerate

    // Internal unpacked output arrays; pack to flat ports
    logic [63:0] x_out_arr [0:2];
    logic [63:0] P_out_arr [0:8];

    generate
        for (ku = 0; ku < 3; ku++) begin : pack_xout
            assign x_out[ku*64 +: 64] = x_out_arr[ku];
        end
        for (ku = 0; ku < 9; ku++) begin : pack_pout
            assign P_out[ku*64 +: 64] = P_out_arr[ku];
        end
    endgenerate

    // Registered intermediate values
    logic [63:0] z_reg, y_tilde, S_reg, S_inv;
    logic [63:0] x_reg  [0:2];
    logic [63:0] P_reg  [0:8];
    logic [63:0] K_reg  [0:2];
    logic [63:0] IKH    [0:8];
    logic [63:0] sx_reg;         // intermediate S*nr_x for NR iterations
    logic [63:0] ky_arr [0:2];   // intermediate K[i]*y_tilde

    // NR intermediate
    logic [63:0] nr_x;

    // gemm_systolic signals
    logic        gs_start, gs_done, gs_busy;
    logic [63:0] gs_A [0:8], gs_B [0:8], gs_C [0:8];

    logic [575:0] gs_A_flat, gs_B_flat, gs_C_flat;

    genvar gk;
    generate
        for (gk = 0; gk < 9; gk++) begin : gs_flat
            assign gs_A_flat[gk*64 +: 64] = gs_A[gk];
            assign gs_B_flat[gk*64 +: 64] = gs_B[gk];
            assign gs_C[gk] = gs_C_flat[gk*64 +: 64];
        end
    endgenerate

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
    // Shared multiplier: 1 f64_mul reused across NR, K_COMP, KY_COMP states.
    // Inputs muxed by always_comb below; result = shared_r.
    // -------------------------------------------------------------------------
    logic [63:0] shared_a, shared_b, shared_r;
    f64_mul u_shared_mul (.a(shared_a), .b(shared_b), .result(shared_r));

    // Negate helper functions (F64: flip sign bit)
    function automatic logic [63:0] f64_neg (input logic [63:0] x);
        f64_neg = {~x[63], x[62:0]};
    endfunction

    // Newton-Raphson seed: bit-trick 1/S approximation (F64)
    function automatic logic [63:0] nr_seed (input logic [63:0] s);
        nr_seed = 64'h7FDE_6000_0000_0000 - s;
    endfunction

    // Pre-computed negations as continuous wires
    wire [63:0] neg_sx_reg = {~sx_reg[63], sx_reg[62:0]};  // -sx_reg for u_nr_sub
    wire [63:0] neg_k1     = {~K_reg[1][63], K_reg[1][62:0]};
    wire [63:0] neg_k2     = {~K_reg[2][63], K_reg[2][62:0]};

    // -------------------------------------------------------------------------
    // f64_add instances (7 total) — kept parallel, no time-multiplex needed.
    // -------------------------------------------------------------------------

    // NR: two_minus_sx = 2.0 - sx_reg (replaces inline -sx wire from u_nr_mul1)
    logic [63:0] two_minus_sx;
    f64_add u_nr_sub  (.a(F64_TWO), .b(neg_sx_reg),          .result(two_minus_sx));

    // x_out[i] = x_reg[i] + ky_arr[i]
    logic [63:0] xout0, xout1, xout2;
    f64_add u_xo0 (.a(x_reg[0]), .b(ky_arr[0]), .result(xout0));
    f64_add u_xo1 (.a(x_reg[1]), .b(ky_arr[1]), .result(xout1));
    f64_add u_xo2 (.a(x_reg[2]), .b(ky_arr[2]), .result(xout2));

    // IKH[0,0] = 1 - K[0]
    logic [63:0] one_minus_k0;
    f64_add u_1mk0 (.a(F64_ONE), .b(f64_neg(K_reg[0])), .result(one_minus_k0));

    // Combinational y_tilde and S_comb
    logic [63:0] y_tilde_comb, S_comb;
    f64_add u_ytilde (.a(z_reg),    .b(f64_neg(x_reg[0])), .result(y_tilde_comb));
    f64_add u_scomb  (.a(P_reg[0]), .b(r_val),              .result(S_comb));

    // -------------------------------------------------------------------------
    // Shared multiplier mux (always_comb — no constant bit-selects)
    // -------------------------------------------------------------------------
    always_comb begin
        shared_a = 64'h0;
        shared_b = 64'h0;
        case (state)
            NR0, NR1, NR2, NR3: begin
                if (sub_cnt == 2'd0) begin
                    shared_a = S_reg;  // sub 0: compute S * nr_x → sx_reg
                    shared_b = nr_x;
                end else begin
                    shared_a = nr_x;             // sub 1: nr_x * (2 - sx) → new nr_x
                    shared_b = two_minus_sx;
                end
            end
            K_COMP: begin
                shared_b = S_inv;
                case (sub_cnt)
                    2'd0: shared_a = P_reg[0];  // K[0] = P[0][0] * S_inv
                    2'd1: shared_a = P_reg[3];  // K[1] = P[1][0] * S_inv
                    2'd2: shared_a = P_reg[6];  // K[2] = P[2][0] * S_inv
                    default: shared_a = 64'h0;
                endcase
            end
            KY_COMP: begin
                shared_b = y_tilde;
                case (sub_cnt)
                    2'd0: shared_a = K_reg[0];
                    2'd1: shared_a = K_reg[1];
                    2'd2: shared_a = K_reg[2];
                    default: shared_a = 64'h0;
                endcase
            end
            default: begin
                shared_a = 64'h0;
                shared_b = 64'h0;
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
            S_COMP:  begin state_nxt = NR0;     sub_nxt = 2'd0; end
            NR0:     if (sub_cnt == 2'd1) begin state_nxt = NR1;    sub_nxt = 2'd0; end
            NR1:     if (sub_cnt == 2'd1) begin state_nxt = NR2;    sub_nxt = 2'd0; end
            NR2:     if (sub_cnt == 2'd1) begin state_nxt = NR3;    sub_nxt = 2'd0; end
            NR3:     if (sub_cnt == 2'd1) begin state_nxt = K_COMP; sub_nxt = 2'd0; end
            K_COMP:  if (sub_cnt == 2'd2) begin state_nxt = KY_COMP; sub_nxt = 2'd0; end
            KY_COMP: if (sub_cnt == 2'd2) begin state_nxt = X_ADD;  sub_nxt = 2'd0; end
            X_ADD:   begin state_nxt = P_UPD;   sub_nxt = 2'd0; end
            P_UPD:   begin state_nxt = WAIT_P;  sub_nxt = 2'd0; end
            WAIT_P:  begin
                if (gs_done) begin state_nxt = DONE_S; sub_nxt = 2'd0; end
                else              sub_nxt = 2'd0;  // hold counter at 0 while waiting
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
            for (ii = 0; ii < 9; ii++) begin
                gs_A[ii]  <= 64'h0;
                gs_B[ii]  <= 64'h0;
                IKH[ii]   <= 64'h0;
                P_reg[ii] <= 64'h0;
            end
            for (ii = 0; ii < 3; ii++) begin
                x_reg[ii]     <= 64'h0;
                K_reg[ii]     <= 64'h0;
                ky_arr[ii]    <= 64'h0;
                x_out_arr[ii] <= 64'h0;
            end
            for (ii = 0; ii < 9; ii++) P_out_arr[ii] <= 64'h0;
            z_reg   <= 64'h0;
            S_reg   <= 64'h0;
            S_inv   <= 64'h0;
            nr_x    <= 64'h0;
            sx_reg  <= 64'h0;
            y_tilde <= 64'h0;
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

                S_COMP: begin
                    // Latch S, seed, y_tilde using combinational outputs
                    nr_x    <= nr_seed(S_comb);
                    y_tilde <= y_tilde_comb;
                    S_reg   <= S_comb;
                end

                // NR0..NR3: 2 sub-cycles each
                // sub 0: shared_r = S_reg * nr_x → latch to sx_reg
                // sub 1: shared_r = nr_x * (2 - sx_reg) → new nr_x (and S_inv for last)
                NR0, NR1, NR2: begin
                    if (sub_cnt == 2'd0) begin
                        sx_reg <= shared_r;
                    end else begin  // sub_cnt == 1
                        nr_x  <= shared_r;
                        S_inv <= shared_r;
                    end
                end

                NR3: begin
                    if (sub_cnt == 2'd0) begin
                        sx_reg <= shared_r;
                    end else begin  // sub_cnt == 1: final iteration
                        S_inv <= shared_r;  // don't update nr_x (no more iterations)
                    end
                end

                // K_COMP: 3 sub-cycles
                // sub 0: K_reg[0] = P_reg[0] * S_inv
                // sub 1: K_reg[1] = P_reg[3] * S_inv
                // sub 2: K_reg[2] = P_reg[6] * S_inv
                K_COMP: begin
                    case (sub_cnt)
                        2'd0: K_reg[0] <= shared_r;
                        2'd1: K_reg[1] <= shared_r;
                        2'd2: K_reg[2] <= shared_r;
                        default: ;
                    endcase
                end

                // KY_COMP: 3 sub-cycles
                // sub 0: ky_arr[0] = K_reg[0] * y_tilde
                // sub 1: ky_arr[1] = K_reg[1] * y_tilde
                // sub 2: ky_arr[2] = K_reg[2] * y_tilde
                KY_COMP: begin
                    case (sub_cnt)
                        2'd0: ky_arr[0] <= shared_r;
                        2'd1: ky_arr[1] <= shared_r;
                        2'd2: ky_arr[2] <= shared_r;
                        default: ;
                    endcase
                end

                // X_ADD: 1 cycle — ky_arr[0:2] are now populated
                // x_out[i] = x_reg[i] + ky_arr[i] (via u_xo0/1/2 combinationally)
                // Also build IKH = (I - K⊗H): H=[1,0,0]
                X_ADD: begin
                    x_out_arr[0] <= xout0;
                    x_out_arr[1] <= xout1;
                    x_out_arr[2] <= xout2;
                    IKH[0] <= one_minus_k0;
                    IKH[1] <= 64'h0;
                    IKH[2] <= 64'h0;
                    IKH[3] <= neg_k1;
                    IKH[4] <= F64_ONE;
                    IKH[5] <= 64'h0;
                    IKH[6] <= neg_k2;
                    IKH[7] <= 64'h0;
                    IKH[8] <= F64_ONE;
                end

                P_UPD: begin
                    for (ii = 0; ii < 9; ii++) begin
                        gs_A[ii] <= IKH[ii];
                        gs_B[ii] <= P_reg[ii];
                    end
                    gs_start <= 1'b1;
                end

                WAIT_P: begin
                    gs_start <= 1'b0;
                end

                DONE_S: begin
                    for (ii = 0; ii < 9; ii++) P_out_arr[ii] <= gs_C[ii];
                end

                default: ;
            endcase
        end
    end

    assign done = (state == DONE_S);
    assign busy = (state != IDLE) && (state != DONE_S);

endmodule
