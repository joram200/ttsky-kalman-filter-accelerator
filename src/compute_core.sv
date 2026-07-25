`timescale 1ns/1ps
// =============================================================================
// compute_core.sv — M4: Option B Kalman Update Accelerator — Compute Core
// Extracted from post_m3_Minor_Redesign/rtl/top.sv
// Contains: f64_mul, f64_add, gemm_systolic, kalman_update
// Changes from m3:
//   1. kalman_update: r_val input port replaces R_CONST localparam
//   2. NR pipeline: FSM 11→51 states; sx_pipe and nr_x_pipe break the
//      92-level critical path into ~30-level stages
//   3. f64_mul: 3-stage pipeline (Stage 0 added for hidden-bit insert)
//   4. f64_add: 4-stage pipeline (Stage 3 added for LZ count)
// =============================================================================

// -----------------------------------------------------------------------------
// f64_mul -- IEEE-754 double-precision multiplier, 3-stage pipeline
// Stage 0 (always_ff): hidden-bit insert, exp_sum, flags -> s0_* regs
//   Removes LUT->DSP high-fanout path from DSP critical timing.
// Stage 1 (always_ff): mantissa product (DSP cascade) + exp_sum -> s1_* regs
//   Critical path: s0_man_r -> DSP48E1 cascade -> s1_product_reg (no LUTs)
// Stage 2 (always_comb): normalize + mux from s1_* -> result
// 2-cycle latency: result valid 2 cycles after inputs are presented.
// -----------------------------------------------------------------------------
module f64_mul (
    input  logic        clk,
    input  logic [63:0] a,
    input  logic [63:0] b,
    output logic [63:0] result
);

    logic        a_sign, b_sign;
    logic [10:0] a_exp,  b_exp;

    assign a_sign = a[63];
    assign b_sign = b[63];
    assign a_exp  = a[62:52];
    assign b_exp  = b[62:52];

    logic a_nan, b_nan, a_inf, b_inf, a_zero, b_zero;
    assign a_nan  = (a_exp == 11'h7FF) && (a[51:0] != 52'h0);
    assign b_nan  = (b_exp == 11'h7FF) && (b[51:0] != 52'h0);
    assign a_inf  = (a_exp == 11'h7FF) && (a[51:0] == 52'h0);
    assign b_inf  = (b_exp == 11'h7FF) && (b[51:0] == 52'h0);
    assign a_zero = (a_exp == 11'h0)   && (a[51:0] == 52'h0);
    assign b_zero = (b_exp == 11'h0)   && (b[51:0] == 52'h0);

    logic signed [12:0] exp_sum_comb;
    assign exp_sum_comb = $signed({2'b00, a_exp}) + $signed({2'b00, b_exp})
                          - $signed(13'd1023);

    // ── Stage 0: register mantissas (hidden-bit insert) + metadata ──────────
    // Removes the LUT->DSP high-fanout path: s0_man regs feed DSP directly.
    logic [52:0]        s0_a_man, s0_b_man;
    logic signed [12:0] s0_exp_sum;
    logic               s0_r_sign;
    logic               s0_a_nan, s0_b_nan, s0_a_inf, s0_b_inf;
    logic               s0_a_zero, s0_b_zero;

    always_ff @(posedge clk) begin
        s0_a_man   <= (a_exp == 11'h0) ? {1'b0, a[51:0]} : {1'b1, a[51:0]};
        s0_b_man   <= (b_exp == 11'h0) ? {1'b0, b[51:0]} : {1'b1, b[51:0]};
        s0_exp_sum <= exp_sum_comb;
        s0_r_sign  <= a_sign ^ b_sign;
        s0_a_nan   <= a_nan;   s0_b_nan   <= b_nan;
        s0_a_inf   <= a_inf;   s0_b_inf   <= b_inf;
        s0_a_zero  <= a_zero;  s0_b_zero  <= b_zero;
    end

    // ── Stage 1: DSP cascade multiply from s0_man (no LUTs in DSP path) ─────
    logic [105:0]       s1_product;
    logic signed [12:0] s1_exp_sum;
    logic               s1_r_sign;
    logic               s1_a_nan, s1_b_nan, s1_a_inf, s1_b_inf;
    logic               s1_a_zero, s1_b_zero;

    always_ff @(posedge clk) begin
        s1_product <= {53'b0, s0_a_man} * {53'b0, s0_b_man};
        s1_exp_sum <= s0_exp_sum;
        s1_r_sign  <= s0_r_sign;
        s1_a_nan   <= s0_a_nan;   s1_b_nan   <= s0_b_nan;
        s1_a_inf   <= s0_a_inf;   s1_b_inf   <= s0_b_inf;
        s1_a_zero  <= s0_a_zero;  s1_b_zero  <= s0_b_zero;
    end

    // ── Stage 2: normalize (combinational from s1_*) ─────────────────────────
    logic [51:0] norm_man;
    logic [12:0] norm_exp;
    always_comb begin
        if (s1_product[105]) begin
            norm_man = s1_product[104:53];
            norm_exp = s1_exp_sum + 13'd1;
        end else begin
            norm_man = s1_product[103:52];
            norm_exp = s1_exp_sum;
        end
    end

    always_comb begin
        if (s1_a_nan || s1_b_nan) begin
            result = 64'h7FF8_0000_0000_0000;
        end else if ((s1_a_inf && s1_b_zero) || (s1_b_inf && s1_a_zero)) begin
            result = 64'h7FF8_0000_0000_0000;
        end else if (s1_a_inf || s1_b_inf) begin
            result = {s1_r_sign, 11'h7FF, 52'h0};
        end else if (s1_a_zero || s1_b_zero) begin
            result = {s1_r_sign, 63'b0};
        end else if (norm_exp >= $signed(13'd2047)) begin
            result = {s1_r_sign, 11'h7FF, 52'h0};
        end else if (norm_exp <= $signed(13'd0)) begin
            result = {s1_r_sign, 63'b0};
        end else begin
            result = {s1_r_sign, norm_exp[10:0], norm_man};
        end
    end

endmodule

// -----------------------------------------------------------------------------
// f64_add -- IEEE-754 double-precision adder/subtractor, 4-stage pipeline
// Stage 1 (always_ff): compare + swap + shift_amt + barrel shift -> s1_* regs
// Stage 2 (always_ff): 54-bit add/sub result -> s2_sum_raw + metadata regs
// Stage 3 (always_ff): LZ count (priority encoder) -> s3_lz_count + passthrough
// Stage 4 (always_comb): normalize + mux from s3_* -> result
// 3-cycle latency: result valid 3 cycles after inputs are presented.
// Critical path B (Stage 2): s1_regs -> 54-bit adder -> s2_regs   (~8 ns OOC)
// Critical path C (Stage 3): s2_regs -> LZ encoder -> s3_regs      (~7 ns OOC)
// Critical path D (Stage 4): s3_regs -> barrel shift + norm -> result_reg
// -----------------------------------------------------------------------------
module f64_add (
    input  logic        clk,
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

    // ── Stage 1 combinational ─────────────────────────────────────────────────
    logic        big_sign, sml_sign;
    logic [10:0] big_exp,  sml_exp;
    logic [52:0] big_man,  sml_man;
    logic        do_swap;

    always_comb begin
        if (a_exp > b_exp)      do_swap = 1'b0;
        else if (b_exp > a_exp) do_swap = 1'b1;
        else                    do_swap = (b_frac > a_frac);

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

    logic [5:0]  shift_amt_comb;
    logic [52:0] sml_aligned_comb;
    always_comb begin
        shift_amt_comb   = (big_exp - sml_exp > 63) ? 6'd63 : big_exp[5:0] - sml_exp[5:0];
        sml_aligned_comb = sml_man >> shift_amt_comb;
    end

    // ── Stage 1 registers ────────────────────────────────────────────────────
    logic        s1_big_sign, s1_eff_sub, s1_r_sign;
    logic [10:0] s1_big_exp;
    logic [52:0] s1_big_man, s1_sml_aligned;
    logic        s1_a_nan, s1_b_nan, s1_a_inf, s1_b_inf, s1_a_zero, s1_b_zero;
    logic [63:0] s1_a, s1_b;   // full inputs for special-case passthrough

    always_ff @(posedge clk) begin
        s1_big_sign    <= big_sign;
        s1_big_exp     <= big_exp;
        s1_big_man     <= big_man;
        s1_sml_aligned <= sml_aligned_comb;
        s1_eff_sub     <= big_sign ^ sml_sign;
        s1_r_sign      <= big_sign;
        s1_a_nan  <= a_nan;  s1_b_nan  <= b_nan;
        s1_a_inf  <= a_inf;  s1_b_inf  <= b_inf;
        s1_a_zero <= a_zero; s1_b_zero <= b_zero;
        s1_a <= a;  s1_b <= b;
    end

    // ── Stage 2 registers: 54-bit adder result latched from s1_* ────────────
    logic [53:0] s2_sum_raw;
    logic [10:0] s2_big_exp;
    logic        s2_r_sign;
    logic        s2_a_nan, s2_b_nan, s2_a_inf, s2_b_inf, s2_a_zero, s2_b_zero;
    logic [63:0] s2_a, s2_b;

    always_ff @(posedge clk) begin
        if (!s1_eff_sub)
            s2_sum_raw <= {1'b0, s1_big_man} + {1'b0, s1_sml_aligned};
        else
            s2_sum_raw <= {1'b0, s1_big_man} - {1'b0, s1_sml_aligned};
        s2_big_exp <= s1_big_exp;
        s2_r_sign  <= s1_r_sign;
        s2_a_nan   <= s1_a_nan;  s2_b_nan  <= s1_b_nan;
        s2_a_inf   <= s1_a_inf;  s2_b_inf  <= s1_b_inf;
        s2_a_zero  <= s1_a_zero; s2_b_zero <= s1_b_zero;
        s2_a <= s1_a; s2_b <= s1_b;
    end

    // ── Stage 3 registers: LZ count latched from s2_* ───────────────────────
    // Splits the LZ-encoder → normalize fan-out into two short paths.
    logic [5:0]  lz_count_comb;   // combinational LZ count from s2_sum_raw
    always_comb begin : lz_encoder
        lz_count_comb = 6'd53;
        for (int i = 0; i <= 52; i++)
            if (s2_sum_raw[i]) lz_count_comb = 6'(52 - i);
    end

    logic [5:0]  s3_lz_count;
    logic [53:0] s3_sum_raw;
    logic [10:0] s3_big_exp;
    logic        s3_r_sign;
    logic        s3_a_nan, s3_b_nan, s3_a_inf, s3_b_inf, s3_a_zero, s3_b_zero;
    logic [63:0] s3_a, s3_b;

    always_ff @(posedge clk) begin
        s3_lz_count <= lz_count_comb;
        s3_sum_raw  <= s2_sum_raw;
        s3_big_exp  <= s2_big_exp;
        s3_r_sign   <= s2_r_sign;
        s3_a_nan    <= s2_a_nan;  s3_b_nan  <= s2_b_nan;
        s3_a_inf    <= s2_a_inf;  s3_b_inf  <= s2_b_inf;
        s3_a_zero   <= s2_a_zero; s3_b_zero <= s2_b_zero;
        s3_a <= s2_a; s3_b <= s2_b;
    end

    // ── Stage 4: normalize + mux (combinational from s3_*) ──────────────────
    logic [11:0] res_exp;
    logic [51:0] res_man;
    logic [52:0] shifted;
    always_comb begin
        shifted = s3_sum_raw[52:0] << s3_lz_count;
        if (s3_sum_raw[53]) begin
            res_exp = {1'b0, s3_big_exp} + 12'd1;
            res_man = s3_sum_raw[52:1];
        end else if (s3_sum_raw[52]) begin
            res_exp = {1'b0, s3_big_exp};
            res_man = s3_sum_raw[51:0];
        end else begin
            res_exp = {1'b0, s3_big_exp} - {6'b0, s3_lz_count};
            res_man = shifted[51:0];
        end
    end

    always_comb begin
        if (s3_a_nan || s3_b_nan)
            result = 64'h7FF8_0000_0000_0000;
        else if (s3_a_inf && s3_b_inf && (s3_a[63] != s3_b[63]))
            result = 64'h7FF8_0000_0000_0000;
        else if (s3_a_inf || s3_b_inf)
            result = s3_a_inf ? s3_a : s3_b;
        else if (s3_a_zero && s3_b_zero)
            result = 64'h0000_0000_0000_0000;
        else if (s3_a_zero)
            result = s3_b;
        else if (s3_b_zero)
            result = s3_a;
        else if (s3_sum_raw == 54'h0)
            result = 64'h0000_0000_0000_0000;
        else if (res_exp[11] || res_exp == 12'h0)
            result = {s3_r_sign, 63'b0};
        else if (res_exp >= 12'h7FF)
            result = {s3_r_sign, 11'h7FF, 52'h0};
        else
            result = {s3_r_sign, res_exp[10:0], res_man};
    end

endmodule

// -----------------------------------------------------------------------------
// gemm_systolic — 3×3 pipelined systolic GEMM
//
// Pipeline change: f64_mul and f64_add are now 2-stage registered modules
// (1-cycle latency each). Each STEPx is split into STEPxM+STEPxMW (mul) and
// STEPxA+STEPxAW (add) so the caller reads results exactly 1 cycle after
// presenting inputs.
//
// Critical paths after pipelining:
//   STEPxM  → STEPxMW: f64_mul Stage 1 only (DSP cascade, ~7-9 ns)
//   STEPxMW internal: f64_mul Stage 2 (norm, ~3-4 ns) [no external register needed]
//   STEPxA  → STEPxAW: f64_add Stage 1 only (compare+shift, ~8-10 ns)
//   STEPxAW internal: f64_add Stage 2 (add+LZ+norm, ~8-10 ns)
//
// FSM: IDLE->LOAD->STEP0M->STEP0MW->STEP0A->STEP0AW->STEP0AW2->STEP0AW3->...->STEP2AW3->FINISH (21 states)
// GEMM latency increases from 9 to 15 clock cycles (accepted trade-off).
// -----------------------------------------------------------------------------
module gemm_systolic (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [63:0] A [0:8],
    input  logic [63:0] B [0:8],
    output logic [63:0] C [0:8],
    output logic        done,
    output logic        busy
);

    typedef enum logic [4:0] {
        IDLE      = 5'd0,
        LOAD      = 5'd1,
        STEP0M    = 5'd2,   // k=0: present a_k_mux to f64_mul (stage-0 latches)
        STEP0MW   = 5'd3,   // k=0: f64_mul stage-1 (DSP multiply) latches
        STEP0MW2  = 5'd4,   // k=0: f64_mul result valid (stage-2 comb) -> latch mul_reg
        STEP0A    = 5'd5,   // k=0: present mul_reg+acc to f64_add (stage-1 latches)
        STEP0AW   = 5'd6,   // k=0: f64_add stage-2 (s2_sum_raw latches)
        STEP0AW2  = 5'd7,   // k=0: f64_add stage-3 (s3_lz_count latches)
        STEP0AW3  = 5'd8,   // k=0: f64_add result valid (stage-4 comb) -> latch acc
        STEP1M    = 5'd9,
        STEP1MW   = 5'd10,
        STEP1MW2  = 5'd11,
        STEP1A    = 5'd12,
        STEP1AW   = 5'd13,
        STEP1AW2  = 5'd14,
        STEP1AW3  = 5'd15,
        STEP2M    = 5'd16,
        STEP2MW   = 5'd17,
        STEP2MW2  = 5'd18,
        STEP2A    = 5'd19,
        STEP2AW   = 5'd20,
        STEP2AW2  = 5'd21,
        STEP2AW3  = 5'd22,
        FINISH    = 5'd23   // latch C from acc; assert done
    } state_t;

    state_t state, state_nxt;

    // Pre-registered operand pairs — loaded from A/B inputs during LOAD state.
    // a_k[k][gi] = A[gi*3+k],  b_k[k][gj] = B[k*3+gj]
    logic [63:0] a_k[0:2][0:2];
    logic [63:0] b_k[0:2][0:2];

    logic [63:0] acc    [0:8];
    logic [63:0] mul_reg[0:2][0:2];  // registered f64_mul results (Stage M boundary)

    // Pre-registered mux output: holds the k-selected operands for the current
    // (or next) f64_mul stage. Latched in LOAD (k=0), STEP0AW2 (k=1), STEP1AW2 (k=2).
    // Removes FSM→LUT→DSP fan-out from the f64_mul Stage-1 critical path.
    logic [63:0] a_k_mux [0:2];   // a_k_mux[gi] = a_k[k_current][gi], registered
    logic [63:0] b_k_mux [0:2];   // b_k_mux[gj] = b_k[k_current][gj], registered

    logic [63:0] mul_out[0:2][0:2];   // f64_mul Stage-2 combinational result
    logic [63:0] add_out[0:2][0:2];   // f64_add Stage-3 combinational result

    genvar gi, gj;
    generate
        for (gi = 0; gi < 3; gi++) begin : gen_row
            for (gj = 0; gj < 3; gj++) begin : gen_col
                // f64_mul: 1-cycle latency — present in STEPxM, read in STEPxMW
                // Operands pre-registered in a_k_mux/b_k_mux (no FSM→LUT path)
                f64_mul u_mul (
                    .clk    (clk),
                    .a      (a_k_mux[gi]),
                    .b      (b_k_mux[gj]),
                    .result (mul_out[gi][gj])
                );

                // f64_add: 1-cycle latency — present in STEPxA, read in STEPxAW
                // Both inputs (mul_reg and acc) are registered → no feedback path.
                f64_add u_add (
                    .clk    (clk),
                    .a      (acc[gi*3+gj]),
                    .b      (mul_reg[gi][gj]),
                    .result (add_out[gi][gj])
                );
            end
        end
    endgenerate

    always_comb begin
        state_nxt = state;
        case (state)
            IDLE:      if (start) state_nxt = LOAD;
            LOAD:                 state_nxt = STEP0M;
            STEP0M:               state_nxt = STEP0MW;
            STEP0MW:              state_nxt = STEP0MW2;
            STEP0MW2:             state_nxt = STEP0A;
            STEP0A:               state_nxt = STEP0AW;
            STEP0AW:              state_nxt = STEP0AW2;
            STEP0AW2:             state_nxt = STEP0AW3;
            STEP0AW3:             state_nxt = STEP1M;
            STEP1M:               state_nxt = STEP1MW;
            STEP1MW:              state_nxt = STEP1MW2;
            STEP1MW2:             state_nxt = STEP1A;
            STEP1A:               state_nxt = STEP1AW;
            STEP1AW:              state_nxt = STEP1AW2;
            STEP1AW2:             state_nxt = STEP1AW3;
            STEP1AW3:             state_nxt = STEP2M;
            STEP2M:               state_nxt = STEP2MW;
            STEP2MW:              state_nxt = STEP2MW2;
            STEP2MW2:             state_nxt = STEP2A;
            STEP2A:               state_nxt = STEP2AW;
            STEP2AW:              state_nxt = STEP2AW2;
            STEP2AW2:             state_nxt = STEP2AW3;
            STEP2AW3:             state_nxt = FINISH;
            FINISH:               state_nxt = IDLE;
            default:              state_nxt = IDLE;
        endcase
    end

    integer i, j;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            for (i = 0; i < 9; i++) begin
                acc[i] <= 64'h0;
                C[i]   <= 64'h0;
            end
            for (i = 0; i < 3; i++) begin
                a_k_mux[i] <= 64'h0;
                b_k_mux[i] <= 64'h0;
                for (j = 0; j < 3; j++) begin
                    a_k[i][j]     <= 64'h0;
                    b_k[i][j]     <= 64'h0;
                    mul_reg[i][j] <= 64'h0;
                end
            end
        end else begin
            state <= state_nxt;

            case (state)
                LOAD: begin
                    for (i = 0; i < 9; i++) acc[i] <= 64'h0;
                    for (i = 0; i < 3; i++) begin  // i = k
                        for (j = 0; j < 3; j++) begin  // j = row/col
                            a_k[i][j] <= A[j*3+i];
                            b_k[i][j] <= B[i*3+j];
                        end
                    end
                    // Pre-select k=0 operands for STEP0M (a_k not yet written,
                    // so read directly from A/B ports)
                    for (j = 0; j < 3; j++) begin
                        a_k_mux[j] <= A[j*3+0];   // a_k[0][j] = A[j*3+0]
                        b_k_mux[j] <= B[j];         // b_k[0][j] = B[0*3+j]
                    end
                end

                // f64_mul: 2-cycle latency. STEPxMW is no-op wait (stage-0→1).
                // STEPxMW2: stage-2 (normalize comb) result valid -> latch mul_reg.
                STEP0MW2, STEP1MW2, STEP2MW2: begin
                    for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                            mul_reg[i][j] <= mul_out[i][j];
                end

                // f64_add: 3-cycle latency. STEPxAW/AW2 are wait states.
                // STEPxAW3: stage-4 result valid -> latch acc.
                // Pre-select next k operands one cycle before each STEPxM.
                // STEPxM arrives 2 cycles after STEPxMW2, so pre-select in STEPxAW2.
                STEP0AW2: begin
                    // Pre-select k=1 for STEP1M (STEP0AW3->STEP1M, so latch here)
                    for (i = 0; i < 3; i++) begin
                        a_k_mux[i] <= a_k[1][i];
                        b_k_mux[i] <= b_k[1][i];
                    end
                end

                STEP0AW3: begin
                    for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                            acc[i*3+j] <= add_out[i][j];
                end

                STEP1AW2: begin
                    // Pre-select k=2 for STEP2M
                    for (i = 0; i < 3; i++) begin
                        a_k_mux[i] <= a_k[2][i];
                        b_k_mux[i] <= b_k[2][i];
                    end
                end

                STEP1AW3: begin
                    for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                            acc[i*3+j] <= add_out[i][j];
                end

                STEP2AW3: begin
                    for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                            acc[i*3+j] <= add_out[i][j];
                end

                FINISH: begin
                    for (i = 0; i < 9; i++) C[i] <= acc[i];
                end

                default: ;
            endcase
        end
    end

    assign done = (state == FINISH);
    assign busy = (state != IDLE) && (state != FINISH);

endmodule


// -----------------------------------------------------------------------------
// kalman_update — Kalman filter update kernel, Option B
// Changes from m3 / previous iterations:
//   - r_val input port replaces R_CONST localparam
//   - f64_mul/f64_add are now 2-stage pipelined (1-cycle latency each)
//   - FSM expanded to 31 states: each arithmetic op gets a "present" + "latch"
//     state pair, accounting for the 1-cycle pipeline delay
//   - NR chain: NRx_A, NRx_AW, NRx_AS, NRx_BS, NRx_BM, NRx_BMW (6 per iter)
//   - X_CORR: X_CORR_M + X_CORR_A + X_CORR_AW
//   - K_COMP: K_COMP + K_COMP_W + K_WAIT
//   - INNOV_W between INNOV and S_COMP for y_tilde/S pipeline settle
// -----------------------------------------------------------------------------
module kalman_update (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [63:0] z,
    input  logic [63:0] x_in  [0:2],
    input  logic [63:0] P_in  [0:8],
    input  logic [63:0] r_val,           // programmable R (from R_REG at 0xE8)
    output logic [63:0] x_out [0:2],
    output logic [63:0] P_out [0:8],
    output logic        done,
    output logic        busy
);

    localparam logic [63:0] F64_TWO = 64'h4000_0000_0000_0000;
    localparam logic [63:0] F64_ONE = 64'h3FF0_0000_0000_0000;

    // -------------------------------------------------------------------------
    // FSM state encoding -- 51 states (6-bit, values 0-50)
    //
    // f64_mul: 2-cycle latency (Stage-0 mantissa reg added).
    // f64_add: 3-cycle latency (Stages 1-3 registered).
    //
    // NR chain (10 states per iteration x 3 = 30 states):
    //   NRx_A:   present S_reg+nr_x to u_nr_mul1   [f64_mul stage-0 latches]
    //   NRx_AW:  f64_mul stage-1 (DSP) latches
    //   NRx_AW2: f64_mul result valid -> latch sx_pipe, nr_x_pipe
    //   NRx_AS:  present to u_nr_sub [f64_add stage-1 latches]
    //   NRx_AS2: u_nr_sub stage-2 (s2_sum_raw) latches
    //   NRx_AS3: u_nr_sub stage-3 (s3_lz_count) latches
    //   NRx_BS:  u_nr_sub result valid -> latch two_minus_sx_reg
    //   NRx_BM:  present to u_nr_mul2 [f64_mul stage-0 latches]
    //   NRx_BMW: f64_mul stage-1 latches
    //   NRx_BMW2:f64_mul result valid -> latch nr_x, S_inv
    // -------------------------------------------------------------------------
    typedef enum logic [5:0] {
        IDLE       = 6'd0,
        INNOV      = 6'd1,   // latch z_reg, x_reg, P_reg from ports
        INNOV_W    = 6'd2,   // u_ytilde/u_scomb stage-1 latches
        INNOV_W2   = 6'd3,   // u_ytilde/u_scomb stage-2 (s2_sum_raw) latches
        INNOV_W3   = 6'd4,   // u_ytilde/u_scomb stage-3 (s3_lz_count) latches
        S_COMP     = 6'd5,   // y_tilde/S_comb valid (stage-4) -> latch
        NR0_A      = 6'd6,   NR0_AW   = 6'd7,  NR0_AW2 = 6'd8,
        NR0_AS     = 6'd9,   NR0_AS2  = 6'd10, NR0_AS3 = 6'd11,
        NR0_BS     = 6'd12,  NR0_BM   = 6'd13, NR0_BMW = 6'd14, NR0_BMW2 = 6'd15,
        NR1_A      = 6'd16,  NR1_AW   = 6'd17, NR1_AW2 = 6'd18,
        NR1_AS     = 6'd19,  NR1_AS2  = 6'd20, NR1_AS3 = 6'd21,
        NR1_BS     = 6'd22,  NR1_BM   = 6'd23, NR1_BMW = 6'd24, NR1_BMW2 = 6'd25,
        NR2_A      = 6'd26,  NR2_AW   = 6'd27, NR2_AW2 = 6'd28,
        NR2_AS     = 6'd29,  NR2_AS2  = 6'd30, NR2_AS3 = 6'd31,
        NR2_BS     = 6'd32,  NR2_BM   = 6'd33, NR2_BMW = 6'd34, NR2_BMW2 = 6'd35,
        K_COMP     = 6'd36,  K_COMP_W = 6'd37, K_COMP_W2 = 6'd38,
        K_WAIT     = 6'd39,
        K_WAIT_W   = 6'd40,  // f64_mul stage-1 wait; f64_add stage-2 wait
        K_WAIT_W2  = 6'd41,  // f64_mul result valid -> latch ky_reg + IKH[1:8]
        K_WAIT_W3  = 6'd42,  // f64_add stage-3 wait
        X_CORR_M   = 6'd43,  // IKH[0]=one_minus_k0 latched (f64_add 3-cycle)
        X_CORR_A   = 6'd44,  X_CORR_AW = 6'd45,  X_CORR_AW2 = 6'd46, X_CORR_AW3 = 6'd47,
        P_UPD      = 6'd48,
        WAIT_P     = 6'd49,
        DONE_S     = 6'd50
    } state_t;

    state_t state, state_nxt;

    logic [63:0] z_reg, y_tilde, S_reg, S_inv;
    logic [63:0] x_reg  [0:2];
    logic [63:0] P_reg  [0:8];
    logic [63:0] K_reg  [0:2];
    logic [63:0] IKH    [0:8];

    // NR pipeline registers
    logic [63:0] nr_x;
    logic [63:0] sx_pipe;            // latched u_nr_mul1 result
    logic [63:0] nr_x_pipe;          // nr_x registered alongside sx_pipe
    logic [63:0] two_minus_sx_reg;   // latched u_nr_sub result

    // X_CORR pipeline register
    logic [63:0] ky_reg [0:2];       // latched u_ky* results

    logic        gs_start, gs_done, gs_busy;
    logic [63:0] gs_A [0:8], gs_B [0:8], gs_C [0:8];

    gemm_systolic u_gemm (
        .clk   (clk),
        .rst_n (rst_n),
        .start (gs_start),
        .A     (gs_A),
        .B     (gs_B),
        .C     (gs_C),
        .done  (gs_done),
        .busy  (gs_busy)
    );

    // -------------------------------------------------------------------------
    // Arithmetic units — all 2-stage pipelined (clk added)
    // Timing contract: present inputs in state X; read result in state X+1.
    // -------------------------------------------------------------------------

    // y_tilde = z_reg - x_reg[0]  (presented in INNOV_W, latched in S_COMP)
    logic [63:0] y_tilde_comb;
    f64_add u_ytilde (.clk(clk), .a(z_reg), .b(f64_neg(x_reg[0])), .result(y_tilde_comb));

    // S = P[0] + R  (presented in INNOV_W, latched in S_COMP)
    logic [63:0] S_comb;
    f64_add u_scomb  (.clk(clk), .a(P_reg[0]), .b(r_val), .result(S_comb));

    // NR: sx = S_reg * nr_x  (NRx_A → NRx_AW)
    logic [63:0] sx;
    f64_mul u_nr_mul1 (.clk(clk), .a(S_reg), .b(nr_x), .result(sx));

    // NR: two_minus_sx = 2 - sx_pipe  (NRx_AS → NRx_BS)
    logic [63:0] two_minus_sx;
    f64_add u_nr_sub  (.clk(clk), .a(F64_TWO), .b({~sx_pipe[63], sx_pipe[62:0]}),
                       .result(two_minus_sx));

    // NR: nr_x_new = nr_x_pipe * two_minus_sx_reg  (NRx_BM → NRx_BMW)
    logic [63:0] nr_x_new;
    f64_mul u_nr_mul2 (.clk(clk), .a(nr_x_pipe), .b(two_minus_sx_reg), .result(nr_x_new));

    // K[i] = P_reg[i*3] * S_inv  (K_COMP → K_COMP_W)
    logic [63:0] k_comb [0:2];
    f64_mul u_k0 (.clk(clk), .a(P_reg[0]), .b(S_inv), .result(k_comb[0]));
    f64_mul u_k1 (.clk(clk), .a(P_reg[3]), .b(S_inv), .result(k_comb[1]));
    f64_mul u_k2 (.clk(clk), .a(P_reg[6]), .b(S_inv), .result(k_comb[2]));

    // 1 - K[0]  (K_WAIT → X_CORR_M)
    logic [63:0] one_minus_k0;
    f64_add u_1mk0 (.clk(clk), .a(F64_ONE), .b(f64_neg(K_reg[0])), .result(one_minus_k0));

    // ky[i] = K_reg[i] * y_tilde  (K_WAIT → X_CORR_M)
    logic [63:0] ky0, ky1, ky2;
    f64_mul u_ky0 (.clk(clk), .a(K_reg[0]), .b(y_tilde), .result(ky0));
    f64_mul u_ky1 (.clk(clk), .a(K_reg[1]), .b(y_tilde), .result(ky1));
    f64_mul u_ky2 (.clk(clk), .a(K_reg[2]), .b(y_tilde), .result(ky2));

    // xout[i] = x_reg[i] + ky_reg[i]  (X_CORR_A → X_CORR_AW)
    logic [63:0] xout0, xout1, xout2;
    f64_add u_xo0 (.clk(clk), .a(x_reg[0]), .b(ky_reg[0]), .result(xout0));
    f64_add u_xo1 (.clk(clk), .a(x_reg[1]), .b(ky_reg[1]), .result(xout1));
    f64_add u_xo2 (.clk(clk), .a(x_reg[2]), .b(ky_reg[2]), .result(xout2));

    // ── FSM next-state ────────────────────────────────────────────────────────
    always_comb begin
        state_nxt = state;
        case (state)
            IDLE:        if (start)  state_nxt = INNOV;
            INNOV:                   state_nxt = INNOV_W;
            INNOV_W:                 state_nxt = INNOV_W2;
            INNOV_W2:                state_nxt = INNOV_W3;
            INNOV_W3:                state_nxt = S_COMP;
            S_COMP:                  state_nxt = NR0_A;
            NR0_A:                   state_nxt = NR0_AW;
            NR0_AW:                  state_nxt = NR0_AW2;
            NR0_AW2:                 state_nxt = NR0_AS;
            NR0_AS:                  state_nxt = NR0_AS2;
            NR0_AS2:                 state_nxt = NR0_AS3;
            NR0_AS3:                 state_nxt = NR0_BS;
            NR0_BS:                  state_nxt = NR0_BM;
            NR0_BM:                  state_nxt = NR0_BMW;
            NR0_BMW:                 state_nxt = NR0_BMW2;
            NR0_BMW2:                state_nxt = NR1_A;
            NR1_A:                   state_nxt = NR1_AW;
            NR1_AW:                  state_nxt = NR1_AW2;
            NR1_AW2:                 state_nxt = NR1_AS;
            NR1_AS:                  state_nxt = NR1_AS2;
            NR1_AS2:                 state_nxt = NR1_AS3;
            NR1_AS3:                 state_nxt = NR1_BS;
            NR1_BS:                  state_nxt = NR1_BM;
            NR1_BM:                  state_nxt = NR1_BMW;
            NR1_BMW:                 state_nxt = NR1_BMW2;
            NR1_BMW2:                state_nxt = NR2_A;
            NR2_A:                   state_nxt = NR2_AW;
            NR2_AW:                  state_nxt = NR2_AW2;
            NR2_AW2:                 state_nxt = NR2_AS;
            NR2_AS:                  state_nxt = NR2_AS2;
            NR2_AS2:                 state_nxt = NR2_AS3;
            NR2_AS3:                 state_nxt = NR2_BS;
            NR2_BS:                  state_nxt = NR2_BM;
            NR2_BM:                  state_nxt = NR2_BMW;
            NR2_BMW:                 state_nxt = NR2_BMW2;
            NR2_BMW2:                state_nxt = K_COMP;
            K_COMP:                  state_nxt = K_COMP_W;
            K_COMP_W:                state_nxt = K_COMP_W2;
            K_COMP_W2:               state_nxt = K_WAIT;
            K_WAIT:                  state_nxt = K_WAIT_W;
            K_WAIT_W:                state_nxt = K_WAIT_W2;
            K_WAIT_W2:               state_nxt = K_WAIT_W3;
            K_WAIT_W3:               state_nxt = X_CORR_M;
            X_CORR_M:                state_nxt = X_CORR_A;
            X_CORR_A:                state_nxt = X_CORR_AW;
            X_CORR_AW:               state_nxt = X_CORR_AW2;
            X_CORR_AW2:              state_nxt = X_CORR_AW3;
            X_CORR_AW3:              state_nxt = P_UPD;
            P_UPD:                   state_nxt = WAIT_P;
            WAIT_P:     if (gs_done) state_nxt = DONE_S;
            DONE_S:                  state_nxt = IDLE;
            default:                 state_nxt = IDLE;
        endcase
    end

    function automatic logic [63:0] f64_neg (input logic [63:0] x);
        f64_neg = {~x[63], x[62:0]};
    endfunction

    function automatic logic [63:0] nr_seed (input logic [63:0] s);
        nr_seed = 64'h7FDE_6000_0000_0000 - s;
    endfunction

    integer ii;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= IDLE;
            gs_start           <= 1'b0;
            sx_pipe            <= 64'h0;
            nr_x_pipe          <= 64'h0;
            two_minus_sx_reg   <= 64'h0;
            y_tilde            <= 64'h0;
            S_reg              <= 64'h0;
            for (ii = 0; ii < 3; ii++) ky_reg[ii] <= 64'h0;
            for (ii = 0; ii < 9; ii++) begin
                gs_A[ii]  <= 64'h0;
                gs_B[ii]  <= 64'h0;
                IKH[ii]   <= 64'h0;
                P_reg[ii] <= 64'h0;
            end
            for (ii = 0; ii < 3; ii++) begin
                x_reg[ii] <= 64'h0;
                K_reg[ii] <= 64'h0;
                x_out[ii] <= 64'h0;
            end
            for (ii = 0; ii < 9; ii++) P_out[ii] <= 64'h0;
            z_reg  <= 64'h0;
            S_inv  <= 64'h0;
            nr_x   <= 64'h0;
        end else begin
            state    <= state_nxt;
            gs_start <= 1'b0;

            case (state)
                // ── Initialization ────────────────────────────────────────────
                INNOV: begin
                    // Latch inputs; u_ytilde/u_scomb stage-1 will latch at end of INNOV_W
                    z_reg    <= z;
                    for (ii = 0; ii < 3; ii++) x_reg[ii] <= x_in[ii];
                    for (ii = 0; ii < 9; ii++) P_reg[ii] <= P_in[ii];
                end
                // INNOV_W/W2/W3: no assignments -- wait for f64_add stages 1/2/3
                S_COMP: begin
                    // y_tilde_comb/S_comb valid (f64_add stage-4 from INNOV_W)
                    y_tilde <= y_tilde_comb;
                    S_reg   <= S_comb;
                    nr_x    <= nr_seed(P_reg[0]);
                end

                // ── NR Iteration 0 ───────────────────────────────────────────
                // NR0_A: present S_reg+nr_x to u_nr_mul1 (f64_mul stage-0 latches)
                // NR0_AW: f64_mul stage-1 (DSP) latches -- no assignment
                NR0_AW2: begin   // u_nr_mul1.result valid (f64_mul 2-cycle)
                    sx_pipe   <= sx;
                    nr_x_pipe <= nr_x;
                end
                // NR0_AS: sx_pipe stable; present to u_nr_sub (f64_add stage-1 latches)
                // NR0_AS2: f64_add stage-2 latches; NR0_AS3: stage-3 latches
                NR0_BS: begin   // u_nr_sub.result valid (f64_add 3-cycle)
                    two_minus_sx_reg <= two_minus_sx;
                end
                // NR0_BM: present nr_x_pipe+tms to u_nr_mul2 (f64_mul stage-0 latches)
                // NR0_BMW: f64_mul stage-1 (DSP) latches -- no assignment
                NR0_BMW2: begin  // u_nr_mul2.result valid (f64_mul 2-cycle)
                    nr_x  <= nr_x_new;
                    S_inv <= nr_x_new;
                end

                // ── NR Iteration 1 ───────────────────────────────────────────
                NR1_AW2: begin
                    sx_pipe   <= sx;
                    nr_x_pipe <= nr_x;
                end
                NR1_BS: begin
                    two_minus_sx_reg <= two_minus_sx;
                end
                NR1_BMW2: begin
                    nr_x  <= nr_x_new;
                    S_inv <= nr_x_new;
                end

                // ── NR Iteration 2 (final) ────────────────────────────────────
                NR2_AW2: begin
                    sx_pipe   <= sx;
                    nr_x_pipe <= nr_x;
                end
                NR2_BS: begin
                    two_minus_sx_reg <= two_minus_sx;
                end
                NR2_BMW2: begin
                    S_inv <= nr_x_new;   // converged reciprocal
                end

                // ── Kalman gain K ─────────────────────────────────────────────
                // K_COMP: S_inv stable; present P_reg+S_inv to u_k0/1/2
                // K_COMP_W: f64_mul stage-1 (DSP) latches -- no assignment
                K_COMP_W2: begin   // u_k*.result valid (f64_mul 2-cycle from K_COMP)
                    K_reg[0] <= k_comb[0];
                    K_reg[1] <= k_comb[1];
                    K_reg[2] <= k_comb[2];
                end
                // K_WAIT: K_reg stable; present K_reg+y_tilde to u_ky* (f64_mul)
                //         AND F64_ONE+neg(K_reg[0]) to u_1mk0 (f64_add)
                // K_WAIT_W:  f64_mul stage-1 (DSP); f64_add stage-2 -- no assignment

                K_WAIT_W2: begin   // u_ky* results valid (f64_mul 2-cycle from K_WAIT)
                    ky_reg[0] <= ky0;
                    ky_reg[1] <= ky1;
                    ky_reg[2] <= ky2;
                    // non-f64_add IKH entries (no pipeline dependency on u_1mk0)
                    IKH[1] <= 64'h0;
                    IKH[2] <= 64'h0;
                    IKH[3] <= f64_neg(K_reg[1]);
                    IKH[4] <= F64_ONE;
                    IKH[5] <= 64'h0;
                    IKH[6] <= f64_neg(K_reg[2]);
                    IKH[7] <= 64'h0;
                    IKH[8] <= F64_ONE;
                end
                // K_WAIT_W3: u_1mk0 stage-3 (s3_lz_count) latches -- no assignment

                // ── State update ──────────────────────────────────────────────
                X_CORR_M: begin   // u_1mk0 result valid (f64_add 3-cycle from K_WAIT)
                    IKH[0] <= one_minus_k0;
                end
                // X_CORR_A: ky_reg stable; present x_reg+ky_reg to u_xo* (f64_add stage-1)
                // X_CORR_AW/AW2/AW3: wait for f64_add stages 2/3/4
                X_CORR_AW3: begin  // u_xo*.result valid (f64_add 3-cycle from X_CORR_A)
                    x_out[0] <= xout0;
                    x_out[1] <= xout1;
                    x_out[2] <= xout2;
                end

                // ── Covariance update ─────────────────────────────────────────
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
                    for (ii = 0; ii < 9; ii++) P_out[ii] <= gs_C[ii];
                end

                default: ;
            endcase
        end
    end

    assign done = (state == DONE_S);
    assign busy = (state != IDLE) && (state != DONE_S);

endmodule
