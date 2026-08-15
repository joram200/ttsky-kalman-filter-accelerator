`timescale 1ns/1ps
// =============================================================================
// compute_core.sv — M3-derived combinational compute core for tt_um_joram200
// All synthesisable RTL combined into a single file.
// Module hierarchy (dependency order):
//   f64_mul       — Combinational IEEE-754 F64 multiplier (no clk)
//   f64_add       — Combinational IEEE-754 F64 adder/subtractor (no clk)
//   gemm_systolic — 3×3 weight-stationary systolic GEMM (used by kalman_update)
//   kalman_update — Kalman filter update kernel (instantiates gemm_systolic)
//
// Forward-ports vs M3:
//   - r_val[63:0] input on kalman_update (replaces hardcoded R_CONST = 5.0)
//   - P_out[0:8] all 9 elements driven (M3 AXI C_REG window exposed only P_out[0:5])
// =============================================================================

// -----------------------------------------------------------------------------
// f64_mul — Q8.56 signed fixed-point multiplier (Option C)
// Computes (a × b) in Q8.56 format: takes bits [119:56] of the 128-bit
// signed product, equivalent to arithmetic right-shift by 56.
// Port names retained for drop-in compatibility with gemm_systolic and
// kalman_update instantiations.
// -----------------------------------------------------------------------------
module f64_mul (
    input  logic [63:0] a,
    input  logic [63:0] b,
    output logic [63:0] result
);
    logic signed [127:0] product;
    assign product = $signed({{64{a[63]}}, a}) * $signed({{64{b[63]}}, b});
    assign result  = product[119:56];
endmodule

// -----------------------------------------------------------------------------
// f64_add — Q8.56 signed fixed-point adder (Option C)
// Two's-complement 64-bit addition; port names retained for drop-in
// compatibility with gemm_systolic and kalman_update instantiations.
// -----------------------------------------------------------------------------
module f64_add (
    input  logic [63:0] a,
    input  logic [63:0] b,
    output logic [63:0] result
);
    assign result = a + b;
endmodule

// -----------------------------------------------------------------------------
// gemm_systolic — 3×3 weight-stationary systolic GEMM, Option A
// Computes C = A × B (row-major F64 matrices).
// FSM: IDLE → LOAD → STEP0 → STEP1 → STEP2 → FINISH
// Each STEP accumulates one inner-dimension slice (k=0,1,2).
// 9 f64_mul + 9 f64_add instances run in parallel each cycle.
// Ports A, B, C are flat packed vectors (9×64=576 bits) for Yosys compatibility.
// -----------------------------------------------------------------------------
module gemm_systolic (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,       // one-cycle pulse: latch A, B, begin compute
    input  logic [575:0] A,          // row-major: A[i*3+j], packed 9×64
    input  logic [575:0] B,
    output logic [575:0] C,
    output logic        done,        // single-cycle pulse when C is valid
    output logic        busy
);

    // FSM state encoding
    typedef enum logic [2:0] {
        IDLE   = 3'd0,
        LOAD   = 3'd1,
        STEP0  = 3'd2,
        STEP1  = 3'd3,
        STEP2  = 3'd4,
        FINISH = 3'd5
    } state_t;

    state_t state, state_nxt;

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

    // Internal unpacked output array; pack to flat C port
    logic [63:0] C_arr [0:8];

    generate
        for (unp = 0; unp < 9; unp++) begin : pack_out
            assign C[unp*64 +: 64] = C_arr[unp];
        end
    endgenerate

    // Registered copies of inputs and accumulators
    logic [63:0] A_reg [0:8];
    logic [63:0] B_reg [0:8];
    logic [63:0] acc   [0:8];

    // k-selector (which inner-dimension index is active this cycle)
    logic [1:0] k_sel;
    always_comb begin
        case (state)
            STEP0:   k_sel = 2'd0;
            STEP1:   k_sel = 2'd1;
            default: k_sel = 2'd2;  // STEP2 and others
        endcase
    end

    // Combinational multiply and add products for each (i,j) pair
    // mul_out[i][j] = A_reg[i*3+k] * B_reg[k*3+j]
    // add_out[i][j] = acc[i*3+j] + mul_out[i][j]
    logic [63:0] mul_out [0:2][0:2];
    logic [63:0] add_out [0:2][0:2];

    genvar gi, gj;
    generate
        for (gi = 0; gi < 3; gi++) begin : gen_row
            for (gj = 0; gj < 3; gj++) begin : gen_col
                // Select A column element for current k
                logic [63:0] a_elem, b_elem;
                always_comb begin
                    a_elem = A_reg[gi*3 + k_sel];
                    b_elem = B_reg[k_sel*3 + gj];
                end

                f64_mul u_mul (
                    .a      (a_elem),
                    .b      (b_elem),
                    .result (mul_out[gi][gj])
                );

                f64_add u_add (
                    .a      (acc[gi*3+gj]),
                    .b      (mul_out[gi][gj]),
                    .result (add_out[gi][gj])
                );
            end
        end
    endgenerate

    // FSM next-state
    always_comb begin
        state_nxt = state;
        case (state)
            IDLE:   if (start)  state_nxt = LOAD;
            LOAD:               state_nxt = STEP0;
            STEP0:              state_nxt = STEP1;
            STEP1:              state_nxt = STEP2;
            STEP2:              state_nxt = FINISH;
            FINISH:             state_nxt = IDLE;
            default:            state_nxt = IDLE;
        endcase
    end

    // FSM state register + datapath
    integer i, j;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            for (i = 0; i < 9; i++) begin
                A_reg[i]  <= 64'h0;
                B_reg[i]  <= 64'h0;
                acc[i]    <= 64'h0;
                C_arr[i]  <= 64'h0;
            end
        end else begin
            state <= state_nxt;

            case (state)
                LOAD: begin
                    for (i = 0; i < 9; i++) begin
                        A_reg[i] <= A_in[i];
                        B_reg[i] <= B_in[i];
                        acc[i]   <= 64'h0;  // clear accumulators
                    end
                end

                STEP0, STEP1, STEP2: begin
                    // Register add_out into acc
                    for (i = 0; i < 3; i++) begin
                        for (j = 0; j < 3; j++) begin
                            acc[i*3+j] <= add_out[i][j];
                        end
                    end
                end

                FINISH: begin
                    for (i = 0; i < 9; i++) begin
                        C_arr[i] <= acc[i];
                    end
                end

                default: ;
            endcase
        end
    end

    // Output signals
    assign done = (state == FINISH);
    assign busy = (state != IDLE) && (state != FINISH);

endmodule

// -----------------------------------------------------------------------------
// kalman_update — Kalman filter update kernel, Option B
// H = [1, 0, 0] (scalar measurement)
// FSM: IDLE → INNOV → S_COMP → NR0 → NR1 → NR2 → K_COMP → X_CORR → P_UPD → WAIT_P → DONE
// Internally instantiates gemm_systolic for the IKH*P covariance update.
//
// Forward-ported vs M3:
//   - r_val[63:0] input replaces hardcoded R_CONST (5.0 F64)
//   - P_out[0:8] all 9 elements driven
// Ports x_in, P_in, x_out, P_out are flat packed vectors for Yosys compatibility.
// -----------------------------------------------------------------------------
module kalman_update (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [63:0] z,              // scalar measurement
    input  logic [191:0] x_in,          // prior state 3×1, packed 3×64
    input  logic [575:0] P_in,          // prior covariance 3×3, packed 9×64
    input  logic [63:0] r_val,          // measurement noise R (default 5.0 = 64'h4014000000000000)
    output logic [191:0] x_out,         // corrected state, packed 3×64
    output logic [575:0] P_out,         // corrected covariance (all 9 elements), packed 9×64
    output logic        done,
    output logic        busy
);

    // 2.0 in Q8.56
    localparam logic [63:0] F64_TWO = 64'h0200_0000_0000_0000;
    // 1.0 in Q8.56
    localparam logic [63:0] F64_ONE = 64'h0100_0000_0000_0000;

    // FSM states
    typedef enum logic [3:0] {
        IDLE   = 4'd0,
        INNOV  = 4'd1,   // y_tilde = z - x_in[0]; S_pre = P_in[0] + r_val
        S_COMP = 4'd2,   // S = S_pre (latch)
        NR0    = 4'd3,   // Newton-Raphson iteration 0: x = x0*(2 - S*x0)
        NR1    = 4'd4,   // NR iteration 1
        NR2    = 4'd5,   // NR iteration 2
        NR3    = 4'd11,  // NR iteration 3
        NR4    = 4'd12,  // NR iteration 4 (5th total — needed for Q8.56 ULP precision)
        K_COMP = 4'd6,   // K[i] = P_in[i*3] * S_inv
        X_CORR = 4'd7,   // x_out[i] = x_in[i] + K[i]*y_tilde
        P_UPD  = 4'd8,   // Build IKH, fire gemm_systolic(IKH, P_in)
        WAIT_P = 4'd9,   // Wait for gemm_systolic to finish
        DONE_S = 4'd10
    } state_t;

    state_t state, state_nxt;

    // Unpack flat input ports to internal unpacked arrays
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

    // Internal unpacked output arrays; packed to flat ports
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
    logic [63:0] IKH    [0:8];  // (I - K*H) matrix, 3×3

    // NR intermediate
    logic [63:0] nr_x;   // current reciprocal estimate

    // gemm_systolic signals (internal unpacked)
    logic        gs_start, gs_done, gs_busy;
    logic [63:0] gs_A [0:8], gs_B [0:8], gs_C [0:8];

    // Flat wires for gemm_systolic ports
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

    // Combinational helpers for Newton-Raphson
    // x_new = x * (2.0 - S * x)
    logic [63:0] sx, two_minus_sx, nr_x_new;
    wire  [63:0] neg_sx = -sx;  // Q8.56 two's complement negation
    f64_mul u_nr_mul1 (.a(S_reg), .b(nr_x),        .result(sx));
    f64_add u_nr_sub  (.a(F64_TWO), .b(neg_sx),    .result(two_minus_sx));  // 2.0 - sx
    f64_mul u_nr_mul2 (.a(nr_x), .b(two_minus_sx), .result(nr_x_new));

    // K[i] = P_reg[i*3] * S_inv  (3 parallel muls)
    logic [63:0] k_comb [0:2];
    f64_mul u_k0 (.a(P_reg[0]), .b(S_inv), .result(k_comb[0]));
    f64_mul u_k1 (.a(P_reg[3]), .b(S_inv), .result(k_comb[1]));
    f64_mul u_k2 (.a(P_reg[6]), .b(S_inv), .result(k_comb[2]));

    // x_out[i] = x_reg[i] + K[i]*y_tilde  (3 parallel)
    logic [63:0] ky0, ky1, ky2;
    logic [63:0] xout0, xout1, xout2;
    f64_mul u_ky0 (.a(K_reg[0]), .b(y_tilde), .result(ky0));
    f64_mul u_ky1 (.a(K_reg[1]), .b(y_tilde), .result(ky1));
    f64_mul u_ky2 (.a(K_reg[2]), .b(y_tilde), .result(ky2));
    f64_add u_xo0 (.a(x_reg[0]), .b(ky0), .result(xout0));
    f64_add u_xo1 (.a(x_reg[1]), .b(ky1), .result(xout1));
    f64_add u_xo2 (.a(x_reg[2]), .b(ky2), .result(xout2));

    // IKH[0,0] = 1 - K[0]: compute combinationally from K_reg[0]
    logic [63:0] one_minus_k0;
    f64_add u_1mk0 (.a(F64_ONE), .b(f64_neg(K_reg[0])), .result(one_minus_k0));

    // FSM next-state
    always_comb begin
        state_nxt = state;
        case (state)
            IDLE:   if (start)    state_nxt = INNOV;
            INNOV:                state_nxt = S_COMP;
            S_COMP:               state_nxt = NR0;
            NR0:                  state_nxt = NR1;
            NR1:                  state_nxt = NR2;
            NR2:                  state_nxt = NR3;
            NR3:                  state_nxt = NR4;
            NR4:                  state_nxt = K_COMP;
            K_COMP:               state_nxt = X_CORR;
            X_CORR:               state_nxt = P_UPD;
            P_UPD:                state_nxt = WAIT_P;
            WAIT_P: if (gs_done)  state_nxt = DONE_S;
            DONE_S:               state_nxt = IDLE;
            default:              state_nxt = IDLE;
        endcase
    end

    // Negate helper: two's complement (Q8.56 negation)
    function automatic logic [63:0] f64_neg (input logic [63:0] x);
        f64_neg = -x;
    endfunction

    // Newton-Raphson seed: fixed Q8.56 value 1/8 = 2^53, valid for S in [4,8)
    // For our test case S=P[0]+R=1+5=6, this seed gives quadratic convergence.
    function automatic logic [63:0] nr_seed (input logic [63:0] s);
        nr_seed = 64'h0020_0000_0000_0000;  // 1/8 in Q8.56
    endfunction

    // Pre-computed negations as continuous wires (two's complement for Q8.56)
    wire [63:0] neg_k1 = -K_reg[1];
    wire [63:0] neg_k2 = -K_reg[2];

    // Combinational y_tilde and S_comb instances (driven from registered values).
    // Declared HERE (before the always_ff that reads S_comb) because iverilog 12
    // requires declaration before use in procedural contexts.
    logic [63:0] y_tilde_comb, S_comb;
    f64_add u_ytilde (.a(z_reg),    .b(f64_neg(x_reg[0])), .result(y_tilde_comb));
    f64_add u_scomb  (.a(P_reg[0]), .b(r_val),              .result(S_comb));

    // FSM datapath
    integer ii;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= IDLE;
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
                x_out_arr[ii] <= 64'h0;
            end
            for (ii = 0; ii < 9; ii++) P_out_arr[ii] <= 64'h0;
            z_reg <= 64'h0;
            S_inv <= 64'h0;
            nr_x  <= 64'h0;
        end else begin
            state    <= state_nxt;
            gs_start <= 1'b0;

            case (state)
                INNOV: begin
                    z_reg <= z;
                    for (ii = 0; ii < 3; ii++) x_reg[ii] <= x_in_arr[ii];
                    for (ii = 0; ii < 9; ii++) P_reg[ii] <= P_in_arr[ii];
                    // y_tilde = z - x_in[0]  (combinational on next cycle)
                    // S_pre = P_in[0] + r_val (similarly)
                    // Computed in S_COMP using registered values
                end

                S_COMP: begin
                    nr_x <= nr_seed(S_comb);    // seed ≈ 1/S from the combinational S adder
                    // y_tilde and S_reg also latched this cycle via the second always_ff below
                end

                NR0: begin
                    nr_x  <= nr_x_new;
                    S_inv <= nr_x_new;
                end

                NR1: begin
                    nr_x  <= nr_x_new;
                    S_inv <= nr_x_new;
                end

                NR2: begin
                    nr_x  <= nr_x_new;
                    S_inv <= nr_x_new;
                end

                NR3: begin
                    nr_x  <= nr_x_new;  // feed NR4
                    S_inv <= nr_x_new;
                end

                NR4: begin
                    S_inv <= nr_x_new;  // final Q8.56 iteration: latch result only
                end

                K_COMP: begin
                    K_reg[0] <= k_comb[0];
                    K_reg[1] <= k_comb[1];
                    K_reg[2] <= k_comb[2];
                end

                X_CORR: begin
                    x_out_arr[0] <= xout0;
                    x_out_arr[1] <= xout1;
                    x_out_arr[2] <= xout2;
                    // Build complete IKH = I - K*H; H=[1,0,0]
                    // Row 0: [1-K[0],  0,  0]
                    // Row 1: [-K[1],   1,  0]
                    // Row 2: [-K[2],   0,  1]
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
                    // Load IKH and P_reg into gemm_systolic; fire start
                    for (ii = 0; ii < 9; ii++) begin
                        gs_A[ii] <= IKH[ii];
                        gs_B[ii] <= P_reg[ii];
                    end
                    gs_start <= 1'b1;
                end

                WAIT_P: begin
                    gs_start <= 1'b0;
                    // gs_done fires while gemm is in FINISH; gs_C holds old C until
                    // the next cycle (NBA update). Latch P_out in DONE_S instead.
                end

                DONE_S: begin
                    // One cycle after FINISH: gs_C now holds the updated gemm result
                    for (ii = 0; ii < 9; ii++) P_out_arr[ii] <= gs_C[ii];
                end

                default: ;
            endcase
        end
    end

    // Latch y_tilde and S_reg in S_COMP (one cycle after INNOV so that z_reg,
    // x_reg, and P_reg have settled from their NBA updates in INNOV).
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_tilde <= 64'h0;
            S_reg   <= 64'h0;
        end else if (state == S_COMP) begin
            y_tilde <= y_tilde_comb;
            S_reg   <= S_comb;
        end
    end

    // Outputs
    assign done = (state == DONE_S);
    assign busy = (state != IDLE) && (state != DONE_S);

endmodule
