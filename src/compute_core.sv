`timescale 1ns/1ps
// =============================================================================
// compute_core.sv — F32 + Time-multiplexed compute core for tt_um_joram200
// Task 6 (iteration 3): Inline gemm into kalman_update.
//
// vs iter2: gemm_systolic module eliminated entirely; its 27-cycle GEMM
//   computation folded into kalman FSM. This removes 1 f32_mul + 1 f32_add
//   instance (largest single reduction possible).
//
//   Also: x_reg, z_reg, P_reg eliminated — ports read directly (inputs are
//   stable from SPI write through core_done). INNOV state removed.
//
// Module hierarchy:
//   f32_mul       — 1 instance: shared across NR, K, KY, and GEMM_COMPUTE
//   f32_add       — 1 instance: shared across S_COMP, X_ADD, NR_chain, GEMM_COMPUTE
//   kalman_update — FSM with inlined GEMM (27-cycle GEMM_COMPUTE state)
//
// FSM (sub-cycles in parens):
//   IDLE → S_COMP(2) → NR0(2) → NR1(2) → NR2(2) → NR3(2)
//   → K_COMP(3) → KY_COMP(3) → X_ADD(4)
//   → GEMM_INIT(1) → GEMM_COMPUTE(27) → DONE_S(1) → IDLE
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
// kalman_update — Kalman filter update with inlined 3×3 GEMM (F32, iter3)
// H = [1, 0, 0] (scalar measurement)
//
// Single shared f32_mul and f32_add handle all operations:
//   NR iterations, K_COMP, KY_COMP, GEMM_COMPUTE, X_ADD, S_COMP
//
// GEMM_COMPUTE (27 cycles): C = IKH × P_in
//   counter order: k outer (0..2), i middle (0..2), j inner (0..2)
//   each cycle: acc[i*3+j] += IKH_elem[i*3+k] × P_in_elem[k*3+j]
//   IKH and P_in are available as combinational wires (from K_reg + P_in port).
//
// Inputs x_in, P_in, z read directly from ports (stable after SPI write).
// Ports x_in, P_in, x_out, P_out are flat packed for Yosys compatibility.
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

    // -------------------------------------------------------------------------
    // FSM states
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        IDLE         = 4'd0,
        S_COMP       = 4'd1,   // 2 sub: y_tilde then S_reg+nr_x seed
        NR0          = 4'd2,   // NR iteration 0 (2 sub-cycles)
        NR1          = 4'd3,   // NR iteration 1
        NR2          = 4'd4,   // NR iteration 2
        NR3          = 4'd5,   // NR iteration 3 (final)
        K_COMP       = 4'd6,   // K[i] = P[i*3] * S_inv (3 sub-cycles)
        KY_COMP      = 4'd7,   // ky[i] = K[i] * y_tilde (3 sub-cycles)
        X_ADD        = 4'd8,   // 4 sub: x_out[0..2] then one_minus_k0_r
        GEMM_INIT    = 4'd9,   // zero acc, reset k/i/j counters
        GEMM_COMPUTE = 4'd10,  // 27 cycles: acc[i*3+j] += IKH[i*3+k]*P[k*3+j]
        DONE_S       = 4'd11
    } state_t;

    state_t state, state_nxt;
    logic [1:0] sub_cnt, sub_nxt;

    // -------------------------------------------------------------------------
    // Unpack flat input ports to wire arrays (read directly — no registered copies)
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    // Output packing
    // -------------------------------------------------------------------------
    logic [31:0] x_out_arr [0:2];
    generate
        for (ku = 0; ku < 3; ku++) begin : pack_xout
            assign x_out[ku*32 +: 32] = x_out_arr[ku];
        end
    endgenerate

    // P_out driven from acc (GEMM output accumulator) — stable after GEMM_COMPUTE
    logic [31:0] acc [0:8];
    generate
        for (ku = 0; ku < 9; ku++) begin : pack_pout
            assign P_out[ku*32 +: 32] = acc[ku];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Registered intermediate values (minimal set)
    // -------------------------------------------------------------------------
    logic [31:0] y_tilde;      // z - x_in[0]
    logic [31:0] S_reg;        // P_in[0] + r_val
    logic [31:0] S_inv;        // 1/S via Newton-Raphson
    logic [31:0] nr_x;         // NR iterate
    logic [31:0] sx_reg;       // S * nr_x (intermediate for NR)
    logic [31:0] K_reg [0:2];  // Kalman gain K[i] = P[i*3]*S_inv
    logic [31:0] ky_arr [0:2]; // K[i] * y_tilde
    logic [31:0] one_minus_k0_r; // 1 - K[0], used as IKH[0,0]

    // GEMM counters: k outer (0..2), i middle (0..2), j inner (0..2)
    logic [1:0] k_cnt, i_cnt, j_cnt;

    // -------------------------------------------------------------------------
    // IKH matrix — combinational from K_reg and one_minus_k0_r
    // IKH = (I - K*H), H=[1,0,0]:
    //   [0,0]=1-K[0]  [0,1]=0       [0,2]=0
    //   [1,0]=-K[1]   [1,1]=1       [1,2]=0
    //   [2,0]=-K[2]   [2,1]=0       [2,2]=1
    // Stored row-major: IKH_arr[i*3+j]
    // -------------------------------------------------------------------------
    logic [31:0] IKH_arr [0:8];
    assign IKH_arr[0] = one_minus_k0_r;
    assign IKH_arr[1] = 32'h0;
    assign IKH_arr[2] = 32'h0;
    assign IKH_arr[3] = {~K_reg[1][31], K_reg[1][30:0]};  // -K[1]
    assign IKH_arr[4] = F32_ONE;
    assign IKH_arr[5] = 32'h0;
    assign IKH_arr[6] = {~K_reg[2][31], K_reg[2][30:0]};  // -K[2]
    assign IKH_arr[7] = 32'h0;
    assign IKH_arr[8] = F32_ONE;

    // -------------------------------------------------------------------------
    // GEMM index computation: 4-bit to avoid overflow (max index = 8)
    // IKH element: row i, col k → index i*3+k
    // P_in element: row k, col j → index k*3+j
    // acc element:  row i, col j → index i*3+j
    // -------------------------------------------------------------------------
    logic [3:0] ikha_idx, pb_idx, acc_idx;
    assign ikha_idx = {2'b0, i_cnt} * 4'd3 + {2'b0, k_cnt};
    assign pb_idx   = {2'b0, k_cnt} * 4'd3 + {2'b0, j_cnt};
    assign acc_idx  = {2'b0, i_cnt} * 4'd3 + {2'b0, j_cnt};

    // 9-to-1 muxes for GEMM element selection (avoids dynamic bit-selects)
    logic [31:0] ikha_elem, pb_elem;
    always_comb begin
        case (ikha_idx)
            4'd0: ikha_elem = IKH_arr[0];
            4'd1: ikha_elem = IKH_arr[1];
            4'd2: ikha_elem = IKH_arr[2];
            4'd3: ikha_elem = IKH_arr[3];
            4'd4: ikha_elem = IKH_arr[4];
            4'd5: ikha_elem = IKH_arr[5];
            4'd6: ikha_elem = IKH_arr[6];
            4'd7: ikha_elem = IKH_arr[7];
            4'd8: ikha_elem = IKH_arr[8];
            default: ikha_elem = 32'h0;
        endcase
    end
    always_comb begin
        case (pb_idx)
            4'd0: pb_elem = P_in_arr[0];
            4'd1: pb_elem = P_in_arr[1];
            4'd2: pb_elem = P_in_arr[2];
            4'd3: pb_elem = P_in_arr[3];
            4'd4: pb_elem = P_in_arr[4];
            4'd5: pb_elem = P_in_arr[5];
            4'd6: pb_elem = P_in_arr[6];
            4'd7: pb_elem = P_in_arr[7];
            4'd8: pb_elem = P_in_arr[8];
            default: pb_elem = 32'h0;
        endcase
    end
    // acc read mux
    logic [31:0] acc_elem;
    always_comb begin
        case (acc_idx)
            4'd0: acc_elem = acc[0];
            4'd1: acc_elem = acc[1];
            4'd2: acc_elem = acc[2];
            4'd3: acc_elem = acc[3];
            4'd4: acc_elem = acc[4];
            4'd5: acc_elem = acc[5];
            4'd6: acc_elem = acc[6];
            4'd7: acc_elem = acc[7];
            4'd8: acc_elem = acc[8];
            default: acc_elem = 32'h0;
        endcase
    end

    // -------------------------------------------------------------------------
    // 1 shared f32_mul + 1 shared f32_add
    // -------------------------------------------------------------------------
    logic [31:0] shared_mul_a, shared_mul_b, shared_mul_r;
    logic [31:0] shared_add_a, shared_add_b, shared_add_r;
    f32_mul u_shared_mul (.a(shared_mul_a), .b(shared_mul_b), .result(shared_mul_r));
    f32_add u_shared_add (.a(shared_add_a), .b(shared_add_b), .result(shared_add_r));

    // Newton-Raphson seed for F32 reciprocal
    function automatic logic [31:0] nr_seed (input logic [31:0] s);
        nr_seed = 32'h7EF1_27EA - s;
    endfunction

    // GEMM all-done flag
    wire gemm_done = (k_cnt == 2'd2) && (i_cnt == 2'd2) && (j_cnt == 2'd2);

    // -------------------------------------------------------------------------
    // Shared adder mux
    // NR sub=1: output feeds shared_mul b (combinational chain)
    // GEMM_COMPUTE: acc_elem + shared_mul_r (multiply-accumulate in 1 cycle)
    // -------------------------------------------------------------------------
    always_comb begin
        shared_add_a = 32'h0;
        shared_add_b = 32'h0;
        case (state)
            S_COMP: begin
                if (sub_cnt == 2'd0) begin
                    shared_add_a = z;
                    shared_add_b = {~x_in_arr[0][31], x_in_arr[0][30:0]};  // z - x_in[0]
                end else begin
                    shared_add_a = P_in_arr[0];
                    shared_add_b = r_val;   // S = P[0,0] + R
                end
            end
            NR0, NR1, NR2, NR3: begin
                if (sub_cnt == 2'd1) begin
                    // two_minus_sx = 2.0 - sx_reg (feeds mul b-input, same cycle)
                    shared_add_a = F32_TWO;
                    shared_add_b = {~sx_reg[31], sx_reg[30:0]};
                end
            end
            X_ADD: begin
                case (sub_cnt)
                    2'd0: begin shared_add_a = x_in_arr[0]; shared_add_b = ky_arr[0]; end
                    2'd1: begin shared_add_a = x_in_arr[1]; shared_add_b = ky_arr[1]; end
                    2'd2: begin shared_add_a = x_in_arr[2]; shared_add_b = ky_arr[2]; end
                    2'd3: begin
                        shared_add_a = F32_ONE;
                        shared_add_b = {~K_reg[0][31], K_reg[0][30:0]};  // 1 - K[0]
                    end
                endcase
            end
            GEMM_COMPUTE: begin
                // MAC: acc[i*3+j] += IKH[i*3+k] * P[k*3+j]
                shared_add_a = acc_elem;
                shared_add_b = shared_mul_r;  // product from current cycle
            end
            default: ;
        endcase
    end

    // -------------------------------------------------------------------------
    // Shared multiplier mux
    // -------------------------------------------------------------------------
    always_comb begin
        shared_mul_a = 32'h0;
        shared_mul_b = 32'h0;
        case (state)
            NR0, NR1, NR2, NR3: begin
                if (sub_cnt == 2'd0) begin
                    shared_mul_a = S_reg;   // sub 0: S * nr_x → sx_reg
                    shared_mul_b = nr_x;
                end else begin
                    shared_mul_a = nr_x;            // sub 1: nr_x * (2-sx) → new nr_x
                    shared_mul_b = shared_add_r;    // two_minus_sx (combinational chain)
                end
            end
            K_COMP: begin
                shared_mul_b = S_inv;
                case (sub_cnt)
                    2'd0: shared_mul_a = P_in_arr[0];
                    2'd1: shared_mul_a = P_in_arr[3];
                    2'd2: shared_mul_a = P_in_arr[6];
                    default: shared_mul_a = 32'h0;
                endcase
            end
            KY_COMP: begin
                shared_mul_b = y_tilde;
                case (sub_cnt)
                    2'd0: shared_mul_a = K_reg[0];
                    2'd1: shared_mul_a = K_reg[1];
                    2'd2: shared_mul_a = K_reg[2];
                    default: shared_mul_a = 32'h0;
                endcase
            end
            GEMM_COMPUTE: begin
                // ikha_elem × pb_elem; result feeds shared_add (MAC)
                shared_mul_a = ikha_elem;
                shared_mul_b = pb_elem;
            end
            default: ;
        endcase
    end

    // -------------------------------------------------------------------------
    // FSM next-state + sub_nxt
    // -------------------------------------------------------------------------
    always_comb begin
        state_nxt = state;
        sub_nxt   = sub_cnt + 2'd1;

        case (state)
            IDLE:         if (start) begin state_nxt = S_COMP;       sub_nxt = 2'd0; end
                          else             sub_nxt = 2'd0;
            S_COMP:       if (sub_cnt == 2'd1) begin state_nxt = NR0;        sub_nxt = 2'd0; end
            NR0:          if (sub_cnt == 2'd1) begin state_nxt = NR1;        sub_nxt = 2'd0; end
            NR1:          if (sub_cnt == 2'd1) begin state_nxt = NR2;        sub_nxt = 2'd0; end
            NR2:          if (sub_cnt == 2'd1) begin state_nxt = NR3;        sub_nxt = 2'd0; end
            NR3:          if (sub_cnt == 2'd1) begin state_nxt = K_COMP;     sub_nxt = 2'd0; end
            K_COMP:       if (sub_cnt == 2'd2) begin state_nxt = KY_COMP;    sub_nxt = 2'd0; end
            KY_COMP:      if (sub_cnt == 2'd2) begin state_nxt = X_ADD;      sub_nxt = 2'd0; end
            X_ADD:        if (sub_cnt == 2'd3) begin state_nxt = GEMM_INIT;  sub_nxt = 2'd0; end
            GEMM_INIT:    begin state_nxt = GEMM_COMPUTE; sub_nxt = 2'd0; end
            GEMM_COMPUTE: if (gemm_done) begin state_nxt = DONE_S;      sub_nxt = 2'd0; end
                          else               sub_nxt = 2'd0;
            DONE_S:       begin state_nxt = IDLE;         sub_nxt = 2'd0; end
            default:      begin state_nxt = IDLE;         sub_nxt = 2'd0; end
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
            y_tilde  <= 32'h0;
            S_reg    <= 32'h0;
            S_inv    <= 32'h0;
            nr_x     <= 32'h0;
            sx_reg   <= 32'h0;
            for (ii = 0; ii < 3; ii++) begin
                K_reg[ii]     <= 32'h0;
                ky_arr[ii]    <= 32'h0;
                x_out_arr[ii] <= 32'h0;
            end
            one_minus_k0_r <= 32'h0;
            k_cnt <= 2'd0; i_cnt <= 2'd0; j_cnt <= 2'd0;
            for (ii = 0; ii < 9; ii++) acc[ii] <= 32'h0;
        end else begin
            state   <= state_nxt;
            sub_cnt <= sub_nxt;

            case (state)

                // S_COMP: 2 sub-cycles using shared adder
                // sub 0: y_tilde = z - x_in[0]     (ports read directly)
                // sub 1: S_reg = P_in[0] + r_val;  nr_x = nr_seed(S_reg)
                S_COMP: begin
                    if (sub_cnt == 2'd0) begin
                        y_tilde <= shared_add_r;
                    end else begin
                        S_reg <= shared_add_r;
                        nr_x  <= nr_seed(shared_add_r);
                    end
                end

                // NR0..NR2: sub 0 → sx=S*nr_x; sub 1 → nr_x = nr_x*(2-sx)
                NR0, NR1, NR2: begin
                    if (sub_cnt == 2'd0) begin
                        sx_reg <= shared_mul_r;
                    end else begin
                        nr_x  <= shared_mul_r;
                        S_inv <= shared_mul_r;
                    end
                end

                NR3: begin
                    if (sub_cnt == 2'd0) begin
                        sx_reg <= shared_mul_r;
                    end else begin
                        S_inv <= shared_mul_r;  // final; don't update nr_x
                    end
                end

                // K_COMP: 3 sub-cycles — K[i] = P_in[i*3] * S_inv
                K_COMP: begin
                    case (sub_cnt)
                        2'd0: K_reg[0] <= shared_mul_r;
                        2'd1: K_reg[1] <= shared_mul_r;
                        2'd2: K_reg[2] <= shared_mul_r;
                        default: ;
                    endcase
                end

                // KY_COMP: 3 sub-cycles — ky[i] = K[i] * y_tilde
                KY_COMP: begin
                    case (sub_cnt)
                        2'd0: ky_arr[0] <= shared_mul_r;
                        2'd1: ky_arr[1] <= shared_mul_r;
                        2'd2: ky_arr[2] <= shared_mul_r;
                        default: ;
                    endcase
                end

                // X_ADD: 4 sub-cycles using shared adder
                // sub 0-2: x_out[i] = x_in[i] + ky[i]
                // sub 3:   one_minus_k0_r = 1 - K[0]  (for IKH[0,0])
                X_ADD: begin
                    case (sub_cnt)
                        2'd0: x_out_arr[0] <= shared_add_r;
                        2'd1: x_out_arr[1] <= shared_add_r;
                        2'd2: x_out_arr[2] <= shared_add_r;
                        2'd3: one_minus_k0_r <= shared_add_r;
                    endcase
                end

                // GEMM_INIT: zero accumulators and counters
                GEMM_INIT: begin
                    for (ii = 0; ii < 9; ii++) acc[ii] <= 32'h0;
                    k_cnt <= 2'd0; i_cnt <= 2'd0; j_cnt <= 2'd0;
                end

                // GEMM_COMPUTE: 27 cycles
                // Each cycle: product = ikha_elem * pb_elem (shared_mul)
                //             new_acc = acc_elem + product   (shared_add, chains mul)
                //             acc[acc_idx] <= new_acc
                // Counter advance: j fastest, then i, then k
                GEMM_COMPUTE: begin
                    // Write accumulator — use acc_idx registered at start of this cycle
                    case (acc_idx)
                        4'd0: acc[0] <= shared_add_r;
                        4'd1: acc[1] <= shared_add_r;
                        4'd2: acc[2] <= shared_add_r;
                        4'd3: acc[3] <= shared_add_r;
                        4'd4: acc[4] <= shared_add_r;
                        4'd5: acc[5] <= shared_add_r;
                        4'd6: acc[6] <= shared_add_r;
                        4'd7: acc[7] <= shared_add_r;
                        4'd8: acc[8] <= shared_add_r;
                        default: ;
                    endcase

                    // Advance counters: j fastest, then i, then k
                    if (j_cnt == 2'd2) begin
                        j_cnt <= 2'd0;
                        if (i_cnt == 2'd2) begin
                            i_cnt <= 2'd0;
                            if (k_cnt != 2'd2) k_cnt <= k_cnt + 2'd1;
                        end else begin
                            i_cnt <= i_cnt + 2'd1;
                        end
                    end else begin
                        j_cnt <= j_cnt + 2'd1;
                    end
                end

                // DONE_S: acc holds P_out; x_out_arr holds x_out.
                // Both wired combinationally to output ports — no copy needed.
                DONE_S: ;

                default: ;
            endcase
        end
    end

    assign done = (state == DONE_S);
    assign busy = (state != IDLE) && (state != DONE_S);

endmodule
