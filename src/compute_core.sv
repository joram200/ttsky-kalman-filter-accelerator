`timescale 1ns/1ps
// =============================================================================
// compute_core.sv — F32 + Time-multiplexed compute core for tt_um_joram200
// Task 6 (iteration 4): Bit-serial f32_mul to eliminate 24×24 combinational
//   multiplier (~3 k cells) and replace with shift-add loop (~400 cells).
//
// vs iter3: f32_mul module changed from 1-cycle combinational to a 26-cycle
//   sequential shift-and-add implementation.  The kalman_update FSM now stalls
//   in multiply-bound states until mul_done_w is asserted.  f32_add remains
//   combinational (adder area is small).
//
// Timing (approx, at 50 MHz):
//   NR×4: ~220 cyc; K/KY_COMP: 81 cyc each; GEMM: ~703 cyc → ~1100 cyc total
//   1100 / 50e6 = 22 µs — well within the SPI transfer window (~300 µs).
//
// Module hierarchy:
//   f32_mul_seq   — 1 instance: bit-serial 26-cycle multiplier
//   f32_add       — 1 instance: combinational adder (unchanged)
//   kalman_update — FSM with inlined GEMM, mul handshake
//
// FSM (state durations with sequential mul):
//   IDLE → S_COMP(2) → NR0..NR3(~55 cyc each) → K_COMP(81) → KY_COMP(81)
//   → X_ADD(4) → GEMM_INIT(1) → GEMM_COMPUTE(703) → DONE_S(1) → IDLE
// =============================================================================

// -----------------------------------------------------------------------------
// f32_mul_seq — Sequential IEEE-754 single-precision multiplier (shift-and-add)
//
// Latency: 26 clock cycles from start to done.
//   Cycle 0      : start=1; capture inputs; init shift registers.
//   Cycles 1..24 : 24 shift-add steps (LSB-first of a_man).
//   Cycle 25     : normalize & register result; assert done for 1 cycle.
//   Cycle 26     : done=1, result valid.
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

    // Registered per-operation flags / exponent (captured on start)
    logic        r_sign_r;
    logic signed [9:0] exp_sum_r;
    logic        a_nan_r, b_nan_r, a_inf_r, b_inf_r, a_zero_r, b_zero_r;

    // Shift-add datapath
    logic [47:0] accum;       // running product accumulator
    logic [47:0] b_shifted;   // b_man shifted left each cycle
    logic [23:0] a_shifted;   // a_man shifted right each cycle (LSB tested)
    logic [4:0]  bit_cnt;     // 0..24 (0..23 = shift-add; 24 = normalize)
    logic        busy;

    // Combinational next-accumulator (used during shift-add cycles)
    logic [47:0] new_accum;
    assign new_accum = a_shifted[0] ? (accum + b_shifted) : accum;

    // Combinational normalize (used on bit_cnt==24, reading registered accum)
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
            busy      <= 1'b0;
            done      <= 1'b0;
            result    <= 32'h0;
            bit_cnt   <= 5'd0;
            accum     <= 48'h0;
            b_shifted <= 48'h0;
            a_shifted <= 24'h0;
            r_sign_r  <= 1'b0;
            exp_sum_r <= 10'h0;
            a_nan_r   <= 1'b0; b_nan_r  <= 1'b0;
            a_inf_r   <= 1'b0; b_inf_r  <= 1'b0;
            a_zero_r  <= 1'b0; b_zero_r <= 1'b0;
        end else begin
            done <= 1'b0;  // default: deassert

            if (start && !busy) begin
                // Capture sign, exponent sum, special-case flags
                r_sign_r  <= a[31] ^ b[31];
                exp_sum_r <= $signed({2'b00, a[30:23]})
                           + $signed({2'b00, b[30:23]})
                           - $signed(10'd127);
                a_nan_r   <= (a[30:23] == 8'hFF) && (a[22:0] != 23'h0);
                b_nan_r   <= (b[30:23] == 8'hFF) && (b[22:0] != 23'h0);
                a_inf_r   <= (a[30:23] == 8'hFF) && (a[22:0] == 23'h0);
                b_inf_r   <= (b[30:23] == 8'hFF) && (b[22:0] == 23'h0);
                a_zero_r  <= (a[30:23] == 8'h0)  && (a[22:0] == 23'h0);
                b_zero_r  <= (b[30:23] == 8'h0)  && (b[22:0] == 23'h0);
                // Initialise shift registers with implicit-1 mantissas
                a_shifted <= (a[30:23] == 8'h0) ? {1'b0, a[22:0]} : {1'b1, a[22:0]};
                b_shifted <= {24'b0, ((b[30:23] == 8'h0) ? {1'b0, b[22:0]} : {1'b1, b[22:0]})};
                accum     <= 48'h0;
                bit_cnt   <= 5'd0;
                busy      <= 1'b1;

            end else if (busy) begin
                if (bit_cnt < 5'd24) begin
                    // Shift-and-add step
                    accum     <= new_accum;
                    b_shifted <= {b_shifted[46:0], 1'b0};  // shift b_man left
                    a_shifted <= {1'b0, a_shifted[23:1]};   // shift a_man right
                    bit_cnt   <= bit_cnt + 5'd1;
                end else begin
                    // bit_cnt == 24: accum holds complete product — normalise
                    result  <= final_result;
                    done    <= 1'b1;
                    busy    <= 1'b0;
                    bit_cnt <= 5'd0;
                end
            end
        end
    end

endmodule

// -----------------------------------------------------------------------------
// f32_add — Combinational IEEE-754 single-precision adder/subtractor
// (unchanged from iter3)
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
// kalman_update — Kalman filter update with inlined 3×3 GEMM (F32, iter4)
// H = [1, 0, 0] (scalar measurement)
//
// Single shared f32_mul_seq (sequential) and f32_add (combinational).
//
// Multiply handshake:
//   mul_start_w  — combinational pulse: asserted for exactly 1 cycle when
//                  the FSM is in a mul-needing state and mul_active=0.
//   mul_active   — registered: set on mul_start_w, cleared on mul_done_w.
//   mul_done_w   — from f32_mul_seq: 1-cycle pulse 26 cycles after start.
//   mul_result   — from f32_mul_seq: valid when mul_done_w=1.
//
// States that stall until mul_done_w:
//   NR0..NR3  sub_cnt 0 (first mul S*nr_x) and sub_cnt 2 (second mul nr_x*(2-sx))
//   K_COMP    sub_cnt 0, 1, 2 (one mul per K element)
//   KY_COMP   sub_cnt 0, 1, 2 (one mul per ky element)
//   GEMM_COMPUTE (one mul per MAC, k/i/j advance on mul_done_w)
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
        S_COMP       = 4'd1,   // 2 sub: y_tilde, then S+r seed
        NR0          = 4'd2,   // NR iteration 0 (3 sub-cycles with mul stalls)
        NR1          = 4'd3,
        NR2          = 4'd4,
        NR3          = 4'd5,
        K_COMP       = 4'd6,   // 3 sub-cycles, each stalls until mul_done
        KY_COMP      = 4'd7,
        X_ADD        = 4'd8,   // 4 sub cycles (adder only)
        GEMM_INIT    = 4'd9,
        GEMM_COMPUTE = 4'd10,  // 27 MACs, each stalls until mul_done
        DONE_S       = 4'd11
    } state_t;

    state_t state, state_nxt;
    logic [1:0] sub_cnt, sub_nxt;

    // -------------------------------------------------------------------------
    // Unpack flat input ports to wire arrays
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

    logic [31:0] acc [0:8];
    generate
        for (ku = 0; ku < 9; ku++) begin : pack_pout
            assign P_out[ku*32 +: 32] = acc[ku];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Registered intermediate values
    // -------------------------------------------------------------------------
    logic [31:0] y_tilde;
    logic [31:0] S_reg;
    logic [31:0] S_inv;
    logic [31:0] nr_x;
    logic [31:0] sx_reg;
    logic [31:0] K_reg [0:2];
    logic [31:0] ky_arr [0:2];
    logic [31:0] one_minus_k0_r;

    // GEMM counters: k outer (0..2), i middle (0..2), j inner (0..2)
    logic [1:0] k_cnt, i_cnt, j_cnt;

    // -------------------------------------------------------------------------
    // IKH matrix (combinational from K_reg)
    // -------------------------------------------------------------------------
    logic [31:0] IKH_arr [0:8];
    assign IKH_arr[0] = one_minus_k0_r;
    assign IKH_arr[1] = 32'h0;
    assign IKH_arr[2] = 32'h0;
    assign IKH_arr[3] = {~K_reg[1][31], K_reg[1][30:0]};
    assign IKH_arr[4] = F32_ONE;
    assign IKH_arr[5] = 32'h0;
    assign IKH_arr[6] = {~K_reg[2][31], K_reg[2][30:0]};
    assign IKH_arr[7] = 32'h0;
    assign IKH_arr[8] = F32_ONE;

    // -------------------------------------------------------------------------
    // GEMM index computation
    // -------------------------------------------------------------------------
    logic [3:0] ikha_idx, pb_idx, acc_idx;
    assign ikha_idx = {2'b0, i_cnt} * 4'd3 + {2'b0, k_cnt};
    assign pb_idx   = {2'b0, k_cnt} * 4'd3 + {2'b0, j_cnt};
    assign acc_idx  = {2'b0, i_cnt} * 4'd3 + {2'b0, j_cnt};

    logic [31:0] ikha_elem, pb_elem;
    always_comb begin
        case (ikha_idx)
            4'd0: ikha_elem = IKH_arr[0]; 4'd1: ikha_elem = IKH_arr[1];
            4'd2: ikha_elem = IKH_arr[2]; 4'd3: ikha_elem = IKH_arr[3];
            4'd4: ikha_elem = IKH_arr[4]; 4'd5: ikha_elem = IKH_arr[5];
            4'd6: ikha_elem = IKH_arr[6]; 4'd7: ikha_elem = IKH_arr[7];
            4'd8: ikha_elem = IKH_arr[8];
            default: ikha_elem = 32'h0;
        endcase
    end
    always_comb begin
        case (pb_idx)
            4'd0: pb_elem = P_in_arr[0]; 4'd1: pb_elem = P_in_arr[1];
            4'd2: pb_elem = P_in_arr[2]; 4'd3: pb_elem = P_in_arr[3];
            4'd4: pb_elem = P_in_arr[4]; 4'd5: pb_elem = P_in_arr[5];
            4'd6: pb_elem = P_in_arr[6]; 4'd7: pb_elem = P_in_arr[7];
            4'd8: pb_elem = P_in_arr[8];
            default: pb_elem = 32'h0;
        endcase
    end
    logic [31:0] acc_elem;
    always_comb begin
        case (acc_idx)
            4'd0: acc_elem = acc[0]; 4'd1: acc_elem = acc[1];
            4'd2: acc_elem = acc[2]; 4'd3: acc_elem = acc[3];
            4'd4: acc_elem = acc[4]; 4'd5: acc_elem = acc[5];
            4'd6: acc_elem = acc[6]; 4'd7: acc_elem = acc[7];
            4'd8: acc_elem = acc[8];
            default: acc_elem = 32'h0;
        endcase
    end

    // -------------------------------------------------------------------------
    // Shared sequential multiplier + combinational adder
    // -------------------------------------------------------------------------
    logic [31:0] mul_a, mul_b;
    logic        mul_start_w;
    logic [31:0] mul_result;
    logic        mul_done_w;
    logic        mul_active;   // registered: 1 while mul is running

    f32_mul_seq u_shared_mul (
        .clk(clk), .rst_n(rst_n),
        .start(mul_start_w),
        .a(mul_a), .b(mul_b),
        .result(mul_result),
        .done(mul_done_w)
    );

    logic [31:0] shared_add_a, shared_add_b, shared_add_r;
    f32_add u_shared_add (.a(shared_add_a), .b(shared_add_b), .result(shared_add_r));

    // Newton-Raphson seed
    function automatic logic [31:0] nr_seed (input logic [31:0] s);
        nr_seed = 32'h7EF1_27EA - s;
    endfunction

    // -------------------------------------------------------------------------
    // mul_start_w — combinational: start a multiply when state needs one
    //   and no multiply is currently running.
    //
    // For NR states: fire on sub_cnt==0 (first mul S*nr_x) and sub_cnt==2
    //   (second mul nr_x*(2-sx)), NOT on sub_cnt==1 (gap cycle for adder).
    // For K_COMP, KY_COMP: fire on every sub_cnt when not active.
    // For GEMM_COMPUTE: fire unless counters are at last-MAC position
    //   AND that last MAC is already done (detected by mul_active transitioning
    //   away).  Guard: only start if k/i/j not at final position or mul_active.
    //   Simpler guard: don't start when all counters are at max AND !mul_active
    //   (which means the last mul already fired and we're about to leave).
    // -------------------------------------------------------------------------
    logic gemm_last;
    assign gemm_last = (k_cnt == 2'd2) && (i_cnt == 2'd2) && (j_cnt == 2'd2);

    always_comb begin
        mul_start_w = 1'b0;
        if (!mul_active) begin
            case (state)
                NR0, NR1, NR2, NR3:
                    if (sub_cnt == 2'd0 || sub_cnt == 2'd2) mul_start_w = 1'b1;
                K_COMP:    mul_start_w = 1'b1;
                KY_COMP:   mul_start_w = 1'b1;
                GEMM_COMPUTE:
                    // Start every MAC including the last one (k=i=j=2).
                    // Termination is in next-state: mul_done_w && gemm_last → DONE_S.
                    mul_start_w = 1'b1;
                default: ;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // mul_a / mul_b mux
    // -------------------------------------------------------------------------
    always_comb begin
        mul_a = 32'h0;
        mul_b = 32'h0;
        case (state)
            NR0, NR1, NR2, NR3: begin
                if (sub_cnt == 2'd0) begin
                    mul_a = S_reg;
                    mul_b = nr_x;
                end else begin
                    // sub_cnt == 2: nr_x * (2 - sx_reg)
                    mul_a = nr_x;
                    mul_b = shared_add_r;   // F32_TWO - sx_reg (combinational)
                end
            end
            K_COMP: begin
                mul_b = S_inv;
                case (sub_cnt)
                    2'd0: mul_a = P_in_arr[0];
                    2'd1: mul_a = P_in_arr[3];
                    2'd2: mul_a = P_in_arr[6];
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
            GEMM_COMPUTE: begin
                mul_a = ikha_elem;
                mul_b = pb_elem;
            end
            default: ;
        endcase
    end

    // -------------------------------------------------------------------------
    // Shared adder mux
    //   S_COMP sub 0 : z - x_in[0]
    //   S_COMP sub 1 : P_in[0] + r_val
    //   NR sub 1,2   : F32_TWO - sx_reg  (gap + mul-b capture cycle)
    //   X_ADD        : x_in[i] + ky[i], then 1 - K[0]
    //   GEMM_COMPUTE : acc_elem + mul_result  (valid on mul_done_w)
    // -------------------------------------------------------------------------
    always_comb begin
        shared_add_a = 32'h0;
        shared_add_b = 32'h0;
        case (state)
            S_COMP: begin
                if (sub_cnt == 2'd0) begin
                    shared_add_a = z;
                    shared_add_b = {~x_in_arr[0][31], x_in_arr[0][30:0]};
                end else begin
                    shared_add_a = P_in_arr[0];
                    shared_add_b = r_val;
                end
            end
            NR0, NR1, NR2, NR3: begin
                // Sub_cnt 1 (gap) and sub_cnt 2 (mul start): adder = 2 - sx_reg
                if (sub_cnt == 2'd1 || sub_cnt == 2'd2) begin
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
                        shared_add_b = {~K_reg[0][31], K_reg[0][30:0]};
                    end
                endcase
            end
            GEMM_COMPUTE: begin
                shared_add_a = acc_elem;
                shared_add_b = mul_result;  // valid when mul_done_w=1
            end
            default: ;
        endcase
    end

    // -------------------------------------------------------------------------
    // FSM next-state + sub_nxt
    //
    // States NR*, K_COMP, KY_COMP, GEMM_COMPUTE stall (sub_nxt = sub_cnt) while
    // mul_active=1 and mul_done_w=0.
    // -------------------------------------------------------------------------
    always_comb begin
        state_nxt = state;
        sub_nxt   = sub_cnt + 2'd1;  // default: advance sub_cnt

        case (state)
            IDLE:
                if (start) begin state_nxt = S_COMP; sub_nxt = 2'd0; end
                else             sub_nxt = 2'd0;

            S_COMP:
                if (sub_cnt == 2'd1) begin state_nxt = NR0; sub_nxt = 2'd0; end

            NR0, NR1, NR2, NR3: begin
                if (sub_cnt == 2'd0) begin
                    // First multiply: stall until done
                    if (!mul_done_w) sub_nxt = 2'd0;
                    // else sub_nxt = 1 (default)
                end else if (sub_cnt == 2'd1) begin
                    // Gap cycle: auto-advance to 2 (default)
                end else begin
                    // sub_cnt == 2: second multiply, stall until done
                    if (!mul_done_w) begin
                        sub_nxt = 2'd2;
                    end else begin
                        sub_nxt = 2'd0;
                        case (state)
                            NR0: state_nxt = NR1;
                            NR1: state_nxt = NR2;
                            NR2: state_nxt = NR3;
                            NR3: state_nxt = K_COMP;
                            default: state_nxt = K_COMP;
                        endcase
                    end
                end
            end

            K_COMP: begin
                if (!mul_done_w) sub_nxt = sub_cnt;  // stall
                else if (sub_cnt == 2'd2) begin state_nxt = KY_COMP; sub_nxt = 2'd0; end
            end

            KY_COMP: begin
                if (!mul_done_w) sub_nxt = sub_cnt;
                else if (sub_cnt == 2'd2) begin state_nxt = X_ADD; sub_nxt = 2'd0; end
            end

            X_ADD:
                if (sub_cnt == 2'd3) begin state_nxt = GEMM_INIT; sub_nxt = 2'd0; end

            GEMM_INIT:
                begin state_nxt = GEMM_COMPUTE; sub_nxt = 2'd0; end

            GEMM_COMPUTE: begin
                sub_nxt = 2'd0;  // sub_cnt unused here
                if (mul_done_w && gemm_last) state_nxt = DONE_S;
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
            state    <= IDLE;
            sub_cnt  <= 2'd0;
            mul_active <= 1'b0;
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

            // mul_active tracking
            if (mul_start_w)  mul_active <= 1'b1;
            else if (mul_done_w) mul_active <= 1'b0;

            case (state)

                // S_COMP: combinational adder only
                S_COMP: begin
                    if (sub_cnt == 2'd0) begin
                        y_tilde <= shared_add_r;
                    end else begin
                        S_reg <= shared_add_r;
                        nr_x  <= nr_seed(shared_add_r);
                    end
                end

                // NR0..NR2: sub 0 → sx=S*nr_x on mul_done;
                //           sub 1 → gap (adder computes 2-sx combinationally);
                //           sub 2 → nr_x = nr_x*(2-sx) on mul_done
                NR0, NR1, NR2: begin
                    if (mul_done_w) begin
                        if (sub_cnt == 2'd0) begin
                            sx_reg <= mul_result;
                        end else begin  // sub_cnt == 2
                            nr_x  <= mul_result;
                            S_inv <= mul_result;
                        end
                    end
                end

                // NR3: final iteration — update S_inv but not nr_x
                NR3: begin
                    if (mul_done_w) begin
                        if (sub_cnt == 2'd0) begin
                            sx_reg <= mul_result;
                        end else begin  // sub_cnt == 2
                            S_inv <= mul_result;
                        end
                    end
                end

                // K_COMP: K[i] = P_in[i*3] * S_inv
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

                // X_ADD: combinational adder only
                X_ADD: begin
                    case (sub_cnt)
                        2'd0: x_out_arr[0] <= shared_add_r;
                        2'd1: x_out_arr[1] <= shared_add_r;
                        2'd2: x_out_arr[2] <= shared_add_r;
                        2'd3: one_minus_k0_r <= shared_add_r;
                    endcase
                end

                // GEMM_INIT: zero accumulators, reset counters
                GEMM_INIT: begin
                    for (ii = 0; ii < 9; ii++) acc[ii] <= 32'h0;
                    k_cnt <= 2'd0; i_cnt <= 2'd0; j_cnt <= 2'd0;
                end

                // GEMM_COMPUTE: one MAC per mul_done_w pulse
                //   acc[acc_idx] += ikha_elem * pb_elem (shared_add feeds mul_result)
                //   Advance k/i/j after each MAC (j fastest)
                GEMM_COMPUTE: begin
                    if (mul_done_w) begin
                        // Write accumulator
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
                end

                DONE_S: ;
                default: ;
            endcase
        end
    end

    assign done = (state == DONE_S);
    assign busy = (state != IDLE) && (state != DONE_S);

endmodule
