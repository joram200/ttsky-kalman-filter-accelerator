// =============================================================================
// tb_top.sv — post_m3_Minor_Redesign co-simulation testbench
// All testbench modules combined into a single file.
`timescale 1ns/1ps
// Module order (dependency):
//   clk_gen              — Simulation-only clock generator
//   veerwolf_bfm         — AXI4-Lite master BFM (models SweRV EL2)
//   axi_monitor          — Passive AXI protocol monitor + SVAs
//   compute_core_checker — FSM contract SVAs for compute core
//   result_checker       — F64 ULP comparator (wraps compute_core_checker)
//   program_block        — 15 Kalman profiling data sets
//   tb_top               — Top-level testbench
//
// Register map (8-byte stride):
//   0x00  CTRL   R/W  [0]=start, [1]=soft_rst
//   0x08  STAT   RO   [0]=done,  [1]=busy
//   0x10  z             A_REG[0]   WO
//   0x18–0x28  x_in[0:2]  A_REG[1:3]  WO
//   0x30–0x40  x_out[0:2] A_REG[4:6]  RO  ← new
//   0x58–0x98  P_in[0:8]  B_REG[0:8]  WO
//   0xA0–0xE0  P_out[0:8] C_REG[0:8]  RO  (all 9 elements)
//   0xE8  R_REG  R/W  programmable R (default 5.0 = 0x4014_0000_0000_0000)
// =============================================================================

// -----------------------------------------------------------------------------
// clk_gen — Simulation-only clock generator. NOT synthesisable.
// -----------------------------------------------------------------------------
module clk_gen #(
    parameter real CLK_PERIOD_NS = 10.0   // 100 MHz default
)(
    output logic clk
);
    initial clk = 1'b0;
    always #(CLK_PERIOD_NS / 2.0) clk = ~clk;
endmodule

// -----------------------------------------------------------------------------
// veerwolf_bfm — AXI4-Lite master BFM (64-bit data width)
// -----------------------------------------------------------------------------
module veerwolf_bfm (
    input  logic        clk,
    input  logic        rst_n,
    // AXI4-Lite master port — 64-bit data, 32-bit address
    output logic [31:0] m_awaddr,
    output logic        m_awvalid,
    input  logic        m_awready,
    output logic [63:0] m_wdata,
    output logic [7:0]  m_wstrb,
    output logic        m_wvalid,
    input  logic        m_wready,
    input  logic [1:0]  m_bresp,
    input  logic        m_bvalid,
    output logic        m_bready,
    output logic [31:0] m_araddr,
    output logic        m_arvalid,
    input  logic        m_arready,
    input  logic [63:0] m_rdata,
    input  logic [1:0]  m_rresp,
    input  logic        m_rvalid,
    output logic        m_rready
);

    // Default idle state
    initial begin
        m_awaddr  = 32'h0;
        m_awvalid = 1'b0;
        m_wdata   = 64'h0;
        m_wstrb   = 8'hFF;
        m_wvalid  = 1'b0;
        m_bready  = 1'b1;
        m_araddr  = 32'h0;
        m_arvalid = 1'b0;
        m_rready  = 1'b1;
    end

    // -----------------------------------------------------------------------
    // axi_write — issue one 64-bit write transaction
    // -----------------------------------------------------------------------
    task automatic axi_write(input logic [31:0] addr, input logic [63:0] data);
        @(posedge clk);
        m_awaddr  <= addr;
        m_awvalid <= 1'b1;
        m_wdata   <= data;
        m_wstrb   <= 8'hFF;
        m_wvalid  <= 1'b1;

        fork
            begin
                wait (m_awready);
                @(posedge clk);
                m_awvalid <= 1'b0;
            end
            begin
                wait (m_wready);
                @(posedge clk);
                m_wvalid <= 1'b0;
            end
        join

        m_bready <= 1'b1;
        wait (m_bvalid);
        @(posedge clk);
    endtask

    // -----------------------------------------------------------------------
    // axi_read — issue one 64-bit read transaction
    // -----------------------------------------------------------------------
    task automatic axi_read(input logic [31:0] addr, output logic [63:0] data);
        @(posedge clk);
        m_araddr  <= addr;
        m_arvalid <= 1'b1;
        m_rready  <= 1'b1;

        wait (m_arready);
        @(posedge clk);
        m_arvalid <= 1'b0;

        wait (m_rvalid);
        data = m_rdata;
        @(posedge clk);
    endtask

    // -----------------------------------------------------------------------
    // write_matrix_A — write z + x_in to A_REG (base 0x10, 9 beats)
    //   A[0] = z (0x10), A[1:3] = x_in[0:2] (0x18–0x28)
    //   A[4:6] = x_out slots (RO — writes silently ignored by slave)
    //   A[7:8] = unused (silently ignored)
    // -----------------------------------------------------------------------
    task automatic write_matrix_A(input logic [63:0] A [0:8]);
        for (int i = 0; i < 9; i++) begin
            axi_write(32'h10 + (i * 8), A[i]);
        end
    endtask

    // -----------------------------------------------------------------------
    // write_matrix_B — write P_in to B_REG (base 0x58, 9 beats)
    // -----------------------------------------------------------------------
    task automatic write_matrix_B(input logic [63:0] B [0:8]);
        for (int i = 0; i < 9; i++) begin
            axi_write(32'h58 + (i * 8), B[i]);
        end
    endtask

    // -----------------------------------------------------------------------
    // read_xout — read x_out[0:2] from A_REG[4:6] (0x30, 0x38, 0x40)
    // -----------------------------------------------------------------------
    task automatic read_xout(output logic [63:0] xo [0:2]);
        axi_read(32'h30, xo[0]);
        axi_read(32'h38, xo[1]);
        axi_read(32'h40, xo[2]);
    endtask

    // -----------------------------------------------------------------------
    // read_matrix_C — read P_out[0:8] from C_REG (base 0xA0, 9 beats)
    // -----------------------------------------------------------------------
    task automatic read_matrix_C(output logic [63:0] C [0:8]);
        for (int i = 0; i < 9; i++) begin
            axi_read(32'hA0 + (i * 8), C[i]);
        end
    endtask

    // -----------------------------------------------------------------------
    // fire_and_wait — write CTRL=1, poll STAT[1] (busy) until done
    // -----------------------------------------------------------------------
    task automatic fire_and_wait();
        logic [63:0] stat;
        axi_write(32'h00, 64'h0000_0000_0000_0001);
        stat = 64'h0;
        while (!stat[1]) axi_read(32'h08, stat);  // wait for busy to assert
        while  (stat[1]) axi_read(32'h08, stat);  // wait for busy to drop
    endtask

endmodule

// -----------------------------------------------------------------------------
// axi_monitor — Passive AXI4-Lite protocol monitor with SVAs
// -----------------------------------------------------------------------------
module axi_monitor (
    input logic        clk,
    input logic        rst_n,
    input logic [31:0] s_awaddr,
    input logic        s_awvalid,
    input logic        s_awready,
    input logic [63:0] s_wdata,
    input logic [7:0]  s_wstrb,
    input logic        s_wvalid,
    input logic        s_wready,
    input logic [1:0]  s_bresp,
    input logic        s_bvalid,
    input logic        s_bready,
    input logic [31:0] s_araddr,
    input logic        s_arvalid,
    input logic        s_arready,
    input logic [63:0] s_rdata,
    input logic [1:0]  s_rresp,
    input logic        s_rvalid,
    input logic        s_rready,
    input logic [63:0] core_C [0:8],
    input logic        core_done,
    input logic        core_busy
);

    // 1. AWVALID must not deassert before AWREADY
    property p_awvalid_stable;
        @(posedge clk) disable iff (!rst_n)
        (s_awvalid && !s_awready) |=> s_awvalid;
    endproperty
    assert property (p_awvalid_stable)
        else $error("AXI MONITOR: AWVALID withdrew before AWREADY");

    // 2. WVALID must not deassert before WREADY
    property p_wvalid_stable;
        @(posedge clk) disable iff (!rst_n)
        (s_wvalid && !s_wready) |=> s_wvalid;
    endproperty
    assert property (p_wvalid_stable)
        else $error("AXI MONITOR: WVALID withdrew before WREADY");

    // 3. ARVALID must not deassert before ARREADY
    property p_arvalid_stable;
        @(posedge clk) disable iff (!rst_n)
        (s_arvalid && !s_arready) |=> s_arvalid;
    endproperty
    assert property (p_arvalid_stable)
        else $error("AXI MONITOR: ARVALID withdrew before ARREADY");

    // 4. BRESP must be OKAY on all accepted write responses
    property p_bresp_okay;
        @(posedge clk) disable iff (!rst_n)
        (s_bvalid && s_bready) |-> (s_bresp == 2'b00);
    endproperty
    assert property (p_bresp_okay)
        else $error("AXI MONITOR: Write response non-OKAY");

    // 5. RRESP must be OKAY on all accepted read data beats
    property p_rresp_okay;
        @(posedge clk) disable iff (!rst_n)
        (s_rvalid && s_rready) |-> (s_rresp == 2'b00);
    endproperty
    assert property (p_rresp_okay)
        else $error("AXI MONITOR: Read response non-OKAY");

    // 6. core_done must be a single-cycle pulse
    property p_done_pulse;
        @(posedge clk) disable iff (!rst_n)
        $rose(core_done) |=> !core_done;
    endproperty
    assert property (p_done_pulse)
        else $error("AXI MONITOR: core_done held HIGH for more than one cycle");

    // 7. core_C (P_out) must be stable when core is idle
    property p_c_stable;
        @(posedge clk) disable iff (!rst_n)
        (!core_busy && !core_done) |=> $stable(core_C);
    endproperty
    assert property (p_c_stable)
        else $error("AXI MONITOR: core_C changed while core is idle");

    // Cover groups — updated for new register map
    covergroup axi_write_targets @(posedge clk);
        cp_wr: coverpoint s_awaddr[7:3] iff (s_awvalid && s_awready) {
            bins ctrl    = {5'd0};    // 0x00 CTRL
            bins z_reg   = {5'd2};    // 0x10 z
            bins x_in_0  = {5'd3};    // 0x18 x_in[0]
            bins x_in_2  = {5'd5};    // 0x28 x_in[2]
            bins b_reg_0 = {5'd11};   // 0x58 P_in[0]
            bins b_reg_8 = {5'd19};   // 0x98 P_in[8]
            bins r_reg   = {5'd29};   // 0xE8 R_REG
        }
    endgroup

    covergroup axi_read_targets @(posedge clk);
        cp_rd: coverpoint s_araddr[7:3] iff (s_arvalid && s_arready) {
            bins stat    = {5'd1};    // 0x08 STAT
            bins x_out_0 = {5'd6};    // 0x30 x_out[0]
            bins x_out_2 = {5'd8};    // 0x40 x_out[2]
            bins c_reg_0 = {5'd20};   // 0xA0 P_out[0]
            bins c_reg_8 = {5'd28};   // 0xE0 P_out[8]
            bins r_reg   = {5'd29};   // 0xE8 R_REG
        }
    endgroup

    axi_write_targets cg_wr = new();
    axi_read_targets  cg_rd = new();

endmodule

// -----------------------------------------------------------------------------
// compute_core_checker — SVA assertions on core FSM contracts
// -----------------------------------------------------------------------------
module compute_core_checker (
    input logic clk,
    input logic core_start,
    input logic core_busy,
    input logic core_done,
    input logic core_rst_n
);

    // 1. start must not assert while busy
    property p_no_start_while_busy;
        @(posedge clk) disable iff (!core_rst_n)
        !(core_start && core_busy);
    endproperty
    assert property (p_no_start_while_busy)
        else $error("CHECKER: start asserted while core is busy");

    // 2. busy must rise within one cycle of start
    property p_busy_follows_start;
        @(posedge clk) disable iff (!core_rst_n)
        core_start |=> core_busy;
    endproperty
    assert property (p_busy_follows_start)
        else $error("CHECKER: busy did not assert the cycle after start");

    // 3. busy must hold HIGH for at least 3 cycles (minimum NR pipeline latency)
    property p_busy_min_duration;
        @(posedge clk) disable iff (!core_rst_n)
        $rose(core_busy) |-> core_busy [*3];
    endproperty
    assert property (p_busy_min_duration)
        else $error("CHECKER: busy deasserted in fewer than 3 cycles (pipeline underrun)");

    // 4. done must not assert without busy having been HIGH
    property p_done_requires_busy;
        @(posedge clk) disable iff (!core_rst_n)
        $rose(core_done) |-> $past(core_busy, 1);
    endproperty
    assert property (p_done_requires_busy)
        else $error("CHECKER: done rose without a preceding busy cycle");

    // 5. busy and done are mutually exclusive
    property p_busy_done_mutex;
        @(posedge clk) disable iff (!core_rst_n)
        !(core_busy && core_done);
    endproperty
    assert property (p_busy_done_mutex)
        else $error("CHECKER: busy and done asserted simultaneously");

    // 6. After done pulses, busy must be LOW
    property p_idle_after_done;
        @(posedge clk) disable iff (!core_rst_n)
        $rose(core_done) |=> !core_busy;
    endproperty
    assert property (p_idle_after_done)
        else $error("CHECKER: core still busy after done pulse");

    // 7. done is a single-cycle pulse
    property p_done_one_cycle;
        @(posedge clk) disable iff (!core_rst_n)
        $rose(core_done) |=> !core_done;
    endproperty
    assert property (p_done_one_cycle)
        else $error("CHECKER: done held HIGH for more than one cycle");

    // Cover: full start → busy → done sequence
    cover property (@(posedge clk) disable iff (!core_rst_n)
        core_start ##1 core_busy [*1:$] ##1 core_done)
        $display("CHECKER cover: full compute cycle observed");

endmodule : compute_core_checker

// -----------------------------------------------------------------------------
// result_checker — ULP comparator + FSM checker wrapper
// Extended to 12 elements per iteration: x_out[0:2] + P_out[0:8]
// Expected: 15 iterations × 12 = 180 checks on PASS
// -----------------------------------------------------------------------------
module result_checker #(
    parameter real ULP_TOLERANCE = 1.0
)(
    input logic        clk,
    input logic        check_en,        // one-cycle pulse after done
    input logic [63:0] dut_xo  [0:2],  // x_out from accelerator
    input logic [63:0] ref_xo  [0:2],  // reference x_out
    input logic [63:0] dut_P   [0:8],  // P_out from accelerator (all 9 elements)
    input logic [63:0] ref_P   [0:8],  // reference P_out
    // Core signals forwarded to FSM checker
    input logic        core_start,
    input logic        core_busy,
    input logic        core_done,
    input logic        core_rst_n
);

    compute_core_checker u_core_chk (
        .clk        (clk),
        .core_start (core_start),
        .core_busy  (core_busy),
        .core_done  (core_done),
        .core_rst_n (core_rst_n)
    );

    // ULP distance: treats F64 as unsigned 64-bit integer.
    function automatic logic [63:0] ulp_dist(
        input logic [63:0] x, y
    );
        logic [63:0] ax, ay;
        ax = {1'b0, x[62:0]};
        ay = {1'b0, y[62:0]};
        if (x[63] == y[63]) begin
            ulp_dist = (ax >= ay) ? (ax - ay) : (ay - ax);
        end else begin
            ulp_dist = ax + ay;
        end
    endfunction

    int unsigned pass_count = 0, fail_count = 0;

    always @(posedge clk) begin
        if (check_en) begin
            // Check x_out[0:2]
            for (int i = 0; i < 3; i++) begin
                automatic logic [63:0] ulp_d = ulp_dist(dut_xo[i], ref_xo[i]);
                if (ulp_d > $realtobits(ULP_TOLERANCE)) begin
                    $error("CHECKER: x_out[%0d] ULP error=%0d  dut=%h  ref=%h",
                           i, ulp_d, dut_xo[i], ref_xo[i]);
                    fail_count++;
                end else begin
                    pass_count++;
                end
            end
            // Check P_out[0:8]
            for (int i = 0; i < 9; i++) begin
                automatic logic [63:0] ulp_d = ulp_dist(dut_P[i], ref_P[i]);
                if (ulp_d > $realtobits(ULP_TOLERANCE)) begin
                    $error("CHECKER: P_out[%0d] ULP error=%0d  dut=%h  ref=%h",
                           i, ulp_d, dut_P[i], ref_P[i]);
                    fail_count++;
                end else begin
                    pass_count++;
                end
            end
        end
    end

    final begin
        $display("CHECKER summary: %0d elements passed, %0d failed",
                 pass_count, fail_count);
        if (fail_count > 0) $error("CHECKER: %0d ULP failures detected", fail_count);
    end

endmodule : result_checker

// -----------------------------------------------------------------------------
// program_block — 15 F64 P covariance snapshots from RVfpgaEL2 profiling.
// (Unchanged from M3 — same profiling data.)
// -----------------------------------------------------------------------------
module program_block (
    output logic [63:0] A_pairs [0:14][0:8],
    output logic [63:0] B_pairs [0:14][0:8]
);

    localparam logic [63:0] ONE   = 64'h3FF0_0000_0000_0000;
    localparam logic [63:0] ZERO  = 64'h0000_0000_0000_0000;
    localparam logic [63:0] DT    = 64'h3FA1_1111_1111_1111;  // 1/30 s

    localparam logic [63:0] A_CONST [0:8] = '{
        ONE, DT,  ZERO,
        ZERO, ONE, DT,
        ZERO, ZERO, ONE
    };

    localparam logic [63:0] B_DATA [0:14][0:8] = '{
        // iter 0 — P0 = 100*I
        '{64'h4059_0000_0000_0000, ZERO, ZERO,
          ZERO, 64'h4059_0000_0000_0000, ZERO,
          ZERO, ZERO, 64'h4059_0000_0000_0000},
        // iter 1
        '{64'h4050_F76B_A2BE_E1C0, 64'h404E_5B11_7F3D_0A76, ZERO,
          64'h404E_5B11_7F3D_0A76, 64'h4050_F76B_A2BE_E1C0, 64'h404E_5B11_7F3D_0A76,
          ZERO, 64'h404E_5B11_7F3D_0A76, 64'h4050_F76B_A2BE_E1C0},
        // iter 2
        '{64'h404C_1A4D_1C7B_A8A5, 64'h4047_4328_6A0D_4A4A, ZERO,
          64'h4047_4328_6A0D_4A4A, 64'h404C_1A4D_1C7B_A8A5, 64'h4047_4328_6A0D_4A4A,
          ZERO, 64'h4047_4328_6A0D_4A4A, 64'h404C_1A4D_1C7B_A8A5},
        // iter 3
        '{64'h4045_9B22_F699_0F8A, 64'h403E_F165_7BBF_03A6, ZERO,
          64'h403E_F165_7BBF_03A6, 64'h4045_9B22_F699_0F8A, 64'h403E_F165_7BBF_03A6,
          ZERO, 64'h403E_F165_7BBF_03A6, 64'h4045_9B22_F699_0F8A},
        // iter 4
        '{64'h4040_6B19_B83B_A7A1, 64'h4035_E823_5B1D_5B24, ZERO,
          64'h4035_E823_5B1D_5B24, 64'h4040_6B19_B83B_A7A1, 64'h4035_E823_5B1D_5B24,
          ZERO, 64'h4035_E823_5B1D_5B24, 64'h4040_6B19_B83B_A7A1},
        // iter 5
        '{64'h403C_8B8E_C5F9_A037, 64'h402E_7C4D_1FBB_3A22, ZERO,
          64'h402E_7C4D_1FBB_3A22, 64'h403C_8B8E_C5F9_A037, 64'h402E_7C4D_1FBB_3A22,
          ZERO, 64'h402E_7C4D_1FBB_3A22, 64'h403C_8B8E_C5F9_A037},
        // iter 6
        '{64'h4037_F31D_E0C0_A820, 64'h4025_F4B6_6B7E_2B1C, ZERO,
          64'h4025_F4B6_6B7E_2B1C, 64'h4037_F31D_E0C0_A820, 64'h4025_F4B6_6B7E_2B1C,
          ZERO, 64'h4025_F4B6_6B7E_2B1C, 64'h4037_F31D_E0C0_A820},
        // iter 7
        '{64'h4032_CE1D_ECD3_3B75, 64'h401C_7B60_5547_9D35, ZERO,
          64'h401C_7B60_5547_9D35, 64'h4032_CE1D_ECD3_3B75, 64'h401C_7B60_5547_9D35,
          ZERO, 64'h401C_7B60_5547_9D35, 64'h4032_CE1D_ECD3_3B75},
        // iter 8
        '{64'h402D_9C1D_DF84_6A2C, 64'h4010_C4E9_E8A2_9C1D, ZERO,
          64'h4010_C4E9_E8A2_9C1D, 64'h402D_9C1D_DF84_6A2C, 64'h4010_C4E9_E8A2_9C1D,
          ZERO, 64'h4010_C4E9_E8A2_9C1D, 64'h402D_9C1D_DF84_6A2C},
        // iter 9
        '{64'h4026_8879_79EA_2B3A, 64'h4003_A2BA_4A91_6C19, ZERO,
          64'h4003_A2BA_4A91_6C19, 64'h4026_8879_79EA_2B3A, 64'h4003_A2BA_4A91_6C19,
          ZERO, 64'h4003_A2BA_4A91_6C19, 64'h4026_8879_79EA_2B3A},
        // iter 10
        '{64'h401E_F46F_7CF1_C71B, 64'h3FF3_4F72_9F19_67A3, ZERO,
          64'h3FF3_4F72_9F19_67A3, 64'h401E_F46F_7CF1_C71B, 64'h3FF3_4F72_9F19_67A3,
          ZERO, 64'h3FF3_4F72_9F19_67A3, 64'h401E_F46F_7CF1_C71B},
        // iter 11
        '{64'h4015_A3D2_6FF4_5924, 64'h3FE6_90C0_9382_8F5C, ZERO,
          64'h3FE6_90C0_9382_8F5C, 64'h4015_A3D2_6FF4_5924, 64'h3FE6_90C0_9382_8F5C,
          ZERO, 64'h3FE6_90C0_9382_8F5C, 64'h4015_A3D2_6FF4_5924},
        // iter 12
        '{64'h400B_CE28_F5C2_8F5C, 64'h3FD0_F5C2_8F5C_28F6, ZERO,
          64'h3FD0_F5C2_8F5C_28F6, 64'h400B_CE28_F5C2_8F5C, 64'h3FD0_F5C2_8F5C_28F6,
          ZERO, 64'h3FD0_F5C2_8F5C_28F6, 64'h400B_CE28_F5C2_8F5C},
        // iter 13
        '{64'h4001_4B6B_B09E_92C1, 64'h3FAD_1745_D174_5D17, ZERO,
          64'h3FAD_1745_D174_5D17, 64'h4001_4B6B_B09E_92C1, 64'h3FAD_1745_D174_5D17,
          ZERO, 64'h3FAD_1745_D174_5D17, 64'h4001_4B6B_B09E_92C1},
        // iter 14 — steady-state P
        '{64'h3FF8_E38E_38E3_8E39, 64'h3F8A_3D70_A3D7_0A3D, ZERO,
          64'h3F8A_3D70_A3D7_0A3D, 64'h3FF8_E38E_38E3_8E39, 64'h3F8A_3D70_A3D7_0A3D,
          ZERO, 64'h3F8A_3D70_A3D7_0A3D, 64'h3FF8_E38E_38E3_8E39}
    };

    always_comb begin
        for (int iter = 0; iter < 15; iter++) begin
            for (int el = 0; el < 9; el++) begin
                A_pairs[iter][el] = A_CONST[el];
                B_pairs[iter][el] = B_DATA[iter][el];
            end
        end
    end

endmodule

// =============================================================================
// tb_top — End-to-end co-simulation testbench (post_m3_Minor_Redesign)
// Exercises the redesigned register map:
//   - R=5.0 written to R_REG (0xE8) once before the loop
//   - x_out read from A_REG[4:6] (0x30–0x40) after each compute
//   - P_out[0:8] read from C_REG (0xA0–0xE0) — all 9 elements now exposed
// Checker verifies 15×12 = 180 elements total (x_out[0:2] + P_out[0:8]).
// =============================================================================

module tb_top;

    logic clk, rst_n;

    clk_gen #(.CLK_PERIOD_NS(10.0)) u_clk (.clk(clk));

    initial begin
        $dumpfile("sim/cosim_waveform.vcd");
        $dumpvars(1, tb_top);

        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
    end

    // AXI4-Lite signals
    logic [31:0] m_awaddr;
    logic        m_awvalid, m_awready;
    logic [63:0] m_wdata;
    logic [7:0]  m_wstrb;
    logic        m_wvalid,  m_wready;
    logic [1:0]  m_bresp;
    logic        m_bvalid,  m_bready;
    logic [31:0] m_araddr;
    logic        m_arvalid, m_arready;
    logic [63:0] m_rdata;
    logic [1:0]  m_rresp;
    logic        m_rvalid,  m_rready;

    // BFM
    veerwolf_bfm u_bfm (
        .clk       (clk),
        .rst_n     (rst_n),
        .m_awaddr  (m_awaddr),  .m_awvalid (m_awvalid), .m_awready (m_awready),
        .m_wdata   (m_wdata),   .m_wstrb   (m_wstrb),
        .m_wvalid  (m_wvalid),  .m_wready  (m_wready),
        .m_bresp   (m_bresp),   .m_bvalid  (m_bvalid),  .m_bready  (m_bready),
        .m_araddr  (m_araddr),  .m_arvalid (m_arvalid), .m_arready (m_arready),
        .m_rdata   (m_rdata),   .m_rresp   (m_rresp),
        .m_rvalid  (m_rvalid),  .m_rready  (m_rready)
    );

    // DUT
    top #(.ADDR_WIDTH(32), .DATA_WIDTH(64)) u_dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .s_awaddr  (m_awaddr),  .s_awvalid (m_awvalid), .s_awready (m_awready),
        .s_wdata   (m_wdata),   .s_wstrb   (m_wstrb),
        .s_wvalid  (m_wvalid),  .s_wready  (m_wready),
        .s_bresp   (m_bresp),   .s_bvalid  (m_bvalid),  .s_bready  (m_bready),
        .s_araddr  (m_araddr),  .s_arvalid (m_arvalid), .s_arready (m_arready),
        .s_rdata   (m_rdata),   .s_rresp   (m_rresp),
        .s_rvalid  (m_rvalid),  .s_rready  (m_rready)
    );

    // Hierarchical references to core signals
    wire        core_start = u_dut.core_start;
    wire        core_busy  = u_dut.core_busy;
    wire        core_done  = u_dut.core_done;
    wire        core_rst_n = u_dut.core_rst_n;
    wire [63:0] core_C [0:8];
    genvar gi;
    generate
        for (gi = 0; gi < 9; gi++) begin : gen_c_tap
            assign core_C[gi] = u_dut.slave_C[gi];
        end
    endgenerate

    // AXI monitor
    axi_monitor u_mon (
        .clk       (clk),
        .rst_n     (rst_n),
        .s_awaddr  (m_awaddr),  .s_awvalid (m_awvalid), .s_awready (m_awready),
        .s_wdata   (m_wdata),   .s_wstrb   (m_wstrb),
        .s_wvalid  (m_wvalid),  .s_wready  (m_wready),
        .s_bresp   (m_bresp),   .s_bvalid  (m_bvalid),  .s_bready  (m_bready),
        .s_araddr  (m_araddr),  .s_arvalid (m_arvalid), .s_arready (m_arready),
        .s_rdata   (m_rdata),   .s_rresp   (m_rresp),
        .s_rvalid  (m_rvalid),  .s_rready  (m_rready),
        .core_C    (core_C),
        .core_done (core_done),
        .core_busy (core_busy)
    );

    // Program block
    logic [63:0] A_pairs [0:14][0:8];
    logic [63:0] B_pairs [0:14][0:8];

    program_block u_prog (
        .A_pairs (A_pairs),
        .B_pairs (B_pairs)
    );

    // Checker — 12 elements per iteration (3 x_out + 9 P_out)
    logic        check_en;
    logic [63:0] dut_xo [0:2];
    logic [63:0] ref_xo [0:2];
    logic [63:0] dut_P  [0:8];
    logic [63:0] ref_P  [0:8];

    result_checker #(.ULP_TOLERANCE(4.0)) u_chk (
        .clk        (clk),
        .check_en   (check_en),
        .dut_xo     (dut_xo),
        .ref_xo     (ref_xo),
        .dut_P      (dut_P),
        .ref_P      (ref_P),
        .core_start (core_start),
        .core_busy  (core_busy),
        .core_done  (core_done),
        .core_rst_n (core_rst_n)
    );

    // Software reference Kalman update: H=[1,0,0], R=5.0, z=0.5
    localparam real R_REF  = 5.0;
    localparam real Z_MEAS = 0.5;

    function automatic void kalman_ref(
        input  logic [63:0] x_in  [0:2],
        input  logic [63:0] P_in  [0:8],
        output logic [63:0] x_out [0:2],
        output logic [63:0] P_out [0:8]
    );
        real x[3], P[9], K[3], x_new[3], P_new[9];
        real y_tilde, S, S_inv;

        for (int i = 0; i < 3; i++) x[i] = $bitstoreal(x_in[i]);
        for (int i = 0; i < 9; i++) P[i] = $bitstoreal(P_in[i]);

        y_tilde = Z_MEAS - x[0];
        S       = P[0] + R_REF;
        S_inv   = 1.0 / S;

        for (int i = 0; i < 3; i++) K[i] = P[i*3] * S_inv;

        for (int i = 0; i < 3; i++) x_new[i] = x[i] + K[i] * y_tilde;

        for (int i = 0; i < 3; i++) begin
            for (int j = 0; j < 3; j++) begin
                P_new[i*3+j] = 0.0;
                for (int k = 0; k < 3; k++) begin
                    real ikH_ik;
                    ikH_ik = ((i == k) ? 1.0 : 0.0) - K[i] * (k == 0 ? 1.0 : 0.0);
                    P_new[i*3+j] = P_new[i*3+j] + ikH_ik * P[k*3+j];
                end
            end
        end

        for (int i = 0; i < 3; i++) x_out[i] = $realtobits(x_new[i]);
        for (int i = 0; i < 9; i++) P_out[i] = $realtobits(P_new[i]);
    endfunction

    // Main test sequence
    logic [63:0] x_iter [0:2];
    logic [63:0] A_in   [0:8];

    initial begin
        check_en = 1'b0;
        for (int i = 0; i < 3; i++) x_iter[i] = 64'h0;

        wait (rst_n === 1'b1);
        repeat (3) @(posedge clk);

        // Write R = 5.0 to R_REG (0xE8) — demonstrates R programmability
        u_bfm.axi_write(32'hE8, 64'h4014_0000_0000_0000);

        for (int iter = 0; iter < 15; iter++) begin
            $display("TB_TOP: Starting iteration %0d", iter);

            // Write z (A_REG[0]), x_in[0:2] (A_REG[1:3]), P_in (B_REG[0:8])
            A_in[0] = $realtobits(Z_MEAS);
            for (int i = 0; i < 3; i++) A_in[1+i] = x_iter[i];
            for (int i = 4; i < 9; i++) A_in[i] = 64'h0;  // x_out/unused slots — ignored

            u_bfm.write_matrix_A(A_in);
            u_bfm.write_matrix_B(B_pairs[iter]);

            u_bfm.fire_and_wait();

            // Read x_out (A_REG[4:6] at 0x30–0x40) and P_out (C_REG at 0xA0–0xE0)
            u_bfm.read_xout(dut_xo);
            u_bfm.read_matrix_C(dut_P);

            // Compute reference
            kalman_ref(x_iter, B_pairs[iter], ref_xo, ref_P);

            // Propagate x for next iteration
            for (int i = 0; i < 3; i++) x_iter[i] = ref_xo[i];

            // Trigger checker (one-cycle pulse)
            @(posedge clk);
            check_en = 1'b1;
            @(posedge clk);
            check_en = 1'b0;

            $display("TB_TOP: Iteration %0d complete", iter);
        end

        repeat (10) @(posedge clk);
        $display("TB_TOP: All 15 iterations complete.");
        $finish;
    end

    // Final PASS/FAIL verdict
    final begin
        if (u_chk.fail_count == 0)
            $display("SIMULATION RESULT: PASS — all checks passed, no errors detected");
        else
            $display("SIMULATION RESULT: FAIL — %0d ULP failures detected", u_chk.fail_count);
    end

    // Timeout watchdog (2 ms — generous for 15 iterations with NR pipeline)
    initial begin
        #2000000;
        $fatal(1, "TB_TOP: Simulation timeout");
    end

endmodule
