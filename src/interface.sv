`timescale 1ns/1ps
// =============================================================================
// interface.sv — M4: Option B Kalman Update Accelerator — AXI4-Lite Interface
// Extracted from post_m3_Minor_Redesign/rtl/top.sv
// Contains: axilite_slave
// Register map:
//   0x00 CTRL   R/W  [0]=start, [1]=sw_rst
//   0x08 STAT   RO   [0]=done,  [1]=busy
//   0x10 z      WO   A_REG[0]
//   0x18 x_in0  WO   A_REG[1]
//   0x20 x_in1  WO   A_REG[2]
//   0x28 x_in2  WO   A_REG[3]
//   0x30 x_out0 RO   A_REG[4]  (core_x_out[0])
//   0x38 x_out1 RO   A_REG[5]  (core_x_out[1])
//   0x40 x_out2 RO   A_REG[6]  (core_x_out[2])
//   0x58–0x98  P_in[0:8]  WO   B_REG[0:8]
//   0xA0–0xE0  P_out[0:8] RO   C_REG[0:8]  (core_C[0:8])
//   0xE8 R_REG  R/W  programmable measurement noise (default 5.0)
// =============================================================================

// -----------------------------------------------------------------------------
// axilite_slave — AXI4-Lite register file + core handshake
// -----------------------------------------------------------------------------
module axilite_slave #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 64
)(
    input  logic                      clk,
    input  logic                      rst_n,
    // AXI4-Lite write address channel
    input  logic [ADDR_WIDTH-1:0]     s_awaddr,
    input  logic                      s_awvalid,
    output logic                      s_awready,
    // AXI4-Lite write data channel
    input  logic [DATA_WIDTH-1:0]     s_wdata,
    input  logic [(DATA_WIDTH/8)-1:0] s_wstrb,
    input  logic                      s_wvalid,
    output logic                      s_wready,
    // AXI4-Lite write response channel
    output logic [1:0]                s_bresp,
    output logic                      s_bvalid,
    input  logic                      s_bready,
    // AXI4-Lite read address channel
    input  logic [ADDR_WIDTH-1:0]     s_araddr,
    input  logic                      s_arvalid,
    output logic                      s_arready,
    // AXI4-Lite read data channel
    output logic [DATA_WIDTH-1:0]     s_rdata,
    output logic [1:0]                s_rresp,
    output logic                      s_rvalid,
    input  logic                      s_rready,
    // Core control outputs
    output logic                      core_start,
    output logic                      core_rst_n,
    output logic [63:0]               core_A [0:8],
    output logic [63:0]               core_B [0:8],
    output logic [63:0]               core_R,           // R_REG → kalman_update.r_val
    // Core status inputs
    input  logic [63:0]               core_C [0:8],     // P_out[0:8]
    input  logic [63:0]               core_x_out [0:2], // x_out[0:2] → A_REG[4:6] reads
    input  logic                      core_done,
    input  logic                      core_busy
);

    logic [63:0] reg_ctrl;
    logic [63:0] reg_A [0:8];
    logic [63:0] reg_B [0:8];
    logic [63:0] reg_R;           // programmable R, default 5.0

    logic        aw_done;
    logic        w_done;
    logic [ADDR_WIDTH-1:0] aw_addr_lat;
    logic [DATA_WIDTH-1:0] w_data_lat;

    logic start_pending;

    // -----------------------------------------------------------------------
    // Write address channel
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_awready   <= 1'b1;
            aw_done     <= 1'b0;
            aw_addr_lat <= '0;
        end else begin
            if (s_awvalid && s_awready) begin
                aw_addr_lat <= s_awaddr;
                aw_done     <= 1'b1;
                s_awready   <= 1'b0;
            end else if (aw_done && w_done) begin
                aw_done   <= 1'b0;
                s_awready <= 1'b1;
            end
        end
    end

    // -----------------------------------------------------------------------
    // Write data channel
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_wready   <= 1'b1;
            w_done     <= 1'b0;
            w_data_lat <= '0;
        end else begin
            if (s_wvalid && s_wready) begin
                w_data_lat <= s_wdata;
                w_done     <= 1'b1;
                s_wready   <= 1'b0;
            end else if (aw_done && w_done) begin
                w_done    <= 1'b0;
                s_wready  <= 1'b1;
            end
        end
    end

    // -----------------------------------------------------------------------
    // Write execute
    // Index decode: addr[7:3]
    //   0  = CTRL
    //   1  = STAT (RO, ignored)
    //   2–5   = A_REG[0:3]  (z, x_in[0:2])
    //   6–8   = A_REG[4:6]  (x_out — RO, writes ignored)
    //   9–10  = A_REG[7:8]  (unused, ignored)
    //   11–19 = B_REG[0:8]  (P_in)
    //   20–28 = C_REG[0:8]  (P_out — RO, ignored)
    //   29    = R_REG
    // -----------------------------------------------------------------------
    logic write_commit;
    assign write_commit = aw_done && w_done;

    logic [4:0] wr_idx;
    assign wr_idx = aw_addr_lat[7:3];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_ctrl      <= '0;
            reg_R         <= 64'h4014_0000_0000_0000;  // default R = 5.0
            start_pending <= 1'b0;
            for (int i = 0; i < 9; i++) begin
                reg_A[i] <= '0;
                reg_B[i] <= '0;
            end
        end else begin
            start_pending <= 1'b0;

            if (write_commit) begin
                case (wr_idx)
                    5'd0: begin
                        reg_ctrl <= w_data_lat;
                        if (w_data_lat[0]) start_pending <= 1'b1;
                    end
                    // idx 1 = STAT (RO, ignored)
                    5'd2:  reg_A[0] <= w_data_lat;   // z
                    5'd3:  reg_A[1] <= w_data_lat;   // x_in[0]
                    5'd4:  reg_A[2] <= w_data_lat;   // x_in[1]
                    5'd5:  reg_A[3] <= w_data_lat;   // x_in[2]
                    // idx 6–8 = x_out (RO, ignored)
                    // idx 9–10 = unused (ignored)
                    5'd11: reg_B[0] <= w_data_lat;
                    5'd12: reg_B[1] <= w_data_lat;
                    5'd13: reg_B[2] <= w_data_lat;
                    5'd14: reg_B[3] <= w_data_lat;
                    5'd15: reg_B[4] <= w_data_lat;
                    5'd16: reg_B[5] <= w_data_lat;
                    5'd17: reg_B[6] <= w_data_lat;
                    5'd18: reg_B[7] <= w_data_lat;
                    5'd19: reg_B[8] <= w_data_lat;
                    // idx 20–28 = P_out (RO, ignored)
                    5'd29: reg_R <= w_data_lat;       // R_REG
                    default: ;
                endcase
            end
        end
    end

    // -----------------------------------------------------------------------
    // Write response channel
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_bvalid <= 1'b0;
            s_bresp  <= 2'b00;
        end else begin
            if (write_commit && !s_bvalid) begin
                s_bvalid <= 1'b1;
                s_bresp  <= 2'b00;
            end else if (s_bvalid && s_bready) begin
                s_bvalid <= 1'b0;
            end
        end
    end

    // -----------------------------------------------------------------------
    // Read channel
    // -----------------------------------------------------------------------
    logic [4:0] rd_idx;
    assign rd_idx = s_araddr[7:3];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_arready <= 1'b1;
            s_rvalid  <= 1'b0;
            s_rdata   <= '0;
            s_rresp   <= 2'b00;
        end else begin
            if (s_arvalid && s_arready) begin
                s_arready <= 1'b0;
                s_rvalid  <= 1'b1;
                s_rresp   <= 2'b00;
                case (rd_idx)
                    5'd0:  s_rdata <= reg_ctrl;
                    5'd1:  s_rdata <= {62'b0, core_busy, core_done};
                    5'd2:  s_rdata <= reg_A[0];            // z
                    5'd3:  s_rdata <= reg_A[1];            // x_in[0]
                    5'd4:  s_rdata <= reg_A[2];            // x_in[1]
                    5'd5:  s_rdata <= reg_A[3];            // x_in[2]
                    5'd6:  s_rdata <= core_x_out[0];       // 0x30 — x_out[0]
                    5'd7:  s_rdata <= core_x_out[1];       // 0x38 — x_out[1]
                    5'd8:  s_rdata <= core_x_out[2];       // 0x40 — x_out[2]
                    5'd9:  s_rdata <= reg_A[7];
                    5'd10: s_rdata <= reg_A[8];
                    5'd11: s_rdata <= reg_B[0];
                    5'd12: s_rdata <= reg_B[1];
                    5'd13: s_rdata <= reg_B[2];
                    5'd14: s_rdata <= reg_B[3];
                    5'd15: s_rdata <= reg_B[4];
                    5'd16: s_rdata <= reg_B[5];
                    5'd17: s_rdata <= reg_B[6];
                    5'd18: s_rdata <= reg_B[7];
                    5'd19: s_rdata <= reg_B[8];
                    5'd20: s_rdata <= core_C[0];           // 0xA0 — P_out[0,0]
                    5'd21: s_rdata <= core_C[1];           // 0xA8 — P_out[0,1]
                    5'd22: s_rdata <= core_C[2];           // 0xB0 — P_out[0,2]
                    5'd23: s_rdata <= core_C[3];           // 0xB8 — P_out[1,0]
                    5'd24: s_rdata <= core_C[4];           // 0xC0 — P_out[1,1]
                    5'd25: s_rdata <= core_C[5];           // 0xC8 — P_out[1,2]
                    5'd26: s_rdata <= core_C[6];           // 0xD0 — P_out[2,0]
                    5'd27: s_rdata <= core_C[7];           // 0xD8 — P_out[2,1]
                    5'd28: s_rdata <= core_C[8];           // 0xE0 — P_out[2,2]
                    5'd29: s_rdata <= reg_R;               // 0xE8 — R_REG
                    default: s_rdata <= '0;
                endcase
            end else if (s_rvalid && s_rready) begin
                s_rvalid  <= 1'b0;
                s_arready <= 1'b1;
            end
        end
    end

    // -----------------------------------------------------------------------
    // Core control outputs
    // -----------------------------------------------------------------------
    assign core_start = start_pending;
    assign core_rst_n = rst_n & ~reg_ctrl[1];
    assign core_R     = reg_R;

    generate
        for (genvar gi = 0; gi < 9; gi++) begin : gen_core_ab
            assign core_A[gi] = reg_A[gi];
            assign core_B[gi] = reg_B[gi];
        end
    endgenerate

endmodule
