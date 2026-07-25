`timescale 1ns/1ps
// =============================================================================
// top.sv — M4: Option B Kalman Update Accelerator — Integration Top
// Extracted from post_m3_Minor_Redesign/rtl/top.sv
// Contains: top (integration wrapper only)
// Instantiates: axilite_slave (interface.sv), kalman_update (compute_core.sv)
// =============================================================================
module top #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 64
)(
    input  logic                      clk,
    input  logic                      rst_n,
    input  logic [ADDR_WIDTH-1:0]     s_awaddr,
    input  logic                      s_awvalid,
    output logic                      s_awready,
    input  logic [DATA_WIDTH-1:0]     s_wdata,
    input  logic [(DATA_WIDTH/8)-1:0] s_wstrb,
    input  logic                      s_wvalid,
    output logic                      s_wready,
    output logic [1:0]                s_bresp,
    output logic                      s_bvalid,
    input  logic                      s_bready,
    input  logic [ADDR_WIDTH-1:0]     s_araddr,
    input  logic                      s_arvalid,
    output logic                      s_arready,
    output logic [DATA_WIDTH-1:0]     s_rdata,
    output logic [1:0]                s_rresp,
    output logic                      s_rvalid,
    input  logic                      s_rready
);

    logic        core_start, core_rst_n, core_done, core_busy;
    logic [63:0] slave_A [0:8];  // A_REG: [0]=z, [1:3]=x_in, [4:6]=unused writes
    logic [63:0] slave_B [0:8];  // B_REG: [0:8]=P_in
    logic [63:0] slave_C [0:8];  // C_REG: [0:8]=P_out (all 9)
    logic [63:0] r_wire;         // R_REG value → kalman_update.r_val

    // x_in extracted from slave_A[1:3]
    logic [63:0] x_in_wire [0:2];
    assign x_in_wire[0] = slave_A[1];
    assign x_in_wire[1] = slave_A[2];
    assign x_in_wire[2] = slave_A[3];

    // kalman_update outputs
    logic [63:0] x_out [0:2];
    logic [63:0] P_out [0:8];

    // C_REG[0:8] = P_out[0:8] (all 9 covariance output elements)
    generate
        for (genvar gi = 0; gi < 9; gi++) begin : gen_slave_c
            assign slave_C[gi] = P_out[gi];
        end
    endgenerate

    axilite_slave #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .DATA_WIDTH (DATA_WIDTH)
    ) u_slave (
        .clk        (clk),
        .rst_n      (rst_n),
        .s_awaddr   (s_awaddr),  .s_awvalid (s_awvalid), .s_awready (s_awready),
        .s_wdata    (s_wdata),   .s_wstrb   (s_wstrb),
        .s_wvalid   (s_wvalid),  .s_wready  (s_wready),
        .s_bresp    (s_bresp),   .s_bvalid  (s_bvalid),  .s_bready  (s_bready),
        .s_araddr   (s_araddr),  .s_arvalid (s_arvalid), .s_arready (s_arready),
        .s_rdata    (s_rdata),   .s_rresp   (s_rresp),
        .s_rvalid   (s_rvalid),  .s_rready  (s_rready),
        .core_start (core_start),
        .core_rst_n (core_rst_n),
        .core_A     (slave_A),
        .core_B     (slave_B),
        .core_R     (r_wire),
        .core_C     (slave_C),
        .core_x_out (x_out),     // x_out readable at A_REG[4:6] (0x30–0x40)
        .core_done  (core_done),
        .core_busy  (core_busy)
    );

    kalman_update u_core (
        .clk   (clk),
        .rst_n (core_rst_n),
        .start (core_start),
        .z     (slave_A[0]),
        .x_in  (x_in_wire),
        .P_in  (slave_B),
        .r_val (r_wire),
        .x_out (x_out),
        .P_out (P_out),
        .done  (core_done),
        .busy  (core_busy)
    );

endmodule
