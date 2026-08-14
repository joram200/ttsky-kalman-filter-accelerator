// =============================================================================
// tb_top.sv — SPI master BFM testbench for tt_um_joram200
// All testbench modules combined into a single file.
//
// Module hierarchy:
//   spi_master_bfm   — Synthesisable-style SPI master with named tasks
//   result_checker   — F64 ULP comparator (4-ULP threshold)
//   program_block    — One Kalman measurement-update scenario
//   tb_top           — Top-level testbench
//
// SPI protocol: CPOL=0, CPHA=0, MSB-first, 8-bit frames.
// Command byte: [7]=R/W (1=write, 0=read), [6:0]=register index.
// Data: 8 bytes per 64-bit register, MSB-first. Auto-increment in burst.
//
// Register map:
//   0   CTRL       W    [0]=start, [1]=sw_rst
//   1   STAT       R    [0]=done,  [1]=busy
//   2   z          W    measurement F64
//   3-5  x_in[0:2] W   prior state
//   6-8  x_out[0:2] R  corrected state
//   9-17 P_in[0:8] W   prior covariance (row-major)
//  18-26 P_out[0:8] R  updated covariance
//  27   R_REG      R/W  measurement noise R (default 5.0)
// =============================================================================
`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// spi_master_bfm — SPI master bus functional model
// Drives SCLK, MOSI, CS_N and samples MISO.
// All tasks are blocking (fork/join style via wait).
// SCLK period = SPI_CLK_NS (default 40 ns = 25 MHz).
// -----------------------------------------------------------------------------
module spi_master_bfm #(
    parameter real SPI_CLK_NS = 40.0  // SCLK period in ns
)(
    input  logic        clk_sys,
    output logic        sclk,
    output logic        mosi,
    output logic        cs_n,
    input  logic        miso
);
    initial begin
        sclk = 1'b0;
        mosi = 1'b0;
        cs_n = 1'b1;
    end

    // -----------------------------------------------------------------------
    // spi_xfer_byte — transfer one byte MSB-first, return received byte
    // -----------------------------------------------------------------------
    task automatic spi_xfer_byte (
        input  logic [7:0] tx,
        output logic [7:0] rx
    );
        rx = 8'h0;
        for (int b = 7; b >= 0; b--) begin
            mosi = tx[b];
            #(SPI_CLK_NS / 2.0);
            sclk = 1'b1;
            #(SPI_CLK_NS / 2.0);
            rx[b] = miso;
            sclk = 1'b0;
        end
    endtask

    // -----------------------------------------------------------------------
    // spi_write — write a single 64-bit register
    // -----------------------------------------------------------------------
    task automatic spi_write (
        input logic [6:0]  addr,
        input logic [63:0] data
    );
        logic [7:0] dummy;
        cs_n = 1'b0;
        #(SPI_CLK_NS);
        spi_xfer_byte(8'h80 | {1'b0, addr}, dummy);  // write command
        for (int byte_idx = 7; byte_idx >= 0; byte_idx--) begin
            spi_xfer_byte(data[byte_idx*8 +: 8], dummy);
        end
        #(SPI_CLK_NS);
        cs_n = 1'b1;
        #(SPI_CLK_NS * 2.0);
    endtask

    // -----------------------------------------------------------------------
    // spi_read — read a single 64-bit register
    // -----------------------------------------------------------------------
    task automatic spi_read (
        input  logic [6:0]  addr,
        output logic [63:0] data
    );
        logic [7:0] dummy;
        data = 64'h0;
        cs_n = 1'b0;
        #(SPI_CLK_NS);
        spi_xfer_byte(8'h00 | {1'b0, addr}, dummy);  // read command
        for (int byte_idx = 7; byte_idx >= 0; byte_idx--) begin
            logic [7:0] rx_b;
            spi_xfer_byte(8'h00, rx_b);
            data[byte_idx*8 +: 8] = rx_b;
        end
        #(SPI_CLK_NS);
        cs_n = 1'b1;
        #(SPI_CLK_NS * 2.0);
    endtask

    // -----------------------------------------------------------------------
    // spi_write_burst — write N contiguous 64-bit registers
    // -----------------------------------------------------------------------
    task automatic spi_write_burst (
        input logic [6:0]  start_addr,
        input logic [63:0] data [],
        input int          n
    );
        logic [7:0] dummy;
        cs_n = 1'b0;
        #(SPI_CLK_NS);
        spi_xfer_byte(8'h80 | {1'b0, start_addr}, dummy);
        for (int reg_i = 0; reg_i < n; reg_i++) begin
            for (int byte_idx = 7; byte_idx >= 0; byte_idx--) begin
                spi_xfer_byte(data[reg_i][byte_idx*8 +: 8], dummy);
            end
        end
        #(SPI_CLK_NS);
        cs_n = 1'b1;
        #(SPI_CLK_NS * 2.0);
    endtask

    // -----------------------------------------------------------------------
    // spi_read_burst — read N contiguous 64-bit registers
    // -----------------------------------------------------------------------
    task automatic spi_read_burst (
        input  logic [6:0]  start_addr,
        output logic [63:0] data [],
        input  int          n
    );
        logic [7:0] dummy;
        cs_n = 1'b0;
        #(SPI_CLK_NS);
        spi_xfer_byte(8'h00 | {1'b0, start_addr}, dummy);
        for (int reg_i = 0; reg_i < n; reg_i++) begin
            data[reg_i] = 64'h0;
            for (int byte_idx = 7; byte_idx >= 0; byte_idx--) begin
                logic [7:0] rx_b;
                spi_xfer_byte(8'h00, rx_b);
                data[reg_i][byte_idx*8 +: 8] = rx_b;
            end
        end
        #(SPI_CLK_NS);
        cs_n = 1'b1;
        #(SPI_CLK_NS * 2.0);
    endtask

    // -----------------------------------------------------------------------
    // poll_done — polls STAT until done is set, with timeout
    // -----------------------------------------------------------------------
    task automatic poll_done (
        input int timeout_ns
    );
        logic [63:0] stat;
        int elapsed = 0;
        do begin
            #200;
            elapsed += 200;
            spi_read(7'd1, stat);
        end while (!stat[0] && elapsed < timeout_ns);
        if (!stat[0]) $fatal(1, "TIMEOUT: done never asserted");
    endtask

endmodule


// -----------------------------------------------------------------------------
// result_checker — compare F64 hardware outputs against pre-computed golden ref
// Checks x_out[0:2] and P_out[0:8]; fails if any exceeds ULP_THRESHOLD.
// -----------------------------------------------------------------------------
module result_checker #(
    parameter int ULP_THRESHOLD = 4
)();
    // Golden reference values for the single test scenario:
    //   z=1.5, x_in=[0,0,0], P_in=I3, r=5.0
    // After one Kalman update (3-iteration NR reciprocal):
    //   S = 1.0 + 5.0 = 6.0
    //   S_inv ≈ 1/6
    //   K = [1/6, 0, 0]
    //   y = 1.5
    //   x_out = [1.5/6, 0, 0] = [0.25, 0, 0]
    //   P_out = (I - K*H)*P = diag(5/6, 1, 1)
    //
    // Note: NR gives exact result for S=6.0 within F64 precision.
    localparam logic [63:0] REF_XOUT [0:2] = '{
        64'h3FD0000000000000,  // 0.25
        64'h0000000000000000,  // 0.0
        64'h0000000000000000   // 0.0
    };
    localparam logic [63:0] REF_POUT [0:8] = '{
        64'h3FE5555555555555,  // 5/6 ≈ 0.8333...
        64'h0000000000000000,
        64'h0000000000000000,
        64'h0000000000000000,
        64'h3FF0000000000000,  // 1.0
        64'h0000000000000000,
        64'h0000000000000000,
        64'h0000000000000000,
        64'h3FF0000000000000   // 1.0
    };

    // ULP distance (signed-integer reinterpret)
    function automatic longint unsigned ulp_dist (
        input logic [63:0] a, b
    );
        longint signed ai, bi;
        ai = $signed(a);
        bi = $signed(b);
        ulp_dist = (ai > bi) ? longint'(ai - bi) : longint'(bi - ai);
    endfunction

    task automatic check (
        input logic [63:0] x_out [0:2],
        input logic [63:0] P_out [0:8]
    );
        int pass = 1;
        longint unsigned d;
        for (int i = 0; i < 3; i++) begin
            d = ulp_dist(x_out[i], REF_XOUT[i]);
            if (d > ULP_THRESHOLD) begin
                $error("x_out[%0d]: hw=%016h ref=%016h ULP=%0d FAIL", i, x_out[i], REF_XOUT[i], d);
                pass = 0;
            end else begin
                $display("x_out[%0d]: ULP=%0d PASS", i, d);
            end
        end
        for (int i = 0; i < 9; i++) begin
            d = ulp_dist(P_out[i], REF_POUT[i]);
            if (d > ULP_THRESHOLD) begin
                $error("P_out[%0d]: hw=%016h ref=%016h ULP=%0d FAIL", i, P_out[i], REF_POUT[i], d);
                pass = 0;
            end else begin
                $display("P_out[%0d]: ULP=%0d PASS", i, d);
            end
        end
        if (pass)
            $display("RESULT_CHECKER: ALL OUTPUTS WITHIN %0d ULP — PASS", ULP_THRESHOLD);
        else
            $fatal(1, "RESULT_CHECKER: ONE OR MORE OUTPUTS EXCEEDED ULP THRESHOLD");
    endtask

endmodule


// -----------------------------------------------------------------------------
// program_block — test scenario data and top-level control
// Scenario: z=1.5, x_in=[0,0,0], P_in=I3, r=5.0
// -----------------------------------------------------------------------------
module program_block (
    input  logic clk
);
    // Scenario inputs (F64 bit patterns)
    localparam logic [63:0] Z_VAL    = 64'h3FF8000000000000; // 1.5
    localparam logic [63:0] X_IN [0:2] = '{
        64'h0000000000000000,  // 0.0
        64'h0000000000000000,
        64'h0000000000000000
    };
    localparam logic [63:0] P_IN [0:8] = '{
        64'h3FF0000000000000, 64'h0000000000000000, 64'h0000000000000000,  // row 0
        64'h0000000000000000, 64'h3FF0000000000000, 64'h0000000000000000,  // row 1
        64'h0000000000000000, 64'h0000000000000000, 64'h3FF0000000000000   // row 2
    };
    // R_REG default (5.0) is already loaded at reset — no explicit write needed
endmodule


// -----------------------------------------------------------------------------
// tb_top — top-level testbench
// -----------------------------------------------------------------------------
module tb_top;

    // Clock and reset
    logic clk, rst_n;
    initial clk = 1'b0;
    always #10 clk = ~clk;  // 50 MHz (20 ns period)

    // DUT signals
    logic [7:0] ui_in, uo_out, uio_in, uio_out, uio_oe;
    logic       ena;

    // SPI signals wired to uio_in/uio_out
    logic sclk_bfm, mosi_bfm, cs_n_bfm, miso_bfm;
    assign uio_in[0] = sclk_bfm;
    assign uio_in[1] = mosi_bfm;
    assign uio_in[3] = cs_n_bfm;
    assign miso_bfm  = uio_out[2];
    assign uio_in[7:4] = 4'h0;
    assign uio_in[2]   = 1'b0;
    assign ui_in       = 8'h0;
    assign ena         = 1'b1;

    // DUT
    tt_um_joram200 dut (
        .ui_in   (ui_in),
        .uo_out  (uo_out),
        .uio_in  (uio_in),
        .uio_out (uio_out),
        .uio_oe  (uio_oe),
        .ena     (ena),
        .clk     (clk),
        .rst_n   (rst_n)
    );

    // SPI master BFM
    spi_master_bfm #(.SPI_CLK_NS(80.0)) u_bfm (
        .clk_sys (clk),
        .sclk    (sclk_bfm),
        .mosi    (mosi_bfm),
        .cs_n    (cs_n_bfm),
        .miso    (miso_bfm)
    );

    // Result checker
    result_checker #(.ULP_THRESHOLD(4)) u_checker ();

    // Program data
    program_block u_prog (.clk(clk));

    // FST dump
    initial begin
        $dumpfile("tb_top.fst");
        $dumpvars(0, tb_top);
    end

    // Test sequence
    initial begin
        logic [63:0] write_buf [];
        logic [63:0] x_out_hw  [0:2];
        logic [63:0] P_out_hw  [0:8];
        logic [63:0] x_rd      [];
        logic [63:0] P_rd      [];

        // Reset
        rst_n = 1'b0;
        repeat(20) @(posedge clk);
        rst_n = 1'b1;
        repeat(10) @(posedge clk);

        $display("=== SPI Kalman update test ===");

        // Write z, x_in[0:2] in one burst (regs 2-5)
        write_buf = new[4];
        write_buf[0] = u_prog.Z_VAL;
        write_buf[1] = u_prog.X_IN[0];
        write_buf[2] = u_prog.X_IN[1];
        write_buf[3] = u_prog.X_IN[2];
        u_bfm.spi_write_burst(7'd2, write_buf, 4);

        // Write P_in[0:8] (regs 9-17)
        write_buf = new[9];
        for (int i = 0; i < 9; i++) write_buf[i] = u_prog.P_IN[i];
        u_bfm.spi_write_burst(7'd9, write_buf, 9);

        // Fire: write CTRL.start = 1
        u_bfm.spi_write(7'd0, 64'h0000000000000001);

        // Poll done
        $display("Waiting for done...");
        u_bfm.poll_done(500000);
        $display("Done asserted.");

        // Read x_out[0:2]
        x_rd = new[3];
        u_bfm.spi_read_burst(7'd6, x_rd, 3);
        for (int i = 0; i < 3; i++) x_out_hw[i] = x_rd[i];

        // Read P_out[0:8]
        P_rd = new[9];
        u_bfm.spi_read_burst(7'd18, P_rd, 9);
        for (int i = 0; i < 9; i++) P_out_hw[i] = P_rd[i];

        // Check results
        u_checker.check(x_out_hw, P_out_hw);

        $display("=== Test complete ===");
        $finish;
    end

    // Timeout watchdog
    initial begin
        #10_000_000;  // 10 ms sim time limit
        $fatal(1, "WATCHDOG: simulation exceeded 10 ms limit");
    end

endmodule
