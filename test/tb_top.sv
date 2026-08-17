// =============================================================================
// tb_top.sv — SPI master BFM testbench for tt_um_joram200
// INT16 Q8.8 fixed-point, 1D (scalar) state.
//
// Protocol: 3-byte SPI (1 cmd byte + 2 data bytes for 16-bit INT16).
//
// Register map (8 registers):
//   0   CTRL       W    [0]=start, [1]=sw_rst
//   1   STAT       R    [0]=done,  [1]=busy
//   2   z          W    measurement Q8.8
//   3   x_in       W    prior scalar state
//   4   x_out      R    corrected scalar state
//   5   P_in       W    prior scalar covariance
//   6   P_out      R    updated scalar covariance
//   7   R_REG      R/W  measurement noise R (default 5.0 Q8.8 = 0x0500)
//
// Scenario: z=1.5, x_in=0, P_in=1.0, R=5.0
//   Expected: x_out=0x003F (63/256≈0.246), P_out=0x00D6 (214/256≈0.836)
// =============================================================================
`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// spi_master_bfm — SPI master bus functional model (3-byte protocol)
// -----------------------------------------------------------------------------
module spi_master_bfm #(
    parameter real SPI_CLK_NS = 80.0
)(
    input  logic        clk_sys,
    output logic        sclk,
    output logic        mosi,
    output logic        cs_n,
    input  logic        miso
);
    logic [15:0] burst_buf [0:8];

    integer _bb;
    initial begin
        sclk = 1'b0;
        mosi = 1'b0;
        cs_n = 1'b1;
        for (_bb = 0; _bb < 9; _bb = _bb + 1)
            burst_buf[_bb] = 16'h0;
    end

    task automatic spi_xfer_byte (
        input  logic [7:0] tx,
        output logic [7:0] rx
    );
        integer b;
        rx = 8'h0;
        for (b = 7; b >= 0; b = b - 1) begin
            mosi = tx[b];
            #(SPI_CLK_NS / 2.0);
            sclk = 1'b1;
            #(SPI_CLK_NS / 2.0);
            rx[b] = miso;
            sclk = 1'b0;
        end
    endtask

    // Write: 1 cmd byte + 2 data bytes (16-bit, MSB first)
    task automatic spi_write (
        input logic [6:0]  addr,
        input logic [15:0] data
    );
        logic [7:0] dummy;
        cs_n = 1'b0;
        #(SPI_CLK_NS);
        spi_xfer_byte(8'h80 | {1'b0, addr}, dummy);
        spi_xfer_byte(data[15:8], dummy);
        spi_xfer_byte(data[7:0],  dummy);
        #(SPI_CLK_NS);
        cs_n = 1'b1;
        #(SPI_CLK_NS * 2.0);
    endtask

    // Read: 1 cmd byte + 2 data bytes (16-bit, MSB first)
    task automatic spi_read (
        input  logic [6:0]  addr,
        output logic [15:0] data
    );
        logic [7:0] dummy, rx_b;
        data = 16'h0;
        cs_n = 1'b0;
        #(SPI_CLK_NS);
        spi_xfer_byte(8'h00 | {1'b0, addr}, dummy);
        spi_xfer_byte(8'h00, rx_b); data[15:8] = rx_b;
        spi_xfer_byte(8'h00, rx_b); data[7:0]  = rx_b;
        #(SPI_CLK_NS);
        cs_n = 1'b1;
        #(SPI_CLK_NS * 2.0);
    endtask

    task automatic spi_write_burst (
        input logic [6:0] start_addr,
        input int         n
    );
        logic [7:0] dummy;
        integer reg_i;
        cs_n = 1'b0;
        #(SPI_CLK_NS);
        spi_xfer_byte(8'h80 | {1'b0, start_addr}, dummy);
        for (reg_i = 0; reg_i < n; reg_i = reg_i + 1) begin
            spi_xfer_byte(burst_buf[reg_i][15:8], dummy);
            spi_xfer_byte(burst_buf[reg_i][7:0],  dummy);
        end
        #(SPI_CLK_NS);
        cs_n = 1'b1;
        #(SPI_CLK_NS * 2.0);
    endtask

    task automatic spi_read_burst (
        input logic [6:0] start_addr,
        input int         n
    );
        logic [7:0] dummy, rx_b;
        integer reg_i;
        cs_n = 1'b0;
        #(SPI_CLK_NS);
        spi_xfer_byte(8'h00 | {1'b0, start_addr}, dummy);
        for (reg_i = 0; reg_i < n; reg_i = reg_i + 1) begin
            burst_buf[reg_i] = 16'h0;
            spi_xfer_byte(8'h00, rx_b); burst_buf[reg_i][15:8] = rx_b;
            spi_xfer_byte(8'h00, rx_b); burst_buf[reg_i][7:0]  = rx_b;
        end
        #(SPI_CLK_NS);
        cs_n = 1'b1;
        #(SPI_CLK_NS * 2.0);
    endtask

    task automatic poll_done (
        input int timeout_ns
    );
        logic [15:0] stat;
        int elapsed;
        elapsed = 0;
        do begin
            #200;
            elapsed = elapsed + 200;
            spi_read(7'd1, stat);
        end while (!stat[0] && elapsed < timeout_ns);
        if (!stat[0]) $fatal(1, "TIMEOUT: done never asserted");
    endtask

endmodule


// -----------------------------------------------------------------------------
// result_checker — compare INT16 Q8.8 scalar outputs against golden reference
//
// Scenario: z=1.5, x=0, P=1.0, R=5.0  (Q8.8)
//   x_out = 0x003F (63/256 ≈ 0.246)
//   P_out = 0x00D6 (214/256 ≈ 0.836)
// -----------------------------------------------------------------------------
module result_checker #(
    parameter int ULP_THRESHOLD = 4
)();
    logic [15:0] REF_XOUT;
    logic [15:0] REF_POUT;
    logic [15:0] hw_x;
    logic [15:0] hw_p;

    initial begin
        REF_XOUT = 16'h003F;
        REF_POUT = 16'h00D6;
        hw_x = 16'h0;
        hw_p = 16'h0;
    end

    function automatic longint unsigned ulp_dist (
        input logic [15:0] a, b
    );
        longint signed ai, bi;
        ai = $signed(a);
        bi = $signed(b);
        ulp_dist = (ai > bi) ? longint'(ai - bi) : longint'(bi - ai);
    endfunction

    task automatic check ();
        int pass;
        longint unsigned d;
        pass = 1;

        d = ulp_dist(hw_x, REF_XOUT);
        if (d > ULP_THRESHOLD) begin
            $error("x_out: hw=%04h ref=%04h ULP=%0d FAIL", hw_x, REF_XOUT, d);
            pass = 0;
        end else begin
            $display("x_out: ULP=%0d PASS", d);
        end

        d = ulp_dist(hw_p, REF_POUT);
        if (d > ULP_THRESHOLD) begin
            $error("P_out: hw=%04h ref=%04h ULP=%0d FAIL", hw_p, REF_POUT, d);
            pass = 0;
        end else begin
            $display("P_out: ULP=%0d PASS", d);
        end

        if (pass)
            $display("RESULT_CHECKER: ALL OUTPUTS WITHIN %0d ULP — PASS", ULP_THRESHOLD);
        else
            $fatal(1, "RESULT_CHECKER: ONE OR MORE OUTPUTS EXCEEDED ULP THRESHOLD");
    endtask

endmodule


// -----------------------------------------------------------------------------
// program_block — INT16 Q8.8 test scenario data (1D scalar)
// Scenario: z=1.5, x_in=0, P_in=1.0, R=5.0
// -----------------------------------------------------------------------------
module program_block (
    input  logic clk
);
    logic [15:0] Z_VAL;
    logic [15:0] X_IN;
    logic [15:0] P_IN;

    initial begin
        Z_VAL = 16'h0180;  // 1.5 Q8.8
        X_IN  = 16'h0000;  // 0.0 Q8.8
        P_IN  = 16'h0100;  // 1.0 Q8.8
    end
endmodule


// -----------------------------------------------------------------------------
// tb_top — top-level testbench
// -----------------------------------------------------------------------------
module tb_top;

    logic clk, rst_n;
    initial clk = 1'b0;
    always #10 clk = ~clk;  // 50 MHz

    logic [7:0] ui_in, uo_out, uio_in, uio_out, uio_oe;
    logic       ena;

    logic sclk_bfm, mosi_bfm, cs_n_bfm, miso_bfm;
    assign uio_in[0] = sclk_bfm;
    assign uio_in[1] = mosi_bfm;
    assign uio_in[3] = cs_n_bfm;
    assign miso_bfm  = uio_out[2];
    assign uio_in[7:4] = 4'h0;
    assign uio_in[2]   = 1'b0;
    assign ui_in       = 8'h0;
    assign ena         = 1'b1;

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

    spi_master_bfm #(.SPI_CLK_NS(80.0)) u_bfm (
        .clk_sys (clk),
        .sclk    (sclk_bfm),
        .mosi    (mosi_bfm),
        .cs_n    (cs_n_bfm),
        .miso    (miso_bfm)
    );

    result_checker #(.ULP_THRESHOLD(4)) u_checker ();
    program_block u_prog (.clk(clk));

    initial begin
        $dumpfile("tb_top.fst");
        $dumpvars(0, tb_top);
    end

    initial begin
        rst_n = 1'b0;
        repeat(20) @(posedge clk);
        rst_n = 1'b1;
        repeat(10) @(posedge clk);

        $display("=== SPI Kalman update test (INT16 Q8.8, 1D scalar, 3-byte SPI) ===");

        // Write z (reg 2), x_in (reg 3) in one burst of 2
        u_bfm.burst_buf[0] = u_prog.Z_VAL;
        u_bfm.burst_buf[1] = u_prog.X_IN;
        u_bfm.spi_write_burst(7'd2, 2);

        // Write P_in (reg 5) individually
        u_bfm.spi_write(7'd5, u_prog.P_IN);

        // Fire: write CTRL.start = 1 (bit 0)
        u_bfm.spi_write(7'd0, 16'h0001);

        // Poll done (500 µs timeout)
        $display("Waiting for done...");
        u_bfm.poll_done(500000);
        $display("Done asserted.");

        // Read x_out (reg 4)
        u_bfm.spi_read(7'd4, u_checker.hw_x);

        // Read P_out (reg 6)
        u_bfm.spi_read(7'd6, u_checker.hw_p);

        // ULP comparison against golden reference
        u_checker.check();

        $display("=== Test complete ===");
        $finish;
    end

    // Timeout watchdog
    initial begin
        #10_000_000;
        $fatal(1, "WATCHDOG: simulation exceeded 10 ms limit");
    end

endmodule
