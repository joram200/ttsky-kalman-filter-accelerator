// =============================================================================
// tb_top.sv — Parallel GPIO testbench for tt_um_joram200
// INT16 Q8.8 fixed-point, 1D (scalar) state.
//
// Write protocol (byte-serial, MSB first):
//   Set addr, byte_sel=0, wr_data=MSB, pulse wr_en ≥1 clk.
//   Set byte_sel=1, wr_data=LSB, pulse wr_en ≥1 clk.
//   (2-FF synchroniser: hold signals stable for ≥2 clk after wr_en rises)
//
// Read protocol (combinational):
//   Set addr, byte_sel; read uo_out same cycle.
//
// Register map:
//   1   STAT      R    [0]=done_latch, [1]=busy
//   2   z         W    measurement Q8.8
//   3   x_in      W    prior scalar state
//   4   x_out     R    corrected scalar state
//   5   P_in      W    prior scalar covariance
//   6   P_out     R    updated scalar covariance
//   7   R_REG     R/W  measurement noise R (default 5.0 Q8.8 = 0x0500)
//
// Scenario: z=1.5, x_in=0, P_in=1.0, R=5.0
//   Expected: x_out=0x003F (63/256≈0.246), P_out=0x00D6 (214/256≈0.836)
// =============================================================================
`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// par_master_bfm — parallel GPIO bus functional model
// -----------------------------------------------------------------------------
module par_master_bfm (
    input  logic        clk,
    output logic [7:0]  ui_out,    // → ui_in of DUT  (write data)
    output logic [7:0]  uio_out,   // → uio_in of DUT (addr/ctrl)
    input  logic [7:0]  uo_in      // ← uo_out of DUT (read data)
);

    initial begin
        ui_out  = 8'h0;
        uio_out = 8'h0;
    end

    // Write one byte to MSB or LSB of addressed register.
    // Holds wr_en high for 3 posedge cycles so the 2-FF sync captures it.
    task automatic par_write_byte (
        input logic [2:0] addr,
        input logic       byte_sel,
        input logic [7:0] data
    );
        ui_out[7:0]  = data;
        uio_out[2:0] = addr;
        uio_out[3]   = byte_sel;
        uio_out[4]   = 1'b0;      // wr_en low
        @(posedge clk); #1;
        uio_out[4]   = 1'b1;      // wr_en rising edge
        @(posedge clk); #1;       // sync stage 1: wr_en_r1 = 1
        @(posedge clk); #1;       // sync stage 2: wr_rise = 1, write happens
        uio_out[4]   = 1'b0;
        @(posedge clk); #1;       // settling
    endtask

    // Write full 16-bit value: MSB byte then LSB byte.
    task automatic par_write (
        input logic [2:0]  addr,
        input logic [15:0] data
    );
        par_write_byte(addr, 1'b0, data[15:8]);
        par_write_byte(addr, 1'b1, data[7:0]);
    endtask

    // Read one byte (MSB or LSB) from addressed register.
    // rd_data is combinational; just set addr/byte_sel and sample uo_in.
    task automatic par_read_byte (
        input  logic [2:0] addr,
        input  logic       byte_sel,
        output logic [7:0] data
    );
        uio_out[2:0] = addr;
        uio_out[3]   = byte_sel;
        @(posedge clk); #1;
        data = uo_in;
    endtask

    // Read full 16-bit value.
    task automatic par_read (
        input  logic [2:0]  addr,
        output logic [15:0] data
    );
        logic [7:0] b;
        par_read_byte(addr, 1'b0, b); data[15:8] = b;
        par_read_byte(addr, 1'b1, b); data[7:0]  = b;
    endtask

    // Assert start pulse (rising-edge one-shot, 2-FF sync inside DUT).
    task automatic par_start ();
        uio_out[6] = 1'b0;
        @(posedge clk); #1;
        uio_out[6] = 1'b1;
        @(posedge clk); #1;
        @(posedge clk); #1;
        uio_out[6] = 1'b0;
        @(posedge clk); #1;
    endtask

    // Poll STAT register until done_latch (bit 0 of LSB byte) asserts.
    task automatic poll_done (input int timeout_cycles);
        logic [7:0] stat;
        int elapsed;
        elapsed = 0;
        uio_out[2:0] = 3'd1;   // STAT register
        uio_out[3]   = 1'b1;   // LSB byte (done in bit 0)
        do begin
            @(posedge clk); #1;
            stat    = uo_in;
            elapsed = elapsed + 1;
        end while (!stat[0] && elapsed < timeout_cycles);
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
        hw_x     = 16'h0;
        hw_p     = 16'h0;
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
        end else
            $display("x_out: ULP=%0d PASS", d);

        d = ulp_dist(hw_p, REF_POUT);
        if (d > ULP_THRESHOLD) begin
            $error("P_out: hw=%04h ref=%04h ULP=%0d FAIL", hw_p, REF_POUT, d);
            pass = 0;
        end else
            $display("P_out: ULP=%0d PASS", d);

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

    assign ena = 1'b1;

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

    par_master_bfm u_bfm (
        .clk     (clk),
        .ui_out  (ui_in),
        .uio_out (uio_in),
        .uo_in   (uo_out)
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
        repeat(5)  @(posedge clk);

        $display("=== Parallel GPIO Kalman update test (INT16 Q8.8, 1D scalar) ===");

        // Write z (reg 2), x_in (reg 3), P_in (reg 5)
        u_bfm.par_write(3'd2, u_prog.Z_VAL);
        u_bfm.par_write(3'd3, u_prog.X_IN);
        u_bfm.par_write(3'd5, u_prog.P_IN);

        // Fire: rising edge on start pin
        u_bfm.par_start();

        // Poll STAT until done (250 000 cycle timeout)
        $display("Waiting for done...");
        u_bfm.poll_done(250_000);
        $display("Done asserted.");

        // Read x_out (reg 4) and P_out (reg 6)
        u_bfm.par_read(3'd4, u_checker.hw_x);
        u_bfm.par_read(3'd6, u_checker.hw_p);

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
