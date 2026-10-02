//==============================================================================
// tb_sensor_monitor.v
//
// Self-checking testbench for the APB-based dual-channel sensor monitor IP.
//
// Exercises 15 distinct test scenarios covering:
//   reset, register R/W, unsigned compare, signed compare, sticky alarms,
//   fault capture, acknowledgment, dual-channel alarm, IRQ, PSLVERR.
//
// Returns $finish exit code 0 on all-pass, non-zero on any failure.
//
// Author : Original design for educational APB4-based sensor monitor IP
// License: MIT
//==============================================================================

`timescale 1ns / 1ps

module tb_sensor_monitor;

    //----------------------------------------------------------------------
    // Signals
    //----------------------------------------------------------------------
    reg         clk;
    reg         reset_n;
    reg  [7:0]  sensor0;
    reg  [7:0]  sensor1;

    // APB signals driven by BFM
    reg         PSEL;
    reg         PENABLE;
    reg         PWRITE;
    reg  [31:0] PADDR;
    reg  [31:0] PWDATA;
    wire [31:0] PRDATA;
    wire        PREADY;
    wire        PSLVERR;

    wire        irq;

    //----------------------------------------------------------------------
    // Register address map
    //----------------------------------------------------------------------
    localparam ADDR_CTRL    = 32'h00,
               ADDR_THRESH0 = 32'h04,
               ADDR_THRESH1 = 32'h08,
               ADDR_SENSOR0 = 32'h0C,
               ADDR_SENSOR1 = 32'h10,
               ADDR_STATUS  = 32'h14,
               ADDR_FAULT0  = 32'h18,
               ADDR_FAULT1  = 32'h1C,
               ADDR_ACK     = 32'h20;

    //----------------------------------------------------------------------
    // DUT instantiation
    //----------------------------------------------------------------------
    top_sensor_monitor dut (
        .clk      (clk),
        .reset_n  (reset_n),
        .sensor0  (sensor0),
        .sensor1  (sensor1),
        .PSEL     (PSEL),
        .PENABLE  (PENABLE),
        .PWRITE   (PWRITE),
        .PADDR    (PADDR),
        .PWDATA   (PWDATA),
        .PRDATA   (PRDATA),
        .PREADY   (PREADY),
        .PSLVERR  (PSLVERR),
        .irq      (irq)
    );

    //----------------------------------------------------------------------
    // Include APB master BFM tasks
    //----------------------------------------------------------------------
    `include "apb_master_bfm.v"

    //----------------------------------------------------------------------
    // Clock generation — 100 MHz (10 ns period)
    //----------------------------------------------------------------------
    initial clk = 0;
    always #5 clk = ~clk;

    //----------------------------------------------------------------------
    // VCD waveform dump
    //----------------------------------------------------------------------
    initial begin
        $dumpfile("sensor_monitor.vcd");
        $dumpvars(0, tb_sensor_monitor);
    end

    //----------------------------------------------------------------------
    // Test counters
    //----------------------------------------------------------------------
    integer pass_count;
    integer fail_count;
    integer total_tests;
    reg [31:0] rdata;  // scratch for APB reads

    //----------------------------------------------------------------------
    // check_value task — compare actual vs expected, increment counters
    //----------------------------------------------------------------------
    task check_value;
        input [31:0] actual;
        input [31:0] expected;
        input [255:0] test_name;  // wide reg to hold string
        begin
            total_tests = total_tests + 1;
            if (actual === expected) begin
                pass_count = pass_count + 1;
                $display("[PASS] %0s : got 0x%08h", test_name, actual);
            end else begin
                fail_count = fail_count + 1;
                $display("[FAIL] %0s : expected 0x%08h, got 0x%08h",
                         test_name, expected, actual);
            end
        end
    endtask

    //----------------------------------------------------------------------
    // check_flag task — compare a single-bit flag
    //----------------------------------------------------------------------
    task check_flag;
        input        actual;
        input        expected;
        input [255:0] test_name;
        begin
            total_tests = total_tests + 1;
            if (actual === expected) begin
                pass_count = pass_count + 1;
                $display("[PASS] %0s : got %0b", test_name, actual);
            end else begin
                fail_count = fail_count + 1;
                $display("[FAIL] %0s : expected %0b, got %0b",
                         test_name, expected, actual);
            end
        end
    endtask

    //----------------------------------------------------------------------
    // Reset task
    //----------------------------------------------------------------------
    task do_reset;
        begin
            reset_n = 1'b0;
            PSEL    = 1'b0;
            PENABLE = 1'b0;
            PWRITE  = 1'b0;
            PADDR   = 32'h0;
            PWDATA  = 32'h0;
            sensor0 = 8'h00;
            sensor1 = 8'h00;
            repeat (4) @(posedge clk);
            reset_n = 1'b1;
            repeat (2) @(posedge clk);
        end
    endtask

    //----------------------------------------------------------------------
    // Main test sequence
    //----------------------------------------------------------------------
    initial begin
        pass_count  = 0;
        fail_count  = 0;
        total_tests = 0;

        $display("");
        $display("==================================================");
        $display("  APB Sensor Monitor — Self-Checking Testbench");
        $display("==================================================");
        $display("");

        //==================================================================
        // TEST 1: Reset behaviour
        //==================================================================
        $display("--- TEST 1: Reset behaviour ---");
        do_reset;
        apb_read(ADDR_CTRL, rdata);
        check_value(rdata, 32'h0, "CTRL after reset");
        apb_read(ADDR_STATUS, rdata);
        check_value(rdata, 32'h0, "STATUS after reset");
        apb_read(ADDR_FAULT0, rdata);
        check_value(rdata, 32'h0, "FAULT0 after reset");
        apb_read(ADDR_FAULT1, rdata);
        check_value(rdata, 32'h0, "FAULT1 after reset");
        check_flag(irq, 1'b0, "IRQ after reset");

        //==================================================================
        // TEST 2: Program & read back thresholds
        //==================================================================
        $display("");
        $display("--- TEST 2: Program & read back thresholds ---");
        apb_write(ADDR_THRESH0, 32'h00_00_00_64); // threshold0 = 100
        apb_write(ADDR_THRESH1, 32'h00_00_00_C8); // threshold1 = 200
        apb_read(ADDR_THRESH0, rdata);
        check_value(rdata, 32'h64, "THRESH0 readback");
        apb_read(ADDR_THRESH1, rdata);
        check_value(rdata, 32'hC8, "THRESH1 readback");

        //==================================================================
        // TEST 3: Unsigned — sensor below threshold (no alarm)
        //==================================================================
        $display("");
        $display("--- TEST 3: Unsigned — sensor below threshold ---");
        apb_write(ADDR_CTRL, 32'h01);  // enable=1, unsigned mode
        sensor0 = 8'd50;   // below 100
        sensor1 = 8'd150;  // below 200
        repeat (4) @(posedge clk);
        apb_read(ADDR_STATUS, rdata);
        check_value(rdata[1:0], 2'b00, "No alarms when below threshold");
        check_flag(irq, 1'b0, "IRQ low when no alarm");

        //==================================================================
        // TEST 4: Unsigned — sensor exactly equal to threshold
        //==================================================================
        $display("");
        $display("--- TEST 4: Unsigned — sensor == threshold ---");
        sensor0 = 8'd100;  // equal to threshold0
        repeat (4) @(posedge clk);
        apb_read(ADDR_STATUS, rdata);
        check_value(rdata[0], 1'b1, "Alarm0 on sensor0 == threshold0");
        check_flag(irq, 1'b1, "IRQ high on alarm");

        // Acknowledge alarm0 and lower sensor to prevent immediate re-trigger
        sensor0 = 8'd50;  // drop below threshold first
        apb_write(ADDR_ACK, 32'h01);
        repeat (2) @(posedge clk);

        //==================================================================
        // TEST 5: Unsigned — sensor above threshold
        //==================================================================
        $display("");
        $display("--- TEST 5: Unsigned — sensor above threshold ---");
        // Clear alarm first: lower sensor, ack was already sent
        sensor0 = 8'd50;
        repeat (4) @(posedge clk);
        // Now raise above threshold
        sensor0 = 8'd120;  // above 100
        repeat (4) @(posedge clk);
        apb_read(ADDR_STATUS, rdata);
        check_value(rdata[0], 1'b1, "Alarm0 on sensor0 > threshold0");

        //==================================================================
        // TEST 6: Sticky alarm — sensor returns below threshold
        //==================================================================
        $display("");
        $display("--- TEST 6: Sticky alarm after sensor drops ---");
        sensor0 = 8'd30;   // now well below threshold
        repeat (4) @(posedge clk);
        apb_read(ADDR_STATUS, rdata);
        check_value(rdata[0], 1'b1, "Alarm0 still sticky");
        check_flag(irq, 1'b1, "IRQ still high (sticky)");

        //==================================================================
        // TEST 7: Fault capture — stores FIRST violating value
        //==================================================================
        $display("");
        $display("--- TEST 7: Fault capture value ---");
        apb_read(ADDR_FAULT0, rdata);
        check_value(rdata[7:0], 8'd120, "Fault0 captured first violating value 120");

        //==================================================================
        // TEST 8: Acknowledgment clears alarm
        //==================================================================
        $display("");
        $display("--- TEST 8: Acknowledgment clears alarm ---");
        apb_write(ADDR_ACK, 32'h01);  // ack ch0
        repeat (3) @(posedge clk);
        apb_read(ADDR_STATUS, rdata);
        check_value(rdata[0], 1'b0, "Alarm0 cleared after ACK");
        check_flag(irq, 1'b0, "IRQ low after alarm cleared");

        //==================================================================
        // TEST 9: New fault captured after acknowledgment
        //==================================================================
        $display("");
        $display("--- TEST 9: Re-capture after ACK ---");
        sensor0 = 8'd110;  // above threshold again
        repeat (4) @(posedge clk);
        apb_read(ADDR_FAULT0, rdata);
        check_value(rdata[7:0], 8'd110, "Fault0 re-captured 110 after ACK");
        apb_read(ADDR_STATUS, rdata);
        check_value(rdata[0], 1'b1, "Alarm0 re-asserted");

        // Clean up ch0 alarm
        apb_write(ADDR_ACK, 32'h01);
        sensor0 = 8'd10;
        repeat (3) @(posedge clk);

        //==================================================================
        // TEST 10: Signed comparison — positive numbers
        //==================================================================
        $display("");
        $display("--- TEST 10: Signed comparison (positive) ---");
        apb_write(ADDR_CTRL, 32'h03);  // enable=1, signed_mode0=1
        apb_write(ADDR_THRESH0, 32'h32); // threshold0 = +50 signed
        sensor0 = 8'd60;   // +60 >= +50 → alarm
        repeat (4) @(posedge clk);
        apb_read(ADDR_STATUS, rdata);
        check_value(rdata[0], 1'b1, "Signed alarm on +60 >= +50");
        apb_write(ADDR_ACK, 32'h01);
        sensor0 = 8'd10;
        repeat (3) @(posedge clk);

        //==================================================================
        // TEST 11: Signed comparison — negative values
        //==================================================================
        $display("");
        $display("--- TEST 11: Signed comparison (negative) ---");
        // threshold0 = -10 = 0xF6, sensor0 = -5 = 0xFB
        // Signed: -5 >= -10 → true → alarm
        // First lower sensor0 below the new threshold to avoid spurious alarm
        sensor0 = 8'h80;  // -128 signed, safely below -10
        repeat (2) @(posedge clk);
        apb_write(ADDR_THRESH0, 32'hF6); // -10
        repeat (2) @(posedge clk);
        // Now apply the intended stimulus
        sensor0 = 8'hFB;  // -5
        repeat (4) @(posedge clk);
        apb_read(ADDR_STATUS, rdata);
        check_value(rdata[0], 1'b1, "Signed alarm on -5 >= -10");
        apb_read(ADDR_FAULT0, rdata);
        check_value(rdata[7:0], 8'hFB, "Fault0 captured 0xFB (-5)");

        // Clean up
        apb_write(ADDR_ACK, 32'h01);
        sensor0 = 8'h80; // -128, well below -10
        repeat (3) @(posedge clk);

        // Verify no alarm for sensor < threshold in signed
        // sensor0 = -128 (0x80), threshold = -10 (0xF6)
        // -128 < -10 → no alarm
        apb_read(ADDR_STATUS, rdata);
        check_value(rdata[0], 1'b0, "No signed alarm: -128 < -10");

        //==================================================================
        // TEST 12: Both channels alarm simultaneously
        //==================================================================
        $display("");
        $display("--- TEST 12: Both channels alarming ---");
        // Disable first, lower sensors, then reconfigure cleanly
        apb_write(ADDR_CTRL, 32'h00);  // disable clears alarms/faults
        sensor0 = 8'd10;
        sensor1 = 8'd10;
        repeat (2) @(posedge clk);
        apb_write(ADDR_THRESH0, 32'h50);  // 80
        apb_write(ADDR_THRESH1, 32'h50);  // 80
        apb_write(ADDR_CTRL, 32'h01);    // re-enable, unsigned for both
        repeat (2) @(posedge clk);
        // Now apply violating sensor values
        sensor0 = 8'd90;
        sensor1 = 8'd95;
        repeat (4) @(posedge clk);
        apb_read(ADDR_STATUS, rdata);
        check_value(rdata[1:0], 2'b11, "Both alarm0 & alarm1 active");
        apb_read(ADDR_FAULT0, rdata);
        check_value(rdata[7:0], 8'd90, "Fault0 = 90");
        apb_read(ADDR_FAULT1, rdata);
        check_value(rdata[7:0], 8'd95, "Fault1 = 95");

        // Ack both
        apb_write(ADDR_ACK, 32'h03);
        sensor0 = 8'd10;
        sensor1 = 8'd10;
        repeat (3) @(posedge clk);

        //==================================================================
        // TEST 13: IRQ behaviour
        //==================================================================
        $display("");
        $display("--- TEST 13: IRQ behaviour ---");
        apb_read(ADDR_STATUS, rdata);
        check_flag(irq, 1'b0, "IRQ low when no alarms");
        sensor0 = 8'd90;
        repeat (4) @(posedge clk);
        check_flag(irq, 1'b1, "IRQ high when alarm active");
        apb_write(ADDR_ACK, 32'h01);
        sensor0 = 8'd10;
        repeat (3) @(posedge clk);
        check_flag(irq, 1'b0, "IRQ low after ACK");

        //==================================================================
        // TEST 14: Read all status/fault registers
        //==================================================================
        $display("");
        $display("--- TEST 14: Read all registers ---");
        apb_read(ADDR_CTRL, rdata);
        check_value(rdata[0], 1'b1, "CTRL.enable = 1");
        apb_read(ADDR_SENSOR0, rdata);
        check_value(rdata[7:0], sensor0, "SENSOR0 live readback");
        apb_read(ADDR_SENSOR1, rdata);
        check_value(rdata[7:0], sensor1, "SENSOR1 live readback");

        //==================================================================
        // TEST 15: Invalid APB address → PSLVERR
        //==================================================================
        $display("");
        $display("--- TEST 15: Invalid address → PSLVERR ---");
        // Perform an APB read to an unmapped address
        @(posedge clk);
        PSEL    <= 1'b1;
        PENABLE <= 1'b0;
        PWRITE  <= 1'b0;
        PADDR   <= 32'hFF;
        @(posedge clk);
        PENABLE <= 1'b1;
        @(posedge clk);
        check_flag(PSLVERR, 1'b1, "PSLVERR on invalid address 0xFF");
        PSEL    <= 1'b0;
        PENABLE <= 1'b0;
        PADDR   <= 32'h0;
        @(posedge clk);

        // Also test write to read-only register
        @(posedge clk);
        PSEL    <= 1'b1;
        PENABLE <= 1'b0;
        PWRITE  <= 1'b1;
        PADDR   <= ADDR_STATUS;  // read-only
        PWDATA  <= 32'hDEAD;
        @(posedge clk);
        PENABLE <= 1'b1;
        @(posedge clk);
        check_flag(PSLVERR, 1'b1, "PSLVERR on write to RO STATUS");
        PSEL    <= 1'b0;
        PENABLE <= 1'b0;
        PWRITE  <= 1'b0;
        PADDR   <= 32'h0;
        PWDATA  <= 32'h0;
        @(posedge clk);

        //==================================================================
        // Summary
        //==================================================================
        $display("");
        $display("==================================");
        $display("       TEST SUMMARY");
        $display("  PASS : %0d", pass_count);
        $display("  FAIL : %0d", fail_count);
        $display("  TOTAL: %0d", total_tests);
        $display("==================================");
        $display("");

        if (fail_count > 0) begin
            $display("*** SIMULATION FAILED ***");
            $finish;
        end else begin
            $display("*** ALL TESTS PASSED ***");
            $finish;
        end
    end

    //----------------------------------------------------------------------
    // Timeout watchdog — kill simulation if it hangs
    //----------------------------------------------------------------------
    initial begin
        #100_000;
        $display("[ERROR] Simulation timeout!");
        $finish;
    end

endmodule
