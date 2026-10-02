//==============================================================================
// top_sensor_monitor.v
//
// Top-level wrapper that integrates:
//   • apb_sensor_monitor  — APB4 slave register interface
//   • sensor_monitor_core — dual-channel monitoring engine
//
// This module is the Design Under Test (DUT) instantiated by the testbench
// and the synthesis entry point.
//
// Author : Original design for educational APB4-based sensor monitor IP
// License: MIT
//==============================================================================

module top_sensor_monitor (
    // Clock and active-low reset
    input  wire        clk,
    input  wire        reset_n,

    // Sensor data inputs (directly from external pads / upstream logic)
    input  wire [7:0]  sensor0,
    input  wire [7:0]  sensor1,

    // APB4 slave interface
    input  wire        PSEL,
    input  wire        PENABLE,
    input  wire        PWRITE,
    input  wire [31:0] PADDR,
    input  wire [31:0] PWDATA,
    output wire [31:0] PRDATA,
    output wire        PREADY,
    output wire        PSLVERR,

    // Interrupt output
    output wire        irq
);

    //--------------------------------------------------------------------------
    // Internal wiring between APB register block and sensor core
    //--------------------------------------------------------------------------
    wire        enable;
    wire        signed_mode0;
    wire        signed_mode1;
    wire [7:0]  threshold0;
    wire [7:0]  threshold1;
    wire        ack0;
    wire        ack1;

    wire        alarm0;
    wire        alarm1;
    wire [7:0]  fault_value0;
    wire [7:0]  fault_value1;
    wire        core_irq;

    //--------------------------------------------------------------------------
    // APB4 slave — register interface
    //--------------------------------------------------------------------------
    apb_sensor_monitor u_apb (
        .PCLK         (clk),
        .PRESETn      (reset_n),
        .PSEL         (PSEL),
        .PENABLE      (PENABLE),
        .PWRITE       (PWRITE),
        .PADDR        (PADDR),
        .PWDATA       (PWDATA),
        .PRDATA       (PRDATA),
        .PREADY       (PREADY),
        .PSLVERR      (PSLVERR),

        .sensor0      (sensor0),
        .sensor1      (sensor1),

        .alarm0       (alarm0),
        .alarm1       (alarm1),
        .fault_value0 (fault_value0),
        .fault_value1 (fault_value1),
        .irq          (core_irq),

        .enable       (enable),
        .signed_mode0 (signed_mode0),
        .signed_mode1 (signed_mode1),
        .threshold0   (threshold0),
        .threshold1   (threshold1),
        .ack0         (ack0),
        .ack1         (ack1)
    );

    //--------------------------------------------------------------------------
    // Sensor monitor core — dual-channel comparator + FSM + fault capture
    //--------------------------------------------------------------------------
    sensor_monitor_core u_core (
        .clk          (clk),
        .reset_n      (reset_n),

        .sensor0      (sensor0),
        .sensor1      (sensor1),

        .threshold0   (threshold0),
        .threshold1   (threshold1),

        .enable       (enable),
        .signed_mode0 (signed_mode0),
        .signed_mode1 (signed_mode1),
        .ack0         (ack0),
        .ack1         (ack1),

        .alarm0       (alarm0),
        .alarm1       (alarm1),
        .fault_value0 (fault_value0),
        .fault_value1 (fault_value1),
        .irq          (core_irq)
    );

    //--------------------------------------------------------------------------
    // IRQ output
    //--------------------------------------------------------------------------
    assign irq = core_irq;

endmodule
