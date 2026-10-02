//==============================================================================
// sensor_monitor_core.v
//
// Dual-channel sensor monitoring engine with configurable thresholds,
// signed/unsigned comparison, sticky alarms, fault-value capture,
// and interrupt generation.
//
// Author : Original design for educational APB4-based sensor monitor IP
// License: MIT
//==============================================================================

module sensor_monitor_core (
    input  wire        clk,
    input  wire        reset_n,

    // Live sensor inputs
    input  wire [7:0]  sensor0,
    input  wire [7:0]  sensor1,

    // Programmable thresholds (from register interface)
    input  wire [7:0]  threshold0,
    input  wire [7:0]  threshold1,

    // Control signals (from register interface)
    input  wire        enable,        // Global monitoring enable
    input  wire        signed_mode0,  // 1 = signed comparison for ch0
    input  wire        signed_mode1,  // 1 = signed comparison for ch1
    input  wire        ack0,          // Software acknowledge for ch0
    input  wire        ack1,          // Software acknowledge for ch1

    // Status outputs (to register interface)
    output reg         alarm0,
    output reg         alarm1,
    output reg  [7:0]  fault_value0,
    output reg  [7:0]  fault_value1,
    output wire        irq
);

    //--------------------------------------------------------------------------
    // FSM State Encoding
    //--------------------------------------------------------------------------
    // IDLE    (2'b00) : Monitoring disabled. All outputs inactive.
    // MONITOR (2'b01) : Actively comparing sensors against thresholds.
    // ALARM   (2'b10) : At least one channel has a sticky alarm active.
    //--------------------------------------------------------------------------
    localparam [1:0] ST_IDLE    = 2'b00,
                     ST_MONITOR = 2'b01,
                     ST_ALARM   = 2'b10;

    reg [1:0] state, next_state;

    //--------------------------------------------------------------------------
    // Threshold violation detection — combinational
    //--------------------------------------------------------------------------
    // Unsigned : alarm when sensor >= threshold
    // Signed   : interpret both as signed 8-bit and compare
    //--------------------------------------------------------------------------
    wire violate0_unsigned = (sensor0 >= threshold0);
    wire violate1_unsigned = (sensor1 >= threshold1);

    wire violate0_signed   = ($signed(sensor0) >= $signed(threshold0));
    wire violate1_signed   = ($signed(sensor1) >= $signed(threshold1));

    wire violate0 = signed_mode0 ? violate0_signed : violate0_unsigned;
    wire violate1 = signed_mode1 ? violate1_signed : violate1_unsigned;

    //--------------------------------------------------------------------------
    // IRQ — active whenever any alarm is asserted
    //--------------------------------------------------------------------------
    assign irq = alarm0 | alarm1;

    //--------------------------------------------------------------------------
    // FSM — next-state logic (combinational)
    //--------------------------------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            ST_IDLE: begin
                if (enable)
                    next_state = ST_MONITOR;
            end

            ST_MONITOR: begin
                if (!enable)
                    next_state = ST_IDLE;
                else if (violate0 | violate1)
                    next_state = ST_ALARM;
            end

            ST_ALARM: begin
                if (!enable)
                    next_state = ST_IDLE;
                else if (!alarm0 && !alarm1)
                    // Both alarms have been acknowledged
                    next_state = ST_MONITOR;
            end

            default:
                next_state = ST_IDLE;
        endcase
    end

    //--------------------------------------------------------------------------
    // FSM — state register
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n)
            state <= ST_IDLE;
        else
            state <= next_state;
    end

    //--------------------------------------------------------------------------
    // Sticky alarm logic — Channel 0
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            alarm0       <= 1'b0;
            fault_value0 <= 8'h00;
        end else if (!enable) begin
            alarm0       <= 1'b0;
            fault_value0 <= 8'h00;
        end else if (ack0) begin
            // Software acknowledgment clears alarm & fault capture
            alarm0       <= 1'b0;
            fault_value0 <= 8'h00;
        end else if (violate0 && !alarm0 &&
                     (state == ST_MONITOR || state == ST_ALARM)) begin
            // First violation — capture the offending value
            alarm0       <= 1'b1;
            fault_value0 <= sensor0;
        end
        // else: alarm0 stays sticky, fault_value0 not overwritten
    end

    //--------------------------------------------------------------------------
    // Sticky alarm logic — Channel 1
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            alarm1       <= 1'b0;
            fault_value1 <= 8'h00;
        end else if (!enable) begin
            alarm1       <= 1'b0;
            fault_value1 <= 8'h00;
        end else if (ack1) begin
            alarm1       <= 1'b0;
            fault_value1 <= 8'h00;
        end else if (violate1 && !alarm1 &&
                     (state == ST_MONITOR || state == ST_ALARM)) begin
            alarm1       <= 1'b1;
            fault_value1 <= sensor1;
        end
    end

endmodule
