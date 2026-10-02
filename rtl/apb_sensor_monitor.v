//==============================================================================
// apb_sensor_monitor.v
//
// AMBA APB4-compatible slave register interface for the sensor monitor IP.
// Provides memory-mapped access to control, threshold, status, fault-capture,
// and acknowledgment registers.
//
// Register Map (32-bit data, word-aligned):
//   0x00  CTRL_REG       [RW]  bit0=enable, bit1=signed_mode0, bit2=signed_mode1
//   0x04  THRESH0_REG    [RW]  bits[7:0]=threshold0
//   0x08  THRESH1_REG    [RW]  bits[7:0]=threshold1
//   0x0C  SENSOR0_REG    [RO]  bits[7:0]=live sensor0
//   0x10  SENSOR1_REG    [RO]  bits[7:0]=live sensor1
//   0x14  STATUS_REG     [RO]  bit0=alarm0, bit1=alarm1, bit2=irq
//   0x18  FAULT0_REG     [RO]  bits[7:0]=captured fault value ch0
//   0x1C  FAULT1_REG     [RO]  bits[7:0]=captured fault value ch1
//   0x20  ACK_REG        [WO]  bit0=ack0, bit1=ack1 (auto-clears)
//
// Author : Original design for educational APB4-based sensor monitor IP
// License: MIT
//==============================================================================

module apb_sensor_monitor (
    // APB4 interface
    input  wire        PCLK,
    input  wire        PRESETn,
    input  wire        PSEL,
    input  wire        PENABLE,
    input  wire        PWRITE,
    input  wire [31:0] PADDR,
    input  wire [31:0] PWDATA,
    output reg  [31:0] PRDATA,
    output wire        PREADY,
    output reg         PSLVERR,

    // Sensor data inputs (directly from pads / upstream logic)
    input  wire [7:0]  sensor0,
    input  wire [7:0]  sensor1,

    // Status from sensor_monitor_core
    input  wire        alarm0,
    input  wire        alarm1,
    input  wire [7:0]  fault_value0,
    input  wire [7:0]  fault_value1,
    input  wire        irq,

    // Control outputs to sensor_monitor_core
    output wire        enable,
    output wire        signed_mode0,
    output wire        signed_mode1,
    output wire [7:0]  threshold0,
    output wire [7:0]  threshold1,
    output reg         ack0,
    output reg         ack1
);

    //--------------------------------------------------------------------------
    // Address decode constants
    //--------------------------------------------------------------------------
    localparam ADDR_CTRL    = 8'h00,
               ADDR_THRESH0 = 8'h04,
               ADDR_THRESH1 = 8'h08,
               ADDR_SENSOR0 = 8'h0C,
               ADDR_SENSOR1 = 8'h10,
               ADDR_STATUS  = 8'h14,
               ADDR_FAULT0  = 8'h18,
               ADDR_FAULT1  = 8'h1C,
               ADDR_ACK     = 8'h20;

    //--------------------------------------------------------------------------
    // Internal registers
    //--------------------------------------------------------------------------
    reg [2:0]  ctrl_reg;       // {signed_mode1, signed_mode0, enable}
    reg [7:0]  thresh0_reg;
    reg [7:0]  thresh1_reg;

    //--------------------------------------------------------------------------
    // Drive control outputs
    //--------------------------------------------------------------------------
    assign enable       = ctrl_reg[0];
    assign signed_mode0 = ctrl_reg[1];
    assign signed_mode1 = ctrl_reg[2];
    assign threshold0   = thresh0_reg;
    assign threshold1   = thresh1_reg;

    //--------------------------------------------------------------------------
    // APB ready — zero-wait-state (always ready in ACCESS phase)
    //--------------------------------------------------------------------------
    assign PREADY = 1'b1;

    //--------------------------------------------------------------------------
    // Address validity check
    //--------------------------------------------------------------------------
    wire addr_valid;
    assign addr_valid = (PADDR[7:0] == ADDR_CTRL)    ||
                        (PADDR[7:0] == ADDR_THRESH0) ||
                        (PADDR[7:0] == ADDR_THRESH1) ||
                        (PADDR[7:0] == ADDR_SENSOR0) ||
                        (PADDR[7:0] == ADDR_SENSOR1) ||
                        (PADDR[7:0] == ADDR_STATUS)  ||
                        (PADDR[7:0] == ADDR_FAULT0)  ||
                        (PADDR[7:0] == ADDR_FAULT1)  ||
                        (PADDR[7:0] == ADDR_ACK);

    //--------------------------------------------------------------------------
    // Write-to-read-only detection
    //--------------------------------------------------------------------------
    wire write_ro;
    assign write_ro = PWRITE && (
        (PADDR[7:0] == ADDR_SENSOR0) ||
        (PADDR[7:0] == ADDR_SENSOR1) ||
        (PADDR[7:0] == ADDR_STATUS)  ||
        (PADDR[7:0] == ADDR_FAULT0)  ||
        (PADDR[7:0] == ADDR_FAULT1)
    );

    //--------------------------------------------------------------------------
    // PSLVERR generation
    //--------------------------------------------------------------------------
    always @(*) begin
        if (PSEL && PENABLE) begin
            if (!addr_valid || write_ro)
                PSLVERR = 1'b1;
            else
                PSLVERR = 1'b0;
        end else begin
            PSLVERR = 1'b0;
        end
    end

    //--------------------------------------------------------------------------
    // Register write logic
    //--------------------------------------------------------------------------
    wire apb_write = PSEL && PENABLE && PWRITE && !PSLVERR;
    wire apb_read  = PSEL && PENABLE && !PWRITE && !PSLVERR;

    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            ctrl_reg    <= 3'b000;
            thresh0_reg <= 8'h00;
            thresh1_reg <= 8'h00;
            ack0        <= 1'b0;
            ack1        <= 1'b0;
        end else begin
            // Default: acknowledgment pulses are one-cycle
            ack0 <= 1'b0;
            ack1 <= 1'b0;

            if (apb_write) begin
                case (PADDR[7:0])
                    ADDR_CTRL:    ctrl_reg    <= PWDATA[2:0];
                    ADDR_THRESH0: thresh0_reg <= PWDATA[7:0];
                    ADDR_THRESH1: thresh1_reg <= PWDATA[7:0];
                    ADDR_ACK: begin
                        ack0 <= PWDATA[0];
                        ack1 <= PWDATA[1];
                    end
                    default: ; // no-op
                endcase
            end
        end
    end

    //--------------------------------------------------------------------------
    // Register read logic
    //--------------------------------------------------------------------------
    always @(*) begin
        PRDATA = 32'h0000_0000;
        if (PSEL && !PWRITE) begin
            case (PADDR[7:0])
                ADDR_CTRL:    PRDATA = {29'b0, ctrl_reg};
                ADDR_THRESH0: PRDATA = {24'b0, thresh0_reg};
                ADDR_THRESH1: PRDATA = {24'b0, thresh1_reg};
                ADDR_SENSOR0: PRDATA = {24'b0, sensor0};
                ADDR_SENSOR1: PRDATA = {24'b0, sensor1};
                ADDR_STATUS:  PRDATA = {29'b0, irq, alarm1, alarm0};
                ADDR_FAULT0:  PRDATA = {24'b0, fault_value0};
                ADDR_FAULT1:  PRDATA = {24'b0, fault_value1};
                ADDR_ACK:     PRDATA = 32'h0000_0000; // write-only
                default:      PRDATA = 32'h0000_0000;
            endcase
        end
    end

endmodule
