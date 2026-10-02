//==============================================================================
// apb_master_bfm.v
//
// Simple APB4 Bus-Functional Model (BFM) used by the testbench to drive
// APB read/write transactions to the DUT.
//
// Provides two tasks:
//   apb_write(addr, data)  — execute a single APB write transaction
//   apb_read (addr, data)  — execute a single APB read  transaction
//
// The BFM follows the two-phase APB protocol: SETUP → ACCESS.
// Zero-wait-state transfers are assumed (PREADY always high from slave).
//
// Author : Original design for educational APB4-based sensor monitor IP
// License: MIT
//==============================================================================

// NOTE: This file is `included inside the testbench module so that it can
// share the testbench's clock and APB signal declarations.
// It is NOT a standalone synthesizable module.

//--------------------------------------------------------------------------
// APB Write Task
//
// Phase 1 (SETUP)  : assert PSEL, PWRITE, PADDR, PWDATA; PENABLE=0
// Phase 2 (ACCESS) : assert PENABLE; capture PREADY, PSLVERR on posedge
// Phase 3 (IDLE)   : de-assert PSEL, PENABLE
//--------------------------------------------------------------------------
task apb_write;
    input [31:0] addr;
    input [31:0] data;
    begin
        @(posedge clk);
        // SETUP phase
        PSEL    <= 1'b1;
        PENABLE <= 1'b0;
        PWRITE  <= 1'b1;
        PADDR   <= addr;
        PWDATA  <= data;

        @(posedge clk);
        // ACCESS phase
        PENABLE <= 1'b1;

        @(posedge clk);
        // Return to IDLE
        PSEL    <= 1'b0;
        PENABLE <= 1'b0;
        PWRITE  <= 1'b0;
        PADDR   <= 32'h0;
        PWDATA  <= 32'h0;
    end
endtask

//--------------------------------------------------------------------------
// APB Read Task
//
// Phase 1 (SETUP)  : assert PSEL, PADDR; PWRITE=0, PENABLE=0
// Phase 2 (ACCESS) : assert PENABLE; sample PRDATA on posedge
// Phase 3 (IDLE)   : de-assert PSEL, PENABLE
//--------------------------------------------------------------------------
task apb_read;
    input  [31:0] addr;
    output [31:0] data;
    begin
        @(posedge clk);
        // SETUP phase
        PSEL    <= 1'b1;
        PENABLE <= 1'b0;
        PWRITE  <= 1'b0;
        PADDR   <= addr;
        PWDATA  <= 32'h0;

        @(posedge clk);
        // ACCESS phase
        PENABLE <= 1'b1;

        @(posedge clk);
        // Sample read data
        data = PRDATA;

        // Return to IDLE
        PSEL    <= 1'b0;
        PENABLE <= 1'b0;
        PADDR   <= 32'h0;
    end
endtask
