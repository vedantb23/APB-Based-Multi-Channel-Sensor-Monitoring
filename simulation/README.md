# Simulation Directory

This directory is used for simulation output files generated during testbench execution.

## Generated Files

| File | Description |
|------|-------------|
| `sensor_monitor.vcd` | Value Change Dump waveform file |

## Running Simulation

From the project root:

```bash
# Compile
iverilog -o simulation/sim.vvp -I tb rtl/sensor_monitor_core.v rtl/apb_sensor_monitor.v rtl/top_sensor_monitor.v tb/tb_sensor_monitor.v

# Simulate
cd simulation && vvp sim.vvp
```

## Viewing Waveforms

If GTKWave is installed:

```bash
gtkwave simulation/sensor_monitor.vcd
```

GTKWave is optional and not required for CI or basic verification.

## Key Signals to Observe

| Signal | Description |
|--------|-------------|
| `clk` | System clock |
| `reset_n` | Active-low reset |
| `sensor0`, `sensor1` | Live sensor inputs |
| `dut.u_core.state` | FSM state (00=IDLE, 01=MONITOR, 10=ALARM) |
| `dut.u_core.alarm0`, `alarm1` | Sticky alarm flags |
| `dut.u_core.fault_value0`, `fault_value1` | Captured fault values |
| `irq` | Interrupt output |
| `PSEL`, `PENABLE`, `PWRITE` | APB control signals |
| `PADDR`, `PWDATA`, `PRDATA` | APB address/data |
| `PREADY`, `PSLVERR` | APB status |
