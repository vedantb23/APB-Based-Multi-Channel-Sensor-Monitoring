# Synthesis Directory

This directory contains the Yosys synthesis script and output files.

## Files

| File | Description |
|------|-------------|
| `synth.ys` | Yosys synthesis script |
| `netlist.v` | Synthesized gate-level netlist (generated) |

## Running Synthesis

```bash
yosys -s synth/synth.ys
```

The script will:
1. Read all RTL source files
2. Perform technology-independent synthesis
3. Map to generic cells
4. Write the synthesized netlist to `synth/netlist.v`
5. Print cell statistics

## Notes

- This uses Yosys's built-in generic cell library — no vendor-specific technology mapping.
- Cell counts reported by `stat` are actual Yosys output and should not be interpreted as ASIC/FPGA-specific area estimates.
- No area, power, or timing numbers are claimed — those require a specific technology library and physical design flow.
