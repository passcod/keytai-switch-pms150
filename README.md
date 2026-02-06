# VHDL FPPA PDK Simulation Model

A probably-functional, timing accurate VHDL model for simulating Padauk FPPA microcontrollers.

Note that AI has been used for this; see the upstream for very old but AI-free code.

## Status

| Core | Chips | Status |
|------|-------|--------|
| PDK14 | PFS154 | ✅ Supported |
| PDK13 | PMS150C | ✅ Supported |

### Supported Features

#### PDK14 (PFS154)
- Full instruction set (including extended instructions: `oram`, `andam`, `xoram`, `subam`, `decm`, `izsnm`, `dzsnm`, `xchm`)
- GPIO with configurable pull-ups (Port A and Port B)
- Timers (TM2, TM3)
- PWM output (PWMG)
- External interrupts
- Comparator with programmable reference (for SAR ADC simulation)

#### PDK13 (PMS150C)
- Full instruction set (SYM_84B)
- GPIO with configurable pull-ups (Port A only)
- Timer TM2 with PWM output
- External interrupts
- Comparator with programmable reference

## Requirements

- [GHDL](https://github.com/ghdl/ghdl) - VHDL simulator
- [SDCC](https://sdcc.sourceforge.net/) - C compiler with PDK support
- `binutils` (for `objcopy`)
- [GTKWave](http://gtkwave.sourceforge.net/) (optional, for viewing waveforms)

## Building

First, build the `bin2rom` tool:

```bash
cd tools
make
```

Then build and run an example:

```bash
cd examples/test
make
./test --wave=waveform.ghw
```

View waveforms with GTKWave:

```bash
gtkwave waveform.ghw
```

## Examples

- **[test](examples/test/)** - Basic PDK14 test demonstrating PWM output and timers
- **[controller](examples/controller/)** - PDK14 bitbanged protocol with SAR ADC for analog inputs, change detection, and interrupt-driven communication
- **[pms150c](examples/pms150c/)** - Basic PDK13 (PMS150C) test demonstrating TM2 PWM output

## Open Questions

These implementation details are not fully documented by Padauk:

1. **T-cycle timing**: Is a "T" cycle a clock cycle, or do instructions take 2 clocks? The CPU sysclk is at most IHRC/2, but can also use ILRC and EOSC without a divider.

2. **Pin alternate function priority**: When multiple peripherals are configured for the same pin (e.g., TM2PWM and PG2PWM on PA3), which takes priority?

3. **Input synchronization**: GPIO inputs use 2 FFs clocked by sysclk to avoid metastability, but the actual implementation is unknown.

4. **Cross-domain register sync**: What is the delay between setting a register (e.g., PWM0GC.4) and the peripheral responding?

5. **Clock switching glitches**: How are narrow pulses avoided when switching clocks? Currently uses a simple multiplexer.

