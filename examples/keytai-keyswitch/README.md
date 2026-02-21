# Keytai Keyswitch firmware

Features a custom bitbanged protocol for reading analog inputs (coordinates, buttons) and controlling a WS2812 NeoPixel LED.

## Pin Assignment

| Pin | Function | Direction |
|-----|----------|-----------|
| PA0 | Clock | Input (interrupt on rising edge) |
| PA3 | WS2812 data output | Output |
| PA4 | Comparator- for buttons | Input (analog) |
| PA5 | Data / Ready signal | Bidirectional |
| PA6 | Comparator- for `coord_y` | Input (analog) |
| PA7 | Comparator- for `coord_x` | Input (analog) |

## Ready Signal (PA5)

The keyswitch signals data availability via PA5:

- **PA5 LOW**: No new data, or data already read
- **PA5 HIGH**: Fresh data available for reading

The keyboard controller can either:
1. Poll PA5 and only initiate exchange when HIGH (efficient)
2. Periodically poll regardless of PA5 state (gets cached values)

When the controller triggers an exchange (rising edge on PA0), PA5 is immediately cleared.

## Protocol Sequence

All data is clocked on PA0 rising edges. PA5 carries bidirectional data.

```
Phase 1: Keyswitch → Controller (14 bits)
  [4 clocks] Sync pattern (PA5 low)
  [4 clocks] coord_x (MSB first, 4 bits)
  [4 clocks] coord_y (MSB first, 4 bits)
  [1 clock]  btn1
  [1 clock]  btn2

Phase 2: Controller → Keyswitch (6 or 30 bits)
  [4 clocks] Sync pattern (PA5 low)
  [2 clocks] Command (MSB first):
             0b00 = skip (no colour update, phase ends here)
             0b01 = RGB colour follows
  [8 clocks] Red (MSB first, 8 bits)    ─┐
  [8 clocks] Green (MSB first, 8 bits)   ├─ only if command = 0b01
  [8 clocks] Blue (MSB first, 8 bits)   ─┘
```

Total: 20 clock pulses (skip) or 44 clock pulses (colour update).

## Timing Requirements

### Clock (PA0)

| Parameter | Min | Typ | Max | Notes |
|-----------|-----|-----|-----|-------|
| Clock high time | 10 µs | 20 µs | — | Keyswitch samples on rising edge |
| Clock low time | 10 µs | 20 µs | — | Data setup time |
| Clock period | 20 µs | 40 µs | — | ~25-50 kHz max clock rate |

### Ready Signal Detection

| Parameter | Min | Notes |
|-----------|-----|-------|
| Wait after mock change | 1000 µs | Full SAR cycle must complete |
| Ready to trigger delay | 0 µs | Can trigger immediately after PA5 high |

### Phase Transitions

| Parameter | Min | Notes |
|-----------|-----|-------|
| Trigger to first sync | 0 µs | Trigger pulse counts as first sync clock |
| After keyswitch data, before controller sync | 20 µs | Keyswitch needs time to switch PA5 to input |
| After controller data, before next exchange | 100 µs | Keyswitch needs time to process LED value |

### SAR ADC Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Coordinate read | ~50 µs | 4-bit SAR, 4 comparisons |
| Button decode | ~25 µs | 2 comparisons only (optimized circuit) |
| Full sample cycle | ~400 µs | 2 coord × ~150 µs + buttons ~100 µs |
| Change detection latency | ~600 µs | Worst case for all channels |

## Button Circuit

Optimized for fast 2-comparison decode:

```
Vcc ─┬─ BTN1 ─ 4.7kΩ ─┬─ PA4 ─ 10kΩ ─ GND
     └─ BTN2 ─ 10kΩ  ─┘
```

| State | Voltage | Quadrant | SAR Range |
|-------|---------|----------|-----------|
| No buttons | 0V | 0 | 0-3 |
| BTN2 only | 1.65V | 1 | 4-7 |
| BTN1 only | 2.25V | 2 | 8-11 |
| Both | 2.48V | 3 | 12-15 |

Decode algorithm:
1. Compare at reference 8 → splits low (none, BTN2) vs high (BTN1, both)
2. Compare at 4 or 12 → identifies exact state

## Data Encoding

### Coordinates (`coord_x`, `coord_y`)

4-bit values (0-15) from SAR ADC. Due to `>` comparison in SAR:
- Input value N typically reads as N-1
- Example: Analog input representing 10 → SAR result 9

### Buttons

Decoded via 2-comparison binary search (see Button Circuit above):

| Quadrant | Comparator Value | btn1 | btn2 |
|----------|------------------|------|------|
| 0 (0-3) | No buttons | 0 | 0 |
| 1 (4-7) | BTN2 only | 0 | 1 |
| 2 (8-11) | BTN1 only | 1 | 0 |
| 3 (12-15) | Both | 1 | 1 |

### LED Value

24-bit RGB color (R, G, B bytes) sent over the protocol, output via bitbanged WS2812 on PA3.

## Example Waveform

```
         ___     ___     ___     ___     ___
PA0  ___|   |___|   |___|   |___|   |___|   |___  ...
        ^       ^       ^       ^       ^
        |       |       |       |       |
        trigger sync1   sync2   sync3   sync4

PA5  ‾‾‾|_______|_______|_______|_______|_____ ...
     ready      sync pattern (low)
     cleared
```

## Building

Requires SDCC (with PDK13 support) and GHDL:

```bash
make          # Build and run simulation
make clean    # Remove generated files
./test        # Run simulation (after make)
./test --wave=waveform.ghw  # Generate waveform for GTKWave
```

## Test Scenarios

The testbench validates three scenarios:

1. **Ready signal wait**: Change mock values, wait for PA5 high, exchange
2. **Stale data poll**: Poll without PA5 high, verify cached values returned
3. **Value change detection**: Change mock values, verify PA5 goes high, verify new values
