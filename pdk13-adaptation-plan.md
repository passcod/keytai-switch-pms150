# PDK13 Adaptation Plan

## Goal
Adapt the existing PDK14 (PFS154) emulator to support PDK13 (PMS150C).

## Key Differences
- 13-bit instructions (vs 14-bit)
- 1K ROM / 64B RAM / 32 IO regs (vs 2K/128B/64)
- SYM_84B instruction set (vs SYM_85A)
- All bit fields shift down by 1

## Files to Create
1. `hdl/pdk13pkg.vhd` - Types with adjusted widths
2. `hdl/pdk13decode.vhd` - Decoder with shifted bit positions
3. `hdl/pdk13.vhd` - Core with smaller memories, PMS150C register map

## Reference
- Instruction set: https://free-pdk.github.io/instruction-sets/PDK13
- Chip info: https://free-pdk.github.io/chips/PMS150C
- PMS150C datasheet for peripheral register addresses

## Approach
1. Copy pdk14pkg.vhd → pdk13pkg.vhd, adjust type widths
2. Copy pdk14decode.vhd → pdk13decode.vhd, shift all bit indices down by 1
3. Copy pdk14.vhd → pdk13.vhd, update memory sizes and IO register map per PMS150C datasheet
4. Create test example with SDCC's `-mpdk13` flag
