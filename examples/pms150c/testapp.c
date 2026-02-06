// Simple PMS150C test - blink PA3 using TM2
// Compiled with: sdcc -mpdk13

#include <stdint.h>

// PMS150C register addresses
__sfr __at(0x00) FLAG;
__sfr __at(0x02) SP;
__sfr __at(0x03) CLKMD;
__sfr __at(0x04) INTEN;
__sfr __at(0x05) INTRQ;
__sfr __at(0x06) T16M;
__sfr __at(0x10) PA;
__sfr __at(0x11) PAC;
__sfr __at(0x12) PAPH;
__sfr __at(0x17) TM2S;
__sfr __at(0x1C) TM2C;
__sfr __at(0x1D) TM2CT;
__sfr __at(0x09) TM2B;

// Delay counter
volatile uint8_t delay_count;

void main(void) {
    // Configure PA3 as output
    PAC = 0x08;  // PA3 output, others input
    PA = 0x00;   // Start low
    
    // Configure TM2 for PWM output on PA3
    // TM2C format in simulation:
    //   bits 7:4 = clock source (0x2 = IHRC)
    //   bits 3:2 = output enable (0x2 = "10" = enable on PA3)
    //   bit 1 = mode (1 = PWM, 0 = toggle)
    //   bit 0 = invert output
    TM2B = 0x40;  // Bound value = 64
    TM2S = 0x40;  // Scale value = 64 for ~50% duty
    TM2C = 0x2A;  // IHRC clock (0x2), output on PA3 (bits 3:2=10), PWM mode (bit 1=1)
    
    // Main loop - just keep running
    while(1) {
        delay_count++;
        __asm__("nop");
    }
}
