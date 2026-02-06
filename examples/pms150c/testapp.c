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
    // TM2C: bit 7-6=output mode, bit 5-4=clock, bit 3=enable, bit 2-0=prescaler
    TM2B = 0x40;  // Bound value
    TM2S = 0x40;  // Scale value for ~50% duty
    TM2C = 0xC8;  // Enable TM2, IHRC/16, PWM output on PA3
    
    // Main loop - just keep running
    while(1) {
        delay_count++;
        __asm__("nop");
    }
}
