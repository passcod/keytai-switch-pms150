// Controller for bitbanged protocol with comparator-based analog reads
// Target: PMS150C (PDK13)

#include <stdint.h>

// === Register definitions (PMS150C - PDK13) ===
__sfr __at(0x00) flag;
__sfr __at(0x02) sp;
__sfr __at(0x03) clkmd;
__sfr __at(0x04) inten;
__sfr __at(0x05) intrq;
__sfr __at(0x0c) integs;
__sfr __at(0x0d) padier;

// Port A (PMS150C has only Port A)
__sfr __at(0x10) pa;
__sfr __at(0x11) pac;       // direction: 1=output
__sfr __at(0x12) paph;      // pull-up

// Comparator
__sfr __at(0x18) gpcc;      // comparator control
__sfr __at(0x19) gpcs;      // comparator select

// === Pin definitions ===
#define PIN_CLK     0       // PA0 - clock input
#define PIN_DATA    5       // PA5 - bidirectional data
#define PIN_LED     3       // PA3 - WS2812 data output
#define PIN_COMPN_X 7       // PA7 - comparator- for coord_x
#define PIN_COMPN_Y 6       // PA6 - comparator- for coord_y
#define PIN_COMPN_B 4       // PA4 - comparator- for buttons

// === Device capabilities (hardcoded) ===
#define CAP_NUM_DIGITAL  2    // btn1, btn2
#define CAP_NUM_ANALOG   2    // coord_x, coord_y
#define CAP_ANALOG_RES_0 4    // coord_x: 4-bit SAR
#define CAP_ANALOG_RES_1 4    // coord_y: 4-bit SAR
#define CAP_NUM_LEDS     1    // Single WS2812
#define CAP_LED_TYPE_0   1    // 0=mono (8-bit), 1=RGB (24-bit)

// Length prefix values (compile-time constants)
#define CAP_LENGTH  (4 + 4 + 4*CAP_NUM_ANALOG + 4 + CAP_NUM_LEDS)  // 21
#define DATA_LENGTH (CAP_ANALOG_RES_0 + CAP_ANALOG_RES_1 + CAP_NUM_DIGITAL)  // 10

// === Cached values (updated in main loop) ===
volatile uint8_t capabilities_sent;
volatile uint8_t coord_x;
volatile uint8_t coord_y;
volatile uint8_t btn1;
volatile uint8_t btn2;
volatile uint8_t led_r;
volatile uint8_t led_g;
volatile uint8_t led_b;
volatile uint8_t led_update;  // Flag: ISR sets to 1 when new LED data received

// === Comparator helpers ===

// Set internal voltage reference level (0-15 for 4-bit resolution)
// GPCC: bit7=enable, bit6=output_enable, bit4=result, bits3:0=reference
static void comp_set_reference(uint8_t level) {
    gpcc = 0x80 | (level & 0x0F);  // enable comparator, set reference
}

// Select which pin is COMP- input
// GPCS bits select the negative input source
static void comp_select_input(uint8_t pin) {
    switch (pin) {
        case PIN_COMPN_X: gpcs = 0x01; break;  // PA7
        case PIN_COMPN_Y: gpcs = 0x02; break;  // PA6
        case PIN_COMPN_B: gpcs = 0x03; break;  // PA4
    }
}

// Read comparator result (1 if input > reference)
static uint8_t comp_read(void) {
    return (gpcc >> 4) & 1;
}

// SAR ADC: 4-bit successive approximation read
static uint8_t sar_read_4bit(uint8_t input_pin) {
    uint8_t result = 0;
    uint8_t bit;
    
    comp_select_input(input_pin);
    
    // 4-bit SAR: test from MSB to LSB
    for (bit = 8; bit > 0; bit >>= 1) {
        comp_set_reference(result | bit);
        // Small delay for comparator to settle
        __asm__("nop\nnop\nnop\nnop");
        if (comp_read()) {
            // Input > reference, keep the bit set
            result |= bit;
        }
        // If input <= reference, don't set the bit (try lower value)
    }
    return result & 0x0F;
}

// === WS2812 bitbang ===
// WS2812 timing (at ~8MHz IHRC):
//   T0H: ~375ns (3 cycles), T0L: ~875ns (7 cycles)
//   T1H: ~875ns (7 cycles), T1L: ~375ns (3 cycles)
//   Reset: >50us low
// WS2812 colour order is GRB
//
// Sends a single byte on PA3 using WS2812 protocol
static void ws2812_byte(uint8_t byte) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (byte & 0x80) {
            pa |= (1 << PIN_LED);
            __asm__("nop\nnop\nnop\nnop\nnop");
            pa &= ~(1 << PIN_LED);
            __asm__("nop");
        } else {
            pa |= (1 << PIN_LED);
            __asm__("nop");
            pa &= ~(1 << PIN_LED);
            __asm__("nop\nnop\nnop\nnop\nnop");
        }
        byte <<= 1;
    }
}

// === Bitbang protocol ===

// Wait for clock rising edge
static void wait_clk_high(void) {
    while (!(pa & (1 << PIN_CLK)));
}

// Wait for clock falling edge
static void wait_clk_low(void) {
    while (pa & (1 << PIN_CLK));
}

// Send one bit on PA5, clocked by external PA0
static void send_bit(uint8_t bit) {
    if (bit) {
        pa |= (1 << PIN_DATA);
    } else {
        pa &= ~(1 << PIN_DATA);
    }
    wait_clk_high();
    wait_clk_low();
}

// Receive one bit from PA5, clocked by external PA0
static uint8_t recv_bit(void) {
    uint8_t bit;
    wait_clk_high();
    bit = (pa >> PIN_DATA) & 1;
    wait_clk_low();
    return bit;
}

// Detect sync pattern: PA5 held low for 4 clock beats
static uint8_t detect_sync(void) {
    uint8_t count = 0;
    for (count = 0; count < 4; count++) {
        wait_clk_high();
        if (pa & (1 << PIN_DATA)) return 0;  // Not a sync
        wait_clk_low();
    }
    return 1;
}

// Send a 4-bit nibble MSB first
static void send_nibble(uint8_t val) {
    uint8_t mask = 0x08;
    while (mask) {
        send_bit((val & mask) ? 1 : 0);
        mask >>= 1;
    }
}

// Send an 8-bit byte MSB first
static void send_byte(uint8_t val) {
    uint8_t mask = 0x80;
    while (mask) {
        send_bit((val & mask) ? 1 : 0);
        mask >>= 1;
    }
}

// Receive an 8-bit byte MSB first
static uint8_t recv_byte(void) {
    uint8_t val = 0;
    uint8_t i;
    for (i = 0; i < 8; i++) val = (val << 1) | recv_bit();
    return val;
}

// Send device capabilities (first exchange only)
static void send_capabilities(void) {
    send_byte(CAP_LENGTH);
    send_nibble(CAP_NUM_DIGITAL);
    send_nibble(CAP_NUM_ANALOG);
    send_nibble(CAP_ANALOG_RES_0);
    send_nibble(CAP_ANALOG_RES_1);
    send_nibble(CAP_NUM_LEDS);
    send_bit(CAP_LED_TYPE_0);
}

// Send sensor data (normal exchange)
static void send_data(void) {
    send_byte(DATA_LENGTH);
    send_nibble(coord_x);
    send_nibble(coord_y);
    send_bit(btn1);
    send_bit(btn2);
}

// Receive LED colour data (24-bit RGB)
static void recv_led_rgb(void) {
    led_r = recv_byte();
    led_g = recv_byte();
    led_b = recv_byte();
    led_update = 1;
}

// === Protocol handler (called from ISR) ===
static void protocol_exchange(void) {
    uint8_t cmd;
    
    // PA5 is already output mode (set in init), just make sure it's low
    pa &= ~(1 << PIN_DATA);
    
    if (!capabilities_sent) {
        send_capabilities();
        capabilities_sent = 1;
        coord_x = 0xFF;  // Force change detection on next main loop pass
    } else {
        send_data();
    }
    
    // Switch PA5 to input for reception
    pac &= ~(1 << PIN_DATA);
    paph |= (1 << PIN_DATA);  // Enable pull-up
    
    // Wait for incoming sync
    while (!detect_sync());
    
    // Receive 2-bit command (MSB first)
    cmd = (recv_bit() << 1) | recv_bit();
    
    if (cmd == 0x01 || cmd == 0x02) {
        // 0b01: data for each LED, 0b10: one data for all LEDs
        // For single LED, both are equivalent
        recv_led_rgb();
    } else if (cmd == 0x03) {
        // 0b11: turn off all LEDs
        led_r = 0;
        led_g = 0;
        led_b = 0;
        led_update = 1;
    }
    // 0b00: skip (no colour update)
    
    // Switch PA5 back to output mode, driving low (ready cleared)
    pa &= ~(1 << PIN_DATA);
    paph &= ~(1 << PIN_DATA);  // Disable pull-up
    pac |= (1 << PIN_DATA);    // Output mode
}

// === Decode button value from comparator reading ===
static void decode_buttons(uint8_t comp_val) {
    // 4-bit value: 0-15
    // 0-5: (0,0), 5-8: (1,0), 8-10: (0,1), 10-15: (1,1)
    if (comp_val < 5) {
        btn1 = 0; btn2 = 0;
    } else if (comp_val < 8) {
        btn1 = 1; btn2 = 0;
    } else if (comp_val < 11) {
        btn1 = 0; btn2 = 1;
    } else {
        btn1 = 1; btn2 = 1;
    }
}

// === Interrupt handler ===
void isr(void) __interrupt(0) {
    if (intrq & 0x01) {         // PA0 interrupt
        intrq &= ~0x01;         // Clear flag
        
        // Clear ready signal immediately (PA5 low, also start of data phase)
        pa &= ~(1 << PIN_DATA);
        
        protocol_exchange();
        
        // Clear any spurious interrupt flags from clock pulses during protocol
        intrq &= ~0x01;
    }
}

// === Startup ===
uint8_t __sdcc_external_startup(void) {
    clkmd = 0x30;               // IHRC, no watchdog
    return 0;
}

// === Main ===
void main(void) {
    // Local variables for sampling (compared against cache)
    uint8_t new_x, new_y, new_btn1, new_btn2;
    uint8_t changed;
    
    // Initialize ports
    pa = 0;
    pac = (1 << PIN_LED) | (1 << PIN_DATA);  // PA3 output for WS2812, PA5 output for ready signal (starts low)
    paph = (1 << PIN_CLK);  // Pull-up on clock only (PA5 is output-driven now)
    
    // Initialize variables
    coord_x = 0;
    coord_y = 0;
    btn1 = 0;
    btn2 = 0;
    led_r = 0;
    led_g = 0;
    led_b = 0;
    led_update = 0;
    capabilities_sent = 0;
    
    // Send initial WS2812 reset (LED off)
    __asm__("disgint");
    ws2812_byte(0);
    ws2812_byte(0);
    ws2812_byte(0);
    __asm__("engint");
    
    // Configure PA0 interrupt (rising edge)
    padier = (1 << PIN_CLK);    // Enable PA0 digital input
    integs = 0x01;              // PA0 rising edge
    inten = 0x01;               // Enable PA0 interrupt
    intrq = 0;                  // Clear pending interrupts
    
    // Enable global interrupts
    __asm__("engint");
    
    // Main loop: continuously sample analog inputs
    while (1) {
        // Sample all channels into local variables first
        new_x = sar_read_4bit(PIN_COMPN_X);
        new_y = sar_read_4bit(PIN_COMPN_Y);
        
        // Fast 2-comparison button decode
        // Circuit: R1=4.7k, R2=10k, Rgnd=10k gives SAR values 0, 7, 10, 12
        // These fall in quadrants [0-3], [4-7], [8-11], [12-15]
        {
            uint8_t high_half;
            comp_select_input(PIN_COMPN_B);
            
            // First comparison: ref=8 splits into low/high pairs
            comp_set_reference(8);
            high_half = comp_read();
            
            if (high_half) {
                // SAR >= 8: either BTN1 only (10) or both (12)
                comp_set_reference(12);
                if (comp_read()) {
                    new_btn1 = 1; new_btn2 = 1;  // Both buttons (SAR 12)
                } else {
                    new_btn1 = 1; new_btn2 = 0;  // BTN1 only (SAR 10)
                }
            } else {
                // SAR < 8: either none (0) or BTN2 only (7)
                comp_set_reference(4);
                if (comp_read()) {
                    new_btn1 = 0; new_btn2 = 1;  // BTN2 only (SAR 7)
                } else {
                    new_btn1 = 0; new_btn2 = 0;  // No buttons (SAR 0)
                }
            }
        }
        
        // Check if any value changed
        changed = (new_x != coord_x) || (new_y != coord_y) ||
                  (new_btn1 != btn1) || (new_btn2 != btn2);
        
        if (changed) {
            // Atomic update: disable interrupts during cache update
            __asm__("disgint");
            coord_x = new_x;
            coord_y = new_y;
            btn1 = new_btn1;
            btn2 = new_btn2;
            __asm__("engint");
            
            // Signal ready: set PA5 high (already output mode from init)
            pa |= (1 << PIN_DATA);
        }
        
        // Update WS2812 LED if new data received from ISR
        if (led_update) {
            led_update = 0;
            __asm__("disgint");
            ws2812_byte(led_g);
            ws2812_byte(led_r);
            ws2812_byte(led_b);
            __asm__("engint");
        }
    }
}
