// Controller for bitbanged protocol with comparator-based analog reads
// Target: PFS154 (PDK14) - adaptable to PMS150C (PDK13)

#include <stdint.h>

// === Register definitions (PFS154) ===
__sfr __at(0x00) flag;
__sfr __at(0x02) sp;
__sfr __at(0x03) clkmd;
__sfr __at(0x04) inten;
__sfr __at(0x05) intrq;
__sfr __at(0x0c) integs;
__sfr __at(0x0d) padier;

// Port A
__sfr __at(0x10) pa;
__sfr __at(0x11) pac;       // direction: 1=output
__sfr __at(0x12) paph;      // pull-up

// Port B
__sfr __at(0x14) pb;
__sfr __at(0x15) pbc;
__sfr __at(0x16) pbph;

// Timer2 (PWM)
__sfr __at(0x1c) tm2c;
__sfr __at(0x17) tm2s;
__sfr __at(0x09) tm2b;

// Comparator
__sfr __at(0x1a) gpcc;      // comparator control
__sfr __at(0x1b) gpcs;      // comparator select

// === Pin definitions ===
#define PIN_CLK     0       // PA0 - clock input
#define PIN_DATA    5       // PA5 - bidirectional data
#define PIN_PWM     3       // PA3 - PWM output
#define PIN_COMPN_X 7       // PA7 - comparator- for coord_x
#define PIN_COMPN_Y 6       // PA6 - comparator- for coord_y
#define PIN_COMPN_B 4       // PA4 - comparator- for buttons

// === Cached values (updated in main loop) ===
volatile uint8_t coord_x;
volatile uint8_t coord_y;
volatile uint8_t btn1;
volatile uint8_t btn2;
volatile uint8_t led_value;

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
        if (!comp_read()) {
            result |= bit;
        }
    }
    return result & 0x0F;
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

// Send sync pattern: hold PA5 low for 4 clock beats
static void send_sync(void) {
    uint8_t i;
    pa &= ~(1 << PIN_DATA);
    for (i = 0; i < 4; i++) {
        wait_clk_high();
        wait_clk_low();
    }
}

// === Protocol handler (called from ISR) ===
static void protocol_exchange(void) {
    uint8_t i;
    uint8_t new_led = 0;
    
    // Switch PA5 to output for transmission
    pac |= (1 << PIN_DATA);
    
    // Send sync (4 beats low)
    send_sync();
    
    // Send coord_x (4 bits, MSB first)
    for (i = 0; i < 4; i++) {
        send_bit((coord_x >> (3 - i)) & 1);
    }
    
    // Send coord_y (4 bits, MSB first)
    for (i = 0; i < 4; i++) {
        send_bit((coord_y >> (3 - i)) & 1);
    }
    
    // Send btn1, btn2
    send_bit(btn1);
    send_bit(btn2);
    
    // Switch PA5 to input for reception
    pac &= ~(1 << PIN_DATA);
    paph |= (1 << PIN_DATA);  // Enable pull-up
    
    // Wait for incoming sync
    while (!detect_sync());
    
    // Receive LED value (8 bits, MSB first)
    for (i = 0; i < 8; i++) {
        new_led = (new_led << 1) | recv_bit();
    }
    
    led_value = new_led;
    
    // Update PWM duty cycle
    tm2b = led_value;
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
        protocol_exchange();
    }
}

// === Startup ===
uint8_t _sdcc_external_startup(void) {
    clkmd = 0x30;               // IHRC, no watchdog
    return 0;
}

// === Main ===
void main(void) {
    // Initialize ports
    pa = 0;
    pac = (1 << PIN_PWM);       // PA3 output for PWM
    paph = (1 << PIN_CLK) | (1 << PIN_DATA);  // Pull-ups on clock/data
    
    // Initialize variables
    coord_x = 0;
    coord_y = 0;
    btn1 = 0;
    btn2 = 0;
    led_value = 0;
    
    // Configure Timer2 for PWM on PA3
    tm2c = 0x1A;                // PWM on PA3, IHRC, prescaler 1
    tm2b = 0x00;                // Start with 0 duty cycle
    tm2s = 0x00;
    
    // Configure PA0 interrupt (rising edge)
    padier = (1 << PIN_CLK);    // Enable PA0 digital input
    integs = 0x01;              // PA0 rising edge
    inten = 0x01;               // Enable PA0 interrupt
    intrq = 0;                  // Clear pending interrupts
    
    // Enable global interrupts
    __asm__("engint");
    
    // Main loop: continuously sample analog inputs
    while (1) {
        // Sample coord_x from PA7
        coord_x = sar_read_4bit(PIN_COMPN_X);
        
        // Sample coord_y from PA6
        coord_y = sar_read_4bit(PIN_COMPN_Y);
        
        // Sample buttons from PA4 and decode
        decode_buttons(sar_read_4bit(PIN_COMPN_B));
    }
}
