/*
 * Project: Pad-Pal Sleeping Pad Inflator Firmware
 * Target: ATtiny10
 * Author/Engineer: Tyler Reeves
 * Description: 
 * Controls a Brushless DC ESC (BLHeli) via PWM. 
 * Manages single-button interface for 4 power levels + Off.
 * Implements auto-off safety timer by 'piggybacking' on the PWM timer interrupt.
 * * * Hardware Resource Usage:
 * - Timer0: Fast PWM Mode (50Hz) for ESC Control
 * - Timer0 Overflow Interrupt: Used for 20ms timekeeping ticks
 */

/*----------Referance Attiny10 hardware pins-------------*/
 //          +====+
//  PWMA/PB0 |*   | PB3 (RESET)
//       GND |    | Vcc
//  PWMB/PB1 |    | PB2 (CLKO)
//           +====+

#include <avr/io.h>
#include <avr/interrupt.h> // Required for ISR and sei()

// ---- Hardware Definitions ---//
#define PWM_PIN         PB0
#define BUTTON_PIN      PB1

// --- Physics & Calibration ---//
// Timer Config: 8MHz / 1 prescaler = 8MHz ticks.
// 50Hz target = 20ms period. TOP = 159,999.
// ESC uses standard servo pulses: 1000us (Off) to 2000us (Full)
#define PWM_PULSE_MIN   8000   // 1.0ms (Motor Off)
#define PWM_LEVEL_1     9600   // 1.2ms
#define PWM_LEVEL_2     10200  // 1.275ms
#define PWM_LEVEL_3     11000  // 1.375ms
#define PWM_LEVEL_4     12000  // 1.5ms

#define RAMP_STEP       100    // PWM counts per step
#define RAMP_TICKS      3      // Wait 3 ticks (60ms) between ramp steps

// --- Safety ---//
// 6 minutes * 60 seconds * 50Hz = 18000 ticks
#define AUTO_OFF_TICKS  18000  // Auto-off threshold in 20ms ticks

// --- State Machine -----//
typedef enum {
    STATE_OFF,
    STATE_LEVEL_1,
    STATE_LEVEL_2,
    STATE_LEVEL_3,
    STATE_LEVEL_4
} SystemState_t;

// Global Variables
// Volatile is required for variables shared with Interrupts
volatile uint16_t sys_ticks = 0;

SystemState_t currentState = STATE_OFF;
uint16_t currentPWM = PWM_PULSE_MIN;
uint16_t targetPWM = PWM_PULSE_MIN;

// Variables for non-blocking timing logic
uint16_t lastRampTick = 0;   
uint16_t stateStartTick = 0;

// --- Interrupt Service Routine (ISR) ---
// This fires every 20ms when the PWM timer overflows/resets
ISR(TIM0_OVF_vect) {
    sys_ticks++;
}

// --- Low Level Hardware Init ----//
void initHardware() {
    // 1. Configure Ports: PB0 Output (PWM), PB1 Input (Button)
    DDRB |= (1 << PWM_PIN);         
    DDRB &= ~(1 << BUTTON_PIN);     
    PUEB |= (1 << BUTTON_PIN);      // Enable Internal Pull-up (Active Low Logic)

    // 2. Timer0 Setup: Fast PWM, Mode 14 (ICR0 as TOP)
    // CS0[2:0] = 001 for No Prescaling (8MHz)
    TCCR0A = (1 << COM0A1) | (1 << WGM01); 
    TCCR0B = (1 << WGM03) | (1 << WGM02) | (1 << CS00);
    
    // 3. Set Frequency (50Hz)
    ICR0 = 159999;     
    
    // 4. Set Initial Duty Cycle
    OCR0A = PWM_PULSE_MIN; 

    // 5. Enable Overflow Interrupt (for our timekeeping)
    TIMSK0 |= (1 << TOIE0);
}

// --- Non-Blocking PWM Ramping ---//
void updateMotor() {
    // Check if enough "ticks" (20ms periods) have passed
    if ((sys_ticks - lastRampTick) >= RAMP_TICKS) {
        lastRampTick = sys_ticks;
        
        // Ramp Up
        if (currentPWM < targetPWM) {
            currentPWM += RAMP_STEP;
            if (currentPWM > targetPWM) currentPWM = targetPWM;
        } 
        // Ramp Down
        else if (currentPWM > targetPWM) {
            currentPWM -= RAMP_STEP;
            if (currentPWM < targetPWM) currentPWM = targetPWM;
        }
        
        OCR0A = currentPWM;
    }
}

// --- Main Application Entry ----//
int main(void) {
    initHardware();
    sei(); // Enable Global Interrupts (Start the "sys_ticks" counter)

    while(1) {
        // --- Button Logic (Input is Active Low due to Pull-up) -----//
        if (!(PINB & (1 << BUTTON_PIN))) { // Check if button is pressed (LOW)
            // Simple Debounce: Wait a few ticks (~200ms)
            uint16_t start_debounce = sys_ticks;
            while(!(PINB & (1 << BUTTON_PIN)) && (sys_ticks - start_debounce < 10)); 

            // Verify button is still held after debounce
            if (!(PINB & (1 << BUTTON_PIN))) {
                stateStartTick = sys_ticks; // Reset auto-off
                
                // State Transition
                switch (currentState) {
                    case STATE_OFF:     currentState = STATE_LEVEL_1; break;
                    case STATE_LEVEL_1: currentState = STATE_LEVEL_2; break;
                    case STATE_LEVEL_2: currentState = STATE_LEVEL_3; break;
                    case STATE_LEVEL_3: currentState = STATE_LEVEL_4; break;
                    case STATE_LEVEL_4: currentState = STATE_OFF;     break;
                }

                // Wait for button release to prevent immediate re-triggering
                while(!(PINB & (1 << BUTTON_PIN)));
            }
        }

        // --- Output Logic ----//
        switch (currentState) {
            case STATE_OFF:     targetPWM = PWM_PULSE_MIN; break;
            case STATE_LEVEL_1: targetPWM = PWM_LEVEL_1; break;
            case STATE_LEVEL_2: targetPWM = PWM_LEVEL_2; break;
            case STATE_LEVEL_3: targetPWM = PWM_LEVEL_3; break;
            case STATE_LEVEL_4: targetPWM = PWM_LEVEL_4; break;
        }

        // --- Safety Logic -----//
        if (currentState != STATE_OFF && (sys_ticks - stateStartTick > AUTO_OFF_TICKS)) {
            currentState = STATE_OFF;
        }

        // --- Physics Update -----//
        updateMotor();
    }
    return 0;
}