/*
 * Project: Pad-Pal Sleeping Pad Inflator Firmware
 * Target: ATtiny10
 * Author/Engineer: Tyler Reeves
 * Description: 
 * Controls a Brushless DC ESC (BLHeli) via PWM. 
 * Manages single-button interface for 4 power levels + Off.
 * Implements auto-off safety timer.
 * * Hardware Timer Config:
 * 8MHz Clock / 1 (Prescaler) = 8MHz Timer Clock
 * PWM Frequency target = 50Hz (Standard ESC/Servo pulse)
 * TOP = (8,000,000 / 50) - 1 = 159,999
 */

/*----------Referance Attiny10 hardware pins-------------*/
 //          +====+
//  PWMA/PB0 |*   | PB3 (RESET)
//       GND |    | Vcc
//  PWMB/PB1 |    | PB2 (CLKO)
//           +====+

#include "Arduino.h"	//inclusion of arduino libary to simplify button state pulling
#include <avr/io.h>

// --- Hardware Definitions ---//
#define PWM_PIN         PB0
#define BUTTON_PIN      PB1

// --- Physics & Calibration ---//
// ESC uses standard servo pulses: 1000us (Off) to 2000us (Full)
// Timer Tick = 1 / 8MHz = 0.125us
// 1000us / 0.125us = 8000 ticks
#define PWM_PULSE_MIN   8000   // 1.0ms (Motor Off)
#define PWM_LEVEL_1     9600   // 1.2ms
#define PWM_LEVEL_2     10200  // 1.275ms
#define PWM_LEVEL_3     11000  // 1.375ms
#define PWM_LEVEL_4     12000  // 1.5ms

#define RAMP_STEP       100    // How much to increase PWM per cycle
#define RAMP_INTERVAL   50     // ms between ramp steps

// --- Safety ---//
#define AUTO_OFF_MS     360000UL // 6 minutes (example)

// --- State Machine ---
typedef enum {
    STATE_OFF,
    STATE_LEVEL_1,
    STATE_LEVEL_2,
    STATE_LEVEL_3,
    STATE_LEVEL_4
} SystemState_t;

// Global Variables
SystemState_t currentState = STATE_OFF;
uint16_t currentPWM = PWM_PULSE_MIN;
uint16_t targetPWM = PWM_PULSE_MIN;
unsigned long lastRampTime = 0;
unsigned long stateStartTime = 0;

// --- Low Level Hardware Init ---
void initHardware() {
    // Configure PB0 as Output (PWM), PB1 as Input (Button)
    DDRB = (1 << PWM_PIN);
    pinMode(BUTTON_PIN, INPUT); // Arduino helper or use DDRB &= ~(1<<BUTTON_PIN)

    // Timer0 Setup: Fast PWM, Mode 14 (ICR0 as TOP)
    // Prescaler 1:1 (CS00)
    TCCR0A = (1 << COM0A1) | (1 << WGM01); 
    TCCR0B = (1 << WGM03) | (1 << WGM02) | (1 << CS00);
    
    ICR0 = 159999;     // Set 50Hz frequency
    OCR0A = PWM_PULSE_MIN; // Init at 0 throttle
}


// --- Non-Blocking PWM Ramping ---
void updateMotor() {
    unsigned long now = millis();
    
    // Only update if interval has passed
    if (now - lastRampTime >= RAMP_INTERVAL) {
        lastRampTime = now;
        
        if (currentPWM < targetPWM) {
            currentPWM += RAMP_STEP;
            if (currentPWM > targetPWM) currentPWM = targetPWM;
        } 
        else if (currentPWM > targetPWM) {
            currentPWM -= RAMP_STEP;
            if (currentPWM < targetPWM) currentPWM = targetPWM;
        }
        
        // Write to Hardware Register
        OCR0A = currentPWM;
    }
}

// --- Non-Blocking PWM Ramping ---
void updateMotor() {
    unsigned long now = millis();
    
    // Only update if interval has passed
    if (now - lastRampTime >= RAMP_INTERVAL) {
        lastRampTime = now;
        
        if (currentPWM < targetPWM) {
            currentPWM += RAMP_STEP;
            if (currentPWM > targetPWM) currentPWM = targetPWM;
        } 
        else if (currentPWM > targetPWM) {
            currentPWM -= RAMP_STEP;
            if (currentPWM < targetPWM) currentPWM = targetPWM;
        }
        
        // Write to Hardware Register
        OCR0A = currentPWM;
    }
}

// --- Main Logic ---
void loop() {
    // Handle Button Inputs (Debouncing omitted for brevity, but should be here)
    if (digitalRead(BUTTON_PIN) == HIGH) {
        delay(200); // Simple debounce
        stateStartTime = millis(); // Reset auto-off timer on interaction
        
        // State Transition Logic
        switch (currentState) {
            case STATE_OFF:     currentState = STATE_LEVEL_1; break;
            case STATE_LEVEL_1: currentState = STATE_LEVEL_2; break;
            case STATE_LEVEL_2: currentState = STATE_LEVEL_3; break;
            case STATE_LEVEL_3: currentState = STATE_LEVEL_4; break;
            case STATE_LEVEL_4: currentState = STATE_OFF;     break;
        }
    }

    // Handle State Output Targets
    switch (currentState) {
        case STATE_OFF:     targetPWM = PWM_PULSE_MIN; break;
        case STATE_LEVEL_1: targetPWM = PWM_LEVEL_1; break;
        case STATE_LEVEL_2: targetPWM = PWM_LEVEL_2; break;
        case STATE_LEVEL_3: targetPWM = PWM_LEVEL_3; break;
        case STATE_LEVEL_4: targetPWM = PWM_LEVEL_4; break;
    }

    // Safety: Auto-off
    if (currentState != STATE_OFF && (millis() - stateStartTime > AUTO_OFF_MS)) {
        currentState = STATE_OFF;
    }

    // Update Physics
    updateMotor();
}

void setup() {
    initHardware();
}