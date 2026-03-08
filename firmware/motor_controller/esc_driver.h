#pragma once

#include <Arduino.h>
#include "micro_ros_config.h"

/**
 * ESC Driver
 *
 * Controls up to NUM_THRUSTERS bidirectional ESCs via PWM.
 * Standard hobby ESC signal: 50 Hz, 1100-1900 μs pulse width.
 *   1100 μs = full reverse
 *   1500 μs = neutral / stop
 *   1900 μs = full forward
 *
 * Teensy 4.0 uses analogWrite() with analogWriteFrequency() for PWM.
 * PWM resolution set to 14-bit (0-16383) for smooth control.
 *
 * ESC Arming sequence (standard):
 *   1. Power on ESC with signal at neutral (1500 μs)
 *   2. Wait 2-3 seconds for ESC to arm
 *   3. ESC will beep to confirm armed
 *
 * IMPORTANT: Always verify ESC arming with motors disconnected first.
 */

class EscDriver {
public:
    EscDriver() {}

    // Initialize all ESC PWM outputs. Call once in setup().
    // Outputs neutral (1500 μs) on all channels immediately.
    void begin();

    // Set PWM pulse width for a single thruster (0-indexed).
    // pulse_us is clamped to [PWM_MIN_US, PWM_MAX_US].
    void setPwm(uint8_t index, uint16_t pulse_us);

    // Set all thrusters to neutral (1500 μs).
    void allNeutral();

    // Set all thrusters. pulse_us array must have NUM_THRUSTERS elements.
    void setAll(const uint16_t pulse_us[NUM_THRUSTERS]);

    // Returns the last commanded PWM value for a thruster.
    uint16_t getPwm(uint8_t index) const;

private:
    uint16_t _pwm_us[NUM_THRUSTERS];
    static const uint8_t _pins[NUM_THRUSTERS];

    // Convert microseconds to analogWrite counts (14-bit at 50 Hz)
    // Period = 1/50 Hz = 20,000 μs → 16383 counts
    // counts = pulse_us / 20000 * 16383
    static uint16_t usToAnalog(uint16_t pulse_us);
};
