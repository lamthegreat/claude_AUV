#include "esc_driver.h"

const uint8_t EscDriver::_pins[NUM_THRUSTERS] = {
    ESC_PIN_0, ESC_PIN_1, ESC_PIN_2,
    ESC_PIN_3, ESC_PIN_4, ESC_PIN_5,
};

void EscDriver::begin() {
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        pinMode(_pins[i], OUTPUT);
        // Set 50 Hz PWM frequency on this pin
        analogWriteFrequency(_pins[i], PWM_FREQUENCY_HZ);
        _pwm_us[i] = PWM_NEUTRAL_US;
        analogWrite(_pins[i], usToAnalog(PWM_NEUTRAL_US));
    }
}

void EscDriver::setPwm(uint8_t index, uint16_t pulse_us) {
    if (index >= NUM_THRUSTERS) return;
    pulse_us = constrain(pulse_us, PWM_MIN_US, PWM_MAX_US);
    _pwm_us[index] = pulse_us;
    analogWrite(_pins[index], usToAnalog(pulse_us));
}

void EscDriver::allNeutral() {
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        setPwm(i, PWM_NEUTRAL_US);
    }
}

void EscDriver::setAll(const uint16_t pulse_us[NUM_THRUSTERS]) {
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        setPwm(i, pulse_us[i]);
    }
}

uint16_t EscDriver::getPwm(uint8_t index) const {
    if (index >= NUM_THRUSTERS) return PWM_NEUTRAL_US;
    return _pwm_us[index];
}

uint16_t EscDriver::usToAnalog(uint16_t pulse_us) {
    // 14-bit resolution at 50 Hz
    // PWM period = 20,000 μs = 16383 counts
    // counts = pulse_us * 16383 / 20000
    return (uint16_t)((uint32_t)pulse_us * 16383UL / 20000UL);
}
