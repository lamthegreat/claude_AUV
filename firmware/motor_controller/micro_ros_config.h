#pragma once

// microROS transport configuration for Teensy 4.0 Motor Controller

#define MICRO_ROS_BAUD      6000000
#define MICRO_ROS_NAMESPACE "auv"
#define NODE_NAME           "motor_controller_mcu"

// Topic names
#define TOPIC_THRUSTER_CMD  "thrusters/commands"
#define TOPIC_THRUSTER_FB   "thrusters/feedback"

// ESC PWM settings
#define NUM_THRUSTERS       6
#define PWM_NEUTRAL_US      1500
#define PWM_MIN_US          1100
#define PWM_MAX_US          1900
#define PWM_FREQUENCY_HZ    50    // Standard ESC PWM frequency

// Teensy 4.0 hardware PWM pins for 6 ESCs
// Pins 2-9 support hardware PWM timers on Teensy 4.0
#define ESC_PIN_0   2
#define ESC_PIN_1   3
#define ESC_PIN_2   4
#define ESC_PIN_3   5
#define ESC_PIN_4   6
#define ESC_PIN_5   7

// Safety watchdog: if no command received in this many ms, safe to neutral
#define WATCHDOG_TIMEOUT_MS 200

// Feedback publish rate
#define FEEDBACK_HZ         10
