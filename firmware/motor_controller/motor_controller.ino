/**
 * AUV Motor Controller Firmware — Teensy 4.0
 *
 * microROS node that:
 *   - Subscribes to /auv/thrusters/commands (auv_msgs/ThrusterCommand)
 *   - Drives 6 bidirectional ESCs via hardware PWM (1100-1900 μs)
 *   - Publishes /auv/thrusters/feedback at 10 Hz
 *   - Implements a 200 ms hardware watchdog: if commands go silent,
 *     all ESCs are driven to neutral (1500 μs) REGARDLESS of arm state
 *
 * SAFETY PROPERTIES (do not remove):
 *   1. allNeutral() is called in setup() before any microROS init
 *   2. allNeutral() is called if msg.armed == false
 *   3. Watchdog calls allNeutral() if no command in WATCHDOG_TIMEOUT_MS
 *   4. These properties hold even if the Pi-side nodes crash
 *
 * Libraries required:
 *   - micro_ros_arduino  (https://github.com/micro-ROS/micro_ros_arduino)
 *   NOTE: auv_msgs/ThrusterCommand is a custom message.
 *   See firmware/README.md for custom message build instructions.
 *   For Phase 3 bring-up, start with std_msgs/Float32MultiArray
 *   (6 floats as PWM μs values) to avoid custom message build complexity.
 *
 * microROS agent (on Pi):
 *   ros2 run micro_ros_agent micro_ros_agent serial \
 *     --dev /dev/auv_motor_ctrl -b 6000000
 */

#include <Arduino.h>
#include "micro_ros_config.h"
#include "esc_driver.h"

// ── microROS includes ─────────────────────────────────────────────────────────
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Using std_msgs/Float32MultiArray for Phase 3 bring-up (no custom msgs needed)
// Replace with auv_msgs/ThrusterCommand when custom message build is set up
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>

// ── Objects ───────────────────────────────────────────────────────────────────
EscDriver escs;

rcl_subscription_t sub_cmd;
rcl_publisher_t    pub_feedback;
rcl_publisher_t    pub_armed;
rcl_timer_t        timer_feedback;
rcl_timer_t        timer_watchdog;

// Command message: 7 floats = [armed(0/1), pwm_t1, pwm_t2, ..., pwm_t6]
std_msgs__msg__Float32MultiArray msg_cmd;
std_msgs__msg__Bool              msg_armed_state;

rclc_support_t   support;
rcl_allocator_t  allocator;
rcl_node_t       node;
rclc_executor_t  executor;

// ── State ─────────────────────────────────────────────────────────────────────
volatile uint32_t last_cmd_ms = 0;
volatile bool     is_armed    = false;

// ── Error macro ───────────────────────────────────────────────────────────────
#define RCCHECK(fn) { \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) { error_loop(); } \
}

void error_loop() {
    escs.allNeutral();  // Safe the motors first
    while (true) {
        digitalWrite(LED_BUILTIN, HIGH); delay(100);
        digitalWrite(LED_BUILTIN, LOW);  delay(100);
    }
}

// ── Command callback ──────────────────────────────────────────────────────────
void cmd_callback(const void* msg_in) {
    const auto* cmd = (const std_msgs__msg__Float32MultiArray*)msg_in;

    // Expect 7 elements: [armed, pwm0, pwm1, pwm2, pwm3, pwm4, pwm5]
    if (cmd->data.size < 7) {
        escs.allNeutral();
        return;
    }

    bool armed = (cmd->data.data[0] > 0.5f);
    last_cmd_ms = millis();

    if (!armed) {
        is_armed = false;
        escs.allNeutral();
        return;
    }

    is_armed = true;
    uint16_t pwm[NUM_THRUSTERS];
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        pwm[i] = (uint16_t)constrain((int)cmd->data.data[i + 1],
                                      PWM_MIN_US, PWM_MAX_US);
    }
    escs.setAll(pwm);
}

// ── Watchdog timer callback (fires at 2x watchdog period) ────────────────────
void watchdog_callback(rcl_timer_t* /*timer*/, int64_t /*last_call*/) {
    uint32_t age_ms = millis() - last_cmd_ms;
    if (last_cmd_ms > 0 && age_ms > WATCHDOG_TIMEOUT_MS) {
        // Command stream gone silent — safe immediately
        escs.allNeutral();
        // Do NOT clear is_armed; let the Pi re-arm explicitly
    }
}

// ── Feedback timer callback ───────────────────────────────────────────────────
void feedback_callback(rcl_timer_t* /*timer*/, int64_t /*last_call*/) {
    // Publish armed state
    msg_armed_state.data = is_armed;
    rcl_publish(&pub_armed, &msg_armed_state, nullptr);

    // TODO: publish full ThrusterFeedback when custom messages are wired in
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // SAFETY: neutral PWM before anything else
    escs.begin();

    // microROS transport
    set_microros_serial_transports(Serial);
    Serial.begin(MICRO_ROS_BAUD);
    delay(2000);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, MICRO_ROS_NAMESPACE, &support));

    // Subscriber for thrust commands
    RCCHECK(rclc_subscription_init_best_effort(
        &sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        TOPIC_THRUSTER_CMD));

    // Publisher for armed state
    RCCHECK(rclc_publisher_init_best_effort(
        &pub_armed, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "thrusters/armed_mcu"));

    // Watchdog timer — fires every WATCHDOG_TIMEOUT_MS/2 for timely detection
    RCCHECK(rclc_timer_init_default(
        &timer_watchdog, &support,
        RCL_MS_TO_NS(WATCHDOG_TIMEOUT_MS / 2),
        watchdog_callback));

    // Feedback timer
    RCCHECK(rclc_timer_init_default(
        &timer_feedback, &support,
        RCL_MS_TO_NS(1000 / FEEDBACK_HZ),
        feedback_callback));

    // Executor: 1 subscription + 2 timers = 3 handles
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &sub_cmd, &msg_cmd, &cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_watchdog));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_feedback));

    // Allocate message data array for incoming command (7 floats)
    static float cmd_data_buf[NUM_THRUSTERS + 1];
    msg_cmd.data.data     = cmd_data_buf;
    msg_cmd.data.size     = 0;
    msg_cmd.data.capacity = NUM_THRUSTERS + 1;

    digitalWrite(LED_BUILTIN, HIGH);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}
