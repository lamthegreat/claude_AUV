/**
 * AUV Sensor Hub Firmware — Teensy 4.1
 *
 * Two operating modes selected at power-on by CALIB_MODE_PIN (see micro_ros_config.h):
 *
 * ── NORMAL MODE (default) ─────────────────────────────────────────────────────
 *   CALIB_MODE_PIN floating / HIGH → normal microROS operation.
 *   Publishes over USB-CDC serial to the Raspberry Pi microROS agent:
 *     auv/sensors/imu/raw              sensor_msgs/Imu        @ 200 Hz
 *     auv/sensors/imu/magnetic_field   sensor_msgs/MagneticField @ 100 Hz
 *
 *   microROS agent invocation (on Pi):
 *     ros2 run micro_ros_agent micro_ros_agent serial \
 *       --dev /dev/auv_sensor_hub -b 6000000
 *
 * ── CALIBRATION MODE ──────────────────────────────────────────────────────────
 *   Pull CALIB_MODE_PIN LOW before powering on → calibration mode.
 *   Connect the Teensy directly to a laptop (no Pi / no microROS agent needed).
 *   Open Serial Monitor at 115200 baud.
 *
 *   What to do:
 *     1. Hold the AUV still for a few seconds (gyro calibrates).
 *     2. Slowly rotate the AUV through all orientations (accel calibrates).
 *     3. Move the AUV in a figure-8 pattern to calibrate the magnetometer.
 *     4. Watch calibration bars improve toward [####] (3/3) for all sensors.
 *     5. When satisfied, send 's' + Enter in Serial Monitor to save to flash.
 *        The save persists across power cycles.
 *     6. Send 'r' to reset back to normal mode.
 *
 *   LED in calibration mode:
 *     Fast blink (100ms)  — sensor init failed
 *     N blinks + 1s pause — current minimum calibration level (0=off, 1-3 blinks)
 *     Solid ON            — fully calibrated (all sensors = 3)
 *
 * ── Libraries required ────────────────────────────────────────────────────────
 *   Install via Arduino Library Manager:
 *     - micro_ros_arduino  (Jazzy release .zip from GitHub releases)
 *     - Adafruit BNO08x    (search "Adafruit BNO08x" in Library Manager)
 *     - Adafruit BusIO     (dependency, auto-installed)
 */

#include <Arduino.h>
#include "micro_ros_config.h"
#include "imu_bno085.h"

// ── microROS (normal mode only) ───────────────────────────────────────────────
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

// ─────────────────────────────────────────────────────────────────────────────
// Globals
// ─────────────────────────────────────────────────────────────────────────────
Bno085Driver imu;
static bool calibration_mode = false;

// microROS objects (used only in normal mode)
rcl_publisher_t     pub_imu_raw;
rcl_publisher_t     pub_imu_mag;
sensor_msgs__msg__Imu            msg_imu;
sensor_msgs__msg__MagneticField  msg_mag;
rclc_support_t    support;
rcl_allocator_t   allocator;
rcl_node_t        node;
rclc_executor_t   executor;
rcl_timer_t       timer_imu;
rcl_timer_t       timer_mag;

// Static frame-ID strings (lifetime must exceed msg use)
static char frame_id_buf[] = "imu_link";

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
static void error_blink_fast() {
    // Fast blink indefinitely — unrecoverable error
    while (true) {
        digitalWrite(LED_BUILTIN, HIGH); delay(50);
        digitalWrite(LED_BUILTIN, LOW);  delay(50);
    }
}

#define RCCHECK(fn) { \
    rcl_ret_t _rc = (fn); \
    if (_rc != RCL_RET_OK) { error_blink_fast(); } \
}

// Blink the LED N times then pause (used in calibration mode for status)
static void blink_n(uint8_t n, uint32_t on_ms = 150, uint32_t pause_ms = 900) {
    for (uint8_t i = 0; i < n; i++) {
        digitalWrite(LED_BUILTIN, HIGH); delay(on_ms);
        digitalWrite(LED_BUILTIN, LOW);  delay(on_ms);
    }
    delay(pause_ms);
}

// ─────────────────────────────────────────────────────────────────────────────
// Normal mode: microROS timer callbacks
// ─────────────────────────────────────────────────────────────────────────────
static void cb_imu(rcl_timer_t* /*timer*/, int64_t /*last*/) {
    imu.update();  // drain FIFO each cycle
    const ImuData& d = imu.data();
    if (!d.valid) return;

    uint32_t now_ms = millis();
    msg_imu.header.stamp.sec     = now_ms / 1000;
    msg_imu.header.stamp.nanosec = (now_ms % 1000) * 1000000UL;

    // Quaternion (ROS: x=i, y=j, z=k, w=real)
    msg_imu.orientation.x = d.quat_i;
    msg_imu.orientation.y = d.quat_j;
    msg_imu.orientation.z = d.quat_k;
    msg_imu.orientation.w = d.quat_real;

    msg_imu.angular_velocity.x    = d.gyro_x;
    msg_imu.angular_velocity.y    = d.gyro_y;
    msg_imu.angular_velocity.z    = d.gyro_z;

    msg_imu.linear_acceleration.x = d.accel_x;
    msg_imu.linear_acceleration.y = d.accel_y;
    msg_imu.linear_acceleration.z = d.accel_z;

    rcl_publish(&pub_imu_raw, &msg_imu, nullptr);
}

static void cb_mag(rcl_timer_t* /*timer*/, int64_t /*last*/) {
    const ImuData& d = imu.data();
    if (!d.valid) return;

    uint32_t now_ms = millis();
    msg_mag.header.stamp.sec     = now_ms / 1000;
    msg_mag.header.stamp.nanosec = (now_ms % 1000) * 1000000UL;

    msg_mag.magnetic_field.x = d.mag_x;
    msg_mag.magnetic_field.y = d.mag_y;
    msg_mag.magnetic_field.z = d.mag_z;

    rcl_publish(&pub_imu_mag, &msg_mag, nullptr);
}

// ─────────────────────────────────────────────────────────────────────────────
// Normal mode: setup
// ─────────────────────────────────────────────────────────────────────────────
static void setup_normal_mode() {
    // microROS transport: USB-CDC serial
    set_microros_serial_transports(Serial);
    Serial.begin(MICRO_ROS_BAUD);
    delay(2000);  // Allow agent time to be ready

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, MICRO_ROS_NAMESPACE, &support));

    RCCHECK(rclc_publisher_init_best_effort(
        &pub_imu_raw, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        TOPIC_IMU_RAW));

    RCCHECK(rclc_publisher_init_best_effort(
        &pub_imu_mag, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        TOPIC_IMU_MAG));

    // Timer periods in nanoseconds
    RCCHECK(rclc_timer_init_default(
        &timer_imu, &support,
        RCL_MS_TO_NS(1000 / IMU_PUBLISH_HZ),
        cb_imu));
    RCCHECK(rclc_timer_init_default(
        &timer_mag, &support,
        RCL_MS_TO_NS(1000 / MAG_PUBLISH_HZ),
        cb_mag));

    // Executor: 2 timer handles
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_mag));

    // Init messages
    sensor_msgs__msg__Imu__init(&msg_imu);
    sensor_msgs__msg__MagneticField__init(&msg_mag);

    // Frame IDs
    msg_imu.header.frame_id.data     = frame_id_buf;
    msg_imu.header.frame_id.size     = strlen(frame_id_buf);
    msg_imu.header.frame_id.capacity = sizeof(frame_id_buf);
    msg_mag.header.frame_id.data     = frame_id_buf;
    msg_mag.header.frame_id.size     = strlen(frame_id_buf);
    msg_mag.header.frame_id.capacity = sizeof(frame_id_buf);

    // Unknown covariance: first diagonal element = -1 signals "unknown" to ROS
    msg_imu.orientation_covariance[0]         = -1.0;
    msg_imu.angular_velocity_covariance[0]    = -1.0;
    msg_imu.linear_acceleration_covariance[0] = -1.0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Calibration mode: loop body
// ─────────────────────────────────────────────────────────────────────────────
static uint32_t last_report_ms = 0;

static void run_calibration_mode() {
    imu.update();

    // LED feedback: solid = fully cal, else blink N = min calibration level
    if (imu.isFullyCalibrated()) {
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        // Non-blocking blink: blink minCalibration() times every ~2 seconds
        static uint32_t blink_next = 0;
        static uint8_t  blink_step = 0;
        static uint8_t  blink_target = 0;
        uint32_t now = millis();

        if (now >= blink_next) {
            uint8_t min_cal = imu.minCalibration();
            if (blink_step == 0) {
                blink_target = min_cal;
            }
            if (blink_step < blink_target * 2) {
                // ON for even steps, OFF for odd
                digitalWrite(LED_BUILTIN, (blink_step % 2 == 0) ? HIGH : LOW);
                blink_next = now + 150;
                blink_step++;
            } else {
                // Pause between cycles
                digitalWrite(LED_BUILTIN, LOW);
                blink_next = now + 1000;
                blink_step = 0;
            }
        }
    }

    // Print calibration report every second
    uint32_t now = millis();
    if (now - last_report_ms >= 1000) {
        last_report_ms = now;
        Serial.println();
        imu.printCalibrationReport(Serial);
        Serial.println(F("Commands: 's' = save calibration  'r' = reboot to normal mode"));
    }

    // Handle serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 's' || cmd == 'S') {
            Serial.println(F("\nSaving calibration to BNO085 flash..."));
            if (imu.saveCalibration()) {
                Serial.println(F("✓ Calibration saved successfully!"));
                // Blink 5 times to confirm save
                for (int i = 0; i < 5; i++) {
                    digitalWrite(LED_BUILTIN, HIGH); delay(100);
                    digitalWrite(LED_BUILTIN, LOW);  delay(100);
                }
            } else {
                Serial.println(F("✗ Save failed — check SH2 connection."));
            }
        } else if (cmd == 'r' || cmd == 'R') {
            Serial.println(F("Rebooting..."));
            delay(100);
            SCB_AIRCR = 0x05FA0004;  // Teensy software reset
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Arduino entry points
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Check calibration mode pin before anything else
    pinMode(CALIB_MODE_PIN, INPUT_PULLUP);
    delay(10);
    calibration_mode = (digitalRead(CALIB_MODE_PIN) == LOW);

    if (calibration_mode) {
        Serial.begin(CALIB_SERIAL_BAUD);
        while (!Serial && millis() < 3000) {}  // Wait for USB serial (laptop)
        Serial.println(F("\n=== BNO085 CALIBRATION MODE ==="));
        Serial.println(F("Initialising sensor..."));
    }

    // Initialise BNO085
    if (!imu.begin(BNO085_CS_PIN, BNO085_INT_PIN, BNO085_RST_PIN)) {
        if (calibration_mode) {
            Serial.println(F("ERROR: BNO085 not found! Check wiring."));
        }
        error_blink_fast();  // Halt — sensor required
    }

    if (calibration_mode) {
        Serial.println(F("BNO085 found. Starting calibration mode.\n"));
        Serial.println(F("Tips:"));
        Serial.println(F("  Gyro    — hold still for a few seconds"));
        Serial.println(F("  Accel   — rotate slowly through all orientations"));
        Serial.println(F("  Mag     — move in figure-8 patterns in all planes"));
        Serial.println(F("  RV      — improves automatically as others calibrate\n"));
        last_report_ms = 0;
    } else {
        setup_normal_mode();
        digitalWrite(LED_BUILTIN, HIGH);  // Solid = microROS running
    }
}

void loop() {
    if (calibration_mode) {
        run_calibration_mode();
    } else {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    }
}
