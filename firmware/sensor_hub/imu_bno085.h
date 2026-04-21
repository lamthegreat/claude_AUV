#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>

#include "micro_ros_config.h"

// BNO085 datasheet typical pitch/roll accuracy: ~1° static, ~2° dynamic.
// Using the dynamic (conservative) spec: (2° × π/180)² ≈ 1.22e-3 rad²
// Use for orientation_covariance roll [0] and pitch [4] diagonal elements.
// Yaw [8] is set per-cycle from the real-time heading_accuracy_rad report.
static constexpr double PITCH_ROLL_VAR_RAD2 = 1.22e-3;  // (2° RMS)²

// ─────────────────────────────────────────────────────────────────────────────
// ImuData — snapshot of all sensor readings from one poll cycle
// ─────────────────────────────────────────────────────────────────────────────
struct ImuData {
    // ARVR Stabilized Rotation Vector quaternion (ROS convention: x,y,z,w)
    float quat_i;       // x
    float quat_j;       // y
    float quat_k;       // z
    float quat_real;    // w
    float heading_accuracy_rad;  // BNO085 estimate of heading error (radians)

    // Calibrated gyroscope (rad/s, body frame)
    float gyro_x;
    float gyro_y;
    float gyro_z;

    // Linear acceleration — gravity compensated (m/s², body frame)
    float accel_x;
    float accel_y;
    float accel_z;

    // Calibrated magnetometer (µT)
    float mag_x;
    float mag_y;
    float mag_z;

    // Game Rotation Vector quaternion (ROS convention: x,y,z,w)
    // 6-DOF: gyro + accel only — no magnetometer, no absolute heading reference.
    // Immune to magnetic interference from motors/ESCs. Use as fallback when
    // heading_accuracy_rad is large or calib_mag < 2.
    float game_quat_i;   // x
    float game_quat_j;   // y
    float game_quat_k;   // z
    float game_quat_real; // w

    // Per-sensor calibration accuracy (0=unreliable, 1=low, 2=med, 3=high)
    // These come from the sh2_SensorValue_t.status field on each report.
    uint8_t calib_rv;       // ARVR stabilized rotation vector (9-DOF)
    uint8_t calib_game_rv;  // Game rotation vector (6-DOF)
    uint8_t calib_gyro;
    uint8_t calib_accel;
    uint8_t calib_mag;

    bool valid;
    uint32_t timestamp_ms;
};

// ─────────────────────────────────────────────────────────────────────────────
// Bno085Driver — wraps Adafruit_BNO08x for the AUV sensor hub
//
// Required library: "Adafruit BNO08x" (install via Arduino Library Manager)
//   https://github.com/adafruit/Adafruit_BNO08x
//
// Usage:
//   Bno085Driver imu;
//   imu.begin(CS_PIN, INT_PIN, RST_PIN);
//   while (true) { imu.update(); }
// ─────────────────────────────────────────────────────────────────────────────
class Bno085Driver {
public:
    Bno085Driver();

    // Initialise BNO085 over SPI and enable sensor reports.
    // cs_pin  — chip select
    // int_pin — data-ready interrupt from sensor
    // rst_pin — hardware reset (-1 to skip, not recommended)
    // Returns true on success, false if sensor not found.
    bool begin(int8_t cs_pin, int8_t int_pin, int8_t rst_pin);

    // Drain the BNO085 event FIFO. Call as often as possible (every loop iteration).
    // Returns true if at least one event was processed this call.
    bool update();

    // Latest fused data snapshot (updated by update()).
    const ImuData& data() const { return _data; }

    // True when all four accuracy fields == 3 (fully calibrated).
    bool isFullyCalibrated() const;

    // Lowest accuracy value across all four sensor types (0–3).
    // Useful for a single LED/display indicator of overall calibration quality.
    uint8_t minCalibration() const;

    // ── Calibration ───────────────────────────────────────────────────────────

    // Save the current Dynamic Calibration Data (DCD) to BNO085 internal flash.
    // This makes calibration persist across power cycles.
    //
    // When to call:
    //   After all calibration fields reach 3 (or an acceptable level).
    //   In calibration mode, after moving the sensor through figure-8 motions
    //   for magnetometer and slow rotations for gyroscope.
    //
    // Blocks until the BNO085 acknowledges the save (~200 ms max).
    // Returns true on success.
    bool saveCalibration();

    // Print a human-readable calibration status table to a Stream (e.g. Serial).
    // Example output:
    //   BNO085 Calibration Status
    //   Rot.Vector  : [###.] (3)  heading err: ±2.3 deg
    //   Gyroscope   : [####] (3)
    //   Accel       : [###.] (3)
    //   Magnetometer: [##..] (2)
    void printCalibrationReport(Stream& stream) const;

private:
    Adafruit_BNO08x _bno;
    ImuData _data{};

    // Enable the four sensor reports at their target publish rates.
    void _enableReports();

    // Unpack one sh2_SensorValue_t event into _data fields.
    void _handleEvent(const sh2_SensorValue_t& ev);

    // Build a 4-char bar string like "###." for a given level 0-3.
    static void _bar(uint8_t level, char out[5]);
};
