#include "imu_bno085.h"

// ─────────────────────────────────────────────────────────────────────────────
// Report intervals (microseconds)
// BNO085 maximum is 400 Hz for rotation vector; 200 Hz is practical with
// the microROS overhead on Teensy 4.1.
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint32_t INTERVAL_RV_US    =  5000;  // 200 Hz
static constexpr uint32_t INTERVAL_GYRO_US  =  5000;  // 200 Hz
static constexpr uint32_t INTERVAL_ACCEL_US =  5000;  // 200 Hz
static constexpr uint32_t INTERVAL_MAG_US   = 10000;  // 100 Hz


// ─────────────────────────────────────────────────────────────────────────────
Bno085Driver::Bno085Driver() : _bno(-1) {
    // -1 passed to Adafruit_BNO08x means "no reset pin managed by library"
    // We handle reset via rst_pin in begin() for reliability.
}

// ─────────────────────────────────────────────────────────────────────────────
bool Bno085Driver::begin(int8_t cs_pin, int8_t int_pin, int8_t rst_pin) {
    // Hardware reset before init improves reliability after power-on
    if (rst_pin >= 0) {
        pinMode(rst_pin, OUTPUT);
        digitalWrite(rst_pin, LOW);
        delay(10);
        digitalWrite(rst_pin, HIGH);
        delay(10);
    }

    // begin_SPI(cs, interrupt, [spi_bus], [speed]) — Adafruit_BNO08x API
    if (!_bno.begin_SPI(cs_pin, int_pin)) {
        return false;
    }

    _enableReports();
    _data.valid = false;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
void Bno085Driver::_enableReports() {
    // ARVR Stabilized Rotation Vector — best for heading in dynamic environments.
    // Uses accelerometer + gyroscope + magnetometer with drift correction.
    _bno.enableReport(SH2_ARVR_STABILIZED_RV,    INTERVAL_RV_US);

    // Game Rotation Vector — 6-DOF (gyro + accel only, no magnetometer).
    // Immune to magnetic interference from motors/ESCs.
    // Heading is relative (arbitrary zero at power-on), not absolute north.
    _bno.enableReport(SH2_GAME_ROTATION_VECTOR,   INTERVAL_RV_US);

    // Calibrated gyroscope — bias-corrected angular rate
    _bno.enableReport(SH2_GYROSCOPE_CALIBRATED,   INTERVAL_GYRO_US);

    // Linear acceleration — gravity-compensated body-frame acceleration
    _bno.enableReport(SH2_LINEAR_ACCELERATION,    INTERVAL_ACCEL_US);

    // Calibrated magnetometer — useful for hard/soft-iron calibration status
    _bno.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, INTERVAL_MAG_US);
}

// ─────────────────────────────────────────────────────────────────────────────
bool Bno085Driver::update() {
    sh2_SensorValue_t ev;
    bool got_any = false;

    // Drain the FIFO — getSensorEvent returns false when empty
    while (_bno.getSensorEvent(&ev)) {
        _handleEvent(ev);
        got_any = true;
    }

    return got_any;
}

// ─────────────────────────────────────────────────────────────────────────────
void Bno085Driver::_handleEvent(const sh2_SensorValue_t& ev) {
    switch (ev.sensorId) {

    case SH2_ARVR_STABILIZED_RV:
        _data.quat_real            = ev.un.arvrStabilizedRV.real;
        _data.quat_i               = ev.un.arvrStabilizedRV.i;
        _data.quat_j               = ev.un.arvrStabilizedRV.j;
        _data.quat_k               = ev.un.arvrStabilizedRV.k;
        _data.heading_accuracy_rad = ev.un.arvrStabilizedRV.accuracy;
        _data.calib_rv             = ev.status & 0x03;  // bits 1:0
        _data.timestamp_ms         = millis();
        _data.valid                = true;
        break;

    case SH2_GYROSCOPE_CALIBRATED:
        _data.gyro_x    = ev.un.gyroscope.x;
        _data.gyro_y    = ev.un.gyroscope.y;
        _data.gyro_z    = ev.un.gyroscope.z;
        _data.calib_gyro = ev.status & 0x03;
        break;

    case SH2_LINEAR_ACCELERATION:
        _data.accel_x    = ev.un.linearAcceleration.x;
        _data.accel_y    = ev.un.linearAcceleration.y;
        _data.accel_z    = ev.un.linearAcceleration.z;
        _data.calib_accel = ev.status & 0x03;
        break;

    case SH2_MAGNETIC_FIELD_CALIBRATED:
        _data.mag_x    = ev.un.magneticField.x;
        _data.mag_y    = ev.un.magneticField.y;
        _data.mag_z    = ev.un.magneticField.z;
        _data.calib_mag = ev.status & 0x03;
        break;

    case SH2_GAME_ROTATION_VECTOR:
        _data.game_quat_real = ev.un.gameRotationVector.real;
        _data.game_quat_i    = ev.un.gameRotationVector.i;
        _data.game_quat_j    = ev.un.gameRotationVector.j;
        _data.game_quat_k    = ev.un.gameRotationVector.k;
        _data.calib_game_rv  = ev.status & 0x03;
        break;

    default:
        break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
bool Bno085Driver::isFullyCalibrated() const {
    return _data.calib_rv    == 3 &&
           _data.calib_gyro  == 3 &&
           _data.calib_accel == 3 &&
           _data.calib_mag   == 3;
}

uint8_t Bno085Driver::minCalibration() const {
    return min({_data.calib_rv, _data.calib_gyro,
                _data.calib_accel, _data.calib_mag});
}

// ─────────────────────────────────────────────────────────────────────────────
bool Bno085Driver::saveCalibration() {
    // sh2_saveDcdNow() writes the current Dynamic Calibration Data to the
    // BNO085's internal flash so it persists across power cycles.
    // It is part of the SH2 HAL that Adafruit_BNO08x links against.
    int rc = sh2_saveDcdNow();
    return (rc == SH2_OK);
}

// ─────────────────────────────────────────────────────────────────────────────
void Bno085Driver::_bar(uint8_t level, char out[5]) {
    // Produces strings like "####", "###.", "##..", "#..."
    for (uint8_t i = 0; i < 4; i++) {
        out[i] = (i < level) ? '#' : '.';
    }
    out[4] = '\0';
}

void Bno085Driver::printCalibrationReport(Stream& stream) const {
    char bar[5];

    stream.println(F("┌─────────────────────────────────────────┐"));
    stream.println(F("│        BNO085 Calibration Status         │"));
    stream.println(F("├─────────────────────────────────────────┤"));

    _bar(_data.calib_rv, bar);
    stream.print(F("│ Rot.Vector  ["));
    stream.print(bar);
    stream.print(F("] ("));
    stream.print(_data.calib_rv);
    stream.print(F("/3)  hdg err ±"));
    stream.print(_data.heading_accuracy_rad * 57.2958f, 1);
    stream.println(F("°       │"));

    _bar(_data.calib_gyro, bar);
    stream.print(F("│ Gyroscope   ["));
    stream.print(bar);
    stream.print(F("] ("));
    stream.print(_data.calib_gyro);
    stream.println(F("/3)                       │"));

    _bar(_data.calib_accel, bar);
    stream.print(F("│ Accel       ["));
    stream.print(bar);
    stream.print(F("] ("));
    stream.print(_data.calib_accel);
    stream.println(F("/3)                       │"));

    _bar(_data.calib_mag, bar);
    stream.print(F("│ Magnetometer["));
    stream.print(bar);
    stream.print(F("] ("));
    stream.print(_data.calib_mag);
    stream.println(F("/3)  <-- move in figure-8 │"));

    stream.println(F("├─────────────────────────────────────────┤"));
    if (isFullyCalibrated()) {
        stream.println(F("│  ✓ FULLY CALIBRATED — press 's' to save │"));
    } else {
        stream.println(F("│  Move sensor to improve calibration...  │"));
    }
    stream.println(F("└─────────────────────────────────────────┘"));
}
