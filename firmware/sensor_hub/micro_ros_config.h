#pragma once

// microROS transport configuration for Teensy 4.1 Sensor Hub
//
// USB-CDC serial at 6 Mbaud (conventional for Teensy microROS projects).
// The actual USB 2.0 link runs at full speed; baud rate is a formality for
// the serial transport API but must match the microROS agent invocation:
//   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/auv_sensor_hub -b 6000000

#define MICRO_ROS_BAUD      6000000

// ROS namespace and node name
#define MICRO_ROS_NAMESPACE "auv"
#define NODE_NAME           "sensor_hub_mcu"

// Topic names (must match Pi-side subscriptions)
#define TOPIC_IMU_RAW       "sensors/imu/raw"
#define TOPIC_IMU_EXTENDED  "sensors/imu/extended"
#define TOPIC_IMU_MAG       "sensors/imu/magnetic_field"
#define TOPIC_DEPTH         "sensors/depth"  // future

// Publish rates
#define IMU_PUBLISH_HZ      200
#define MAG_PUBLISH_HZ      100

// BNO085 SPI pins (Teensy 4.1)
#define BNO085_CS_PIN       10
#define BNO085_WAKE_PIN     9   // not used by Adafruit lib but reserved
#define BNO085_INT_PIN      8
#define BNO085_RST_PIN      7

// Calibration mode pin (Teensy 4.1).
// Pull LOW at power-on to boot into calibration mode instead of normal microROS mode.
// Leave floating (internal pull-up holds HIGH) for normal operation.
#define CALIB_MODE_PIN      6

// Watchdog: if no microROS agent heartbeat for this many ms, attempt reconnect
#define AGENT_TIMEOUT_MS    5000

// Calibration Serial baud (used in calibration mode, separate from microROS baud)
#define CALIB_SERIAL_BAUD   115200
