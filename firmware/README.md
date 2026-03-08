# Firmware

Arduino sketches for the two Teensy MCUs running microROS.

## Directory Structure

| Directory | MCU | Role |
|-----------|-----|------|
| `sensor_hub/` | Teensy 4.1 | Reads BNO085 IMU, publishes IMU data via microROS |
| `motor_controller/` | Teensy 4.0 | Receives thruster commands, drives 6 ESCs via PWM |

## Required Libraries

Install these via **Arduino Library Manager** (Tools → Manage Libraries):

### Both MCUs
- **micro_ros_arduino** — microROS Arduino port
  - Install from: https://github.com/micro-ROS/micro_ros_arduino/releases
  - Download the `.zip` for your ROS2 distro (Jazzy)
  - Install via Sketch → Include Library → Add .ZIP Library

### Sensor Hub (Teensy 4.1)
- **SparkFun BNO08x Arduino Library** (search "SparkFun BNO08x")
  - Or: **Adafruit BNO08x** (search "Adafruit BNO08x")

## Toolchain Setup

1. Install **Arduino IDE** (2.x recommended)
2. Install **Teensyduino** add-on (https://www.pjrc.com/teensy/td_download.html)
3. In Arduino IDE: Tools → Board → Teensyduino → **Teensy 4.1** (sensor hub)
   or **Teensy 4.0** (motor controller)
4. Tools → USB Type → **Serial**
5. Tools → CPU Speed → **600 MHz**

## Flashing

```bash
# From repo root
scripts/flash_sensor_hub.sh      # Flash Teensy 4.1
scripts/flash_motor_controller.sh  # Flash Teensy 4.0
```

Or use Arduino IDE directly (Ctrl+U to compile and upload).

## Custom Message Build (Advanced)

The firmware uses `std_msgs/Float32MultiArray` for Phase 3 bring-up to avoid
the custom message build step. Once the system is working end-to-end, upgrade to
`auv_msgs/ThrusterCommand` by:

1. Follow the micro_ros_arduino custom message build instructions:
   https://github.com/micro-ROS/micro_ros_arduino/tree/jazzy#adding-custom-ros-2-messages
2. Include your `auv_msgs` package in the extra packages list
3. Rebuild the micro_ros_arduino precompiled library
4. Update the `.ino` files to use `auv_msgs/msg/thruster_command.h`

## Testing Without a Pi

Connect Teensy 4.0 to any laptop with micro_ros_agent installed:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000
```
Then publish a test command to safe-check the neutral output:
```bash
# All neutral, armed=0
ros2 topic pub /auv/thrusters/commands std_msgs/msg/Float32MultiArray \
  "{data: [0.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0]}"
```
Verify ESCs stay at neutral (no motor movement).
