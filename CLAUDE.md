# claude_AUV — Project Notes for Claude Code

## Project Overview
Hobbyist Autonomous Underwater Vehicle (AUV) built on ROS2 Jazzy + microROS.
Learning-focused, low-cost, modular. Raspberry Pi 5 as main computer, two Teensy MCUs as microROS nodes.

## Hardware
| Device | Role | Connection |
|--------|------|------------|
| Raspberry Pi 5 (4GB) | Main computer, runs ROS2 | — |
| Teensy 4.1 | Sensor Hub MCU (BNO085 IMU, future depth/sonar) | USB → /dev/auv_sensor_hub |
| Teensy 4.0 | Motor Controller MCU (6x ESC PWM) | USB → /dev/auv_motor_ctrl |
| BNO085 | 9-DOF IMU with onboard fusion | SPI on Teensy 4.1 |
| 6x DI 2205-2207 | Brushless motors + DIY waterproofing | PWM ESCs on Teensy 4.0 |

## Software Stack
- **OS**: Ubuntu 24.04 (Raspberry Pi 5)
- **ROS2**: Jazzy Jalisco
- **Firmware**: Arduino/C++ with microROS Arduino library
- **Pi-side nodes**: Python (rclpy)
- **`ROS_DOMAIN_ID`**: 42 (isolates AUV traffic on local network)

## Repository Layout
```
ros2_ws/src/     — ROS2 colcon workspace packages
firmware/        — Teensy Arduino sketches (microROS)
config/          — udev rules, systemd services
docs/            — Architecture, wiring, commissioning
scripts/         — Setup, flash, calibration helpers
```

## Build Commands
```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build workspace
cd /home/pi/repos/claude_AUV/ros2_ws
colcon build --symlink-install

# Source workspace overlay
source install/setup.bash

# Build only messages first (other packages depend on it)
colcon build --symlink-install --packages-select auv_msgs
```

## Device Names (after udev rules deployed)
- `/dev/auv_sensor_hub` → Teensy 4.1 (sensor hub)
- `/dev/auv_motor_ctrl` → Teensy 4.0 (motor controller)

## microROS Agent Commands
```bash
# Run both agents (each in its own terminal)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/auv_sensor_hub -b 6000000
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/auv_motor_ctrl  -b 6000000
```

## Topic Namespace Convention
All AUV topics live under `/auv/`. Examples:
- `/auv/sensors/imu/raw`
- `/auv/thrusters/commands`
- `/auv/state/auv_state`
- `/auv/safety/emergency_surface`

## Safety Rules
- NEVER command thrusters armed without a human present
- Teensy 4.0 has a 200 ms hardware watchdog — motors safe automatically if commands stop
- Always run `auv_safety` node before enabling thrusters
- Test ESC neutral (1500 μs) before any water entry

## Coding Conventions
- Python nodes use `rclpy` lifecycle nodes where stateful
- All parameters declared and loaded from YAML (no magic numbers in code)
- Log at `INFO` for state changes, `DEBUG` for per-cycle data
- ROS2 package names: `auv_<subsystem>` (snake_case)
- Firmware files: `<subsystem>.<ext>` (snake_case)

## Testing
- Write code that is unit-testable: extract critical logic into pure functions or classes that take plain data and return plain data, keeping ROS2 I/O (publishers, subscribers, timers) separate from business logic
- Include unit tests for all critical logic: control math, thruster allocation, state estimation filters, safety watchdog conditions, sensor parsing
- Tests live in `ros2_ws/src/<package>/test/` and use `pytest` (Python) or `gtest` (C++)
- Test files are named `test_<module>.py` and registered in `CMakeLists.txt` / `setup.cfg`
- Run tests with: `colcon test --packages-select <package> && colcon test-result --verbose`
- Do not mock ROS2 infrastructure when the logic under test does not require it — pass values directly to the function
