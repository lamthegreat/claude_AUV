# AUV Commissioning Checklist

Work through these in order. Never skip to a later phase until the earlier one is verified.

---

## Phase 1: Workspace Build Verification

- [ ] `colcon build --symlink-install` completes without errors
- [ ] `ros2 pkg list | grep auv` shows all 8 `auv_*` packages
- [ ] `ros2 interface show auv_msgs/msg/AuvState` outputs the message definition
- [ ] `ros2 interface show auv_msgs/msg/ThrusterCommand` outputs the message definition

---

## Phase 2: Sensor Hub Bring-Up (Teensy 4.1 + BNO085)

- [ ] BNO085 wired per `docs/hardware_setup.md`
- [ ] `sensor_hub.ino` compiles in Arduino IDE (Board: Teensy 4.1)
- [ ] Firmware flashed, Teensy appears as `/dev/ttyACM*`
- [ ] udev rules updated with sensor hub serial number
- [ ] `/dev/auv_sensor_hub` symlink present after replug
- [ ] microROS agent starts without error:
  `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/auv_sensor_hub -b 6000000`
- [ ] `ros2 topic list` shows `/auv/sensors/imu/raw`
- [ ] `ros2 topic hz /auv/sensors/imu/raw` reports ~200 Hz
- [ ] `ros2 topic echo /auv/sensors/imu/raw` shows valid quaternion (not all zeros)
- [ ] Rotating sensor causes orientation changes in quaternion values

---

## Phase 3: Motor Controller Bring-Up (Teensy 4.0)

**Safety gate: perform ALL of these with motors DISCONNECTED from ESCs.**

- [ ] ESC signal wires connected to Teensy 4.0 pins 2-7
- [ ] `motor_controller.ino` compiles (Board: Teensy 4.0)
- [ ] Firmware flashed, udev rules updated with motor controller serial
- [ ] `/dev/auv_motor_ctrl` symlink present after replug
- [ ] microROS agent for motor controller starts:
  `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/auv_motor_ctrl -b 6000000`
- [ ] `ros2 topic list` shows `/auv/thrusters/commands` (can subscribe)
- [ ] **Watchdog test**: Start agent, wait 5 seconds without publishing any command.
  Verify with oscilloscope that PWM output is 1500 μs on all channels.
- [ ] **Neutral command test**: Publish:
  ```
  ros2 topic pub --once /auv/thrusters/commands std_msgs/msg/Float32MultiArray \
    "{data: [0.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0]}"
  ```
  Verify PWM = 1500 μs on oscilloscope.

**Now connect ESCs (still no motors):**

- [ ] ESC calibration completed (`scripts/calibrate_escs.sh` — follow the manual steps)
- [ ] Publish neutral, ESCs respond with ready beep pattern

**Now connect motors (out of water, no props):**

- [ ] Arm and send slow forward command:
  ```
  ros2 topic pub /auv/thrusters/commands std_msgs/msg/Float32MultiArray \
    "{data: [1.0, 1550.0, 1550.0, 1550.0, 1550.0, 1550.0, 1550.0]}"
  ```
- [ ] All 6 motors spin in expected direction
- [ ] Disarm (set data[0]=0.0), motors stop
- [ ] **Watchdog test with motors**: Kill the topic publisher. Verify motors stop within 200ms.

---

## Phase 4: Motor Bridge + Thruster Allocator

- [ ] Fill in `thruster_config_path` in `motor_controller_params.yaml`
  (e.g., path to `h4_v2.yaml` or `vectored_6dof.yaml`)
- [ ] Launch motor controller bridge:
  `ros2 launch auv_motor_controller motor_controller.launch.py`
- [ ] Call arm service:
  ```
  ros2 service call /auv/thrusters/arm auv_msgs/srv/ArmThrusters \
    "{enable: true, reason: 'bench test'}"
  ```
- [ ] Publish a wrench command and verify correct motor response:
  ```
  ros2 topic pub /auv/control/wrench_output geometry_msgs/msg/WrenchStamped \
    "{wrench: {force: {x: 1.0, y: 0.0, z: 0.0}}}"
  ```

---

## Phase 5: State Estimator

- [ ] Launch sensor hub + state estimator:
  ```
  ros2 launch auv_bringup auv_hardware.launch.py
  ros2 launch auv_state_estimator state_estimator.launch.py
  ```
- [ ] `ros2 topic echo /auv/state/auv_state` shows valid data
- [ ] `ros2 run tf2_tools view_frames` shows `odom → base_link` transform

---

## Phase 6: Depth Hold Test (First Wet Test)

**Prerequisites**: depth sensor wired and firmware updated, pool/tank available.

- [ ] Safety checklist before entering water:
  - [ ] All connectors sealed / potted
  - [ ] Enclosure O-rings seated and greased
  - [ ] Leak sensor installed and tested
  - [ ] Tether attached
  - [ ] Safety_monitor node running and monitoring
  - [ ] Emergency surface trigger tested (manually trigger it and verify response)
- [ ] AUV neutrally buoyant or slightly positive
- [ ] Deploy at 0.5m depth, arm, enable DEPTH_HOLD mode
- [ ] Verify depth is maintained ±0.1m for 30 seconds
- [ ] Manually trigger emergency surface, verify AUV rises

---

## Notes

- Update this checklist as new phases are completed
- Log all test results with date and conditions
- Never skip the watchdog test — it is the last hardware safety net
