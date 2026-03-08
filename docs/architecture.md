# AUV Software Architecture

## Overview

Layered architecture: firmware (microROS on Teensys) → hardware bridges (Pi ROS2 nodes) → autonomy stack (state estimation, control, navigation).

```
┌──────────────────────────── Raspberry Pi 5 (ROS2 Jazzy) ───────────────────────────┐
│                                                                                      │
│  /auv/mission_node          /auv/controller_node        /auv/safety_monitor_node    │
│       │                           │                              │                   │
│       └──→ /auv/control/setpoint  │                              │                   │
│                                   ↓                              │                   │
│  /auv/state_estimator_node ──→ /auv/control/wrench_output        │                   │
│       ↑                           ↓                              ↓                   │
│       │                  /auv/motor_ctrl_bridge    /auv/safety/emergency_surface     │
│       │                           │                                                  │
│  /auv/sensor_hub_node             ↓                                                  │
│       ↑                  /auv/thrusters/commands                                     │
│       │                           │                                                  │
│  [microROS agent]           [microROS agent]                                         │
└───────┼─────────────────────────┼───────────────────────────────────────────────────┘
        │ USB serial              │ USB serial
        │ /dev/auv_sensor_hub     │ /dev/auv_motor_ctrl
        ↓                        ↓
   Teensy 4.1               Teensy 4.0
   Sensor Hub               Motor Controller
   - BNO085 IMU (SPI)       - 6x ESC PWM outputs
   - Future: depth sensor   - 200ms watchdog
```

## Topic Reference

| Topic | Type | Hz | Direction |
|-------|------|----|-----------|
| `/auv/sensors/imu/raw` | `sensor_msgs/Imu` | 200 | Teensy 4.1 → Pi |
| `/auv/sensors/imu/extended` | `auv_msgs/ImuExtended` | 200 | Teensy 4.1 → Pi |
| `/auv/sensors/imu/magnetic_field` | `sensor_msgs/MagneticField` | 100 | Teensy 4.1 → Pi |
| `/auv/sensors/depth` | `auv_msgs/DepthStamped` | 50 | Teensy 4.1 → Pi (future) |
| `/auv/state/auv_state` | `auv_msgs/AuvState` | 50 | state_estimator → all |
| `/auv/state/pose` | `geometry_msgs/PoseStamped` | 50 | state_estimator → all |
| `/auv/control/setpoint/pose` | `geometry_msgs/PoseStamped` | on-demand | mission → controller |
| `/auv/control/setpoint/twist` | `geometry_msgs/TwistStamped` | on-demand | teleop → controller |
| `/auv/control/wrench_output` | `geometry_msgs/WrenchStamped` | 50 | controller → motor bridge |
| `/auv/thrusters/commands` | `auv_msgs/ThrusterCommand` | 50 | motor bridge → Teensy 4.0 |
| `/auv/thrusters/feedback` | `auv_msgs/ThrusterFeedback` | 10 | Teensy 4.0 → Pi |
| `/auv/thrusters/armed` | `std_msgs/Bool` | latched | motor bridge |
| `/auv/safety/emergency_surface` | `std_msgs/Bool` | latched | safety monitor |

## Service Reference

| Service | Type | Server |
|---------|------|--------|
| `/auv/thrusters/arm` | `auv_msgs/ArmThrusters` | motor_controller_node |
| `/auv/thrusters/set_allocation` | `auv_msgs/SetThrusterAllocation` | motor_controller_node |
| `/auv/control/set_mode` | `auv_msgs/SetControlMode` | controller_node |

## TF Tree

```
map
 └── odom
      └── base_link
           ├── imu_link
           ├── thruster_fl_link ... thruster_vr_link
```

## Control Modes

| Mode | Value | Description |
|------|-------|-------------|
| MANUAL | 0 | Setpoint twist passed directly as wrench (teleop) |
| DEPTH_HOLD | 1 | PID on depth; surge/sway/yaw from operator |
| HEADING_HOLD | 2 | PID on heading; depth/surge from operator |
| FULL_AUTO | 3 | All axes under PID control from mission planner |

## Thruster Allocation

The `ThrusterAllocator` in `auv_motor_controller` computes:

```
thrust[N] = pinv(TAM[6×N]) @ wrench[6]
```

Where TAM columns are thrusters and rows are `[Fx, Fy, Fz, Mx, My, Mz]`.
Thrust is normalized, scale-protected, and converted to PWM μs.
Config YAML files in `auv_motor_controller/config/thruster_configs/`.

## Implementation Phases

| Phase | Deliverable | Verification |
|-------|-------------|--------------|
| 0 | Repo scaffold | `colcon build` succeeds |
| 1 | auv_msgs | `ros2 interface show auv_msgs/msg/AuvState` |
| 2 | Sensor hub firmware | `ros2 topic echo /auv/sensors/imu/raw` |
| 3 | Motor controller firmware | Watchdog safe at 200ms silence |
| 4 | Motor bridge + TAM | Wrench → PWM on all 6 ESCs |
| 5 | State estimator | TF tree visible, /auv/state/auv_state publishing |
| 6 | Controller (depth hold) | Hold 0.5m depth in bucket |
| 7 | Safety monitor | Emergency surface on IMU timeout |
| 8 | Full bringup + URDF | `auv_full.launch.py` brings up all nodes |
| 9 | Mission capability | Execute example_mission.yaml |
