# Lightweight Simulation

The repo now includes a lightweight hardware-free smoke test for the thruster
allocation path. It is designed to be fast enough for local iteration and later
reused in CI.

## Allocator Smoke Test

Run from a sourced workspace:

```bash
ros2 launch auv_sim allocator_smoke.launch.py
```

What it validates:

- a non-zero wrench published while disarmed produces neutral thruster output
- the scenario arms through the real `/auv/thrusters/arm` service
- the same wrench then produces active, bounded thruster output
- silence on `/auv/control/wrench_output` longer than the controller timeout
  returns output to neutral

The timeout check matches the current `auv_motor_controller` implementation:
neutral PWM is published after timeout while the command remains marked armed.

## Optional Rosbag Capture

To record a small rosbag for debugging:

```bash
ros2 launch auv_sim allocator_smoke.launch.py record_bag:=true
```

Recorded topics:

- `/auv/control/wrench_output`
- `/auv/thrusters/commands`
- `/auv/thrusters/armed`

By default the bag is written to `/tmp/auv_allocator_smoke_bag`. Override it
with `bag_output_dir:=/path/to/output`.
