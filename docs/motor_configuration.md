# Motor / Thruster Configuration

## DI 2205-2207 Brushless Motors

- **Type**: Brushless outrunner
- **Voltage**: 3S-4S LiPo (11.1V - 14.8V)
- **KV rating**: varies by variant (~2300 KV typical for 2205)
- **Waterproofing**: DIY (conformal coating on windings, sealed bearings, corrosion-inhibiting grease on shaft)

## ESC Selection

Standard PWM hobby ESCs (bidirectional firmware required):
- Must support **bidirectional PWM** (1100=full reverse, 1500=stop, 1900=full forward)
- Many drone ESCs default to unidirectional — must reflash with **BLHeli** bidirectional firmware
  or purchase ESCs pre-configured for bidirectional operation
- Recommended: **BlueRobotics Basic ESC** (designed for underwater, bidirectional by default)
- Alternative: Any 30A+ ESC flashed with BLHeli_32 in bidirectional mode

## PWM Signal

| Value | Meaning |
|-------|---------|
| 1100 μs | Full reverse |
| 1500 μs | Neutral / stop |
| 1900 μs | Full forward |
| 50 Hz | Signal frequency |

## Thruster Allocation Matrix (TAM)

See config files in `ros2_ws/src/auv_motor_controller/config/thruster_configs/`.

Two example configurations are provided:

### h4_v2.yaml — 4 Horizontal + 2 Vertical
Suitable for a box-frame AUV. Easy to build, intuitive behavior.
- 4 horizontal thrusters at 45° to body axes (surge, sway, yaw)
- 2 vertical thrusters (heave, limited roll control)
- **Pitch is not controllable** with this config

### vectored_6dof.yaml — Vectored 6-DOF
More complex, higher control authority. All 6 thrusters at angles.
- Full control over all 6 DOF (surge, sway, heave, roll, pitch, yaw)
- Requires precise measurement of thruster positions/angles
- TAM values in the template are placeholders — must be derived from your geometry

## Deriving the TAM for Your Frame

For each thruster i at position `[px, py, pz]` pointing in direction `[dx, dy, dz]`:

```
TAM column i = [dx, dy, dz, py*dz - pz*dy, pz*dx - px*dz, px*dy - py*dx]
```

This gives the wrench (force + torque) that thruster i produces per unit thrust.
Normalize columns to unit thrust contribution, then use `numpy.linalg.pinv(TAM)` to
compute the pseudoinverse used by `ThrusterAllocator`.

## Motor Direction Convention

With the h4_v2 config:

```
Top view (X = forward, Y = left):

        Front
    FL ↖    ↗ FR
          ×
    RL ↙    ↘ RR
        Rear

Thruster rotation directions must produce the correct force direction.
Swap any two motor wires to reverse rotation direction.
Use a slow forward command (1550 μs) to verify each motor's actual thrust direction
before running closed-loop control.
```

## Thrust Testing

After installation, measure actual thrust with a simple load cell:
1. Mount motor + prop/shroud on a fixture above a scale
2. Command various PWM values, record thrust vs. PWM
3. Update `max_thrust_n` in your thruster config YAML
4. The TAM math assumes linear thrust vs. PWM — this is approximate.
   Real thrust curves are nonlinear. Accept this at low speeds; add linearization later.
