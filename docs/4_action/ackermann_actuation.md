# Ackermann Actuation Pipeline

## Overview

The actuation pipeline converts a `ControlOutput` into Avian3D `ExternalForce` +
`ExternalTorque` across three layers:

```
Layer 1  Controller::compute()  →  ControlOutput
            │  reads: EstimatorComponent (EKF estimated state)
            │  outer loop — setpoint generation, path following, planning
            │
Layer 2  AckermannOutputAdapter::adapt()  →  AckermannCommand { throttle_norm, steering_torque_norm }
            │  reads: Avian3D LinearVelocity, AngularVelocity, Transform
            │  inner loop — low-latency feedback against raw physics state
            │
Layer 3  drive_ackermann_cars  →  ExternalForce + ExternalTorque
            │  pure linear map, no state reads
            ▼
        Avian3D PhysicsSet::Simulate
```

---

## Two Feedback Loops, Two State Sources

The pipeline deliberately uses **different state sources** at each level. This mirrors
real embedded robotics systems:

| | Real hardware | This sim |
|---|---|---|
| Outer loop | EKF / SLAM estimate (global pose, velocity) | `EstimatorComponent` (EKF) |
| Inner loop | IMU gyro + wheel encoders (100–1000 Hz) | Avian3D `LinearVelocity`, `AngularVelocity` |

On a real vehicle, the inner-loop speed and yaw-rate controllers read directly from
low-latency sensors — not from the EKF. The EKF runs slower, introduces lag, and is
designed for navigation-quality global state, not fast actuator feedback. Closing a PID
loop against EKF velocity would require heavy filtering and gain detuning to avoid
instability.

In this sim, Avian3D's `LinearVelocity` and `AngularVelocity` are the equivalent of
those embedded sensors — noiseless for now, but structurally correct. When explicit
sensor simulation (IMU noise, encoder slip) is added in a future pass, those noisy
measurements can replace the raw physics state in the adapter without changing the
architecture.

**Consequence:** A degraded EKF affects Layer 1 (setpoint quality), but Layer 2 remains
stable because it never sees the estimate. End-to-end estimation testing is a future
option; the current priority is a clean architectural split.

---

## Layer 2 — `AckermannOutputAdapter`

### Trait

```rust
pub trait AckermannOutputAdapter: Send + Sync {
    fn adapt(
        &mut self,
        output: &ControlOutput,
        params: &AckermannParameters,
        actuator: &AckermannActuator,
        transform: &Transform,     // Avian3D world transform — inner-loop orientation
        lin_vel: &LinearVelocity,  // Avian3D world-frame velocity — inner-loop speed
        ang_vel: &AngularVelocity, // Avian3D world-frame angular velocity — inner-loop yaw rate
        mass: f32,
        dt: f32,
    ) -> AckermannCommand;
}
```

Stored on the entity as `AckermannAdapterComponent(Box<dyn AckermannOutputAdapter>)`.
Built at scene spawn via the `AutonomyRegistry` adapter factory; configured via
`[adapter]` in the vehicle TOML.

### `AckermannCommand`

```rust
pub struct AckermannCommand {
    pub throttle_norm: f32,        // [-1.0, 1.0]  positive = forward
    pub steering_torque_norm: f32, // [-1.0, 1.0]  positive = left (FLU)
}
```

Both fields normalised to `[-1, 1]`. All kinematics resolved inside the adapter;
Layer 3 is vehicle-agnostic.

---

## Built-in Adapters

### `DefaultAckermannAdapter` — open-loop

`Raw` and `RawActuators` are **direct passthroughs** — values returned as-is without
kinematic conversion. Correct for teleop: the caller expresses normalised intent
("turn this hard"), not a physical steering angle.

| Variant | throttle_norm | steering_torque_norm |
|---|---|---|
| `Raw` | `u[0]` | `u[1]` — direct |
| `RawActuators` | `signals[0]` | `signals[1]` — direct |
| `BodyVelocity` | `linear.x / max_speed` | Ackermann kinematics (see below) |
| `Wrench` | `force.x / max_force` | Ackermann kinematics (see below) |
| `BodyAcceleration` | `linear.x * mass / max_force` | `0.0` |

For `BodyVelocity` and `Wrench`, a `steering_rad` is derived first, then converted:

```
yaw_rate_demand      = |v| / wheelbase * tan(steering_rad)
max_yaw_rate         = max_speed / wheelbase * tan(max_steering_angle)
steering_torque_norm = clamp(yaw_rate_demand / max_yaw_rate, -1, 1)
```

Yaw torque scales with speed: zero at rest, 1.0 at `max_speed` with full lock.
Passive yaw decay comes from `AngularDamping` in `[physics]`.

### `DualSisoPidAdapter` — closed-loop

Two SISO PID channels, both reading raw physics state (inner loop):

| Channel | Error | Output |
|---|---|---|
| Longitudinal | `desired_speed − lin_vel · transform.forward()` | `raw_force / max_force` → `throttle_norm` |
| Lateral | `desired_yaw_rate − ang_vel.y` | `raw_torque / max_torque` → `steering_torque_norm` |

`transform.forward()` = Bevy `-Z` world direction = FLU `+X` in world space.
`ang_vel.y` (Bevy Y-up yaw rate) matches FLU `angular.z` in sign.

**Only fires for `BodyVelocity`.** All other variants fall through to
`DefaultAckermannAdapter`. `DualSisoPid` on a teleop-only vehicle has no effect —
use `Default`.

---

## Choosing an Adapter

| Scenario | Adapter |
|---|---|
| Keyboard / joystick (`RawActuators`) | `Default` |
| Autonomous controller outputting `BodyVelocity` | `DualSisoPid` |
| Model-based controller outputting normalised `Raw` | `Default` |
| Custom feedback (Pacejka, MPC) | Implement `AckermannOutputAdapter` |

---

## Layer 3 — `drive_ackermann_cars`

Pure linear map:

```
force_flu  = throttle_norm        * max_force   (FLU +X = Forward)
torque_flu = steering_torque_norm * max_torque  (FLU +Z = Yaw-up)
```

Both vectors converted to Bevy world frame via `flu_vector_to_bevy_local_vector` +
entity rotation quaternion (`transforms.rs`). No state reads.

---

## TOML Configuration

```toml
kind = "Ackermann"
wheelbase = 2.5
max_steering_angle = 35.0  # degrees
max_steering_rate  = 140.0 # degrees/sec

[physics]
mass             = 1500.0  # kg
friction         = 0.7
linear_damping   = 0.5     # passive drag on release
angular_damping  = 1.5     # passive yaw decay on release

[actuator]
max_force  = 25000.0  # N    must exceed mass * 9.81 * friction to move
max_torque = 10000.0  # N·m
max_speed  = 20.0     # m/s  used by DefaultAdapter for BodyVelocity throttle normalisation

[adapter]
kind = "Default"       # or "DualSisoPid" when using a BodyVelocity autonomous controller
```

### Physics tuning

| Symptom | Fix |
|---|---|
| Won't move | Increase `max_force`; ensure `max_force > mass * 9.81 * friction` |
| Won't turn | Increase `max_torque`; if `BodyVelocity`, check vehicle has non-zero speed (kinematic scaling) |
| Spins out | Decrease `max_torque`; increase `angular_damping` |
| Doesn't stop on throttle release | Increase `linear_damping` |
| `DualSisoPid` oscillates | Reduce `kp`, increase `kd`, check `integral_clamp` |

---

## Implementing a Custom Adapter

```rust
pub struct MyAdapter;

impl AckermannOutputAdapter for MyAdapter {
    fn adapt(&mut self, output: &ControlOutput, _params: &AckermannParameters,
             actuator: &AckermannActuator, transform: &Transform,
             lin_vel: &LinearVelocity, ang_vel: &AngularVelocity,
             _mass: f32, _dt: f32) -> AckermannCommand {
        // Inner-loop physics state:
        let forward_speed = lin_vel.dot(*transform.forward()); // signed m/s
        let yaw_rate = ang_vel.y;                              // rad/s, + = left
        // compute throttle_norm and steering_torque_norm ...
        AckermannCommand { throttle_norm, steering_torque_norm }
    }
}
```

Register in a plugin:
```rust
registry.register_adapter("MyAdapter", |_ctx| Ok(Box::new(MyAdapter)));
```

Activate in TOML: `[adapter] kind = "MyAdapter"`
