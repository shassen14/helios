# Ackermann Actuation — Architecture Notes

## Current State (as of 2026-03-05)

The actuation pipeline converts a `ControlOutput` (produced by a `Controller` in
`helios_core`) into Avian3D `ExternalForce` + `ExternalTorque` in two steps inside
`drive_ackermann_cars` in `helios_sim/src/simulation/plugins/vehicles/ackermann.rs`.

### Step 1 — `control_output_to_ackermann()` → `(throttle_norm, steering_rad)`

A free function that pattern-matches on the `ControlOutput` variant and maps it to
a normalised throttle `[-1, 1]` and a steering angle in radians.

**Problem:** the conversion is hardcoded. There is no way to swap in a PID or LQR
at this level without editing the function. Every variant mapping is a proportional
approximation — no integrators, no feedback on velocity error.

### Step 2 — `(throttle_norm, steering_rad)` → `ExternalForce + ExternalTorque`

```
// Force — currently a proportional speed controller, NOT a dumb linear map
let speed_error = (throttle_norm * max_speed) - current_forward_speed;
let force = speed_error * max_force;        // ← P controller embedded in actuator layer

// Torque — proportional yaw-rate controller, also embedded in actuator layer
let desired_yaw_rate = (speed / wheelbase) * tan(steering_rad);
let yaw_error = desired_yaw_rate - ang_vel.y;
let torque = yaw_error * max_torque;        // ← P controller embedded in actuator layer
```

**Problem:** both calculations contain feedback loops (they read current physics
state to compute an error). This is low-level control logic that belongs in a
pluggable layer, not hardcoded in the dumb actuator.

---

## Planned Architecture — Three Layers

```
Layer 1  Controller::compute()  →  ControlOutput          (helios_core, vehicle-agnostic)
            │
Layer 2  AckermannOutputAdapter::adapt()  →  AckermannCommand { throttle, steering_rad }
            │  (helios_sim, pluggable via Box<dyn AckermannOutputAdapter> component)
            │  DefaultAckermannAdapter  — open-loop, simple ratios, no feedback
            │  VelocityPidAdapter       — PID on velocity/yaw-rate error (user-defined)
            │  LqrAdapter               — full-state feedback (user-defined)
            │
Layer 3  drive_ackermann_cars (dumb actuator)  →  ExternalForce + ExternalTorque
            force  = throttle * max_force                        (linear, no feedback)
            torque = yaw_rate_kinematic * (max_torque / max_yaw_rate)  (kinematic, no feedback)
            where yaw_rate_kinematic = |v| / wheelbase * tan(steering_rad)
```

### Layer 2 — `AckermannOutputAdapter` trait

```rust
pub trait AckermannOutputAdapter: Send + Sync {
    fn adapt(
        &mut self,
        output: &ControlOutput,
        params: &AckermannParameters,
        actuator: &AckermannActuator,
        lin_vel: &LinearVelocity,
        ang_vel: &AngularVelocity,
        dt: f32,
    ) -> AckermannCommand;
}
```

Stored on the entity as `AckermannAdapterComponent(Box<dyn AckermannOutputAdapter>)`.
Spawned with `DefaultAckermannAdapter` by `process_ackermann_logic`; replaced by
inserting a different component at runtime or at spawn time.

### Layer 3 — Dumb actuator (the change from today)

Replace the two P controllers with a purely linear mapping:

```
force  = throttle * max_force
torque = yaw_rate_kinematic * (max_torque / max_yaw_rate)
```

- **No current-speed feedback in the force** — the car accelerates under constant
  throttle; friction/drag from Avian3D provides passive damping. If speed capping is
  needed it belongs in a Layer 2 PID adapter.
- **No angular-velocity feedback in the torque** — Ackermann geometry provides
  speed-scaled yaw rate demand, which is physically correct for a bicycle model.
  Yaw-rate tracking (comparing desired vs. actual ang_vel.y) belongs in the adapter.

### Why not `torque = (steering / max_steering) * max_torque`?

A steering-normalised torque is speed-independent: the car spins freely at rest and
understeers at speed. Ackermann geometry (`yaw_rate = v/L * tan(δ)`) scales the
torque demand with vehicle speed automatically, which is the correct zero-feedback
baseline for a bicycle-model approximation.

---

## Migration Path

1. Add `AckermannCommand` struct and `AckermannOutputAdapter` trait to `ackermann.rs`.
2. Add `AckermannAdapterComponent` Bevy component; spawn `DefaultAckermannAdapter`
   in `process_ackermann_logic`.
3. Rewrite `drive_ackermann_cars` to call the adapter (Layer 2) then apply the dumb
   linear mapping (Layer 3).
4. Move the current P-controller logic from the actuator into `DefaultAckermannAdapter`
   so existing keyboard/teleop behaviour is preserved.
5. As a follow-up, implement `VelocityPidAdapter` using `SisoPid` from `helios_core`
   for accurate speed tracking when a `VelocityPidController` drives the vehicle.

---

## Architectural Pushback & Alternatives Considered

### Pushback 1 — Is Layer 2 even necessary?

**Argument against:** `ControlOutput` already has a `RawActuators` variant that is a
direct passthrough. Any controller that wants to own its own low-level control loop
can output `RawActuators([throttle, steering_rad])` and skip Layer 2 entirely.
The adapter exists only to give semantic variants (`BodyVelocity`, `Wrench`, etc.)
a sensible default mapping.

**Why we keep it:** `BodyVelocity` is the most useful high-level output — planners
and autonomy stacks think in velocities, not throttle percentages. Without an adapter,
every controller that outputs `BodyVelocity` would have to know about `max_speed`,
`wheelbase`, and Ackermann geometry, which are vehicle-specific concerns. The adapter
isolates that knowledge and makes the controller truly vehicle-agnostic.

---

### Pushback 2 — Two feedback loops are better than one

The current embedded P controllers (speed error → force, yaw-rate error → torque)
are actually reasonable for a simple physics simulation. Removing them means the
car's response depends entirely on Avian3D friction and damping, which requires more
careful tuning of `Friction`, `LinearDamping`, and `AngularDamping` to get stable,
predictable behaviour.

**The tradeoff:**

| | Current (P in actuator) | Planned (dumb actuator + adapter) |
|---|---|---|
| Speed response | Fast, clips to target automatically | Depends on friction/damping tuning |
| Yaw response | Stable, resists overshoot | May oscillate without adapter feedback |
| Testability | Can't unit-test the controller without physics | Adapter is pure Rust, testable standalone |
| Pluggability | None | Full — swap adapter per agent |
| Correctness | Physically approximate | Architecturally correct |

**Recommendation:** Keep the P-controller behaviour in `DefaultAckermannAdapter`
(not in the dumb actuator) so it is preserved for teleop and quick prototyping.
The dumb actuator layer should be truly dumb so that users who do tune Avian3D
damping get a predictable linear baseline.

---

### Pushback 3 — Does `BodyVelocity` belong in Layer 1 or Layer 2?

**The overlap:** `VelocityPidController` (Layer 1, `helios_core`) outputs
`BodyVelocity` with the result of a PID computation. `DefaultAckermannAdapter`
(Layer 2) then re-interprets `BodyVelocity.linear.x` as a desired speed and divides
by `max_speed` to get throttle. This is open-loop in Layer 2 — the PID work done in
Layer 1 is essentially thrown away.

**Correct interpretation:** `BodyVelocity` from `VelocityPidController` is a
*velocity correction*, not a *velocity setpoint*. The PID output magnitude is
already the right throttle signal; dividing by `max_speed` rescales it. This
works as long as the PID gains are tuned relative to `max_speed`. It is not ideal
but is acceptable for a first pass.

**Cleaner alternative:** Use `ControlOutput::Raw(u)` for model-based controllers
where `u[0]` is already a normalised throttle. Reserve `BodyVelocity` strictly for
setpoint commands that the Layer 2 adapter must close a loop around. This gives a
clean semantic split:

```
RawActuators / Raw  →  adapter is a passthrough (Layer 2 is trivial)
BodyVelocity        →  adapter closes a velocity PID loop (Layer 2 is non-trivial)
```

---

### Alternative A — Merge Layer 2 into the high-level `Controller`

Instead of an adapter, require Ackermann controllers to output `RawActuators`
directly, and give each controller an `AckermannKinematics` model at construction
time (via `ControlDynamics`). The controller owns all of its own vehicle-specific
maths.

**Rejected because:** this couples `helios_core` controllers to vehicle geometry.
A `VelocityPidController` becomes an `AckermannVelocityPidController`, and any
vehicle type change requires a new controller implementation. The whole point of
`ControlOutput` is to decouple controllers from actuators.

---

### Alternative B — Make the dumb actuator configurable via TOML gains

Add `force_gain` and `torque_gain` scalars to `AckermannActuatorConfig` and
remove `max_force` / `max_torque`. The dumb actuator becomes:

```
force  = throttle * force_gain
torque = yaw_rate_kinematic * torque_gain
```

This is simpler than the normalised `max_yaw_rate` approach and puts physical
tuning entirely in TOML.

**Tradeoff:** `max_force` and `max_torque` have physical meaning (they bound the
actuator output); raw gains do not. Keeping the normalised form makes it easier to
reason about actuator saturation. However, if Avian3D physics makes the normalisation
feel artificial, switching to raw gains is a simple change.

---

### Alternative C — Skip `AckermannCommand` and keep `(throttle, steering)` as a tuple

`AckermannCommand { throttle, steering_rad }` could just be `(f32, f32)`.

**Rejected because:** a named struct makes the adapter contract self-documenting,
allows adding fields later (e.g., `brake: f32`), and avoids positional confusion
between throttle and steering at call sites.

---

## Open Questions

- Should `AckermannOutputAdapter` receive the physics `Mass` component so that
  `BodyAcceleration` → throttle conversion can use the real mass instead of the
  hardcoded 1500 kg in `DefaultAckermannAdapter`?
- Should `AckermannAdapterComponent` be configurable from TOML (e.g.,
  `[vehicle.adapter] kind = "VelocityPid" kp = 0.5`)? If so, it needs a factory
  entry in `AutonomyRegistry` similar to `ControllerFactory`.
