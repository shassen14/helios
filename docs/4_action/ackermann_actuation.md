# Ackermann Actuation

## Where control ends and physics begins

The autonomy pipeline ends at an **actuator terminal** — a per-actuator
`ActuatorCommand` — and the sim host applies it. No control logic lives in the
host; the host is a transducer from setpoints to physics.

```
ControllerNode (DirectTwist)                 helios_runtime
    │  BodyTwist  @ "command"
    ▼
AllocatorNode<KinematicAckermann>            helios_runtime
    │  drive  → Velocity(vx / wheel_radius)   [wheel angular velocity, rad/s]
    │  steer  → Position(atan(L·wz / vx))     [steer angle, rad]
    │  ActuatorCommand  @ "actuators"   (the terminal)
    ▼
read_actuators() → ActuatorCommandComponent  helios_sim (BrainOutput)
    ▼
drive_ackermann_cars                         helios_sim (Actuation)
    │  resolve() enforces the actuator contract
    │  L0 shim folds the setpoints into one chassis wrench
    ▼
ConstantForce + ConstantTorque → Avian3D
```

## The allocator is not optional

The controller emits a `BodyTwist`; the terminal expects an `ActuatorCommand`.
The **allocator node bridges them**. With no `[allocators]` block in the agent
profile there is no allocator node, nothing writes the terminal, and `resolve`
fills every actuator with its safe state — a car that receives a perfectly good
velocity command and sits still. The symptom of a missing allocator is a dead
vehicle, not an error message.

## `resolve` — the actuator contract

`ActuationModel::resolve` runs before application and always returns a *total*
command: exactly one setpoint per declared actuator, in declaration order. It
clamps each to `|limit|`, applies the sign convention, and substitutes
`safe_state` for any setpoint that is missing, of the wrong command space, or
non-finite. It cannot fail, so whatever reaches physics is always safe to apply
directly.

## L0 — the arcade shim (interim)

The vehicle is a single rigid body, so `drive_ackermann_cars` folds the
per-actuator setpoints into one chassis wrench, interpreting each by its command
space:

| Setpoint | Applied as |
|---|---|
| `Velocity` (drive) | forward force `= v · l0_force_gain` |
| `Position` (steer) | yaw torque `= δ · l0_yaw_gain` |
| `Force` / `Torque` | warned and ignored — no place in the L0 map |

The wrench is built in body FLU (+X forward, +Z yaw-up), rotated to world via
`FluVector` + the entity rotation, and applied as `ConstantForce` /
`ConstantTorque`. Passive damping is left to Avian's `LinearDamping` /
`AngularDamping`.

### Known limitation — open-loop speed

`force = velocity · gain` is **open-loop**: a force sets acceleration, not speed.
Steady-state speed is where the applied force balances drag and Coulomb friction
— an affine, nonlinear relation with a constant offset — so no single gain can
make actual speed track the commanded velocity across the range. In practice the
car crawls at low commands (the constant friction term dominates) and is roughly
right at high commands. This is inherent to open-loop force control of a velocity
setpoint and is **not tunable away**; the L0 gains are deliberately left rough
rather than chasing a target they cannot reach.

## L1 — dynamic actuation (deferred milestone)

Closing the speed loop belongs in the **brain**, not the host. A speed
controller in `helios_runtime`/`helios_core` reads the state estimate's velocity,
tracks the commanded velocity, and emits a `Force`/`Torque` setpoint that the
host applies verbatim. That keeps full collision dynamics *and* accurate speed
while leaving the host a dumb transducer — putting a PID in the host would
reintroduce the very control-in-the-host smell this seam removed. L1 arrives with
the `Torque` allocator as one milestone, because the setpoint type couples to the
vehicle-model fidelity. Until then, L0 is the interim and its open-loop error is
accepted.

## Configuration

The vehicle entity file carries the body and its actuator declarations:

```toml
kind = "Ackermann"
wheelbase = 2.5
wheel_radius = 0.3

[physics]
mass = 1500.0
friction = 0.7
linear_damping = 1.0
angular_damping = 1.5

[actuator]              # L0-shim wrench gains — retired wholesale at L1
l0_force_gain = 1200.0
l0_yaw_gain   = 16400.0

[[actuation.actuators]]
id = "drive"
kind = "Velocity"
limit = 66.7            # rad/s wheel speed (~20 m/s at r = 0.3)
safe_state = { Velocity = 0.0 }
sign = "Normal"

[[actuation.actuators]]
id = "steer"
kind = "Position"
limit = 0.61            # rad (~35°)
safe_state = { Position = 0.0 }
sign = "Normal"
```

The allocator is wired in the agent profile, not the entity file:

```toml
[allocators.wheels]
kind = "KinematicAckermann"
wheelbase = 2.5
wheel_radius = 0.3
drive = "drive"
steer = "steer"
```

> **Duplication debt.** `wheelbase` and `wheel_radius` appear in both the
> allocator config and the vehicle entity — one physical truth in two files. The
> allocator is portable runtime config and cannot read the sim-only entity, so
> the values are duplicated for now. This is a genuine single-source violation to
> resolve once a reference path exists — distinct from *deliberate* divergence
> like a filter's believed noise vs a sensor's actual noise. See the single-source
> note in `docs/config_design/composition_and_overrides.md`.
