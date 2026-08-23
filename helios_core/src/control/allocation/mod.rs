//! Allocation — the control pipeline's terminal stage.
//!
//! An [`Allocator`] maps a vehicle-level command (a body twist, wrench, or
//! whatever a family speaks) onto an [`ActuatorCommand`]: one native setpoint per
//! physical actuator. It is the last brain-side node; everything downstream is
//! the host relaying setpoints to hardware or the sim. Each vehicle family
//! supplies its own implementation ([`ackermann`] for a steered car); the trait
//! is only the shared shape.

pub mod wheeled;

use crate::control::actuators::ActuatorCommand;

/// Maps a vehicle-level command onto per-actuator setpoints.
///
/// Three sources feed an allocation, and the type parameters name two of them:
///
/// - `self` — what is *intrinsic* to the allocator: fixed geometry (wheelbase,
///   wheel radius, the actuator ids) and, for a stateful allocator, its memory of
///   its own past outputs.
/// - `In` — the command being converted, i.e. *what the vehicle should do*, e.g. a
///   [`BodyTwist`](crate::control::commands::BodyTwist).
/// - `Inputs` — the current measured/estimated vehicle state the conversion
///   depends on but the allocator neither owns nor produces, read fresh each tick.
///
/// The test for `Inputs`: does the math read a quantity that changes tick-to-tick,
/// comes from a sensor or estimator, and is *not* the command? Fixed → `self`; the
/// setpoint → `In`; the allocator's own history → `self`; current world state →
/// `Inputs`.
///
/// A pure feedforward inverse — the Ackermann bicycle model, which inverts
/// idealized kinematics from the command and fixed geometry alone — never reads
/// vehicle state, so its `Inputs` is `()`. Allocators that must *react* to
/// conditions carry a non-`()` `Inputs`: a traction-limited drive needs current
/// wheel speeds to bound slip; an over-actuated vehicle (more actuators than
/// degrees of freedom) resolves the redundancy against current effector state.
///
/// `allocate` takes `&mut self` so a stateful allocator (slew limiting against its
/// last output, an integrator) can carry state between ticks; a pure feedforward
/// map ignores it. The returned command is *raw* — clamping to each actuator's
/// limit, the sign convention, and the fail-safe belong to the host via the
/// [`ActuationModel`](crate::control::actuation_model::ActuationModel), never here.
pub trait Allocator: Send + Sync {
    type In;
    type Inputs;

    fn allocate(&mut self, command: &Self::In, inputs: &Self::Inputs) -> ActuatorCommand;
}
