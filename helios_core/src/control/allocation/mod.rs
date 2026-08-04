//! Allocation — the control pipeline's terminal stage.
//!
//! An [`Allocator`] maps a vehicle-level command (a body twist, wrench, or
//! whatever a family speaks) onto an [`ActuatorCommand`]: one native setpoint per
//! physical actuator. It is the last brain-side node; everything downstream is
//! the host relaying setpoints to hardware or the sim. Each vehicle family
//! supplies its own implementation ([`ackermann`] for a steered car); the trait
//! is only the shared shape.

pub mod ackermann;

use crate::control::actuators::ActuatorCommand;

/// Maps a vehicle-level command onto per-actuator setpoints.
///
/// - `In` — the incoming command this allocator consumes, e.g. a
///   [`BodyTwist`](crate::control::commands::BodyTwist).
/// - `Inputs` — extra per-tick feedback an allocator needs; `()` for the stateless
///   feedforward maps most kinematic allocators are.
///
/// `allocate` takes `&mut self` so a stateful allocator (slew limiting, an
/// integrator) can carry state between ticks; a pure feedforward map ignores it.
/// The returned command is *raw* — clamping to each actuator's limit, the sign
/// convention, and the fail-safe belong to the host via the
/// [`ActuationModel`](crate::control::actuation_model::ActuationModel), never here.
pub trait Allocator {
    type In;
    type Inputs;

    fn allocate(&mut self, command: &Self::In, inputs: &Self::Inputs) -> ActuatorCommand;
}
