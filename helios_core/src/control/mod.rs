//! Controller abstraction layer. All types are framework-agnostic.
//!
//! Defines the [`Controller`] trait and its inputs. A controller's output is a
//! concrete typed command from [`commands`] (e.g. [`BodyTwist`](commands::BodyTwist),
//! [`BodyWrench`](commands::BodyWrench)) chosen via the `Out` associated type —
//! there is no output enum. [`ControlDynamics`] is re-exported from
//! `models::controls` for caller convenience. Bevy/ECS wrappers and actuator
//! dispatch live in `helios_sim`.

pub mod actuation_model;
pub mod actuators;
pub mod allocation;
pub mod commands;
pub mod direct_twist;
pub mod dynamics;
pub mod siso_pid;

pub use crate::control::dynamics::ControlDynamics;

use crate::{data::messages::TrajectoryPoint, frames::FrameAwareState};

// =========================================================================
// == Core Data Types ==
// =========================================================================

pub struct ControlInputs {
    pub state: FrameAwareState,
    pub reference: Option<TrajectoryPoint>,
}

// =========================================================================
// == Controller Trait ==
// =========================================================================

/// A stateful, mutable controller that maps state estimates to control outputs.
///
/// Implementations include PID, LQR, feedforward-PID, MPC, and RL policies.
/// The trait is intentionally minimal so that any policy (learned or analytical)
/// can implement it by storing whatever internal state it needs.
pub trait Controller: Send + Sync {
    type Inputs;
    type Out;
    /// Compute a control output for the current state and time step.
    fn compute(&mut self, dt: f64, inputs: &Self::Inputs) -> Self::Out;

    /// Reset all internal integrators, accumulators, and filters to zero.
    fn reset(&mut self);
}
