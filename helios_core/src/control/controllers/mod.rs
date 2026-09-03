//! Controllers — the control law: map a reference (and, for feedback, a measured
//! state) to a command.
//!
//! Organized by **function**, not by agent — a control law is the same law on any
//! morphology; the platform enters only as the plant model it is synthesized
//! against (data, not a distinct type). [`feedback`] holds error/state-driven laws
//! (PID, and later the model-based `optimal` family: LQR, H₂, H∞, MPC, iLQR, state
//! feedback); [`feedforward`] holds inverse-plant maps, which are inherently
//! plant-specific and so name their plant. [`direct_twist`] is the passthrough leaf.

pub mod direct_twist;
pub mod feedback;
pub mod feedforward;

use crate::control::reference::ControlReference;
use crate::frames::FrameAwareState;

pub struct ControlInputs<R: ControlReference> {
    pub state: FrameAwareState,
    pub reference: Option<R>,
}

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
