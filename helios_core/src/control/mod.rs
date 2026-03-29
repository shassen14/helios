//! Controller abstraction layer. All types are framework-agnostic.
//!
//! Defines the [`Controller`] trait, [`ControlOutput`] enum, and [`TrajectoryPoint`].
//! [`ControlDynamics`] is re-exported from `models::controls` for caller convenience.
//! Bevy/ECS wrappers and actuator dispatch live in `helios_sim`.

pub mod feedforward_pid;
pub mod lqr;
pub mod metrics;
pub mod pid;
pub mod siso_pid;
pub mod steering_pid;

pub use crate::models::controls::ControlDynamics;

use nalgebra::{DVector, Vector3};

use crate::{
    frames::FrameAwareState,
    types::{TfProvider, TrajectoryPoint},
};

// =========================================================================
// == Core Data Types ==
// =========================================================================

/// Context passed to every `Controller::compute()` call.
pub struct ControlContext<'a> {
    pub tf: Option<&'a dyn TfProvider>,
    pub reference: Option<&'a TrajectoryPoint>,
}

/// The typed output of any `Controller`. All vectors are in the **body FLU frame**, SI units.
///
/// Actuator systems in `helios_sim` are responsible for converting these to
/// Bevy-frame forces via `transforms.rs`.
pub enum ControlOutput {
    /// Desired body-frame velocity (m/s forward/lateral/vertical, rad/s roll/pitch/yaw).
    BodyVelocity {
        linear: Vector3<f64>,
        angular: Vector3<f64>,
    },

    /// Desired body-frame acceleration (m/s², rad/s²).
    /// Actuator multiplies by mass/inertia to produce forces/torques.
    BodyAcceleration {
        linear: Vector3<f64>,
        angular: Vector3<f64>,
    },

    /// Direct force (N) + torque (N·m) in body FLU frame.
    /// For computed-torque / wrench control.
    Wrench {
        force: Vector3<f64>,
        torque: Vector3<f64>,
    },

    /// Raw control vector in the dynamics model's control space.
    /// Escape hatch for model-based controllers (LQR, MPC, feedforward).
    /// Adapter in the helios_sim vehicle plugin converts this to forces.
    Raw(DVector<f64>),

    /// Explicit actuator signals for teleop / keyboard override.
    /// Convention per vehicle type:
    ///   Ackermann:  [throttle -1..1, steering_angle_rad]
    ///   Quadcopter: [r0, r1, r2, r3] rotor speeds (rad/s)
    RawActuators(Vec<f64>),
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
    /// Compute a control output for the current state and time step.
    fn compute(&mut self, state: &FrameAwareState, dt: f64, ctx: &ControlContext) -> ControlOutput;

    /// Reset all internal integrators, accumulators, and filters to zero.
    fn reset(&mut self);
}
