// helios_core/src/types.rs

use crate::frames::FrameAwareState;
use nalgebra::{DVector, Isometry3};
use serde::{Deserialize, Serialize};

// --- Core Type Aliases ---
pub type State = DVector<f64>;
pub type Control = DVector<f64>;

/// A lightweight, copyable identifier for a coordinate frame (agent body, sensor, world).
///
/// In simulation, the bits encode a Bevy `Entity` via `from_entity()`/`to_entity()` (behind
/// the `bevy` feature gate). On hardware, bits encode static calibration IDs assigned at startup.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub struct FrameHandle(pub u64);

impl FrameHandle {
    // A convenience method for use in the Bevy adapter crate.
    #[cfg(feature = "bevy")] // This will only compile if the "bevy" feature is enabled
    pub fn from_entity(entity: bevy_ecs::prelude::Entity) -> Self {
        Self(entity.to_bits())
    }

    #[cfg(feature = "bevy")]
    pub fn to_entity(self) -> bevy_ecs::prelude::Entity {
        bevy_ecs::prelude::Entity::from_bits(self.0)
    }
}

/// Monotonically increasing time in seconds (simulation or hardware clock).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Serialize, Deserialize)]
pub struct MonotonicTime(pub f64);

/// Abstraction over any system that can answer transform queries between coordinate frames.
///
/// `helios_sim` implements this as `TfTree` (Bevy resource). `helios_hw` will implement it
/// as a hardware-clock-backed calibration tree. Filters receive `&dyn TfProvider` via
/// `FilterContext` — they never depend on the concrete host type.
pub trait TfProvider {
    fn get_transform(&self, from: FrameHandle, to: FrameHandle) -> Option<Isometry3<f64>>;

    /// Returns the world (ENU) pose of a frame directly from physics.
    /// Use this to bypass the estimator entirely (e.g. ground-truth mapping).
    fn world_pose(&self, frame: FrameHandle) -> Option<Isometry3<f64>>;
}

/// Reference trajectory point for the current time step.
///
/// Provided by the planner; consumed by path follower and controller.
/// `state_dot` is `None` when the planner cannot or does not provide it
/// (e.g. a waypoint planner). Feedforward controllers degrade gracefully
/// to pure feedback and log a diagnostic when `state_dot` is absent.
#[derive(Clone)]
pub struct TrajectoryPoint {
    pub state: FrameAwareState,
    /// x_dot_ref = f(x_ref, u_nominal, t). None = feedforward unavailable.
    pub state_dot: Option<DVector<f64>>,
    pub time: f64,
}
