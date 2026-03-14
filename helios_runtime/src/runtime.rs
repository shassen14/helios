// helios_runtime/src/runtime.rs
//
// AgentRuntime trait — the only interface the pipeline uses to query world state.
// SimRuntime in helios_sim implements this for simulation.
// On real hardware, a HardwareRuntime would implement it using sensor calibration data.

//! The host-runtime interface consumed by [`AutonomyPipeline`](crate::pipeline::AutonomyPipeline).
//!
//! [`AgentRuntime`] is the only external interface the pipeline uses to query world state.
//! It abstracts over:
//! - `SimRuntime` in `helios_sim` — backed by Bevy's `TfTree` and elapsed time.
//! - `HardwareRuntime` (future) in `helios_hw` — backed by calibration data and system clock.
//!
//! Neither implementation is visible to `helios_runtime`; the trait is the only coupling point.

use helios_core::types::{FrameHandle, MonotonicTime, TfProvider};
use nalgebra::Isometry3;

/// The only external interface [`AutonomyPipeline`](crate::pipeline::AutonomyPipeline)
/// uses to query world state.
///
/// Implementations must be `Send + Sync`. `FrameHandle` IDs encode Bevy `Entity` bits
/// in simulation and static calibration IDs on hardware — the trait is agnostic to the
/// encoding.
///
/// # Implementing for a New Host
///
/// 1. Implement `get_transform`, `world_pose`, and `now`.
/// 2. Pass a reference to your implementation into pipeline methods that accept
///    `runtime: &dyn AgentRuntime`.
///
/// Do not store world state in `helios_runtime` structs — the runtime is the only
/// sanctioned read path for external world state.
pub trait AgentRuntime: Send + Sync {
    /// Returns the transform from `from` frame to `to` frame in ENU world coordinates.
    fn get_transform(&self, from: FrameHandle, to: FrameHandle) -> Option<Isometry3<f64>>;

    /// Returns the world (ENU) pose of a sensor or agent frame.
    fn world_pose(&self, frame: FrameHandle) -> Option<Isometry3<f64>>;

    /// Current monotonic time in seconds.
    fn now(&self) -> MonotonicTime;
}

/// Adapts `&dyn AgentRuntime` to the `TfProvider` trait expected by `FilterContext`.
/// Zero-cost: both `AgentRuntime` and `TfProvider` use `FrameHandle` directly.
pub(crate) struct TfProviderAdapter<'a>(pub &'a dyn AgentRuntime);

impl TfProvider for TfProviderAdapter<'_> {
    fn get_transform(&self, from: FrameHandle, to: FrameHandle) -> Option<Isometry3<f64>> {
        self.0.get_transform(from, to)
    }

    fn world_pose(&self, frame: FrameHandle) -> Option<Isometry3<f64>> {
        self.0.world_pose(frame)
    }
}
