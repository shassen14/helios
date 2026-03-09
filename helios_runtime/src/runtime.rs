// helios_runtime/src/runtime.rs
//
// AgentRuntime trait — the only interface the pipeline uses to query world state.
// SimRuntime in helios_sim implements this for simulation.
// On real hardware, a HardwareRuntime would implement it using sensor calibration data.

use helios_core::types::{FrameHandle, MonotonicTime, TfProvider};
use nalgebra::Isometry3;

/// The only external interface the AutonomyPipeline uses to query world state.
/// FrameHandle IDs are assigned from Bevy Entity bits in sim,
/// or from a sensor config registry on hardware.
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
