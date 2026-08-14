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

use helios_core::data::ports::TfProvider;
use helios_core::data::primitives::MonotonicTime;
use helios_core::frames::transforms::ErasedTransform;
use helios_core::frames::FrameId;

/// The only external interface [`AutonomyPipeline`](crate::pipeline::AutonomyPipeline)
/// uses to query world state.
///
/// Implementations must be `Send + Sync`. `FrameHandle` IDs encode Bevy `Entity` bits
/// in simulation and static calibration IDs on hardware — the trait is agnostic to the
/// encoding.
///
/// # Implementing for a New Host
///
/// 1. Implement `get_transform` and `now`.
/// 2. Pass a reference to your implementation into pipeline methods that accept
///    `runtime: &dyn AgentRuntime`.
///
/// Do not store world state in `helios_runtime` structs — the runtime is the only
/// sanctioned read path for external world state.
pub trait AgentRuntime: Send + Sync {
    /// Returns the transform from `from` frame to `to` frame as an
    /// [`ErasedTransform`] — a raw isometry tagged with the axis convention of
    /// each end. The host is the source of truth for those conventions (per-frame
    /// registration in sim, calibration on hardware); it does not assume a global
    /// convention such as ENU. The caller crosses the erased transform with
    /// [`ErasedTransform::typed`] to recover a compile-time-checked transform,
    /// which fails loudly if the stamped conventions don't match what it expects.
    fn get_transform(
        &self,
        from: FrameId,
        to: FrameId,
        at: MonotonicTime,
    ) -> Option<ErasedTransform>;

    /// Current monotonic time in seconds.
    fn now(&self) -> MonotonicTime;
}

/// Adapts `&dyn AgentRuntime` to the `TfProvider` trait expected by `FilterContext`.
/// Zero-cost: both `AgentRuntime` and `TfProvider` use `FrameHandle` directly.
pub(crate) struct TfProviderAdapter<'a>(pub &'a dyn AgentRuntime);

impl TfProvider for TfProviderAdapter<'_> {
    fn get_transform(
        &self,
        from: FrameId,
        to: FrameId,
        at: MonotonicTime,
    ) -> Option<ErasedTransform> {
        self.0.get_transform(from, to, at)
    }
}
