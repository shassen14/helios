// SimRuntime: implements AgentRuntime over a TfTree snapshot and elapsed time.
// Created fresh each tick by the systems that drive the AutonomyPipeline.

use helios_core::data::primitives::MonotonicTime;
use helios_core::frames::transforms::ErasedTransform;
use helios_core::frames::FrameId;
use helios_runtime::runtime::AgentRuntime;

use crate::core::transforms::TfTree;

/// Wraps a TfTree reference and the current elapsed simulation time.
pub struct SimRuntime<'a> {
    pub tf: &'a TfTree,
    pub elapsed_secs: f64,
}

impl AgentRuntime for SimRuntime<'_> {
    fn get_transform(
        &self,
        from: FrameId,
        to: FrameId,
        at: MonotonicTime,
    ) -> Option<ErasedTransform> {
        debug_assert!(at.0 <= self.elapsed_secs);

        self.tf.erased(from, to)
    }

    fn now(&self) -> MonotonicTime {
        MonotonicTime(self.elapsed_secs)
    }
}
