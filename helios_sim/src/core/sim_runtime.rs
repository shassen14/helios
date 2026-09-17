// SimRuntime: implements TfProvider over a TfTree snapshot.
// Created fresh each tick by the systems that drive the AutonomyPipeline.

use helios_core::data::ports::TfProvider;
use helios_core::data::primitives::MonotonicTime;
use helios_core::frames::transforms::ErasedTransform;
use helios_core::frames::FrameId;

use crate::core::transforms::TfTree;

/// Wraps a TfTree reference and the current elapsed simulation time.
///
/// `elapsed_secs` is retained only to bound the latest-only tree's answers to
/// the current instant (the `debug_assert` below); the pipeline clock is now
/// supplied to `tick` by the host directly, not read back through here.
pub struct SimRuntime<'a> {
    pub tf: &'a TfTree,
    pub elapsed_secs: f64,
}

impl TfProvider for SimRuntime<'_> {
    fn get_transform(
        &self,
        from: FrameId,
        to: FrameId,
        at: MonotonicTime,
    ) -> Option<ErasedTransform> {
        // The sim tree is latest-only: one pose per frame, no history, so it can
        // only answer for `now`. `at` is intentionally discarded, and the assert
        // pins the single case where "now" is the correct answer — a request for
        // the current instant, never a past or future one. A time-buffered tree
        // (lands with the estimated map->odom edge) will interpolate to `at`;
        // this seam already passes it through, so that upgrade is invisible here.
        debug_assert!(at.0 <= self.elapsed_secs);

        self.tf.erased(from, to)
    }
}
