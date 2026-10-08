//! Estimate-channel constructors: where an estimator writes its state, and the
//! one estimate the rest of the stack reads.
//!
//! Every estimator writes its [`FrameAwareState`] on a channel named after its
//! node ([`estimator_output`]), so two estimators in one stack never share a
//! slot. The estimate seam forwards the one the stack's `[estimate]` section
//! names onto [`estimate`], which control, planning, mapping and the host read.
//! A reader never names an estimator: swapping the source, or running a second
//! estimator in shadow, changes no reader.

use crate::port::InternalChannel;

use helios_core::spatial::FrameAwareState;

const ROLE_ESTIMATE: &str = "estimate";

/// The agent's estimate: the authoritative estimator's state, forwarded by
/// the estimate seam.
///
/// Singular fixed type (Internal). Read by the controllers, the path
/// follower, the planners, the mapper and [`AutonomyPipeline::read_state`].
///
/// [`AutonomyPipeline::read_state`]: crate::pipeline::AutonomyPipeline::read_state
pub fn estimate() -> InternalChannel {
    InternalChannel::named::<FrameAwareState>(ROLE_ESTIMATE)
}

/// The channel the estimator node `node` writes its state on.
pub fn estimator_output(node: &str) -> InternalChannel {
    InternalChannel::named::<FrameAwareState>(node)
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::ChannelKey;

    #[test]
    fn two_estimators_and_the_estimate_occupy_distinct_slots() {
        let primary: ChannelKey = estimator_output("primary").into();
        let shadow: ChannelKey = estimator_output("shadow").into();
        let estimate: ChannelKey = estimate().into();

        assert_ne!(primary, shadow);
        assert_ne!(primary, estimate);
        assert_ne!(shadow, estimate);
    }
}
