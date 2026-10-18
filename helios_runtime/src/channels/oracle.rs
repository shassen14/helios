//! The body's truth channels: the true transforms and velocities of its
//! frames, published by a truth source the body carries, such as the
//! simulator or a motion-capture rig.
//!
//! Both channels are keyed by the frame they describe, as `tf` is: a truth
//! source declares the frames it publishes, one instance each, and a reader
//! takes the instance of the frame it wants. Each value names its own frames,
//! so the channel name says nothing about frames and a second truth frame is
//! a second instance, not a second channel.
//!
//! Both are oracle channels: only mock nodes may read them, and the pipeline
//! build refuses an algorithm node that does.
//!
//! Functions (not consts) are required because a channel carries a
//! [`std::any::TypeId`], which is not `const`-constructible.

use crate::port::{ChannelKey, ChannelKind, Keyed};

use helios_core::interchange::motion::StampedTwist;
use helios_core::spatial::conventions::{Flu, Frame};
use helios_core::spatial::transforms::tf::stamped::StampedTransform;
use helios_core::spatial::FrameId;

const TRUTH_TF: &str = "truth.tf";
const TRUTH_TWIST: &str = "truth.twist";

/// A velocity belongs to the instance of the frame that moves.
impl<F: Frame> Keyed for StampedTwist<F> {
    type Instance = FrameId;

    fn instance(&self) -> &FrameId {
        &self.moving
    }
}

/// The true transforms of the body's frames, one instance per child frame:
/// each value is the pose of its `child` in its `parent`, as a `tf` edge is.
///
/// A simulated body publishes `D → base_link`, with D the world's datum.
pub fn truth_tf() -> ChannelKey {
    ChannelKey::keyed::<StampedTransform>(ChannelKind::Oracle, TRUTH_TF)
}

/// The true velocities of the body's frames, one instance per moving frame,
/// each expressed in an FLU frame its value names.
///
/// A simulated body publishes `base_link`'s velocity relative to the world's
/// datum, expressed in `base_link`.
pub fn truth_twist() -> ChannelKey {
    ChannelKey::keyed::<StampedTwist<Flu>>(ChannelKind::Oracle, TRUTH_TWIST)
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::InstanceKey;

    use helios_core::prelude::{AgentId, MonotonicTime};
    use helios_core::spatial::quantities::FreeVector;

    #[test]
    fn truth_channels_are_keyed_oracle_channels() {
        for key in [truth_tf(), truth_twist()] {
            assert_eq!(key.kind(), ChannelKind::Oracle);
            assert!(key.is_keyed(), "{key}");
        }
        assert_ne!(truth_tf(), truth_twist());
    }

    #[test]
    fn a_twist_is_keyed_by_its_moving_frame() {
        let agent = AgentId::new("car");
        let twist = StampedTwist::<Flu> {
            moving: FrameId::base_link(agent.clone()),
            relative_to: FrameId::global("datum"),
            expressed_in: FrameId::base_link(agent),
            stamp: MonotonicTime(0.0),
            linear: FreeVector::zeros(),
            angular: FreeVector::zeros(),
        };
        assert_eq!(
            InstanceKey::of::<StampedTwist<Flu>>(twist.instance()).as_str(),
            "agent.car.base_link"
        );
    }
}
