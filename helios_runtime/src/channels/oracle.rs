//! The body's truth channels: the true transforms and velocities of its
//! frames, and what it touches, published by a truth source the body
//! carries, such as the simulator or a motion-capture rig.
//!
//! Each channel is keyed by the frame it describes, as `tf` is: a truth
//! source declares the frames it publishes, one instance each, and a reader
//! takes the instance of the frame it wants. Each value names its own frames,
//! so the channel name says nothing about frames and a second truth frame is
//! a second instance, not a second channel.
//!
//! All are oracle channels: only mock nodes may read them, and the pipeline
//! build refuses an algorithm node that does.
//!
//! Functions (not consts) are required because a channel carries a
//! [`std::any::TypeId`], which is not `const`-constructible.

use crate::port::{ChannelKey, ChannelKind, Keyed};

use helios_core::interchange::motion::StampedTwist;
use helios_core::interchange::perception::contact::Contacts;
use helios_core::spatial::conventions::{Flu, Frame};
use helios_core::spatial::transforms::tf::stamped::StampedTransform;
use helios_core::spatial::FrameId;

const TRUTH_TF: &str = "truth.tf";
const TRUTH_TWIST: &str = "truth.twist";
const TRUTH_CONTACTS: &str = "truth.contacts";

/// A velocity belongs to the instance of the frame that moves. It is
/// declared bare: the frames it is measured against and expressed in are
/// named by each value.
impl<F: Frame> Keyed for StampedTwist<F> {
    type Instance = FrameId;
    type Declared = FrameId;

    fn instance(&self) -> &FrameId {
        &self.moving
    }

    fn declared_frame(&self) -> Option<&FrameId> {
        None
    }
}

/// What a body touches belongs to the instance of the frame that touched.
impl Keyed for Contacts {
    type Instance = FrameId;
    type Declared = FrameId;

    fn instance(&self) -> &FrameId {
        &self.body
    }

    fn declared_frame(&self) -> Option<&FrameId> {
        None
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

/// What the body's frames touched, one instance per frame that can touch:
/// each value lists every other side its frame touched during the step it
/// is stamped at the end of, and is written every step, empty or not, so a
/// reader sees each touch begin and end.
///
/// A simulated body publishes `base_link`.
pub fn truth_contacts() -> ChannelKey {
    ChannelKey::keyed::<Contacts>(ChannelKind::Oracle, TRUTH_CONTACTS)
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::InstanceKey;

    use helios_core::prelude::{AgentId, MonotonicTime};
    use helios_core::spatial::quantities::FreeVector;

    #[test]
    fn truth_channels_are_keyed_oracle_channels() {
        let keys = [truth_tf(), truth_twist(), truth_contacts()];
        for key in &keys {
            assert_eq!(key.kind(), ChannelKind::Oracle);
            assert!(key.is_keyed(), "{key}");
        }
        assert_ne!(keys[0], keys[1]);
        assert_ne!(keys[1], keys[2]);
        assert_ne!(keys[0], keys[2]);
    }

    #[test]
    fn contacts_are_keyed_by_the_body_that_touched() {
        let contacts = Contacts::new(
            FrameId::base_link(AgentId::new("car")),
            MonotonicTime(0.0),
            [],
        );
        assert_eq!(
            InstanceKey::of::<Contacts>(contacts.instance()).as_str(),
            "agent.car.base_link"
        );
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
