//! The estimated transform tree's channel: one keyed channel, `tf`, holding
//! the latest [`StampedTransform`] of each edge, keyed by the edge's child
//! frame.
//!
//! A producer that owns an edge of the estimated tree (the estimate relay
//! emitting `odom → base_link`, a localizer correcting `map → odom`)
//! dual-publishes it: alongside its rich output it writes the bare transform
//! onto [`tf`], declaring the child frames it writes. The runtime `TfService`
//! drains the channel's changed instances into its buffer.
//!
//! **The child names the edge.** A strict tree gives each frame one parent at
//! a time, so the child frame identifies an edge, as tf2 keys its buffer; a
//! re-parented frame stays on one instance. The instance is read off the
//! message's own `child`, so a producer cannot route a transform onto another
//! edge's instance, and the pipeline build refuses two producers of one
//! child. Edges are internal channels (node-produced); a hardware `/tf`
//! bridge or a Zenoh peer maps each foreign transform onto its child's
//! instance.

use crate::{
    port::{ChannelKey, ChannelKind, Keyed, PortBus},
    Stamped,
};

use helios_core::spatial::transforms::tf::stamped::{FrameEdge, StampedTransform};
use helios_core::spatial::FrameId;

const TF: &str = "tf";

/// A transform belongs to the instance of its child frame.
impl Keyed for StampedTransform {
    type Instance = FrameId;

    fn instance(&self) -> &FrameId {
        &self.child
    }
}

/// The estimated tree's edges, one instance per child frame.
pub fn tf() -> ChannelKey {
    ChannelKey::keyed::<StampedTransform>(ChannelKind::Internal, TF)
}

/// Dual-publish one transform edge: writes `stamped` onto [`tf`], under the
/// instance its own `child` names, so the `TfService` drain can fold it into
/// the buffer.
///
/// A dropped write here is a wiring bug (the producer did not declare the
/// edge's child, or nothing wired `tf`), unlike the oracle channel's
/// *expected* no-consumer miss, so it warns loudly and names the edge rather
/// than going out on `.ok()`.
pub fn publish_edge(bus: &PortBus, stamped: Stamped<StampedTransform>) {
    let edge = FrameEdge {
        child: stamped.value.child.clone(),
        parent: stamped.value.parent.clone(),
    };

    if let Err(error) = bus.write_keyed(tf(), stamped) {
        // A silently unfed edge only surfaces far away, as a missing lookup
        // at some consumer, so name the edge here at the source.
        tracing::warn!(edge = %edge, error = %error, "tf edge dropped");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::{InstanceKey, PortDescriptorBuilder};
    use crate::stamped::Health;

    use helios_core::prelude::{AgentId, MonotonicTime};
    use helios_core::spatial::transforms::{Convention, ErasedTransform};

    use nalgebra::Isometry3;

    fn agent() -> AgentId {
        AgentId::new("bot")
    }

    /// `child→parent`, the two spine edges a single agent's tree carries.
    fn base_to_odom() -> FrameEdge {
        FrameEdge {
            child: FrameId::base_link(agent()),
            parent: FrameId::odom(agent()),
        }
    }

    fn odom_to_map() -> FrameEdge {
        FrameEdge {
            child: FrameId::odom(agent()),
            parent: FrameId::map(agent()),
        }
    }

    #[test]
    fn tf_is_one_keyed_internal_channel_of_transforms() {
        let key = tf();
        assert!(key.is_keyed());
        assert_eq!(key.kind(), ChannelKind::Internal);
        assert_eq!(key.type_id(), std::any::TypeId::of::<StampedTransform>());
        assert_eq!(key, tf());
    }

    /// A sample edge message: an identity `Flu → Enu` transform on `edge`,
    /// stamped at t=1.
    fn sample_message(edge: &FrameEdge) -> Stamped<StampedTransform> {
        Stamped {
            value: StampedTransform {
                parent: edge.parent.clone(),
                child: edge.child.clone(),
                stamp: MonotonicTime(1.0),
                transform: ErasedTransform::from_parts(
                    Isometry3::identity(),
                    Convention::Flu,
                    Convention::Enu,
                ),
            },
            timestamp: MonotonicTime(1.0),
            health: Health::Ok,
            producer: 0,
        }
    }

    /// A bus whose one producer declares the children of `edges`.
    fn bus_wired_for(edges: &[FrameEdge]) -> PortBus {
        let descriptor = PortDescriptorBuilder::algorithm()
            .output_keyed::<StampedTransform>(tf(), edges.iter().map(|edge| edge.child.clone()))
            .build();
        PortBus::new(&[descriptor])
    }

    fn child_key(edge: &FrameEdge) -> InstanceKey {
        InstanceKey::of::<StampedTransform>(&edge.child)
    }

    #[test]
    fn a_transform_is_keyed_by_its_child_frame_spelled() {
        let message = sample_message(&base_to_odom());
        assert_eq!(
            InstanceKey::of::<StampedTransform>(message.value.instance()).as_str(),
            "agent.bot.base_link"
        );
    }

    #[test]
    fn publish_edge_lands_each_transform_on_its_childs_instance() {
        let bus = bus_wired_for(&[base_to_odom(), odom_to_map()]);

        publish_edge(&bus, sample_message(&base_to_odom()));
        publish_edge(&bus, sample_message(&odom_to_map()));

        for edge in [base_to_odom(), odom_to_map()] {
            let read = bus
                .read_instance::<StampedTransform>(tf(), &child_key(&edge))
                .expect("the published edge must be readable on its instance");
            assert_eq!(read.value.parent, edge.parent);
            assert_eq!(read.value.child, edge.child);
        }
    }

    #[test]
    fn publish_edge_of_an_undeclared_child_drops_without_panicking() {
        // The producer declares only `odom`, so `base_link` has no instance.
        // The helper must warn and return (the loud-drop path), never panic,
        // and nothing lands.
        let bus = bus_wired_for(&[odom_to_map()]);

        publish_edge(&bus, sample_message(&base_to_odom()));

        assert!(bus.read_instances::<StampedTransform>(tf()).is_empty());
    }
}
