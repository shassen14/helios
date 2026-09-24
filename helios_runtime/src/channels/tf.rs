//! Transform-edge channel vocabulary: one bus slot per tree edge.
//!
//! A producer that owns a transform edge (an estimator emitting `odom→base_link`,
//! a localizer correcting `map→odom`, a static loader mounting a sensor under
//! `base_link`) dual-publishes it: alongside its rich output it writes a bare
//! [`StampedTransform`] onto the edge's channel here. The runtime `TfService`
//! later drains those channels and folds them into its buffer. This module is
//! the single source of truth for the `(kind, type, name)` tuple each edge
//! occupies, so a producer and the drainer can't drift on a typo.
//!
//! **Every edge is its own channel.** All edges carry the same payload type
//! ([`StampedTransform`]), and the bus keeps one last-known-good slot per
//! [`ChannelKey`] — so if two edges shared a slot the second producer's write
//! would clobber the first. Each edge is therefore disambiguated by an instance
//! string built from its `(child, parent)` identity via [`tf_edge`]. Because the
//! set of live edges is data (which agents, which sensors), there is no single
//! `const` for it and no "read every `StampedTransform`" bus primitive; the
//! service drains an *explicit* list of edge keys, and [`is_tf_edge`] lets the
//! assembler assert that list covers every declared edge.
//!
//! The [`TF_EDGE_PREFIX`] marks these instances (`tf/…`) the way `oracle/` and
//! `health/` mark theirs — visible in the DAG dump, and the discriminator
//! [`is_tf_edge`] keys off. Edges are [`InternalChannel`]s (node-produced), so
//! the value drops straight into the `output_internal` descriptor builder; a
//! hardware `/tf` bridge or a Zenoh peer publishes onto the same channel by
//! projecting its foreign stream through [`tf_edge`].

use crate::{
    port::{ChannelError, ChannelKind, InternalChannel, PortBus},
    ChannelKey, Stamped,
};

use helios_core::spatial::transforms::tf::stamped::{FrameEdge, StampedTransform};

use std::any::TypeId;

const TF_EDGE_PREFIX: &str = "tf/";

pub fn tf_edge(edge: &FrameEdge) -> InternalChannel {
    InternalChannel::named::<StampedTransform>(format!("{}{}", TF_EDGE_PREFIX, edge))
}

/// Dual-publish one transform edge: writes `stamped` onto the [`tf_edge`] slot
/// its own `(child, parent)` names, so the `TfService` drain can fold it into
/// the buffer.
///
/// The edge is derived from the message's own `child`/`parent`, not passed
/// separately — a producer physically cannot route its transform onto the wrong
/// slot. A dropped write here is a genuine wiring bug (the drain list never
/// carried this edge), unlike the oracle channel's *expected* no-consumer miss,
/// so it warns loudly and names the edge rather than going out on `.ok()`.
pub fn publish_edge(bus: &PortBus, stamped: Stamped<StampedTransform>) {
    let edge = FrameEdge {
        child: stamped.value.child.clone(),
        parent: stamped.value.parent.clone(),
    };

    if let Err(e) = bus.write(tf_edge(&edge).into(), stamped) {
        match e {
            // No node drains this edge — its tf-edge channel was never wired
            // into the graph. A silently unfed edge only surfaces far away, as a
            // missing lookup at some consumer, so name the edge here at the
            // source where the fix (add it to the drain list) actually lives.
            ChannelError::UnknownChannel => tracing::warn!(
                edge = %edge,
                "tf edge dropped: no channel wired for this edge; \
                 the TfService drain list is missing it",
            ),
        }
    }
}

pub fn is_tf_edge(key: &ChannelKey) -> bool {
    key.kind() == ChannelKind::Internal
        && key.type_id() == TypeId::of::<StampedTransform>()
        && key.instance().starts_with(TF_EDGE_PREFIX)
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::PortDescriptor;
    use crate::stamped::Health;

    use helios_core::prelude::{AgentId, MonotonicTime};
    use helios_core::spatial::id::FrameId;
    use helios_core::spatial::transforms::{Convention, ErasedTransform};

    use nalgebra::Isometry3;

    /// A payload type that is not [`StampedTransform`], for checking the type
    /// clause of [`is_tf_edge`] in isolation.
    struct NotATransform;

    /// `child→parent`, the two spine edges a single agent's tree carries.
    fn base_to_odom() -> FrameEdge {
        let agent = AgentId::new("bot");
        FrameEdge {
            child: FrameId::base_link(agent.clone()),
            parent: FrameId::odom(agent),
        }
    }

    fn odom_to_map() -> FrameEdge {
        let agent = AgentId::new("bot");
        FrameEdge {
            child: FrameId::odom(agent.clone()),
            parent: FrameId::map(agent),
        }
    }

    #[test]
    fn same_edge_yields_equal_keys() {
        // The single-source guarantee: a producer and the drainer that both name
        // the same edge land on the same slot.
        assert_eq!(tf_edge(&base_to_odom()), tf_edge(&base_to_odom()));
    }

    #[test]
    fn distinct_edges_yield_distinct_keys() {
        // The whole reason edges are per-edge channels: two edges of the same
        // payload type must not collide on one last-known-good slot.
        assert_ne!(tf_edge(&base_to_odom()), tf_edge(&odom_to_map()));
    }

    #[test]
    fn tf_edge_is_internal_kind() {
        let key: ChannelKey = tf_edge(&base_to_odom()).into();
        assert_eq!(key.kind(), ChannelKind::Internal);
    }

    #[test]
    fn tf_edge_binds_the_stamped_transform_type() {
        let key: ChannelKey = tf_edge(&base_to_odom()).into();
        assert_eq!(key.type_id(), TypeId::of::<StampedTransform>());
    }

    #[test]
    fn is_tf_edge_accepts_a_constructed_edge() {
        let key: ChannelKey = tf_edge(&base_to_odom()).into();
        assert!(is_tf_edge(&key));
    }

    #[test]
    fn is_tf_edge_rejects_an_unprefixed_transform_channel() {
        // A `StampedTransform` channel that did not go through `tf_edge` (no
        // `tf/` prefix) is not an edge — this is the clause that keeps the drain
        // list from swallowing unrelated transform slots.
        let key: ChannelKey = InternalChannel::named::<StampedTransform>("some/other").into();
        assert!(!is_tf_edge(&key));
    }

    #[test]
    fn is_tf_edge_rejects_a_prefixed_channel_of_another_type() {
        // The prefix alone is not enough: a `tf/`-named slot carrying a different
        // payload is not a transform edge.
        let key: ChannelKey =
            InternalChannel::named::<NotATransform>("tf/bot/base_link->bot/odom").into();
        assert!(!is_tf_edge(&key));
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

    /// A bus with exactly the given edges' slots allocated as outputs.
    fn bus_wired_for(edges: &[FrameEdge]) -> PortBus {
        let descriptor = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: edges.iter().map(|e| tf_edge(e).into()).collect(),
            rate: None,
        };
        PortBus::new(&[descriptor])
    }

    #[test]
    fn publish_edge_lands_the_transform_on_its_slot() {
        let edge = base_to_odom();
        let bus = bus_wired_for(std::slice::from_ref(&edge));

        publish_edge(&bus, sample_message(&edge));

        let read = bus
            .read::<StampedTransform>(tf_edge(&edge).into())
            .expect("the published edge must be readable on its slot");
        // Routed to the slot its own (child, parent) names.
        assert_eq!(read.value.parent, edge.parent);
        assert_eq!(read.value.child, edge.child);
    }

    #[test]
    fn publish_edge_on_an_unwired_edge_drops_without_panicking() {
        // The bus allocates a *different* edge's slot, so the target edge's
        // channel is unknown. The helper must warn and return (the loud-drop
        // path), never panic, and nothing lands on the target slot.
        let bus = bus_wired_for(&[odom_to_map()]);

        let target = base_to_odom();
        publish_edge(&bus, sample_message(&target));

        assert!(bus
            .read::<StampedTransform>(tf_edge(&target).into())
            .is_none());
    }
}
