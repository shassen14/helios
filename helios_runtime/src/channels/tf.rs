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
    port::{ChannelKind, InternalChannel},
    ChannelKey,
};

use helios_core::frames::transforms::tf::stamped::{FrameEdge, StampedTransform};

use std::any::TypeId;

const TF_EDGE_PREFIX: &str = "tf/";

pub fn tf_edge(edge: &FrameEdge) -> InternalChannel {
    InternalChannel::named::<StampedTransform>(format!("{}{}", TF_EDGE_PREFIX, edge))
}

pub fn is_tf_edge(key: &ChannelKey) -> bool {
    key.kind() == ChannelKind::Internal
        && key.type_id() == TypeId::of::<StampedTransform>()
        && key.instance().starts_with(TF_EDGE_PREFIX)
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::data::AgentId;
    use helios_core::frames::id::FrameId;

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
        let key: ChannelKey = InternalChannel::named::<NotATransform>("tf/bot/base_link->bot/odom").into();
        assert!(!is_tf_edge(&key));
    }
}
