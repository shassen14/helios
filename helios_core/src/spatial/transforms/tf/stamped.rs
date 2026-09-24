//! The transform *message* and the small supporting types the buffer stores and
//! reports with.
//!
//! [`StampedTransform`] is what a producer hands the buffer: one timestamped edge
//! of the tree. [`FrameEdge`], [`TimeSpan`], and [`EdgeKind`] are the buffer's
//! own vocabulary — an edge identity, a time interval, and the two ways an edge
//! answers time. The buffer itself and its query/error types live in `buffer.rs`;
//! this file is only the data those two vocabularies share.

use crate::{
    prelude::MonotonicTime,
    spatial::{id::FrameId, transforms::ErasedTransform},
};

use nalgebra::Isometry3;
use std::{collections::VecDeque, fmt::Display};

/// One timestamped edge of the transform tree: the pose of `child` in `parent`,
/// as it was at `stamp`.
///
/// The message names its two frames **explicitly** rather than overloading a
/// `from`/`to` polarity, because every producer already knows which side is
/// parent-ward — a localizer *corrects* `map→odom`, a static loader *mounts* a
/// sensor under `base_link` — and an implicit polarity is a rule everyone must
/// remember and no one must get wrong.
///
/// The field layout mirrors tf2's `TransformStamped` (`parent` = `header.frame_id`,
/// `child` = `child_frame_id`, `transform`), so bridging a hardware `/tf` stream
/// is a field rename, not a reinterpretation.
#[derive(Debug, Clone)]
pub struct StampedTransform {
    /// The toward-root frame of the edge.
    pub parent: FrameId,
    /// The leaf-ward frame of the edge.
    pub child: FrameId,
    /// When this pose held. Lookups interpolate between stamps; they never
    /// extrapolate past the newest one.
    pub stamp: MonotonicTime,
    /// The pose of `child` expressed in `parent`. An [`ErasedTransform`] carries
    /// no [`FrameId`] — it is identity-erased by design — so `parent`/`child`
    /// above are the only identity carriers, and this field is the one place the
    /// edge's polarity is pinned. Its two convention tags are a deliberate
    /// redundancy with each frame's established intrinsic; the buffer checks them
    /// agree on ingest.
    pub transform: ErasedTransform,
}

/// The identity of a tree edge: the ordered `(child, parent)` pair, with no pose
/// attached.
///
/// A named type rather than a bare `(FrameId, FrameId)` because the buffer's
/// errors reference "an edge" and a tuple is easy to transpose silently. Fields
/// read child-first, matching the `Display`.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct FrameEdge {
    pub child: FrameId,
    pub parent: FrameId,
}

impl Display for FrameEdge {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // ASCII `->`, not a Unicode arrow: this rendering is embedded verbatim
        // in the bus channel key for the edge (`tf_edge`), so it is grepped in
        // DAG dumps and may be reconstructed by hand at a hardware `/tf` bridge.
        // A plain-ASCII, tf2-style `parent`-ward arrow avoids encoding surprises.
        write!(f, "{}->{}", self.child, self.parent)
    }
}

/// A closed time interval `[oldest, newest]`, the span a dynamic edge currently
/// holds samples over.
///
/// Reported inside the buffer's out-of-range lookup error so a caller can see how
/// far its query missed the available history.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TimeSpan {
    pub oldest: MonotonicTime,
    pub newest: MonotonicTime,
}

/// How a single edge answers a time query — a genuine type distinction, not a
/// rate hint. The two kinds answer time fundamentally differently.
#[derive(Debug, Clone)]
pub enum EdgeKind {
    /// Time-invariant calibration (a sensor mount, an extrinsic). One isometry
    /// that answers every `t`.
    Static(Isometry3<f64>),
    /// A history of samples, **sorted ascending by stamp**. Answers a `t` by
    /// interpolating the two bracketing samples; a single sample answers only its
    /// own exact stamp. The sorted invariant is maintained by ingest and relied
    /// on by the lookup's binary search — the `VecDeque` type does not enforce it.
    Dynamic(VecDeque<(MonotonicTime, Isometry3<f64>)>),
}

impl EdgeKind {
    /// The payload-free discriminant of this edge, for reporting a kind conflict
    /// without shipping the sample history.
    pub fn tag(&self) -> EdgeKindTag {
        match self {
            EdgeKind::Static(_) => EdgeKindTag::Static,
            EdgeKind::Dynamic(_) => EdgeKindTag::Dynamic,
        }
    }
}

/// Which flavour of edge — the discriminant of [`EdgeKind`] without its payload.
///
/// Rides in the ingest error for a kind conflict: a caller learns *which* kind it
/// collided with (e.g. sent a dynamic sample to a static edge) without the error
/// carrying a whole `VecDeque` of samples or leaking the buffer's storage layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeKindTag {
    Static,
    Dynamic,
}

#[derive(Debug, Clone, PartialEq)]
pub struct DynamicEdgeStats {
    pub newest: MonotonicTime,
    pub oldest: MonotonicTime,
    pub sample_count: usize,
}

#[cfg(test)]
mod tests {
    use super::FrameEdge;
    use crate::prelude::AgentId;
    use crate::spatial::id::FrameId;

    #[test]
    fn frame_edge_displays_child_then_parent() {
        // A spine edge base_link→odom renders leaf-ward frame first, matching the
        // stored `(child, parent)` field order.
        let agent = AgentId::new("bot");
        let edge = FrameEdge {
            child: FrameId::base_link(agent.clone()),
            parent: FrameId::odom(agent),
        };
        assert_eq!(edge.to_string(), "bot/base_link->bot/odom");
    }
}
