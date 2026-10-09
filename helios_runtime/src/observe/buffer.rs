//! One node's buffer of the values it emits for watchers.
//!
//! Each node owns one buffer, created by the pipeline at build. During a tick
//! the node records into its own buffer only; after the tick the host drains
//! every buffer. Nothing in the brain reads a buffer, so what is watched can
//! never change what the pipeline computes.

use crate::observe::observation::{Observation, ObservedValue};

use helios_core::prelude::MonotonicTime;

use std::sync::{Arc, Mutex};

/// The observations one node emitted since the last drain, and which of its
/// declared leaves are watched.
///
/// Observations are kept in the order the node emitted them. The host must
/// drain after every tick: a buffer with watched leaves that is never drained
/// grows without bound.
///
/// The leaf flags change only through `&mut self`, between ticks, so they need
/// no lock. The list is written through `&self` during a tick, so it sits
/// behind a `Mutex`; only this node writes to it, so the lock is never
/// contended.
pub(crate) struct NodeObservations {
    node_name: Arc<str>,
    leaves: Vec<Leaf>,
    /// True when at least one leaf is watched, so an unwatched node rejects
    /// every emit with one check instead of scanning its leaves.
    any_watched: bool,
    recorded: Mutex<Vec<Observation>>,
}

impl NodeObservations {
    /// A buffer for `node_name` declaring `leaf_names`, none of them watched.
    pub(crate) fn new(
        node_name: impl Into<Arc<str>>,
        leaf_names: impl IntoIterator<Item = impl Into<Arc<str>>>,
    ) -> Self {
        let leaves = leaf_names
            .into_iter()
            .map(|n| Leaf {
                name: n.into(),
                watched: false,
            })
            .collect();

        Self {
            node_name: node_name.into(),
            leaves,
            any_watched: false,
            recorded: Mutex::default(),
        }
    }

    /// The stored name of `leaf_name` if it is declared and watched, else
    /// `None`.
    ///
    /// Returning the stored name lets [`record`](Self::record) build an
    /// observation without allocating the leaf string again.
    pub(crate) fn watched_leaf(&self, leaf_name: &str) -> Option<Arc<str>> {
        if !self.any_watched {
            return None;
        }

        self.leaves
            .iter()
            .find(|leaf| leaf.name.as_ref() == leaf_name && leaf.watched)
            .map(|leaf| leaf.name.clone())
    }

    /// Appends one observation of `leaf`. Callers pass a name returned by
    /// [`watched_leaf`](Self::watched_leaf).
    ///
    /// A poisoned lock drops the sample.
    pub(crate) fn record(&self, leaf: Arc<str>, timestamp: MonotonicTime, value: ObservedValue) {
        let Ok(mut list) = self.recorded.lock() else {
            return;
        };

        list.push(Observation {
            node: self.node_name.clone(),
            leaf,
            timestamp,
            value,
        });
    }

    /// Moves every recorded observation onto the end of `out`, in emit order,
    /// leaving this buffer empty. The buffer keeps its allocation for the next
    /// tick.
    ///
    /// A poisoned lock leaves `out` unchanged.
    pub(crate) fn drain_into(&self, out: &mut Vec<Observation>) {
        let Ok(mut list) = self.recorded.lock() else {
            return;
        };

        out.append(&mut list);
    }

    /// Sets whether `leaf_name` is watched. Returns `false`, changing nothing,
    /// if the node doesn't declare that leaf.
    pub(crate) fn set_watched(&mut self, leaf_name: &str, watched: bool) -> bool {
        let Some(leaf) = self
            .leaves
            .iter_mut()
            .find(|leaf| leaf.name.as_ref() == leaf_name)
        else {
            return false;
        };

        leaf.watched = watched;
        self.any_watched = self.leaves.iter().any(|leaf| leaf.watched);
        true
    }
}

/// One declared leaf and whether it is watched.
struct Leaf {
    name: Arc<str>,
    watched: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    const NODE: &str = "estimator";
    const NIS: &str = "aiding.gps.nis";
    const DROPPED: &str = "aiding.gps.dropped";
    const UNDECLARED: &str = "aiding.gps.undeclared";

    fn buffer() -> NodeObservations {
        NodeObservations::new(NODE, [NIS, DROPPED])
    }

    /// Records `value` on `leaf` the way an emit does: look up, then record.
    fn emit(buffer: &NodeObservations, leaf: &str, t: f64, value: f64) {
        if let Some(leaf) = buffer.watched_leaf(leaf) {
            buffer.record(leaf, MonotonicTime(t), ObservedValue::Scalar(value));
        }
    }

    fn drain(buffer: &NodeObservations) -> Vec<Observation> {
        let mut out = Vec::new();
        buffer.drain_into(&mut out);
        out
    }

    fn observation(node: &str, leaf: &str, t: f64, value: f64) -> Observation {
        Observation {
            node: node.into(),
            leaf: leaf.into(),
            timestamp: MonotonicTime(t),
            value: ObservedValue::Scalar(value),
        }
    }

    #[test]
    fn watched_leaf_records_in_emit_order_and_drain_empties() {
        let mut buffer = buffer();
        assert!(buffer.set_watched(NIS, true));

        emit(&buffer, NIS, 1.0, 0.5);
        emit(&buffer, NIS, 2.0, 1.5);

        assert_eq!(
            drain(&buffer),
            vec![
                observation(NODE, NIS, 1.0, 0.5),
                observation(NODE, NIS, 2.0, 1.5)
            ]
        );
        assert!(drain(&buffer).is_empty());
    }

    #[test]
    fn drain_appends_after_what_out_already_holds() {
        let mut buffer = buffer();
        buffer.set_watched(NIS, true);
        emit(&buffer, NIS, 1.0, 0.5);

        let earlier = observation("other", "x", 0.0, 0.0);
        let mut out = vec![earlier.clone()];
        buffer.drain_into(&mut out);

        assert_eq!(out, vec![earlier, observation(NODE, NIS, 1.0, 0.5)]);
    }

    #[test]
    fn nothing_watched_records_nothing() {
        let buffer = buffer();
        emit(&buffer, NIS, 1.0, 0.5);
        assert!(drain(&buffer).is_empty());
    }

    #[test]
    fn unwatched_leaf_beside_a_watched_one_records_nothing() {
        let mut buffer = buffer();
        buffer.set_watched(NIS, true);

        emit(&buffer, DROPPED, 1.0, 1.0);
        assert!(drain(&buffer).is_empty());
    }

    #[test]
    fn undeclared_leaf_records_nothing() {
        let mut buffer = buffer();
        buffer.set_watched(NIS, true);

        emit(&buffer, UNDECLARED, 1.0, 0.5);
        assert!(drain(&buffer).is_empty());
    }

    #[test]
    fn watching_an_undeclared_leaf_reports_false() {
        let mut buffer = buffer();
        assert!(!buffer.set_watched(UNDECLARED, true));
        assert!(!buffer.any_watched);
    }

    #[test]
    fn unwatching_one_leaf_keeps_the_other_watched() {
        let mut buffer = buffer();
        buffer.set_watched(NIS, true);
        buffer.set_watched(DROPPED, true);
        buffer.set_watched(DROPPED, false);

        emit(&buffer, NIS, 1.0, 0.5);
        emit(&buffer, DROPPED, 1.0, 1.0);

        assert_eq!(drain(&buffer), vec![observation(NODE, NIS, 1.0, 0.5)]);
    }

    #[test]
    fn unwatching_every_leaf_records_nothing() {
        let mut buffer = buffer();
        buffer.set_watched(NIS, true);
        buffer.set_watched(NIS, false);

        assert!(!buffer.any_watched);
        emit(&buffer, NIS, 1.0, 0.5);
        assert!(drain(&buffer).is_empty());
    }
}
