//! One node's buffer of what is recorded during its runs.
//!
//! Each node owns one buffer, created by the pipeline at build. During a tick
//! the node's emissions, and the values it wrote to watched channels, are
//! recorded into its own buffer only; after the tick the host drains every
//! buffer. Nothing in the brain reads a buffer, so what is watched can never
//! change what the pipeline computes.

use crate::observe::{
    key::{RecordedKey, Subject},
    observation::{Observation, ObservedValue},
};

use helios_core::prelude::{AgentId, MonotonicTime};

use std::sync::{Arc, Mutex};

/// The observations recorded during one node's runs since the last drain,
/// and which of its leaves are watched.
///
/// The leaves are the node's emissions only; which of its channels are
/// watched, the pipeline keeps beside each channel. Observations are kept in
/// the order they were recorded. The host must
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
    /// A buffer for `node_name`, a node of `agent`'s pipeline, declaring
    /// `leaf_names`, none of them watched.
    pub(crate) fn new(
        agent: &AgentId,
        node_name: impl Into<Arc<str>>,
        leaf_names: impl IntoIterator<Item = impl Into<Arc<str>>>,
    ) -> Self {
        let node_name: Arc<str> = node_name.into();
        let leaves = leaf_names
            .into_iter()
            .map(|name| {
                let name: Arc<str> = name.into();
                Leaf {
                    key: RecordedKey {
                        agent: agent.clone(),
                        subject: Subject::emission(node_name.clone(), name.clone()),
                    },
                    name,
                    watched: false,
                }
            })
            .collect();

        Self {
            node_name,
            leaves,
            any_watched: false,
            recorded: Mutex::default(),
        }
    }

    /// The key of `leaf_name` if it is declared and watched, else `None`.
    ///
    /// Returning the stored key lets [`record`](Self::record) build an
    /// observation without allocating.
    pub(crate) fn watched_leaf(&self, leaf_name: &str) -> Option<RecordedKey> {
        if !self.any_watched {
            return None;
        }

        self.leaves
            .iter()
            .find(|leaf| leaf.name.as_ref() == leaf_name && leaf.watched)
            .map(|leaf| leaf.key.clone())
    }

    /// Whether the node declares `leaf_name`, watched or not.
    pub(crate) fn declares(&self, leaf_name: &str) -> bool {
        self.leaves
            .iter()
            .any(|leaf| leaf.name.as_ref() == leaf_name)
    }

    /// Panics in debug builds if the node doesn't declare `leaf_name`; does
    /// nothing in release builds.
    ///
    /// An emit of an undeclared leaf can never be watched, so without this a
    /// misspelled leaf would go unnoticed.
    pub(crate) fn assert_declared(&self, leaf_name: &str) {
        debug_assert!(
            self.declares(leaf_name),
            "node \"{}\" emitted undeclared leaf \"{leaf_name}\"",
            self.node_name
        );
    }

    /// Appends one observation under `key`: a key returned by
    /// [`watched_leaf`](Self::watched_leaf), or the key of a watched channel
    /// the node wrote.
    ///
    /// A poisoned lock drops the sample.
    pub(crate) fn record(&self, key: RecordedKey, timestamp: MonotonicTime, value: ObservedValue) {
        let Ok(mut list) = self.recorded.lock() else {
            return;
        };

        list.push(Observation {
            key,
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

    /// Sets whether every declared leaf is watched. A buffer that declares no
    /// leaves stays unwatched.
    pub(crate) fn set_all_watched(&mut self, watched: bool) {
        self.any_watched = watched && !self.leaves.is_empty();

        for leaf in &mut self.leaves {
            leaf.watched = watched;
        }
    }
}

/// One declared leaf, the key it records under, and whether it is watched.
struct Leaf {
    name: Arc<str>,
    key: RecordedKey,
    watched: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    const AGENT: &str = "car";
    const NODE: &str = "estimator";
    const NIS: &str = "aiding.gps.nis";
    const DROPPED: &str = "aiding.gps.dropped";
    const UNDECLARED: &str = "aiding.gps.undeclared";

    fn buffer() -> NodeObservations {
        NodeObservations::new(&AgentId::new(AGENT), NODE, [NIS, DROPPED])
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
            key: RecordedKey {
                agent: AgentId::new(AGENT),
                subject: Subject::emission(node, leaf),
            },
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

    #[test]
    fn declares_answers_whether_watched_or_not() {
        let mut buffer = buffer();
        assert!(buffer.declares(NIS));

        buffer.set_watched(NIS, true);
        assert!(buffer.declares(NIS));
        assert!(!buffer.declares(UNDECLARED));
    }

    #[test]
    fn watching_all_records_every_leaf() {
        let mut buffer = buffer();
        buffer.set_all_watched(true);

        emit(&buffer, NIS, 1.0, 0.5);
        emit(&buffer, DROPPED, 1.0, 1.0);

        assert_eq!(
            drain(&buffer),
            vec![
                observation(NODE, NIS, 1.0, 0.5),
                observation(NODE, DROPPED, 1.0, 1.0)
            ]
        );
    }

    #[test]
    fn unwatching_all_records_nothing_even_after_one_leaf_is_rewatched() {
        let mut buffer = buffer();
        buffer.set_all_watched(true);
        buffer.set_all_watched(false);

        emit(&buffer, NIS, 1.0, 0.5);
        assert!(drain(&buffer).is_empty());

        buffer.set_watched(NIS, true);
        emit(&buffer, NIS, 1.0, 0.5);
        emit(&buffer, DROPPED, 1.0, 1.0);
        assert_eq!(drain(&buffer), vec![observation(NODE, NIS, 1.0, 0.5)]);
    }

    #[test]
    fn buffer_declaring_nothing_stays_unwatched() {
        let mut buffer = NodeObservations::new(&AgentId::new(AGENT), NODE, Vec::<&str>::new());
        buffer.set_all_watched(true);
        assert!(!buffer.any_watched);
    }
}
