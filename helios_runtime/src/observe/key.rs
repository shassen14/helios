//! What a recorded value is a value of: its [`RecordedKey`].
//!
//! The key is the value's identity; a watcher path is only its spelling. A
//! key names the agent and the [`Subject`], and says nothing about who wrote
//! the value, so swapping a writer (one estimator for another) keeps the
//! keys of what it wrote.

use crate::{
    observe::path::{channel_path, node_path},
    port::{ChannelKey, InstanceKey},
};

use helios_core::prelude::AgentId;

use std::sync::Arc;

/// The identity of one recorded series: the agent, and what on that agent
/// the series records.
///
/// Every field is shared, so a key is cloned onto each recorded value with
/// reference-count bumps and no allocation.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct RecordedKey {
    pub agent: AgentId,
    pub subject: Subject,
}

impl RecordedKey {
    /// The watcher path that spells this key. Two instances of one keyed
    /// channel spell the same path, since the path grammar has no place for
    /// an instance: a path names every instance of its channel.
    pub fn path(&self) -> String {
        self.subject.path(&self.agent)
    }
}

/// What on one agent a series records.
///
/// Ordered by kind, then by name: emissions by node and leaf, so a node's
/// leaves sit together, and channels by [`ChannelKey`]'s order, then
/// instance. Both orders are the same in every build.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Subject {
    /// A value a node emitted about itself, under a leaf it declared or one
    /// the pipeline adds to every node.
    Emission {
        node: Arc<str>,
        /// The leaf, e.g. `aiding.gps.nis`.
        leaf: Arc<str>,
    },
    /// The values written to one channel, or to one instance of a keyed
    /// channel, whoever wrote them.
    Channel {
        channel: ChannelKey,
        /// The instance, on a keyed channel; `None` on any other.
        instance: Option<InstanceKey>,
    },
}

impl Subject {
    /// An emission of `node` under `leaf`.
    pub fn emission(node: impl Into<Arc<str>>, leaf: impl Into<Arc<str>>) -> Self {
        Self::Emission {
            node: node.into(),
            leaf: leaf.into(),
        }
    }

    /// The values on `channel`, at `instance` when it is keyed.
    pub fn channel(channel: ChannelKey, instance: Option<InstanceKey>) -> Self {
        Self::Channel { channel, instance }
    }

    /// The watcher path that spells this subject on `agent`:
    /// `agent.<agent>.node.<node>.<leaf>` or
    /// `agent.<agent>.channel.<channel>`.
    pub fn path(&self, agent: &AgentId) -> String {
        match self {
            Self::Emission { node, leaf } => node_path(agent.as_str(), node, leaf),
            Self::Channel { channel, .. } => channel_path(agent.as_str(), channel),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::{ChannelKind, Keyed};

    use helios_core::spatial::FrameId;

    const CAR: &str = "car";

    struct Edge(&'static str);

    impl Keyed for Edge {
        type Instance = &'static str;
        type Declared = &'static str;

        fn instance(&self) -> &&'static str {
            &self.0
        }

        fn declared_frame(&self) -> Option<&FrameId> {
            None
        }
    }

    fn tf() -> ChannelKey {
        ChannelKey::keyed::<Edge>(ChannelKind::Internal, "tf")
    }

    fn edge(frame: &'static str) -> Subject {
        Subject::channel(tf(), Some(InstanceKey::of::<Edge>(&frame)))
    }

    #[test]
    fn an_emission_is_spelled_under_its_node() {
        let key = RecordedKey {
            agent: AgentId::new(CAR),
            subject: Subject::emission("estimator", "aiding.gps.nis"),
        };

        assert_eq!(key.path(), "agent.car.node.estimator.aiding.gps.nis");
    }

    #[test]
    fn every_instance_of_a_channel_is_spelled_by_the_channel_alone() {
        let agent = AgentId::new(CAR);

        assert_eq!(edge("base_link").path(&agent), "agent.car.channel.tf");
        assert_eq!(edge("imu").path(&agent), edge("base_link").path(&agent));
        assert_ne!(edge("imu"), edge("base_link"));
    }

    #[test]
    fn instances_of_a_channel_sort_together_by_spelling() {
        let mut subjects = vec![
            edge("wheel"),
            Subject::emission("estimator", "nis"),
            edge("base_link"),
        ];
        subjects.sort();

        assert_eq!(
            subjects,
            vec![
                Subject::emission("estimator", "nis"),
                edge("base_link"),
                edge("wheel"),
            ]
        );
    }
}
