//! Which subjects of one pipeline record what is emitted or written there.
//!
//! A watch set names [`Subject`]s, without the agent: a pipeline records
//! only its own agent's values, so the agent adds nothing to a set it is
//! handed. A host builds one from the pipeline's catalog of observables and
//! hands it to the pipeline between ticks.

use crate::{
    observe::key::Subject,
    port::{ChannelKey, InstanceKey},
};

use std::{collections::BTreeSet, error::Error, fmt::Display};

/// The subjects a pipeline records, replaced whole each time it is set.
///
/// The default is an empty set of subjects, which watches nothing.
#[derive(Debug, Clone, PartialEq)]
pub enum WatchSet {
    /// Every emission of every node, including nondeterministic ones such as
    /// each node's run duration, and every channel instance a node writes.
    Everything,
    /// Exactly these subjects. Held in order, so the same subject named twice
    /// is kept once.
    Subjects(BTreeSet<Subject>),
}

impl Default for WatchSet {
    fn default() -> Self {
        Self::Subjects(BTreeSet::default())
    }
}

/// Why one entry of a watch set was refused.
///
/// Non-exhaustive, so later ways of naming subjects can add their own
/// refusals.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum WatchError {
    /// No node in the pipeline has this name.
    UnknownNode { node: String, leaf: String },
    /// The node exists but declares no watchable leaf with this name.
    UndeclaredLeaf { node: String, leaf: String },
    /// No node writes this channel, or this instance of it. A channel only
    /// the body or the outside world writes is not recorded yet.
    UnrecordedChannel {
        channel: ChannelKey,
        instance: Option<InstanceKey>,
    },
}

impl Display for WatchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownNode { node, leaf } => {
                write!(f, "no node named \"{node}\" (watching leaf \"{leaf}\")")
            }
            Self::UndeclaredLeaf { node, leaf } => {
                write!(f, "node \"{node}\" declares no watchable leaf \"{leaf}\"")
            }
            Self::UnrecordedChannel {
                channel,
                instance: Some(instance),
            } => write!(
                f,
                "no node writes instance \"{instance}\" of {channel}, so nothing records it"
            ),
            Self::UnrecordedChannel {
                channel,
                instance: None,
            } => write!(f, "no node writes {channel}, so nothing records it"),
        }
    }
}

impl Error for WatchError {}
