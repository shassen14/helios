//! Which declared leaves record what their nodes emit.
//!
//! A watch set names leaves relative to the pipeline, as `(node, leaf)`
//! pairs, because a pipeline doesn't know which agent runs it. A host builds
//! one from the pipeline's catalog of observables and hands it to the
//! pipeline between ticks.

use std::{collections::BTreeSet, error::Error, fmt::Display};

/// The leaves a pipeline records, replaced whole each time it is set.
///
/// The default is an empty set of leaves, which watches nothing.
#[derive(Debug, Clone, PartialEq)]
pub enum WatchSet {
    /// Every leaf of every node, including wall-clock leaves such as each
    /// node's run duration.
    Everything,
    /// Exactly these leaves. Held in order, so the same leaf named twice is
    /// kept once.
    Leaves(BTreeSet<WatchedLeaf>),
}

impl Default for WatchSet {
    fn default() -> Self {
        Self::Leaves(BTreeSet::default())
    }
}

/// One leaf of one node, named relative to the pipeline.
///
/// Ordered by node, then leaf, so a set of them groups each node's leaves
/// together.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct WatchedLeaf {
    /// The node's name.
    pub node: String,
    /// The leaf the node declared, e.g. `aiding.gps.nis`.
    pub leaf: String,
}

impl WatchedLeaf {
    pub fn new(node: impl Into<String>, leaf: impl Into<String>) -> Self {
        Self {
            node: node.into(),
            leaf: leaf.into(),
        }
    }
}

/// Why one entry of a watch set was refused.
///
/// Non-exhaustive, so later ways of naming leaves can add their own refusals.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum WatchError {
    /// No node in the pipeline has this name.
    UnknownNode { node: String, leaf: String },
    /// The node exists but declares no leaf with this name.
    UndeclaredLeaf { node: String, leaf: String },
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
        }
    }
}

impl Error for WatchError {}
