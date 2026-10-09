//! [`PipelineBuildError`], everything [`build`](super::PipelineBuilder::build)
//! can reject, [`Supplier`], who supplies a channel, and [`CycleEdge`], one
//! read inside a dependency loop.

use crate::{
    pipeline::autonomy_pipeline::PIPELINE_LEAF_GROUP,
    port::{ChannelKey, InputNeed},
};

/// Errors produced by [`PipelineBuilder::build`](super::PipelineBuilder::build).
///
/// `build` returns every error found in a stage as a `Vec` rather than
/// stopping at the first. Wiring errors are reported before ordering runs,
/// so a pipeline with both reports the wiring errors first. A node's own
/// config is checked earlier, by its kind's factory; this type checks DAG
/// structure.
#[derive(Debug)]
pub enum PipelineBuildError {
    /// A set of nodes wait on each other's outputs in the same tick, so no
    /// order runs each after its producers. One error is raised per loop.
    ///
    /// `participants` names the nodes in the loop, sorted. A node that is
    /// only downstream of the loop is not listed, though it cannot be placed
    /// either. `edges` lists every same-tick read between participants,
    /// sorted by producer then consumer; a node reading its own output is a
    /// loop of one with a single edge.
    Cycle {
        participants: Vec<String>,
        edges: Vec<CycleEdge>,
    },
    /// A node reads a sensor or internal channel that has no supplier: no
    /// node writes it, the body does not publish it, and it is not declared
    /// as an outside input. Raised for optional inputs too, since optional
    /// means the node runs without a value, not that the wiring is unchecked.
    UnsatisfiedInput {
        node_name: String,
        channel: ChannelKey,
        need: InputNeed,
    },
    /// A node reads an oracle or health channel that the body does not
    /// publish. Only a body supplies these, so the error names the body.
    UnsatisfiedBodyCapabilities {
        node_name: String,
        channel_key: ChannelKey,
        body: String,
        need: InputNeed,
    },
    /// Two suppliers provide the same channel: two nodes, or a node and the
    /// body, or a node and an outside input. Each channel has exactly one
    /// supplier, so every read has one writer.
    MultipleSuppliers {
        channel: ChannelKey,
        first: Supplier,
        second: Supplier,
    },
    /// Two or more nodes share a name. Errors, logs and node order within a
    /// level all identify a node by its name, so names must be unique.
    DuplicateNodeName { name: String },
    /// A node declares a watchable leaf that isn't a well-formed path: it is
    /// empty, has an empty part between dots (`.nis`, `nis.`, `aiding..nis`),
    /// or contains `/`. Paths split on `.`, so such a leaf would mis-split in
    /// a glob or a tree-shaped sink.
    MalformedObservable { node_name: String, leaf: String },
    /// A node declares a watchable leaf in the pipeline's own group
    /// ([`PIPELINE_LEAF_GROUP`]), which holds the leaves the pipeline adds to
    /// every node. Refused even when the pipeline doesn't add that leaf yet,
    /// so adding one later never breaks a node.
    ReservedObservable { node_name: String, leaf: String },
    /// A node declares the same watchable leaf more than once, so a
    /// watcher's path would point at two values. Reported once per leaf per
    /// node.
    DuplicateObservable { node_name: String, leaf: String },
    /// A node declares a leaf that has the same path as one of its own output
    /// channels: under `agent.<agent>.<node>`, `leaf` and the channel's path
    /// segment are the same string, so a watcher or an assertion target
    /// would name two things. Only an exact match is refused; a shared
    /// dotted prefix (`estimate` beside `estimate.cov`) is a group.
    ObservableCollidesWithOutput {
        node_name: String,
        leaf: String,
        channel: ChannelKey,
    },
    /// The level sort stopped with nodes left but no loop was found among
    /// them. Wiring and ordering together rule this out, so it means a bug in
    /// the build, not a mistake in the pipeline's configuration. `nodes`
    /// names the nodes left, in no particular order.
    StuckWithoutCycle { nodes: Vec<String> },
}

// default source() returns None which is okay
impl std::error::Error for PipelineBuildError {}

impl std::fmt::Display for PipelineBuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PipelineBuildError::Cycle {
                participants,
                edges,
            } => {
                write!(
                    f,
                    "nodes [{}] wait on each other's outputs in the same tick, so none can \
                     run first:",
                    participants.join(", ")
                )?;
                for edge in edges {
                    write!(f, " {edge};")?;
                }
                Ok(())
            }
            PipelineBuildError::UnsatisfiedInput {
                node_name,
                channel,
                need,
            } => {
                let need = need_label(*need);
                write!(
                    f,
                    "node \"{node_name}\" reads channel {channel} ({need}) but nothing supplies it: \
                     no node writes it, the body does not publish it, and it is not declared as an \
                     outside input"
                )
            }
            PipelineBuildError::UnsatisfiedBodyCapabilities {
                node_name,
                channel_key,
                body,
                need,
            } => {
                let need = need_label(*need);
                write!(
                    f,
                    "node \"{node_name}\" reads channel {channel_key} ({need}); body \"{body}\" \
                     does not publish it"
                )
            }
            PipelineBuildError::MultipleSuppliers {
                channel,
                first,
                second,
            } => {
                write!(
                    f,
                    "channel {channel} is supplied by both {first} and {second} — each channel \
                     has exactly one supplier"
                )
            }
            PipelineBuildError::DuplicateNodeName { name } => {
                write!(
                    f,
                    "more than one node is named \"{name}\" — node names must be unique"
                )
            }
            PipelineBuildError::MalformedObservable { node_name, leaf } => {
                write!(
                    f,
                    "node \"{node_name}\" declares watchable leaf \"{leaf}\", which is not a \
                     path — a leaf is one or more non-empty parts joined by '.', with no '/'"
                )
            }
            PipelineBuildError::ReservedObservable { node_name, leaf } => {
                write!(
                    f,
                    "node \"{node_name}\" declares watchable leaf \"{leaf}\" in the \
                     \"{PIPELINE_LEAF_GROUP}\" group, which the pipeline owns and adds to every \
                     node — use another name"
                )
            }
            PipelineBuildError::DuplicateObservable { node_name, leaf } => {
                write!(
                    f,
                    "node \"{node_name}\" declares watchable leaf \"{leaf}\" more than once — \
                     each leaf names one value"
                )
            }
            PipelineBuildError::ObservableCollidesWithOutput {
                node_name,
                leaf,
                channel,
            } => {
                write!(
                    f,
                    "node \"{node_name}\" declares watchable leaf \"{leaf}\", which is also \
                     the path of its output channel {channel} — rename the leaf or the channel"
                )
            }
            PipelineBuildError::StuckWithoutCycle { nodes } => {
                write!(
                    f,
                    "internal error: ordering stopped with nodes left but found no cycle \
                     among them: [{}]",
                    nodes.join(", ")
                )
            }
        }
    }
}

/// Who supplies a channel to the graph, as named in a
/// [`PipelineBuildError::MultipleSuppliers`] error.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Supplier {
    /// A node writes it; carries the node's name.
    Node(String),
    /// The body publishes it; carries the body's name.
    Body(String),
    /// It is declared as sent from outside the robot.
    OutsideInput,
}

impl std::fmt::Display for Supplier {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Supplier::Node(name) => write!(f, "node \"{name}\""),
            Supplier::Body(name) => write!(f, "body \"{name}\""),
            Supplier::OutsideInput => write!(f, "an outside input"),
        }
    }
}

/// One same-tick read inside a dependency loop, as listed in a
/// [`PipelineBuildError::Cycle`] error: `consumer` reads `channel`, which
/// `producer` writes, so `consumer` must run after `producer`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CycleEdge {
    /// The node that writes `channel`.
    pub producer: String,
    /// The node that reads `channel` in the same tick.
    pub consumer: String,
    /// The channel read.
    pub channel: ChannelKey,
}

impl std::fmt::Display for CycleEdge {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "\"{}\" reads {} from \"{}\"",
            self.consumer, self.channel, self.producer
        )
    }
}

/// The word an error message uses for an input's need.
fn need_label(need: InputNeed) -> &'static str {
    match need {
        InputNeed::Required => "required",
        InputNeed::Optional => "optional",
    }
}
