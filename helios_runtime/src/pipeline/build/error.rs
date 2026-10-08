//! [`PipelineBuildError`], everything [`build`](super::PipelineBuilder::build)
//! can reject, [`Supplier`], who supplies a channel, and [`CycleEdge`], one
//! read inside a dependency loop.

use crate::port::{ChannelKey, InputNeed};

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
