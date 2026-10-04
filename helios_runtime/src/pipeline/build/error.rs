//! [`PipelineBuildError`], everything [`build`](super::PipelineBuilder::build)
//! can reject, and [`Supplier`], who supplies a channel.

use crate::port::{ChannelKey, InputNeed};

/// Errors produced by [`PipelineBuilder::build`](super::PipelineBuilder::build).
///
/// `build` returns every error found in a stage as a `Vec` rather than
/// stopping at the first. Wiring errors are reported before ordering runs,
/// so a pipeline with both reports the wiring errors first. Disjoint from
/// [`ConfigValidationError`](crate::validation::ConfigValidationError),
/// which checks that TOML strings reference real registry keys — this
/// type checks DAG structure.
#[derive(Debug)]
pub enum PipelineBuildError {
    /// The dependency graph has no valid topological ordering. At least
    /// one set of nodes has every input satisfied only by each others'
    /// outputs. `participants` lists every stranded node whose same-tick
    /// inputs are all covered by the stranded set's collective outputs —
    /// the cycle members plus anything downstream of them.
    Cycle { participants: Vec<String> },
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
}

// default source() returns None which is okay
impl std::error::Error for PipelineBuildError {}

impl std::fmt::Display for PipelineBuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PipelineBuildError::Cycle { participants } => {
                let names = if participants.is_empty() {
                    "<unknown>".to_string()
                } else {
                    participants.join(", ")
                };
                write!(
                    f,
                    "pipeline has a dependency cycle — no valid topological ordering exists; \
                     stranded nodes: [{names}]"
                )
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

/// The word an error message uses for an input's need.
fn need_label(need: InputNeed) -> &'static str {
    match need {
        InputNeed::Required => "required",
        InputNeed::Optional => "optional",
    }
}
