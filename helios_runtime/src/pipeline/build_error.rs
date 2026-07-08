use crate::port::ChannelKey;

/// Errors produced by [`PipelineBuilder::build`](super::PipelineBuilder::build).
///
/// `build` collects every detected error and returns them as a `Vec`
/// rather than short-circuiting, so a misconfigured pipeline reports all
/// problems in one pass. Disjoint from
/// [`ConfigValidationError`](crate::validation::ConfigValidationError),
/// which checks that TOML strings reference real registry keys — this
/// type checks DAG structure.
#[derive(Debug)]
pub enum PipelineBuildError {
    /// The dependency graph has no valid topological ordering. At least
    /// one set of nodes has every input satisfied only by each others'
    /// outputs. `participants` lists every stranded node whose required
    /// inputs are all covered by the stranded set's collective outputs —
    /// the cycle members plus anything downstream of them.
    Cycle { participants: Vec<String> },
    /// A node declared a required input that no other node produces and
    /// that is not in the builder's sensor-signal or host-state lists.
    UnsatisfiedInput {
        node_name: String,
        channel: ChannelKey,
    },
    UnsatisfiedBodyCapabilities {
        node_name: String,
        channel_key: ChannelKey,
        body: String,
    },
    /// Two nodes both declared the same channel as one of their outputs.
    /// At most one producer per channel is allowed.
    MultipleProducers {
        channel: ChannelKey,
        first_node: String,
        second_node: String,
    },
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
            PipelineBuildError::UnsatisfiedInput { node_name, channel } => {
                write!(f, "node \"{node_name}\" requires channel {channel} but no upstream node or signal key produces it")
            }
            PipelineBuildError::UnsatisfiedBodyCapabilities {
                node_name,
                channel_key,
                body,
            } => write!(
                f,
                "Node {node_name} requires channel {channel_key}; body {body} does not advertise it."
            ),
            PipelineBuildError::MultipleProducers {
                channel,
                first_node,
                second_node,
            } => {
                write!(f, "channel {channel} is declared as an output by both \"{first_node}\" and \"{second_node}\" — at most one node may produce a channel")
            }
        }
    }
}
