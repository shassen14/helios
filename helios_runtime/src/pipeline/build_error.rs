use crate::port::ChannelKey;

#[derive(Debug)]
pub enum PipelineBuildError {
    Cycle,
    UnsatisfiedInput {
        node_name: String,
        channel: ChannelKey,
    },
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
            PipelineBuildError::Cycle => write!(
                f,
                "pipeline has a dependency cycle — no valid topological ordering exists"
            ),
            PipelineBuildError::UnsatisfiedInput { node_name, channel } => {
                write!(f, "node \"{node_name}\" requires channel {channel} but no upstream node or signal key produces it")
            }
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
