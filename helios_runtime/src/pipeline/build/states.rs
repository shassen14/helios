//! State schemas: each channel carrying a
//! [`FrameAwareState`](helios_core::spatial::FrameAwareState) with the schema
//! its states are built on and the root they are held in, gathered once the
//! nodes are built. A node gives its own output's; an output forwarding an
//! input takes the input's.

use crate::{pipeline::autonomy_pipeline::ScheduledNode, port::ChannelKey};

use helios_core::estimation::schema::StateSchema;
use helios_core::spatial::FrameId;

use std::sync::Arc;

/// Each state channel with its [`StateDescription`], in level order. Only the
/// builder makes one; a built pipeline exposes it through
/// [`AutonomyPipeline::state_schemas`](crate::AutonomyPipeline::state_schemas).
#[derive(Debug, Clone, Default)]
pub struct StateSchemas {
    states: Vec<(ChannelKey, StateDescription)>,
}

impl StateSchemas {
    /// What the states on `channel` are, or `None` when no writer describes
    /// them.
    pub fn of_channel(&self, channel: &ChannelKey) -> Option<&StateDescription> {
        self.states
            .iter()
            .find(|(key, _)| key == channel)
            .map(|(_, description)| description)
    }

    /// Each described state channel with its description, in level order.
    pub fn iter(&self) -> impl Iterator<Item = (&ChannelKey, &StateDescription)> + '_ {
        self.states
            .iter()
            .map(|(channel, description)| (channel, description))
    }
}

/// What every state a channel carries is: built on `schema`, held in `root`.
/// Fixed for the run, so a reader knows each state's blocks before the first
/// arrives.
#[derive(Debug, Clone)]
pub struct StateDescription {
    root: FrameId,
    schema: Arc<StateSchema>,
}

impl StateDescription {
    /// States built on `schema`, held in `root`, the frame their pose and
    /// motion are relative to.
    pub fn new(root: FrameId, schema: Arc<StateSchema>) -> Self {
        Self { root, schema }
    }

    /// The frame the states are held in.
    pub fn root(&self) -> &FrameId {
        &self.root
    }

    /// The schema every state is built on.
    pub fn schema(&self) -> &Arc<StateSchema> {
        &self.schema
    }
}

/// Every state channel `levels`' nodes write, described: each output its node
/// describes, and each output forwarding an input described by an earlier
/// node. Level order puts a forwarded input's writer first, since a node runs
/// after every writer of its inputs.
pub(super) fn describe_states(levels: &[Vec<ScheduledNode>]) -> StateSchemas {
    let mut states = StateSchemas::default();
    for scheduled in levels.iter().flatten() {
        let node = &scheduled.node;
        for port in node.port_descriptor().output_ports() {
            let description = match port.forwards() {
                Some(input) => states.of_channel(input).cloned(),
                None => node.state_description(port.channel()),
            };
            if let Some(description) = description {
                states.states.push((port.channel().clone(), description));
            }
        }
    }
    states
}
