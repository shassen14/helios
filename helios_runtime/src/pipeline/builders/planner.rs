use helios_core::{
    frames::FrameAwareState,
    mapping::MapData,
    prelude::{PlannerGoal, SearchPlannerInputs},
};

use crate::{
    pipeline::autonomy_pipeline::MISSION_GOAL_INSTANCE,
    port::{ChannelKey, PortBus},
    prelude::{AgentRuntime, TickContext},
};

/// Assembles [`SearchPlannerInputs`] for a [`SearchPlannerNode`] from bus channels.
///
/// Declares the channels it reads via [`required_channels`]/[`optional_channels`]
/// so the node can publish them in its [`PortDescriptor`].
///
/// [`SearchPlannerNode`]: crate::pipeline::nodes::search_planner::SearchPlannerNode
/// [`PortDescriptor`]: crate::port::PortDescriptor
/// [`required_channels`]: SearchPlannerInputBuilder::required_channels
/// [`optional_channels`]: SearchPlannerInputBuilder::optional_channels
pub trait SearchPlannerInputBuilder: Send + Sync {
    fn assemble(
        &self,
        bus: &PortBus,
        runtime: &dyn AgentRuntime,
        tick: &TickContext,
    ) -> Option<SearchPlannerInputs>;

    fn required_channels(&self) -> &[ChannelKey];

    fn optional_channels(&self) -> &[ChannelKey];
}

/// Default builder: reads `FrameAwareState @ ""`, a configurable map channel,
/// and an optional `PlannerGoal @ "mission"` (the canonical mission slot
/// written by `AutonomyPipeline::inject_mission_goal`).
pub struct DefaultSearchPlannerInputBuilder {
    state_channel: ChannelKey,
    map_channel: ChannelKey,
    goal_channel: ChannelKey,
    required: Vec<ChannelKey>,
    optional: Vec<ChannelKey>,
}

impl DefaultSearchPlannerInputBuilder {
    pub fn new(map_channel: ChannelKey) -> Self {
        let state_channel = ChannelKey::of::<FrameAwareState>();
        let goal_channel = ChannelKey::named::<PlannerGoal>(MISSION_GOAL_INSTANCE);

        Self {
            state_channel: state_channel.clone(),
            map_channel: map_channel.clone(),
            goal_channel: goal_channel.clone(),
            required: vec![state_channel, map_channel],
            optional: vec![goal_channel],
        }
    }
}

impl SearchPlannerInputBuilder for DefaultSearchPlannerInputBuilder {
    fn assemble(
        &self,
        bus: &PortBus,
        _runtime: &dyn AgentRuntime,
        _tick: &TickContext,
    ) -> Option<SearchPlannerInputs> {
        let state_stamped = bus.read::<FrameAwareState>(self.state_channel.clone())?;
        let map_stamped = bus.read::<MapData>(self.map_channel.clone())?;
        let goal = bus
            .read::<PlannerGoal>(self.goal_channel.clone())
            .map(|s| s.value.clone());

        Some(SearchPlannerInputs {
            state: state_stamped.value.state.clone(),
            map: map_stamped.value.clone(),
            goal,
        })
    }

    fn required_channels(&self) -> &[ChannelKey] {
        &self.required
    }

    fn optional_channels(&self) -> &[ChannelKey] {
        &self.optional
    }
}
