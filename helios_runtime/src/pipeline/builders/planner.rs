use helios_core::{
    frames::FrameAwareState,
    mapping::MapData,
    prelude::{PlannerGoal, PlannerInputs},
};

use crate::{
    port::{ChannelKey, PortBus},
    prelude::{AgentRuntime, TickContext},
};

pub trait PlannerInputBuilder: Send + Sync {
    fn assemble(
        &self,
        bus: &PortBus,
        runtime: &dyn AgentRuntime,
        tick: &TickContext,
    ) -> Option<PlannerInputs>;

    fn required_channels(&self) -> &[ChannelKey];

    fn optional_channels(&self) -> &[ChannelKey];
}

pub struct DefaultPlannerInputBuilder {
    state_channel: ChannelKey,
    map_channel: ChannelKey,
    goal_channel: ChannelKey,
    required: Vec<ChannelKey>,
    optional: Vec<ChannelKey>,
}

impl DefaultPlannerInputBuilder {
    pub fn new(map_channel: ChannelKey) -> Self {
        let state_channel = ChannelKey::of::<FrameAwareState>();
        let goal_channel = ChannelKey::of::<PlannerGoal>();

        Self {
            state_channel: state_channel.clone(),
            map_channel: map_channel.clone(),
            goal_channel: goal_channel.clone(),
            required: vec![state_channel, map_channel],
            optional: vec![goal_channel],
        }
    }
}

impl PlannerInputBuilder for DefaultPlannerInputBuilder {
    fn assemble(
        &self,
        bus: &PortBus,
        _runtime: &dyn AgentRuntime,
        tick: &TickContext,
    ) -> Option<PlannerInputs> {
        let state_stamped = bus.read::<FrameAwareState>(self.state_channel.clone())?;
        let map_stamped = bus.read::<MapData>(self.map_channel.clone())?;
        let goal = bus
            .read::<PlannerGoal>(self.goal_channel.clone())
            .map(|s| s.value.clone());

        Some(PlannerInputs {
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
