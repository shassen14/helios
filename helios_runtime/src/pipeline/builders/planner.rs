use helios_core::{
    frames::FrameAwareState,
    mapping::MapData,
    prelude::{PlannerGoal, SearchPlannerInputs},
};

use crate::{
    port::{ChannelKey, InternalChannel, PortBus},
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
pub(crate) trait SearchPlannerInputBuilder: Send + Sync {
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
/// and an optional `PlannerGoal` on a configurable goal channel (the planner
/// config's `goal_channel`, defaulting to `"mission"`). Whichever host input
/// declares that channel — a mission dispatcher, teleop, a ground-station
/// behavior tree — is indistinguishable to the planner.
pub(crate) struct DefaultSearchPlannerInputBuilder {
    state_channel: ChannelKey,
    map_channel: ChannelKey,
    goal_channel: ChannelKey,
    required: Vec<ChannelKey>,
    optional: Vec<ChannelKey>,
}

impl DefaultSearchPlannerInputBuilder {
    /// `goal_channel` is the instance name the planner's optional `PlannerGoal`
    /// input is keyed on, resolved from `SearchPlannerConfig`'s `goal_channel`
    /// field at assembly time. The host writes the mission goal to this same
    /// name, so the planner and the host input agree by construction.
    pub(crate) fn new(map_channel: InternalChannel, goal_channel: &str) -> Self {
        let state_channel: ChannelKey = InternalChannel::of::<FrameAwareState>().into();
        let map_channel_key: ChannelKey = map_channel.into();
        let goal_channel: ChannelKey = InternalChannel::named::<PlannerGoal>(goal_channel).into();

        Self {
            state_channel: state_channel.clone(),
            map_channel: map_channel_key.clone(),
            goal_channel: goal_channel.clone(),
            required: vec![state_channel, map_channel_key],
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A custom `goal_channel` reaches the builder's optional input descriptor:
    /// the planner subscribes to `PlannerGoal` on the configured name rather
    /// than the `"mission"` default, so renaming the mission channel in config
    /// rewires the planner to match the host that writes it.
    #[test]
    fn custom_goal_channel_reaches_optional_inputs() {
        let builder =
            DefaultSearchPlannerInputBuilder::new(InternalChannel::named::<MapData>("occupancy"), "waypoints");

        let goal: ChannelKey = InternalChannel::named::<PlannerGoal>("waypoints").into();

        // The goal is an optional input on the configured channel, never required.
        assert!(builder.optional_channels().contains(&goal));
        assert!(!builder.required_channels().contains(&goal));
    }

    /// The optional set carries exactly the goal channel, and the required set
    /// carries state + map — so a renamed goal channel cannot leak into the
    /// required inputs and turn an absent goal into an `UnsatisfiedInput`.
    #[test]
    fn goal_is_the_only_optional_input() {
        let builder =
            DefaultSearchPlannerInputBuilder::new(InternalChannel::named::<MapData>("occupancy"), "mission");

        assert_eq!(builder.optional_channels().len(), 1);
        assert_eq!(&**builder.optional_channels()[0].instance(), "mission");
        assert_eq!(builder.required_channels().len(), 2);
    }
}
