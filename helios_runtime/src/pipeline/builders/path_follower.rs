use helios_core::{frames::FrameAwareState, prelude::PathFollowerInputs};

use crate::{
    port::{ChannelKey, InternalChannel, PortBus},
    prelude::{AgentRuntime, TickContext},
};

pub trait PathFollowerInputBuilder: Send + Sync {
    fn assemble(
        &self,
        bus: &PortBus,
        runtime: &dyn AgentRuntime,
        tick: &TickContext,
    ) -> Option<PathFollowerInputs>;

    fn required_channels(&self) -> &[ChannelKey];

    fn optional_channels(&self) -> &[ChannelKey];
}

pub struct DefaultPathFollowerInputBuilder {
    state_channel: ChannelKey,
    required: Vec<ChannelKey>,
}

impl Default for DefaultPathFollowerInputBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl DefaultPathFollowerInputBuilder {
    pub fn new() -> Self {
        let state_channel: ChannelKey = InternalChannel::of::<FrameAwareState>().into();
        Self {
            state_channel: state_channel.clone(),
            required: vec![state_channel],
        }
    }
}

impl PathFollowerInputBuilder for DefaultPathFollowerInputBuilder {
    fn assemble(
        &self,
        bus: &PortBus,
        _runtime: &dyn AgentRuntime,
        _tick: &TickContext,
    ) -> Option<PathFollowerInputs> {
        let state_stamped = bus.read::<FrameAwareState>(self.state_channel.clone())?;

        Some(PathFollowerInputs {
            state: state_stamped.value.state.clone(),
        })
    }

    fn required_channels(&self) -> &[ChannelKey] {
        &self.required
    }

    fn optional_channels(&self) -> &[ChannelKey] {
        &[]
    }
}
