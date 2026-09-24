use helios_core::{spatial::FrameAwareState, prelude::PathFollowerInputs};

use crate::{
    port::{ChannelKey, InternalChannel, PortBus},
    prelude::TickContext,
};

pub(crate) trait PathFollowerInputBuilder: Send + Sync {
    fn assemble(&self, bus: &PortBus, tick: &TickContext) -> Option<PathFollowerInputs>;

    fn required_channels(&self) -> &[ChannelKey];

    fn optional_channels(&self) -> &[ChannelKey];
}

pub(crate) struct DefaultPathFollowerInputBuilder {
    state_channel: ChannelKey,
    required: Vec<ChannelKey>,
}

impl Default for DefaultPathFollowerInputBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl DefaultPathFollowerInputBuilder {
    pub(crate) fn new() -> Self {
        let state_channel: ChannelKey = InternalChannel::of::<FrameAwareState>().into();
        Self {
            state_channel: state_channel.clone(),
            required: vec![state_channel],
        }
    }
}

impl PathFollowerInputBuilder for DefaultPathFollowerInputBuilder {
    fn assemble(&self, bus: &PortBus, _tick: &TickContext) -> Option<PathFollowerInputs> {
        let state_stamped = bus.read::<FrameAwareState>(self.state_channel.clone())?;

        Some(PathFollowerInputs {
            state: state_stamped.value.clone(),
        })
    }

    fn required_channels(&self) -> &[ChannelKey] {
        &self.required
    }

    fn optional_channels(&self) -> &[ChannelKey] {
        &[]
    }
}
