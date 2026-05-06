use crate::{
    port::{ChannelKey, PortBus},
    prelude::{AgentRuntime, TickContext},
};
use helios_core::{
    data::primitives::TrajectoryPoint, frames::FrameAwareState, prelude::ControlInputs,
};

pub trait ControlInputBuilder: Send + Sync {
    fn assemble(
        &self,
        bus: &PortBus,
        runtime: &dyn AgentRuntime,
        tick: &TickContext,
    ) -> Option<ControlInputs>;

    fn required_channels(&self) -> &[ChannelKey];

    fn optional_channels(&self) -> &[ChannelKey];
}

pub struct DefaultControlInputBuilder {
    state_channel: ChannelKey,
    reference_channel: ChannelKey,
    required: Vec<ChannelKey>,
    optional: Vec<ChannelKey>,
}

impl Default for DefaultControlInputBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl DefaultControlInputBuilder {
    pub fn new() -> Self {
        let state_channel = ChannelKey::of::<FrameAwareState>();
        let reference_channel = ChannelKey::of::<TrajectoryPoint>();

        Self {
            state_channel: state_channel.clone(),
            reference_channel: reference_channel.clone(),
            required: vec![state_channel],
            optional: vec![reference_channel],
        }
    }
}

impl ControlInputBuilder for DefaultControlInputBuilder {
    fn assemble(
        &self,
        bus: &PortBus,
        _runtime: &dyn AgentRuntime,
        _tick: &TickContext,
    ) -> Option<ControlInputs> {
        let state_stamped = bus.read::<FrameAwareState>(self.state_channel.clone())?;

        let reference = bus
            .read::<TrajectoryPoint>(self.reference_channel.clone())
            .map(|stamped| stamped.value.clone());

        Some(ControlInputs {
            state: state_stamped.value.clone(),
            reference,
        })
    }

    fn required_channels(&self) -> &[ChannelKey] {
        &self.required
    }

    fn optional_channels(&self) -> &[ChannelKey] {
        &self.optional
    }
}
