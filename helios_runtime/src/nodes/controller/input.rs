use crate::{
    port::{ChannelKey, InternalChannel, PortBus},
    prelude::{AgentRuntime, TickContext},
};
use helios_core::{
    control::{ControlInputs, ControlReference},
    frames::FrameAwareState,
};

use std::marker::PhantomData;

pub(crate) trait ControlInputBuilder: Send + Sync {
    type Output;

    fn assemble(
        &self,
        bus: &PortBus,
        runtime: &dyn AgentRuntime,
        tick: &TickContext,
    ) -> Option<Self::Output>;

    fn required_channels(&self) -> &[ChannelKey];

    fn optional_channels(&self) -> &[ChannelKey];
}

pub(crate) struct DefaultControlInputBuilder<R> {
    state_channel: ChannelKey,
    reference_channel: ChannelKey,
    required: Vec<ChannelKey>,
    optional: Vec<ChannelKey>,
    _reference: PhantomData<fn() -> R>,
}

impl<R: ControlReference> Default for DefaultControlInputBuilder<R> {
    fn default() -> Self {
        Self::new()
    }
}

impl<R: ControlReference> DefaultControlInputBuilder<R> {
    pub(crate) fn new() -> Self {
        let state_channel: ChannelKey = InternalChannel::of::<FrameAwareState>().into();
        let reference_channel: ChannelKey = InternalChannel::of::<R>().into();

        Self {
            state_channel: state_channel.clone(),
            reference_channel: reference_channel.clone(),
            required: vec![state_channel],
            optional: vec![reference_channel],
            _reference: PhantomData,
        }
    }
}

impl<R: ControlReference + Clone> ControlInputBuilder for DefaultControlInputBuilder<R> {
    type Output = ControlInputs<R>;

    fn assemble(
        &self,
        bus: &PortBus,
        _runtime: &dyn AgentRuntime,
        _tick: &TickContext,
    ) -> Option<ControlInputs<R>> {
        let state_stamped = bus.read::<FrameAwareState>(self.state_channel.clone())?;

        let reference = bus
            .read::<R>(self.reference_channel.clone())
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
