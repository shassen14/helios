//! [`EstimatorInputBuilder`]: assembles the control vector a dynamics model's
//! predict consumes, from the bus; [`DeclaredInput`], a builder declared before
//! the pipeline is wired and built once its channels' mounts are known.

use crate::pipeline::InputMounts;
use crate::port::{ChannelKey, PortBus};
use crate::prelude::TickContext;

use helios_core::estimation::schema::InputSchema;
use helios_core::estimation::EstimatorInputs;
use helios_core::spatial::tf::TfProvider;
use helios_core::spatial::FrameId;

use std::sync::Arc;

/// Reads the bus and assembles one predict's control vector.
///
/// Paired with a dynamics model in a
/// [`DynamicsComponent`](super::DynamicsComponent), which refuses the pair
/// unless [`input_schema`](Self::input_schema) matches the input the dynamics
/// consume, so a builder can't feed a model rows it reads as something else.
pub trait EstimatorInputBuilder: Send + Sync {
    /// The input this builder assembles: each block's quantity, frame and
    /// convention, in row order.
    fn input_schema(&self) -> Arc<InputSchema>;

    /// The control vector for this tick, or why there is none. `tf` resolves
    /// the mounts of the channels it reads, for a builder whose input is
    /// declared in another frame than its readings.
    fn assemble(&self, bus: &PortBus, tf: &dyn TfProvider, tick: &TickContext) -> AssembledInput;

    /// Channels the builder needs before it can assemble anything.
    fn required_channels(&self) -> &[ChannelKey];

    /// Channels the builder reads when present.
    fn optional_channels(&self) -> &[ChannelKey];
}

/// What [`EstimatorInputBuilder::assemble`] returns: the control vector, or
/// why the bus can't supply one this tick.
pub enum AssembledInput {
    Ready(EstimatorInputs),
    Unavailable(InputUnavailable),
}

/// Why a builder assembled no input. The estimator skips predict either way;
/// a loud reason is also warned and degrades the estimate.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InputUnavailable {
    /// Quiet. A channel the builder reads has no reading yet (cold start) or
    /// none now (dropout).
    NoReading,
    /// **Loud.** The transform between a channel's mount and the frame the
    /// input is declared in did not resolve: a missing extrinsic, which would
    /// otherwise stop predict silently.
    MissingTransform { from: FrameId, to: FrameId },
    /// **Loud.** The transform resolved, but its axis conventions are not the
    /// ones the builder reads its readings in.
    ConventionMismatch { from: FrameId, to: FrameId },
}

impl InputUnavailable {
    /// The cause of a loud reason, or `None` for a quiet one.
    pub fn loud_cause(&self) -> Option<String> {
        match self {
            InputUnavailable::NoReading => None,
            InputUnavailable::MissingTransform { from, to } => {
                Some(format!("transform {from:?} → {to:?} unresolved"))
            }
            InputUnavailable::ConventionMismatch { from, to } => {
                Some(format!("convention mismatch between {from:?} and {to:?}"))
            }
        }
    }
}

/// Builds an input builder from the mounts of the channels it reads.
type BuildInput =
    Box<dyn FnOnce(&InputMounts) -> Result<Box<dyn EstimatorInputBuilder>, String> + Send>;

/// An input builder declared before the pipeline is wired: the input it will
/// assemble and the channels it will read, and how to build it once those
/// channels' mounts are known.
///
/// The declaration is what the dynamics are checked against and what the
/// node's ports are declared from; the builder built from it must match it.
pub struct DeclaredInput {
    schema: Arc<InputSchema>,
    required: Vec<ChannelKey>,
    optional: Vec<ChannelKey>,
    build: BuildInput,
}

impl DeclaredInput {
    /// A builder that will assemble `schema` from the `required` and
    /// `optional` channels, made by `build` from those channels' mounts.
    pub fn new<F>(
        schema: Arc<InputSchema>,
        required: Vec<ChannelKey>,
        optional: Vec<ChannelKey>,
        build: F,
    ) -> Self
    where
        F: FnOnce(&InputMounts) -> Result<Box<dyn EstimatorInputBuilder>, String> + Send + 'static,
    {
        Self {
            schema,
            required,
            optional,
            build: Box::new(build),
        }
    }

    /// The input the builder will assemble.
    pub(crate) fn input_schema(&self) -> Arc<InputSchema> {
        Arc::clone(&self.schema)
    }

    /// Channels the builder will need before it can assemble anything.
    pub(crate) fn required_channels(&self) -> &[ChannelKey] {
        &self.required
    }

    /// Channels the builder will read when present.
    pub(crate) fn optional_channels(&self) -> &[ChannelKey] {
        &self.optional
    }

    /// Builds the builder from `mounts`.
    ///
    /// Fails if the build does, or if the builder assembles another input or
    /// reads other channels than declared: the dynamics were checked against
    /// the declared input, and the node's ports carry the declared channels.
    pub(crate) fn build(
        self,
        mounts: &InputMounts,
    ) -> Result<Box<dyn EstimatorInputBuilder>, String> {
        let builder = (self.build)(mounts)?;
        if builder.input_schema().blocks() != self.schema.blocks() {
            return Err(
                "the input builder was built assembling another input than it declared".to_string(),
            );
        }
        if builder.required_channels() != self.required.as_slice()
            || builder.optional_channels() != self.optional.as_slice()
        {
            return Err(
                "the input builder was built reading other channels than it declared".to_string(),
            );
        }
        Ok(builder)
    }
}
