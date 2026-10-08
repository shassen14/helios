//! [`EstimatorInputBuilder`]: assembles the control vector a dynamics model's
//! predict consumes, from the bus.

use crate::port::{ChannelKey, PortBus};
use crate::prelude::TickContext;

use helios_core::estimation::schema::InputSchema;
use helios_core::estimation::EstimatorInputs;

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

    /// The control vector for this tick, or `None` when the bus can't supply
    /// one yet (cold start, sensor dropout).
    fn assemble(&self, bus: &PortBus, tick: &TickContext) -> Option<EstimatorInputs>;

    /// Channels the builder needs before it can assemble anything.
    fn required_channels(&self) -> &[ChannelKey];

    /// Channels the builder reads when present.
    fn optional_channels(&self) -> &[ChannelKey];
}
