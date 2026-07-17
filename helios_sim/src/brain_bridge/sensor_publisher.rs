//! The single path every host sensor uses to publish onto its agent's bus.
//!
//! Rate gating, `SensorReading` construction, and the choice of channel name
//! stay in each sensor's own system — those differ per sensor. Everything
//! after that is identical for GPS, IMU, magnetometer, lidar, and a future
//! camera: resolve the agent's pipeline, wrap the batch in a `Stamped`
//! envelope with the host producer id, build the named channel key, write, and
//! turn a dropped write into one loud warning. That shared tail lives here as
//! [`SensorPublisher`], so there is exactly one place that constructs the
//! envelope and one place that handles a missing channel.

use std::collections::HashSet;

use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use helios_core::data::SensorReading;
use helios_runtime::{port::SensorChannel, ChannelKey, Health, Stamped, HOST_PRODUCER_ID};

use crate::prelude::AutonomyPipelineComponent;

/// Bundles the two things every sensor system needs to publish: the query
/// that resolves an agent entity to its `AutonomyPipeline`, and a per-system
/// set of channels already warned about. A sensor system takes this one param
/// in place of both, and its publish tail collapses to a single
/// [`publish`](Self::publish) call.
///
/// `warned_channels` is a [`Local`], so each system that takes a
/// `SensorPublisher` gets its own set. That is deliberate: a channel is
/// published by exactly one sensor system, so per-system dedup warns once per
/// broken channel — the same guarantee a global set would give, without the
/// shared-mutable resource that would serialize the sensor systems against
/// each other.
#[derive(SystemParam)]
pub struct SensorPublisher<'w, 's> {
    pipelines: Query<'w, 's, &'static AutonomyPipelineComponent>,
    warned_channels: Local<'s, HashSet<String>>,
}

impl SensorPublisher<'_, '_> {
    /// Publishes a batch of readings to `channel` on `agent`'s pipeline bus.
    ///
    /// - An empty batch is a no-op.
    /// - An agent with no pipeline (not yet spawned during scene build, or a
    ///   brainless prop) is a silent no-op — legal, not an error.
    /// - The envelope timestamp is taken from the last reading, so an
    ///   accumulated batch carries its freshest time.
    /// - A write to an unwired channel warns exactly once per channel name,
    ///   naming it: that means the profile's declared channel and a consumer's
    ///   configured input channel disagree, and readings are being dropped.
    pub fn publish<T: Send + Sync + 'static>(
        &mut self,
        agent: Entity,
        channel: &str,
        readings: Vec<SensorReading<T>>,
    ) {
        let Ok(pipeline) = self.pipelines.get(agent) else {
            return;
        };

        let Some(reading) = readings.last() else {
            return;
        };

        let envelope_timestamp = reading.timestamp;

        let stamped_readings = Stamped {
            value: readings,
            timestamp: envelope_timestamp,
            health: Health::Ok,
            producer: HOST_PRODUCER_ID,
        };

        let channel_key: ChannelKey = SensorChannel::named::<Vec<SensorReading<T>>>(channel).into();

        let write_result = pipeline.0.bus().write(channel_key, stamped_readings);

        // A failed write means no consumer is wired to this channel — a config
        // defect (the profile's channel name and a consumer's input channel
        // disagree), unlike an oracle channel where a missing consumer is
        // expected. Warn once per channel: `insert` returns true only the first
        // time a name is seen, so a fixed-rate sensor cannot flood the log.
        if write_result.is_err() && self.warned_channels.insert(channel.to_string()) {
            warn!(
                "Channel '{}' has no slot in the autonomy DAG; readings are being \
                 dropped.",
                channel
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use bevy::ecs::system::SystemState;
    use helios_core::data::primitives::{FrameHandle, MonotonicTime};
    use helios_core::data::sensor::GpsPosition;
    use nalgebra::Vector3;

    fn gps_reading(handle: Entity) -> SensorReading<GpsPosition> {
        SensorReading {
            sensor_handle: FrameHandle::from_entity(handle),
            timestamp: MonotonicTime(0.0),
            data: GpsPosition {
                position: Vector3::zeros(),
            },
        }
    }

    /// An agent without an `AutonomyPipelineComponent` — a prop, or an agent
    /// mid-scene-build — must be a silent no-op, not a panic.
    #[test]
    fn publish_to_agent_without_pipeline_is_a_noop() {
        let mut world = World::new();
        let agent = world.spawn_empty().id();

        let mut state: SystemState<SensorPublisher> = SystemState::new(&mut world);
        let mut publisher = state.get_mut(&mut world).unwrap();

        publisher.publish(agent, "sensor.gps.primary", vec![gps_reading(agent)]);
    }

    /// An empty batch never touches the bus and never warns.
    #[test]
    fn publish_empty_batch_is_a_noop() {
        let mut world = World::new();
        let agent = world.spawn_empty().id();

        let mut state: SystemState<SensorPublisher> = SystemState::new(&mut world);
        let mut publisher = state.get_mut(&mut world).unwrap();

        publisher.publish::<GpsPosition>(agent, "sensor.gps.primary", Vec::new());
    }
}
