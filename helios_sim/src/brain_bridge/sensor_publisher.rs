//! The single path every host sensor uses to publish onto its agent's bus.
//!
//! Rate gating, `SensorReading` construction, and the choice of channel name
//! stay in each sensor's own system — those differ per sensor. Everything
//! after that is identical for GPS, IMU, magnetometer, lidar, and a future
//! camera: resolve the agent's pipeline, wrap the batch in a `Stamped`
//! envelope with the host producer id, write to the sensor's described key,
//! and report a dropped write once. That shared tail lives here as
//! [`SensorPublisher`], so there is exactly one place that constructs the
//! envelope and one place that handles a missing channel.

use crate::brain_bridge::components::SensorPublishChannel;
use crate::prelude::AutonomyPipelineComponent;

use helios_core::prelude::SensorReading;
use helios_runtime::port::ChannelError;
use helios_runtime::{Health, Stamped, HOST_PRODUCER_ID};

use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use std::collections::HashSet;

/// Bundles the two things every sensor system needs to publish: the query
/// that resolves an agent entity to its `AutonomyPipeline`, and a per-system
/// set of channels already reported. A sensor system takes this one param
/// in place of both, and its publish tail collapses to a single
/// [`publish`](Self::publish) call.
///
/// `warned_channels` is a [`Local`], so each system that takes a
/// `SensorPublisher` gets its own set. That is deliberate: a channel is
/// published by exactly one sensor system, so per-system dedup reports once
/// per channel — the same guarantee a global set would give, without the
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
    /// - A write to a channel no node reads is traced once per channel name.
    ///   The body lists every installed sensor, read or not, and the key
    ///   written is the one the body declared, so no slot means only that
    ///   this stack does not read the sensor.
    /// - Any other refused write warns exactly once per channel name, naming
    ///   it and why, since readings are being dropped: the readings are not
    ///   of the type the channel was described with, or a reading names
    ///   another frame than the channel's mount.
    pub fn publish<T: Send + Sync + 'static>(
        &mut self,
        agent: Entity,
        channel: &SensorPublishChannel,
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

        let channel_key = channel.description().key();
        let write_result = pipeline
            .0
            .bus()
            .write_mounted(channel_key.clone(), stamped_readings);

        // Report once per channel: `insert` returns true only the first time
        // a name is seen, so a fixed-rate sensor cannot flood the log.
        let Err(error) = write_result else {
            return;
        };
        let name = channel_key.name();
        if !self.warned_channels.insert(name.to_string()) {
            return;
        }
        match error {
            // The fit installs a sensor no node reads; the build accepts
            // that, since a body lists what is installed. Not a wiring bug:
            // the key is the one the body declared, so it cannot be misspelt.
            ChannelError::UnknownChannel => tracing::trace!(
                channel = %name,
                "sensor channel has no reader in the autonomy DAG; readings are dropped"
            ),
            // The spawner described the channel with another payload or shape
            // than the sensor publishes: a host bug every reader would see as
            // silence.
            error => warn!(
                "Channel '{}' refused a write ({}); readings are being dropped.",
                name, error
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use bevy::ecs::system::SystemState;
    use helios_core::interchange::measurement::sensor::GpsPosition;
    use helios_core::prelude::AgentId;
    use helios_core::spatial::primitives::MonotonicTime;
    use helios_core::spatial::FrameId;
    use nalgebra::Vector3;

    fn gps_reading() -> SensorReading<GpsPosition> {
        SensorReading {
            sensor: FrameId::agent(AgentId::new("test_agent"), "gps"),
            timestamp: MonotonicTime(0.0),
            data: GpsPosition {
                position: Vector3::zeros(),
            },
        }
    }

    fn gps_channel() -> SensorPublishChannel {
        SensorPublishChannel::new::<GpsPosition>(
            "gps.primary",
            "gps",
            FrameId::agent(AgentId::new("test_agent"), "gps.primary"),
        )
    }

    /// An agent without an `AutonomyPipelineComponent` — a prop, or an agent
    /// mid-scene-build — must be a silent no-op, not a panic.
    #[test]
    fn publish_to_agent_without_pipeline_is_a_noop() {
        let mut world = World::new();
        let agent = world.spawn_empty().id();

        let mut state: SystemState<SensorPublisher> = SystemState::new(&mut world);
        let mut publisher = state.get_mut(&mut world).unwrap();

        publisher.publish(agent, &gps_channel(), vec![gps_reading()]);
    }

    /// An empty batch never touches the bus and never warns.
    #[test]
    fn publish_empty_batch_is_a_noop() {
        let mut world = World::new();
        let agent = world.spawn_empty().id();

        let mut state: SystemState<SensorPublisher> = SystemState::new(&mut world);
        let mut publisher = state.get_mut(&mut world).unwrap();

        publisher.publish::<GpsPosition>(agent, &gps_channel(), Vec::new());
    }
}
