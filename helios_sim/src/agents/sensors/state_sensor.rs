//! The state-sensor half of sensor simulation: one trait and one generic
//! system in place of a bespoke publish loop per sensor.
//!
//! A state sensor's reading is a pure function of its agent's ground-truth
//! state and its own world pose — it needs no world query. Every such sensor
//! shares the same orchestration: tick a rate timer, look up the parent
//! agent's truth, convert the Bevy transform into an ENU pose, delegate to a
//! forward model in `helios_core::sensors`, and publish the payload on the
//! profile-declared channel. [`publish_state_sensor`] owns that loop once,
//! generically; a concrete sensor contributes only a component implementing
//! [`StateSensor`].

use crate::core::prng::SensorRng;
use crate::core::transforms::{bevy_transform_to_transform_bevy, FromBevy};
use crate::prelude::{AgentIdComponent, GroundTruthState, SensorPublishChannel, SensorPublisher};

use helios_core::prelude::{MonotonicTime, SensorPayload, SensorReading};
use helios_core::spatial::conventions::{Enu, Flu};
use helios_core::spatial::transforms::Transform as CoreTransform;

use std::collections::HashSet;
use std::time::Duration;

use bevy::ecs::component::{Component, Mutable};
use bevy::ecs::entity::Entity;
use bevy::ecs::hierarchy::ChildOf;
use bevy::ecs::query::With;
use bevy::ecs::system::{Local, Query, Res};
use bevy::log::warn;
use bevy::time::{Time, Timer, TimerMode};
use bevy::transform::components::GlobalTransform;
use nalgebra::Isometry3;
use rand::RngCore;

/// Publish-rate gate carried by every state-sensor entity, spawned alongside
/// the sensor component. The rate is scheduling policy, not sensor physics,
/// so it lives outside the sensor component, where [`publish_state_sensor`]
/// can tick it without knowing the concrete sensor type.
#[derive(Component)]
pub struct SensorTimer(Timer);

impl SensorTimer {
    /// Builds a repeating timer that fires every `1 / rate_hz` seconds.
    ///
    /// Panics if `rate_hz` is zero, negative, or non-finite — a bad config
    /// rate should stop the run at scene build, not limp along.
    pub fn from_rate(rate_hz: f64) -> Self {
        Self(Timer::new(
            Duration::from_secs_f64(1.0 / rate_hz),
            TimerMode::Repeating,
        ))
    }
}

/// A sensor whose reading is a pure function of its agent's ground-truth
/// state and its own world pose — no world query.
///
/// Implementors are thin glue: they hold a forward model from
/// `helios_core::sensors` plus any environmental constants baked in at spawn
/// (gravity, a reference field), and [`sample`](Self::sample) delegates to
/// that model. The `Mutable` bound and `&mut self` leave room for stateful
/// models such as a drifting bias.
pub trait StateSensor: Component<Mutability = Mutable> {
    /// The typed payload this sensor publishes onto the agent's bus.
    type Payload: SensorPayload + Send + Sync + 'static;

    /// Produces one reading.
    ///
    /// `truth` is the parent agent's ground-truth state. `sensor_pose_world`
    /// is the sensor's FLU frame expressed in the ENU world: its translation
    /// is the sensor's world position, and its rotation maps sensor vectors
    /// into world axes — a model that wants world-into-sensor takes the
    /// inverse.
    ///
    /// `rng` is this sensor's own stream, not a shared one. All randomness must
    /// come from it: a model reaching for any other source breaks replay, and
    /// one that draws a variable number of values per call — rather than a
    /// fixed number — makes its own future readings depend on its past inputs.
    fn sample(
        &mut self,
        truth: &GroundTruthState,
        sensor_pose_world: &Isometry3<f64>,
        rng: &mut dyn RngCore,
    ) -> Self::Payload;
}

/// The shared publish loop, registered once per concrete sensor type
/// (`publish_state_sensor::<Gps>`, …) in `SimulationSet::Sensors`.
///
/// For each sensor entity whose timer fires this tick: look up its agent's
/// ground truth, convert the sensor's global transform to an ENU pose, ask the
/// sensor for a reading, stamp it with the channel's mount, and publish it on
/// the entity's declared channel. The agent is the nearest ancestor carrying
/// its identity ([`AgentIdComponent`]): a sensor hangs directly under it, or
/// under its installation's frame entity when one installation carries several
/// sensors (an IMU).
///
/// Noise is drawn from the entity's own [`SensorRng`], so a sensor's readings
/// depend only on its own name, its own tick count, and the run's master seed.
/// The registrations of this system for different sensor types are unordered
/// with respect to each other, and after this loop that no longer matters.
///
/// A sensor with no agent above it, or whose agent has no
/// [`GroundTruthState`], can never produce data — that is a scene-wiring bug,
/// so the entity is skipped with a warning, once per entity.
#[allow(clippy::type_complexity)]
pub fn publish_state_sensor<S: StateSensor>(
    time: Res<Time>,
    truth_query: Query<&GroundTruthState>,
    mut sensor_query: Query<(
        Entity,
        &mut S,
        &mut SensorTimer,
        &mut SensorRng,
        &GlobalTransform,
        &SensorPublishChannel,
    )>,
    parents: Query<&ChildOf>,
    agents: Query<(), With<AgentIdComponent>>,
    mut publisher: SensorPublisher,
    mut warned_missing_truth: Local<HashSet<Entity>>,
) {
    let elapsed = time.elapsed_secs_f64();
    let dt = time.delta();

    for (entity, mut sensor, mut timer, mut rng, transform, channel) in &mut sensor_query {
        timer.0.tick(dt);

        if !timer.0.just_finished() {
            continue;
        }

        let agent = parents
            .iter_ancestors(entity)
            .find(|&ancestor| agents.contains(ancestor));
        let Some((agent, truth)) =
            agent.and_then(|agent| truth_query.get(agent).ok().map(|truth| (agent, truth)))
        else {
            if warned_missing_truth.insert(entity) {
                warn!(
                    "State sensor {:?} skipped: its agent {:?} has no GroundTruthState, \
                     so the sensor will never publish.",
                    entity, agent
                );
            }
            continue;
        };
        let sensor_pose: CoreTransform<Flu, Enu> =
            bevy_transform_to_transform_bevy(transform.compute_transform()).from_bevy();
        let sensor_pose_world = sensor_pose.into_inner();

        let payload = sensor.sample(truth, &sensor_pose_world, &mut rng.0);

        let reading = SensorReading {
            sensor: channel.mount().clone(),
            timestamp: MonotonicTime(elapsed),
            data: payload,
        };

        publisher.publish(agent, channel, vec![reading]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::prelude::AutonomyPipelineComponent;

    use helios_core::interchange::measurement::sensor::GpsPosition;
    use helios_core::prelude::{AgentId, TfProvider};
    use helios_core::spatial::{transforms::Convention, FrameId};
    use helios_runtime::port::{ChannelKey, PortBus, PortDescriptorBuilder};
    use helios_runtime::{
        BodyCapabilities, PipelineBuilder, PipelineNode, PortDescriptor, TickContext,
    };

    use bevy::ecs::schedule::Schedule;
    use bevy::ecs::world::World;
    use nalgebra::Vector3;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;
    use std::collections::HashMap;

    /// Counts `sample` calls so the tests can assert exactly when the generic
    /// system consults a sensor.
    #[derive(Component)]
    struct CountingSensor {
        samples: u32,
    }

    impl StateSensor for CountingSensor {
        type Payload = GpsPosition;

        fn sample(
            &mut self,
            _truth: &GroundTruthState,
            _sensor_pose_world: &Isometry3<f64>,
            _rng: &mut dyn RngCore,
        ) -> GpsPosition {
            self.samples += 1;
            GpsPosition {
                position: Vector3::zeros(),
            }
        }
    }

    const RATE_HZ: f64 = 10.0;

    /// The period the timer was actually built with, so the sampling tests
    /// stay valid for any `RATE_HZ`, including ones whose period is not
    /// exactly representable.
    fn period() -> Duration {
        SensorTimer::from_rate(RATE_HZ).0.duration()
    }

    /// An arbitrary fixed seed. These tests assert on *when* the system samples,
    /// never on the values it produces, so the sensor's stream only has to be
    /// present and reproducible — hence a literal seed rather than
    /// [`SensorRng::from_sensor`], which would couple timer tests to the seed
    /// derivation for no gain.
    const TEST_SEED: u64 = 0;

    const TEST_AGENT: &str = "test_agent";
    const TEST_CHANNEL: &str = "test.channel";
    const TEST_UNIT: &str = "test_unit";
    /// These tests publish and stamp readings, which a reference doesn't
    /// change, so their channels state none and their pipelines declare no
    /// world frame.
    const UNREFERENCED: bool = false;

    /// The test sensor's mount: its installation's frame, named apart from
    /// its channel so a header stamped from the wrong one shows.
    fn test_mount() -> FrameId {
        FrameId::agent(AgentId::new(TEST_AGENT), TEST_UNIT)
    }

    fn world_with_sensor(agent_has_truth: bool) -> (World, Entity) {
        let mut world = World::new();
        world.insert_resource(Time::<()>::default());

        let agent = world.spawn(AgentIdComponent(AgentId::new(TEST_AGENT))).id();
        if agent_has_truth {
            world.entity_mut(agent).insert(GroundTruthState::default());
        }

        let sensor = world
            .spawn((
                CountingSensor { samples: 0 },
                SensorTimer::from_rate(RATE_HZ),
                SensorRng(ChaCha8Rng::seed_from_u64(TEST_SEED)),
                GlobalTransform::default(),
                SensorPublishChannel::new::<GpsPosition>(
                    TEST_CHANNEL,
                    TEST_UNIT,
                    test_mount(),
                    RATE_HZ,
                    UNREFERENCED,
                ),
                ChildOf(agent),
            ))
            .id();

        (world, sensor)
    }

    fn publish_schedule() -> Schedule {
        let mut schedule = Schedule::default();
        schedule.add_systems(publish_state_sensor::<CountingSensor>);
        schedule
    }

    fn advance_and_run(world: &mut World, schedule: &mut Schedule, dt: Duration) {
        world.resource_mut::<Time>().advance_by(dt);
        schedule.run(world);
    }

    fn samples(world: &World, sensor: Entity) -> u32 {
        world.get::<CountingSensor>(sensor).unwrap().samples
    }

    #[test]
    fn from_rate_builds_a_repeating_timer_with_the_period() {
        let timer = SensorTimer::from_rate(RATE_HZ);
        assert!((timer.0.duration().as_secs_f64() - 1.0 / RATE_HZ).abs() < 1e-9);
        assert_eq!(timer.0.mode(), TimerMode::Repeating);
    }

    #[test]
    fn does_not_sample_before_the_period_elapses() {
        let (mut world, sensor) = world_with_sensor(true);
        let mut schedule = publish_schedule();

        advance_and_run(&mut world, &mut schedule, period() / 2);
        assert_eq!(samples(&world, sensor), 0);

        // The partial tick accumulates: one more full period puts it over.
        advance_and_run(&mut world, &mut schedule, period());
        assert_eq!(samples(&world, sensor), 1);
    }

    #[test]
    fn samples_once_each_time_the_period_elapses() {
        let (mut world, sensor) = world_with_sensor(true);
        let mut schedule = publish_schedule();

        advance_and_run(&mut world, &mut schedule, period());
        assert_eq!(samples(&world, sensor), 1);

        advance_and_run(&mut world, &mut schedule, period());
        assert_eq!(samples(&world, sensor), 2);
    }

    #[test]
    fn skips_a_sensor_whose_agent_has_no_truth_state() {
        let (mut world, sensor) = world_with_sensor(false);
        let mut schedule = publish_schedule();

        advance_and_run(&mut world, &mut schedule, period());

        assert_eq!(samples(&world, sensor), 0);
    }

    /// Truth on an entity that is not an agent is not the sensor's: the agent
    /// is found by its identity, not by whatever carries truth.
    #[test]
    fn skips_a_sensor_with_no_agent_above_it() {
        let (mut world, sensor) = world_with_sensor(true);
        let agent = world.get::<ChildOf>(sensor).unwrap().parent();
        world.entity_mut(agent).remove::<AgentIdComponent>();
        let mut schedule = publish_schedule();

        advance_and_run(&mut world, &mut schedule, period());

        assert_eq!(samples(&world, sensor), 0);
    }

    /// One installation carrying several sensors (an IMU) hangs them under
    /// its frame entity, so the agent is a grandparent.
    #[test]
    fn samples_a_sensor_hung_under_its_installation_frame() {
        let (mut world, sensor) = world_with_sensor(true);
        let agent = world.get::<ChildOf>(sensor).unwrap().parent();
        let installation = world.spawn(ChildOf(agent)).id();
        world.entity_mut(sensor).insert(ChildOf(installation));
        let mut schedule = publish_schedule();

        advance_and_run(&mut world, &mut schedule, period());

        assert_eq!(samples(&world, sensor), 1);
    }

    /// Declares the test channel as an optional input, so the bus reserves
    /// a slot the sensor can publish into.
    struct FakeGpsConsumer {
        descriptor: PortDescriptor,
    }

    impl PipelineNode for FakeGpsConsumer {
        fn name(&self) -> &str {
            "fake_gps_consumer"
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {}
    }

    fn test_channel() -> ChannelKey {
        ChannelKey::measurement::<Vec<SensorReading<GpsPosition>>>(TEST_CHANNEL)
    }

    /// Nothing downstream of the estimator reads a state sensor's header, so
    /// a wrong one would pass every run: it must name the channel's mount,
    /// the installation's frame, never the channel.
    #[test]
    fn a_reading_is_stamped_with_its_channels_mount() {
        let (mut world, sensor) = world_with_sensor(true);
        let agent = world.get::<ChildOf>(sensor).unwrap().parent();
        let channel = world.get::<SensorPublishChannel>(sensor).unwrap().clone();
        let consumer = FakeGpsConsumer {
            descriptor: PortDescriptorBuilder::algorithm()
                .inputs_from_slices(&[], &[test_channel()])
                .build(),
        };
        let pipeline = PipelineBuilder::new(AgentId::new(TEST_AGENT))
            .add_node(Box::new(consumer))
            .with_body_capabilities(BodyCapabilities {
                publishes: vec![channel.description(&FrameId::global("datum"))],
                frames: HashMap::from([(test_mount(), Convention::Flu)]),
                ..Default::default()
            })
            .build()
            .expect("a single-node pipeline builds");
        world
            .entity_mut(agent)
            .insert(AutonomyPipelineComponent(pipeline));
        let mut schedule = publish_schedule();

        advance_and_run(&mut world, &mut schedule, period());

        let pipeline = world.get::<AutonomyPipelineComponent>(agent).unwrap();
        let batch = pipeline
            .0
            .bus()
            .read::<Vec<SensorReading<GpsPosition>>>(test_channel())
            .expect("the reading was published");
        let reading = batch.value.last().expect("one reading in the batch");
        assert_eq!(reading.sensor, test_mount());
    }

    // ==================================================================
    // == Stream isolation ==
    // ==================================================================

    /// Records the values it draws, so a test can compare one sensor's stream
    /// across two differently-populated worlds.
    #[derive(Component)]
    struct RecordingSensor {
        draws: Vec<u64>,
    }

    impl StateSensor for RecordingSensor {
        type Payload = GpsPosition;

        fn sample(
            &mut self,
            _truth: &GroundTruthState,
            _sensor_pose_world: &Isometry3<f64>,
            rng: &mut dyn RngCore,
        ) -> GpsPosition {
            self.draws.push(rng.next_u64());
            GpsPosition {
                position: Vector3::zeros(),
            }
        }
    }

    const TEST_MASTER_SEED: u64 = 2024;

    /// Enough ticks that an interleaved stream would diverge unmistakably, and
    /// few enough to stay instant.
    const ISOLATION_TICKS: usize = 5;

    /// Spawns one recording sensor per name under a single agent, runs the
    /// publish system until each has sampled `ISOLATION_TICKS` times, and
    /// returns their recorded draws in the order the names were given.
    fn draws_per_sensor(names: &[&str]) -> Vec<Vec<u64>> {
        let mut world = World::new();
        world.insert_resource(Time::<()>::default());
        let agent = world
            .spawn((
                AgentIdComponent(AgentId::new("car_1")),
                GroundTruthState::default(),
            ))
            .id();

        let mut sensors = Vec::new();
        for name in names {
            let sensor = world
                .spawn((
                    RecordingSensor { draws: Vec::new() },
                    SensorTimer::from_rate(RATE_HZ),
                    SensorRng::from_sensor(TEST_MASTER_SEED, name),
                    GlobalTransform::default(),
                    SensorPublishChannel::new::<GpsPosition>(
                        "test.channel",
                        "test_unit",
                        FrameId::agent(AgentId::new("car_1"), *name),
                        RATE_HZ,
                        UNREFERENCED,
                    ),
                    ChildOf(agent),
                ))
                .id();
            sensors.push(sensor);
        }

        let mut schedule = Schedule::default();
        schedule.add_systems(publish_state_sensor::<RecordingSensor>);

        for _ in 0..ISOLATION_TICKS {
            advance_and_run(&mut world, &mut schedule, period());
        }

        sensors
            .into_iter()
            .map(|sensor| world.get::<RecordingSensor>(sensor).unwrap().draws.clone())
            .collect()
    }

    /// The property the per-sensor RNG exists to provide, and the one a shared
    /// generator cannot have: adding a second sensor to an agent must not
    /// perturb the first one's noise.
    ///
    /// Under a single shared RNG this fails — both sensors draw from one
    /// sequence, so whichever the query visits first takes the earlier values
    /// and the other's readings shift. That made an unrelated scenario edit
    /// (adding a sensor, removing one, a lidar hitting a different surface
    /// count) silently change every other sensor's output for the whole run.
    #[test]
    fn a_sensors_stream_is_unaffected_by_a_sibling() {
        let alone = draws_per_sensor(&["car_1/gps"]);
        let with_sibling = draws_per_sensor(&["car_1/gps", "car_1/imu"]);

        // Guards against a vacuous pass: if the timer never fired, both sides
        // would be empty and compare equal while asserting nothing.
        assert_eq!(alone[0].len(), ISOLATION_TICKS);

        assert_eq!(alone[0], with_sibling[0]);
    }

    /// The companion to isolation: streams must be independent, not identical.
    /// A derivation that ignored the sensor name would pass the test above
    /// while giving every sensor on an agent the same noise.
    #[test]
    fn two_sensors_on_one_agent_draw_different_streams() {
        let draws = draws_per_sensor(&["car_1/gps", "car_1/imu"]);

        assert_ne!(draws[0], draws[1]);
    }
}
