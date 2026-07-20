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

use crate::core::prng::SimulationRng;
use crate::core::transforms::EnuBodyPose;
use crate::prelude::{GroundTruthState, SensorPublishChannel, SensorPublisher};

use helios_core::data::{FrameHandle, MonotonicTime, SensorPayload, SensorReading};

use std::collections::HashSet;
use std::time::Duration;

use bevy::ecs::component::{Component, Mutable};
use bevy::ecs::entity::Entity;
use bevy::ecs::hierarchy::ChildOf;
use bevy::ecs::system::{Local, Query, Res, ResMut};
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
    /// inverse. All randomness must come from `rng` so a seeded run
    /// reproduces exactly.
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
/// For each sensor entity whose timer fires this tick: look up the parent
/// agent's ground truth, convert the sensor's global transform to an ENU
/// pose, ask the sensor for a reading, and publish it on the entity's
/// declared channel.
///
/// A sensor whose parent has no [`GroundTruthState`] can never produce data —
/// that is a scene-wiring bug, so the entity is skipped with a warning, once
/// per entity.
pub fn publish_state_sensor<S: StateSensor>(
    time: Res<Time>,
    mut rng: ResMut<SimulationRng>,
    truth_query: Query<&GroundTruthState>,
    mut sensor_query: Query<(
        Entity,
        &mut S,
        &mut SensorTimer,
        &GlobalTransform,
        &SensorPublishChannel,
        &ChildOf,
    )>,
    mut publisher: SensorPublisher,
    mut warned_missing_truth: Local<HashSet<Entity>>,
) {
    let elapsed = time.elapsed_secs_f64();
    let dt = time.delta();

    for (entity, mut sensor, mut timer, transform, channel, parent) in &mut sensor_query {
        timer.0.tick(dt);

        if !timer.0.just_finished() {
            continue;
        }

        let Ok(truth) = truth_query.get(parent.parent()) else {
            if warned_missing_truth.insert(entity) {
                warn!(
                    "State sensor {:?} skipped: parent agent {:?} has no GroundTruthState, \
                     so the sensor will never publish.",
                    entity,
                    parent.parent()
                );
            }
            continue;
        };
        let sensor_pose_world = EnuBodyPose::from(transform).0;

        let payload = sensor.sample(truth, &sensor_pose_world, &mut rng.0);

        let reading = SensorReading {
            sensor_handle: FrameHandle::from_entity(entity),
            timestamp: MonotonicTime(elapsed),
            data: payload,
        };

        publisher.publish(parent.parent(), channel.0.as_str(), vec![reading]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::data::sensor::GpsPosition;

    use bevy::ecs::schedule::Schedule;
    use bevy::ecs::world::World;
    use nalgebra::Vector3;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

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

    fn world_with_sensor(parent_has_truth: bool) -> (World, Entity) {
        let mut world = World::new();
        world.insert_resource(Time::<()>::default());
        world.insert_resource(SimulationRng(ChaCha8Rng::seed_from_u64(0)));

        let agent = if parent_has_truth {
            world.spawn(GroundTruthState::default()).id()
        } else {
            world.spawn_empty().id()
        };

        let sensor = world
            .spawn((
                CountingSensor { samples: 0 },
                SensorTimer::from_rate(RATE_HZ),
                GlobalTransform::default(),
                SensorPublishChannel("test.channel".to_string()),
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
    fn skips_a_sensor_whose_parent_has_no_truth_state() {
        let (mut world, sensor) = world_with_sensor(false);
        let mut schedule = publish_schedule();

        advance_and_run(&mut world, &mut schedule, period());

        assert_eq!(samples(&world, sensor), 0);
    }
}
