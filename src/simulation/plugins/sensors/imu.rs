use crate::simulation::core::topics::{GroundTruthState, ImuData, TopicBus};
use bevy::prelude::*;

// --- Plugin Definition ---
pub struct ImuPlugin;

impl Plugin for ImuPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(FixedUpdate, tick_imu_timers).add_systems(
            FixedUpdate,
            imu_sensor_system
                .after(tick_imu_timers)
                .run_if(should_run_imu_system),
        );
    }
}

// --- Components ---
#[derive(Component)]
pub struct Imu {
    pub name: String,
    // This is the full topic name this specific IMU publishes to.
    pub topic_to_publish: String,
    pub timer: Timer,
}

// This is the SINGLE component added to an agent. It manages ALL IMUs for that agent.
#[derive(Component, Default)]
pub struct ImuSuite {
    pub imus: Vec<Imu>,
}

// --- Systems ---

// This system ticks ALL IMU timers within ALL suites.
fn tick_imu_timers(mut query: Query<&mut ImuSuite>, time: Res<Time<Fixed>>) {
    for mut suite in query.iter_mut() {
        for imu in &mut suite.imus {
            imu.timer.tick(time.delta());
        }
    }
}

// This run condition checks if ANY IMU in ANY suite is ready.
fn should_run_imu_system(query: Query<&ImuSuite>) -> bool {
    query
        .iter()
        .any(|suite| suite.imus.iter().any(|imu| imu.timer.just_finished()))
}

// The main system iterates through the suites and their IMUs.
fn imu_sensor_system(
    agent_query: Query<(Entity, &ImuSuite, &GroundTruthState)>,
    mut topic_bus: ResMut<TopicBus>,
    time: Res<Time>,
) {
    for (entity, suite, ground_truth) in agent_query.iter() {
        // Loop through each IMU spec in the suite.
        for imu in &suite.imus {
            // Check if THIS specific IMU is ready.
            if imu.timer.just_finished() {
                debug!(
                    "[SENSOR] IMU '{}' firing to topic '{}'.",
                    imu.name, imu.topic_to_publish
                );
                let data = ImuData {
                    entity,
                    sensor_name: imu.name.clone(),
                    timestamp: time.elapsed_secs_f64(),
                    acceleration: ground_truth.linear_velocity,
                    angular_velocity: ground_truth.angular_velocity,
                };
                topic_bus.publish(&imu.topic_to_publish, data);
            }
        }
    }
}
