// src/sensors/imu_plugin.rs
use crate::simulation::topics::{GroundTruthState, ImuData};
use bevy::prelude::*;
use std::time::Duration;
// You'll need your deterministic RNG resource here too

pub struct ImuSensorPlugin;

impl Plugin for ImuSensorPlugin {
    fn build(&self, app: &mut App) {
        // Every plugin that sends a message must register its event type
        app.add_event::<ImuData>().add_systems(
            Update,
            imu_sensor_system
                // Example of a fixed rate using Bevy's built-in tools
                .run_if(on_timer(Duration::from_secs_f32(1.0 / 100.0))),
        );
    }
}

/// A component to be added to agents that have an IMU.
/// Contains the sensor's specific parameters.
#[derive(Component)]
pub struct Imu {
    pub noise_stddev_accel: f64,
    pub noise_stddev_gyro: f64,
}

/// The system that performs the sensor simulation.
fn imu_sensor_system(
    // Query for all entities that have an IMU and a ground truth state
    agent_query: Query<(Entity, &Imu, &GroundTruthState)>,
    mut imu_writer: EventWriter<ImuData>,
    // sim_time: Res<SimulationTime>, // Get the deterministic simulation time
    // mut rng: ResMut<DeterministicRng>, // Get the seeded RNG
) {
    for (entity, imu_params, ground_truth) in agent_query.iter() {
        // 1. Get ground truth data
        // 2. Add noise using rng
        // 3. Fire the ImuData event using imu_writer.send(...)
    }
}
