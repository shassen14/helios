use bevy::prelude::*;
use nalgebra::{DMatrix, Vector3};
use rand_distr::{Distribution, Normal};
use std::time::Duration;

// --- Simulation Crate Imports ---
use crate::prelude::*;
use crate::simulation::core::transforms::bevy_global_transform_to_enu_iso;
use crate::simulation::core::{
    app_state::SimulationSet, events::BevyMeasurementMessage, prng::SimulationRng,
    topics::GroundTruthState,
};

// --- Core Library Imports ---
use helios_core::{
    messages::{MeasurementData, MeasurementMessage},
    models::estimation::measurement::{magnetometer::MagnetometerModel, Measurement},
    types::FrameHandle,
};

// =========================================================================
// == Magnetometer Components & Plugin ==
// =========================================================================

#[derive(Component)]
pub struct Magnetometer {
    pub timer: Timer,
    // Store noise distributions for each axis
    noise_dist_x: Normal<f64>,
    noise_dist_y: Normal<f64>,
    noise_dist_z: Normal<f64>,
}

pub struct MagnetometerPlugin;

impl Plugin for MagnetometerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_magnetometer_sensors.in_set(SceneBuildSet::ProcessSensors),
        )
        .add_systems(
            FixedUpdate,
            magnetometer_sensor_system.in_set(SimulationSet::Sensors),
        );
    }
}

// =========================================================================
// == Spawning System ==
// =========================================================================

fn spawn_magnetometer_sensors(
    mut commands: Commands,
    request_query: Query<(Entity, &Name, &SpawnAgentConfigRequest)>,
) {
    for (agent_entity, agent_name, request) in &request_query {
        for (sensor_name, sensor_config) in &request.0.sensors {
            if let SensorConfig::Magnetometer(mag_config) = sensor_config {
                info!(
                    "  -> Spawning Magnetometer '{}' as child of agent '{}'",
                    sensor_name,
                    agent_name.as_str()
                );

                // --- 1. Create the `helios_core` Measurement Model ---
                let r_matrix = DMatrix::from_diagonal(&nalgebra::DVector::from_vec(vec![
                    mag_config.noise_stddev[0].powi(2) as f64,
                    mag_config.noise_stddev[1].powi(2) as f64,
                    mag_config.noise_stddev[2].powi(2) as f64,
                ]));

                // Define the world's magnetic field in ENU (points North, along +Y).
                let world_magnetic_field = Vector3::new(0.0, 1.0, 0.0).normalize();

                let mut core_model = MagnetometerModel {
                    agent_handle: FrameHandle::from_entity(agent_entity),
                    sensor_handle: FrameHandle(0), // Placeholder
                    r_matrix,
                    world_magnetic_field,
                };

                // --- 2. Spawn the Sensor Entity ---
                let mut sensor_entity_commands = commands.spawn_empty();
                let sensor_entity = sensor_entity_commands.id();

                core_model.sensor_handle = FrameHandle::from_entity(sensor_entity);

                sensor_entity_commands.insert((
                    Name::new(mag_config.name.clone()),
                    Magnetometer {
                        timer: Timer::new(
                            Duration::from_secs_f32(1.0 / mag_config.rate),
                            TimerMode::Repeating,
                        ),
                        noise_dist_x: Normal::new(0.0, mag_config.noise_stddev[0] as f64).unwrap(),
                        noise_dist_y: Normal::new(0.0, mag_config.noise_stddev[1] as f64).unwrap(),
                        noise_dist_z: Normal::new(0.0, mag_config.noise_stddev[2] as f64).unwrap(),
                    },
                    MeasurementModel(Box::new(core_model)),
                    TrackedFrame,
                    mag_config.get_relative_pose().to_bevy_transform(),
                ));

                commands.entity(agent_entity).add_child(sensor_entity);
            }
        }
    }
}

// =========================================================================
// == Runtime System ==
// =========================================================================

fn magnetometer_sensor_system(
    mut measurement_writer: EventWriter<BevyMeasurementMessage>,
    time: Res<Time>,
    mut rng: ResMut<SimulationRng>,
    parent_query: Query<(Entity, &GroundTruthState, &Children)>,
    mut sensor_query: Query<(Entity, &mut Magnetometer, &GlobalTransform)>,
) {
    let dt = time.delta();
    // Re-use the world magnetic field definition.
    let world_magnetic_field_enu = Vector3::new(0.0, 1.0, 0.0).normalize();

    for (agent_entity, ground_truth, children) in &parent_query {
        for &child_entity in children {
            if let Ok((sensor_entity, mut mag, sensor_global_transform)) =
                sensor_query.get_mut(child_entity)
            {
                mag.timer.tick(dt);
                if !mag.timer.just_finished() {
                    continue;
                }

                // --- Simulate Magnetometer Measurement ---
                let sensor_pose_enu = bevy_global_transform_to_enu_iso(sensor_global_transform);
                let q_sensor_from_world = sensor_pose_enu.rotation.inverse();

                // Rotate the true world magnetic field into the sensor's local frame.
                let perfect_mag_reading = q_sensor_from_world * world_magnetic_field_enu;

                // Add noise.
                let noisy_mag_reading = Vector3::new(
                    perfect_mag_reading.x + mag.noise_dist_x.sample(&mut rng.0),
                    perfect_mag_reading.y + mag.noise_dist_y.sample(&mut rng.0),
                    perfect_mag_reading.z + mag.noise_dist_z.sample(&mut rng.0),
                );

                // Create and send the message.
                let pure_message = MeasurementMessage {
                    agent_handle: FrameHandle::from_entity(agent_entity),
                    sensor_handle: FrameHandle::from_entity(sensor_entity),
                    timestamp: time.elapsed_secs_f64(),
                    data: MeasurementData::Magnetometer(noisy_mag_reading),
                };
                measurement_writer.write(BevyMeasurementMessage(pure_message));
            }
        }
    }
}
