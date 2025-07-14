use avian3d::prelude::Gravity;
use bevy::prelude::*;
use nalgebra::{DMatrix, Vector3, Vector6};
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
    models::measurement::imu::Imu6DofModel, // Import our new core model
    types::FrameHandle,
};

// =========================================================================
// == IMU Components & Plugin ==
// =========================================================================

/// A Bevy component attached to a sensor entity, containing its runtime state.
#[derive(Component)]
pub struct Imu {
    pub timer: Timer,
    // We could add noise parameters here if we want them to be configurable per-sensor at runtime
    // pub accel_noise: Normal<f64>,
    // pub gyro_noise: Normal<f64>,
}

pub struct ImuPlugin;

impl Plugin for ImuPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            // The spawning system
            spawn_imu_sensors.in_set(SceneBuildSet::ProcessSensors),
        )
        .add_systems(
            FixedUpdate,
            // The runtime sensor simulation system
            imu_sensor_system.in_set(SimulationSet::Sensors),
        );
    }
}

// =========================================================================
// == Spawning System ==
// =========================================================================

/// Reads the config and spawns IMU sensor entities as children of the appropriate agent.
fn spawn_imu_sensors(
    mut commands: Commands,
    // We query for agents that have a spawn request.
    request_query: Query<(Entity, &Name, &SpawnAgentConfigRequest)>,
    // We need the gravity resource to properly configure the core model.
    gravity: Res<Gravity>,
) {
    for (agent_entity, agent_name, request) in &request_query {
        // Find any IMU configurations for this agent.
        for sensor_config in &request.0.sensors {
            if let SensorConfig::Imu(imu_config) = sensor_config {
                info!(
                    "  -> Spawning IMU '{}' as child of agent '{}'",
                    imu_config.get_name(),
                    agent_name.as_str()
                );

                // --- 1. Create the `helios_core` Model ---
                // This is the pure, framework-agnostic physics model for the sensor.
                let (accel_std, gyro_std) = imu_config.get_noise_stddevs();
                let r_matrix = DMatrix::from_diagonal(&nalgebra::DVector::from_vec(vec![
                    (accel_std[0] as f64).powi(2),
                    (accel_std[1] as f64).powi(2),
                    (accel_std[2] as f64).powi(2),
                    (gyro_std[0] as f64).powi(2),
                    (gyro_std[1] as f64).powi(2),
                    (gyro_std[2] as f64).powi(2),
                ]));

                // --- 2. Spawn the Sensor Entity as a Child ---
                let mut sensor_entity_commands = commands.spawn_empty();
                let sensor_entity = sensor_entity_commands.id();

                // Update the placeholder handle in the core model.
                let final_core_model = Imu6DofModel {
                    agent_handle: FrameHandle::from_entity(agent_entity),

                    sensor_handle: FrameHandle::from_entity(sensor_entity),
                    r_matrix,
                    gravity_magnitude: gravity.0.length() as f64,
                };

                // --- 3. Add all components to the new sensor entity ---
                sensor_entity_commands.insert((
                    Name::new(imu_config.get_name().to_string()),
                    // The Bevy component with runtime state.
                    Imu {
                        timer: Timer::new(
                            Duration::from_secs_f32(1.0 / imu_config.get_rate()),
                            TimerMode::Repeating,
                        ),
                    },
                    // The pure `helios_core` model, wrapped for use in Bevy.
                    MeasurementModel(Box::new(final_core_model)),
                    // Marker so the TF system can find it.
                    TrackedFrame,
                    // Its local transform relative to the parent (the agent).
                    imu_config.get_relative_pose().to_bevy_transform(),
                ));

                // --- 4. Add the sensor as a child of the agent ---
                commands.entity(agent_entity).add_child(sensor_entity);
            }
        }
    }
}

// =========================================================================
// == Runtime System ==
// =========================================================================

/// Runs every frame to simulate IMU physics and publish measurement messages.
fn imu_sensor_system(
    mut measurement_writer: EventWriter<BevyMeasurementMessage>,
    time: Res<Time>,
    mut rng: ResMut<SimulationRng>,
    gravity: Res<Gravity>, // The "true" gravity from the physics engine.

    // This is the modern Bevy pattern for parent-child relationships.
    // 1. Query for all parents (agents) that have children.
    parent_query: Query<(Entity, &GroundTruthState, &Children)>,
    // 2. A separate query to get the components of the children (sensors).
    mut sensor_query: Query<(Entity, &mut Imu, &GlobalTransform)>,
) {
    let dt = time.delta();

    for (agent_entity, ground_truth, children) in &parent_query {
        for &child_entity in children {
            if let Ok((sensor_entity, mut imu, sensor_global_transform)) =
                sensor_query.get_mut(child_entity)
            {
                imu.timer.tick(dt);
                if !imu.timer.just_finished() {
                    continue;
                }

                // --- 1. Get Sensor Orientation in ENU Frame ---
                let sensor_pose_enu = bevy_global_transform_to_enu_iso(sensor_global_transform);
                let rot_sensor_from_world = sensor_pose_enu.rotation;
                let q_sensor_from_world = rot_sensor_from_world.inverse();

                // --- 2. Calculate Proper Acceleration ---
                // The fundamental equation: ProperAccel = CoordAccel - Gravity

                // Get the vehicle's coordinate acceleration from GroundTruthState (already in ENU).
                let coord_accel_world_enu = ground_truth.linear_acceleration;

                // Define the gravity vector in the ENU world frame.
                let gravity_world_enu = Vector3::new(0.0, 0.0, gravity.0.y as f64); // gravity.0.y is -9.81

                // Calculate proper acceleration in the world frame.
                let proper_accel_world_enu = coord_accel_world_enu - gravity_world_enu;
                // For a stationary vehicle: [0,0,0] - [0,0,-9.81] = [0,0,+9.81]

                // --- 3. Express All Vectors in the Sensor's Frame ---
                // Rotate the calculated proper acceleration into the sensor's local frame.
                let perfect_accel_sensor_frame = q_sensor_from_world * proper_accel_world_enu;

                // Rotate the ground truth angular velocity into the sensor's local frame.
                let perfect_gyro_sensor_frame = q_sensor_from_world * ground_truth.angular_velocity;

                // --- 4. Create the Final Measurement Vector ---
                let z = Vector6::new(
                    perfect_accel_sensor_frame.x,
                    perfect_accel_sensor_frame.y,
                    perfect_accel_sensor_frame.z,
                    perfect_gyro_sensor_frame.x,
                    perfect_gyro_sensor_frame.y,
                    perfect_gyro_sensor_frame.z,
                );

                // --- 5. Create and Send the Message ---
                let pure_message = MeasurementMessage {
                    agent_handle: FrameHandle::from_entity(agent_entity),
                    sensor_handle: FrameHandle::from_entity(sensor_entity),
                    timestamp: time.elapsed_secs_f64(),
                    data: MeasurementData::Imu6Dof(z), // Add noise here later
                };

                measurement_writer.write(BevyMeasurementMessage(pure_message));
                // info!("IMU {:?} published measurement.", sensor_entity);
            }
        }
    }

    // Iterate over each parent agent that has children.
    // for (agent_entity, ground_truth, children) in &parent_query {
    //     // Iterate through the list of child entities for this specific agent.
    //     for &child_entity in children {
    //         // Use `get_mut` to check if this child is an IMU sensor we need to process.
    //         if let Ok((sensor_entity, mut imu, sensor_global_transform)) =
    //             sensor_query.get_mut(child_entity)
    //         {
    //             // Tick the sensor's timer.
    //             imu.timer.tick(dt);
    //             if !imu.timer.just_finished() {
    //                 continue; // Not time to publish yet.
    //             }

    //             // --- Simulate IMU Physics ---

    //             // 1. Get the sensor's true orientation in the ENU world frame.
    //             let sensor_pose_enu = bevy_global_transform_to_enu_iso(sensor_global_transform);
    //             let q_sensor_from_world = sensor_pose_enu.rotation.inverse();

    //             // 2. Calculate the "proper acceleration" that an IMU measures.
    //             // Proper_Accel = Coordinate_Accel - Gravity_Vector
    //             // All vectors here are in the ENU world frame.
    //             let gravity_enu = Vector3::new(0.0, 0.0, gravity.0.y as f64);
    //             let proper_accel_world = ground_truth.linear_acceleration - gravity_enu;

    //             // 3. Rotate world-frame vectors into the sensor's local frame.
    //             let perfect_accel = q_sensor_from_world * proper_accel_world;
    //             let perfect_gyro = q_sensor_from_world * ground_truth.angular_velocity;

    //             // 4. Combine into a 6D vector and add optional noise.
    //             let perfect_measurement = Vector6::new(
    //                 perfect_accel.x,
    //                 perfect_accel.y,
    //                 perfect_accel.z,
    //                 perfect_gyro.x,
    //                 perfect_gyro.y,
    //                 perfect_gyro.z,
    //             );
    //             // TODO: Add noise from a Normal distribution using `rng`.
    //             let noisy_measurement = perfect_measurement;

    //             // 5. Create the pure `MeasurementMessage` from `helios_core`.
    //             let pure_message = MeasurementMessage {
    //                 agent_handle: FrameHandle::from_entity(agent_entity),
    //                 sensor_handle: FrameHandle::from_entity(sensor_entity),
    //                 timestamp: time.elapsed_secs_f64(),
    //                 data: MeasurementData::Imu6Dof(noisy_measurement),
    //             };

    //             info!(
    //                 "IMU {:?} published measurement {:?}",
    //                 sensor_entity, &pure_message.data
    //             );
    //             // 6. Wrap it in the Bevy event and send it.
    //             measurement_writer.write(BevyMeasurementMessage(pure_message));
    //         }
    //     }
    // }
}
