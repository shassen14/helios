// src/simulation/plugins/sensors/imu.rs

use std::time::Duration;

use crate::{
    prelude::{AppState, ImuConfig, SensorConfig},
    simulation::core::{
        app_state::SceneBuildSet,
        prng::SimulationRng,
        simulation_setup::SpawnRequestSensorSuite,
        topics::{GroundTruthState, ImuData, TopicBus, TopicTag},
    },
};
use avian3d::prelude::Gravity;
use bevy::prelude::*;
use nalgebra::Vector3;
use rand_distr::{Distribution, Normal};

const WORLD_MAGNETIC_FIELD_VECTOR: Vector3<f64> = Vector3::new(0.0, 25.0, 0.0); // [East, North, Up] in microteslas

// --- Plugin Definition ---
pub struct ImuPlugin;

impl Plugin for ImuPlugin {
    fn build(&self, app: &mut App) {
        // This single system handles the creation of all IMU sensors.
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_imu_sensors.in_set(SceneBuildSet::ProcessRequests),
        );

        // This is the runtime system that makes the sensors work.
        app.add_systems(
            FixedUpdate,
            imu_sensor_system.run_if(in_state(AppState::Running)),
        );
    }
}

// Let's create an enum that mirrors the config structure.
// This will be the data stored in our component.
#[derive(Debug, Clone)]
pub enum ImuModel {
    SixDof {
        accel_noise_stddev: [f64; 3],
        gyro_noise_stddev: [f64; 3],
    },
    NineDof {
        accel_noise_stddev: [f64; 3],
        gyro_noise_stddev: [f64; 3],
        mag_noise_stddev: [f64; 3],
    },
}

#[derive(Component)]
pub struct Imu {
    pub name: String,
    pub topic_to_publish: String,
    pub timer: Timer,
    pub model: ImuModel,
}

// This is the SINGLE component added to an agent. It manages ALL IMUs for that agent.
// #[derive(Component, Default)]
// pub struct ImuSuite {
//     pub imus: Vec<Imu>,
// }

// --- Systems ---

fn spawn_imu_sensors(
    mut commands: Commands,
    mut topic_bus: ResMut<TopicBus>,
    // Query for agents that have a sensor suite request.
    request_query: Query<(Entity, &Name, &SpawnRequestSensorSuite)>,
) {
    for (agent_entity, agent_name, request) in &request_query {
        // We will add all IMU children to the agent entity.
        commands.entity(agent_entity).with_children(|parent| {
            // Iterate through the list of sensor configs from the TOML.
            for sensor_config in &request.0 {
                // This system only cares about the `Imu` variant.
                if let SensorConfig::Imu(imu_config) = sensor_config {
                    info!(
                        "  -> Spawning IMU '{}' as child of agent '{}'",
                        imu_config.get_name(),
                        agent_name
                    );

                    // 1. Create the topic for this specific sensor.
                    let topic_name = format!(
                        "/agent/{}/imu/{}",
                        agent_name.as_str(),
                        imu_config.get_name()
                    );
                    topic_bus.create_topic::<ImuData>(
                        &topic_name,
                        100,
                        TopicTag::Imu,
                        agent_name.as_str().to_string(),
                    );

                    // 2. Build the correct ImuModel from the config.
                    let imu_model = match imu_config {
                        ImuConfig::SixDof {
                            accel_noise_stddev,
                            gyro_noise_stddev,
                            ..
                        } => ImuModel::SixDof {
                            accel_noise_stddev: accel_noise_stddev.map(|v| v as f64),
                            gyro_noise_stddev: gyro_noise_stddev.map(|v| v as f64),
                        },
                        ImuConfig::NineDof {
                            accel_noise_stddev,
                            gyro_noise_stddev,
                            mag_noise_stddev,
                            ..
                        } => ImuModel::NineDof {
                            accel_noise_stddev: accel_noise_stddev.map(|v| v as f64),
                            gyro_noise_stddev: gyro_noise_stddev.map(|v| v as f64),
                            mag_noise_stddev: mag_noise_stddev.map(|v| v as f64),
                        },
                    };

                    // 3. Get the sensor's local transform relative to its parent.
                    let local_transform = imu_config.get_relative_pose().to_bevy_transform();

                    // 4. Spawn the child entity with all its components.
                    parent.spawn((
                        Name::new(format!("Sensor: {}", imu_config.get_name())),
                        Imu {
                            name: imu_config.get_name().to_string(),
                            topic_to_publish: topic_name,
                            timer: Timer::new(
                                Duration::from_secs_f32(1.0 / imu_config.get_rate()),
                                TimerMode::Repeating,
                            ),
                            model: imu_model,
                        },
                        // Attach the local transform. Bevy will automatically compute the GlobalTransform.
                        local_transform,
                    ));
                }
            }
        });
    }
}

fn imu_sensor_system(
    // --- QUERY 1: The "Parent" Query ---
    // We start by finding all agents that have children.
    // We get the data we need from the parent (GroundTruthState).
    agent_query: Query<(&GroundTruthState, &Children)>,

    // --- QUERY 2: The "Child" Query ---
    // This is a separate query that can access any IMU component in the world.
    // We will use the list of children from the first query to look up entities here.
    mut imu_query: Query<(&mut Imu, &GlobalTransform)>,

    // --- Resources ---
    mut topic_bus: ResMut<TopicBus>,
    time: Res<Time<Fixed>>,
    gravity: Res<Gravity>,
    mut rng: ResMut<SimulationRng>,
) {
    // 1. Iterate over each agent (the parent).
    for (ground_truth, children) in &agent_query {
        // `children` is a component containing a list of the agent's child entities.

        // 2. Iterate through the list of child entities for this specific agent.
        for &child_entity in children {
            // `child_entity` is the Entity ID of a sensor.

            // 3. Try to get the IMU components from this child entity using our second query.
            // `get_mut` will return Ok if this child is indeed an IMU, and Err otherwise.
            if let Ok((mut imu, _sensor_global_transform)) = imu_query.get_mut(child_entity) {
                // SUCCESS! We have found an IMU child.
                // At this point, we have everything we need:
                // - `ground_truth` (from the parent agent)
                // - `imu` (the mutable data for this specific sensor)
                // - `sensor_global_transform` (the world pose of this specific sensor)

                imu.timer.tick(time.delta());

                if imu.timer.just_finished() {
                    // The ground truth pose gives us the orientation of the body relative to the world.
                    // We need the inverse to transform vectors FROM the world frame TO the body frame.
                    let q_world_to_body = ground_truth.pose.rotation.inverse();

                    // --- 1. Calculate Angular Velocity (Gyroscope) ---
                    // A real gyro measures angular velocity in its own local reference frame.
                    // The physics engine provides it in the world frame, so we must rotate it.
                    let body_angular_velocity = q_world_to_body * ground_truth.angular_velocity;

                    // --- 2. Calculate Proper Acceleration (Accelerometer) ---
                    // An accelerometer measures the combination of kinematic acceleration and gravity.
                    // This is also measured in the sensor's local reference frame.

                    // First, transform the world-frame gravity vector into the body's frame.
                    let world_gravity_vector = Vector3::new(0.0, gravity.0.y as f64, 0.0);
                    let gravity_in_body_frame = q_world_to_body * world_gravity_vector;

                    // Second, transform the world-frame kinematic acceleration into the body's frame.
                    let kinematic_accel_in_body_frame =
                        q_world_to_body * ground_truth.linear_acceleration;

                    // Proper Acceleration = a_kinematic - g. This is what the accelerometer feels.
                    // For example, an object at rest (a=0) on a table feels an upward force, so it
                    // measures an upward acceleration of +g. Our formula: 0 - (-g) = +g. Correct.
                    let proper_acceleration = kinematic_accel_in_body_frame - gravity_in_body_frame;

                    // --- 3. Generate and Apply Noise ---
                    // We match on the IMU model to get the correct noise parameters.
                    let (noisy_acceleration, noisy_angular_velocity) = match &imu.model {
                        ImuModel::SixDof {
                            accel_noise_stddev,
                            gyro_noise_stddev,
                        } => {
                            // Create Gaussian distributions for the noise.
                            let accel_dist = Normal::new(0.0, 1.0).unwrap(); // Will be scaled by stddev
                            let gyro_dist = Normal::new(0.0, 1.0).unwrap();

                            // Generate noise vectors.
                            let accel_noise = Vector3::new(
                                accel_dist.sample(&mut rng.0) * accel_noise_stddev[0],
                                accel_dist.sample(&mut rng.0) * accel_noise_stddev[1],
                                accel_dist.sample(&mut rng.0) * accel_noise_stddev[2],
                            );
                            let gyro_noise = Vector3::new(
                                gyro_dist.sample(&mut rng.0) * gyro_noise_stddev[0],
                                gyro_dist.sample(&mut rng.0) * gyro_noise_stddev[1],
                                gyro_dist.sample(&mut rng.0) * gyro_noise_stddev[2],
                            );

                            // Add noise to the perfect, body-frame values.
                            (
                                proper_acceleration + accel_noise,
                                body_angular_velocity + gyro_noise,
                            )
                        }
                        ImuModel::NineDof { .. } => {
                            // TODO: Implement magnetometer readings and noise for 9-DOF.
                            // For now, it behaves identically to the 6-DOF model.
                            // You would add a magnetometer noise model here.
                            let noisy_accel = proper_acceleration; // Placeholder
                            let noisy_gyro = body_angular_velocity; // Placeholder
                            (noisy_accel, noisy_gyro)
                        }
                    };

                    // --- 4. Construct and Publish the Message ---
                    let data = ImuData {
                        entity: child_entity,
                        sensor_name: imu.name.clone(),
                        timestamp: time.elapsed_secs_f64(),
                        acceleration: noisy_acceleration,
                        angular_velocity: noisy_angular_velocity,
                    };

                    // println!("{:?}", data);

                    topic_bus.publish(&imu.topic_to_publish, data);
                }
            }
        }
    }
}
