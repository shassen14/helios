// src/simulation/core/simulation_setup.rs

use std::collections::HashMap;
use std::time::Duration;

use crate::prelude::{
    AppState, EKF, EstimatorConfig, GroundTruthState, ImuConfig, ImuData, TopicReader, TopicTag,
    Vehicle,
};
use crate::simulation::core::app_state::SceneBuildSet;
use crate::simulation::core::config::SimulationConfig;
use crate::simulation::core::prng::SimulationRng;
use crate::simulation::core::spawn_requests::SpawnRequestAckermann;
use crate::simulation::core::topics::TopicBus;
use crate::simulation::plugins::estimation::ekf::ImuSubscriptions;
use crate::simulation::plugins::sensors::imu::{Imu, ImuModel, ImuSuite};
use crate::simulation::plugins::vehicles::ackermann::AckermannParameters;
use bevy::prelude::*;
use nalgebra::{DMatrix, DVector, Isometry3, Translation3, UnitQuaternion, Vector3};
use rand::SeedableRng;
use rand::rngs::OsRng;

use rand_chacha::ChaCha8Rng;

pub struct SimulationSetupPlugin;

impl Plugin for SimulationSetupPlugin {
    fn build(&self, app: &mut App) {
        // This plugin's job is to read the config and add resources and startup systems.
        let config = app
            .world()
            .get_resource::<SimulationConfig>()
            .expect("SimulationConfig not found!");

        // --- 1. Add the Deterministic PRNG Resource ---
        let rng = match config.simulation.seed {
            Some(seed) => ChaCha8Rng::seed_from_u64(seed),
            None => ChaCha8Rng::from_rng(&mut OsRng).expect("OS RNG failed"),
        };
        app.insert_resource(SimulationRng(rng));

        // --- 2. Initialize other core resources ---
        app.init_resource::<TopicBus>();

        app.configure_sets(
            OnEnter(AppState::SceneBuilding),
            (
                SceneBuildSet::Logic,
                SceneBuildSet::Physics,
                SceneBuildSet::Validation,
            )
                .chain(), // .chain() enforces the sequence.
        );

        // --- 3. Add the startup systems that depend on the config ---
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            (
                // --- Add Logic Set Systems ---
                (create_topics_from_config, spawn_agents_from_config).in_set(SceneBuildSet::Logic),
                // --- Add Validation Set Systems ---
                // We'll create a placeholder for this system for now.
                validate_agent_pipelines.in_set(SceneBuildSet::Validation),
                // --- Add the final transition system ---
                // This will run after all the sets in the chain are complete.
                transition_to_running_state.after(SceneBuildSet::Validation),
            ),
        );
    }
}

// You can move the create_topics_... and spawn_agents_... systems into this file.
// They will need to take `Res<SimulationConfig>` as an argument.

/// STARTUP PASS 1: Read the config and create all necessary topics on the bus.
fn create_topics_from_config(config: Res<SimulationConfig>, mut topic_bus: ResMut<TopicBus>) {
    for agent_config in &config.agents {
        // Create topics for IMUs
        for name in agent_config.sensors.imu.keys() {
            let topic_name = format!("/agents/{}/imu/{}", agent_config.name, name);
            info!("[SETUP] Creating topic: {}", topic_name);
            topic_bus.create_topic::<ImuData>(
                &topic_name,
                1000,
                TopicTag::Imu,
                agent_config.name.clone(),
            );
        }

        // ... create topics for GPS, Lidar, etc. here ...
    }
}

fn spawn_agents_from_config(
    mut commands: Commands,
    config: Res<SimulationConfig>,
    topic_bus: Res<TopicBus>, // Read-only access to find topics
) {
    for agent_config in &config.agents {
        info!(
            "[SPAWN PASS 1] Creating LOGICAL agent shell: {}",
            &agent_config.name
        );

        // --- Calculate Initial Logical Pose ---
        let start_pos = agent_config.starting_pose.position;
        let start_rot_deg = agent_config.starting_pose.orientation_deg;
        let start_isometry_enu = Isometry3::from_parts(
            Translation3::new(
                start_pos[0] as f64,
                start_pos[1] as f64,
                start_pos[2] as f64,
            ),
            UnitQuaternion::from_euler_angles(
                start_rot_deg[0].to_radians() as f64,
                start_rot_deg[1].to_radians() as f64,
                start_rot_deg[2].to_radians() as f64,
            ),
        );

        // --- Spawn the Logical Entity Shell ---
        // This entity has a name and a logical state, but no physical form yet.
        let mut agent_entity_commands = commands.spawn((
            Name::new(agent_config.name.clone()),
            GroundTruthState {
                pose: start_isometry_enu,
                linear_velocity: Vector3::zeros(),
                angular_velocity: Vector3::zeros(),
                linear_acceleration: Vector3::zeros(),
                last_linear_velocity: Vector3::zeros(),
            },
        ));

        // --- Attach Vehicle-Specific Parameters and Spawn Request ---
        match &agent_config.vehicle {
            Vehicle::Ackermann {
                wheelbase,
                max_steering_angle,
                max_steering_rate: _, // Not used in this component, but could be
            } => {
                info!("    -> Attaching AckermannParameters and SpawnRequest");

                // Attach the static parameters component.
                agent_entity_commands.insert(AckermannParameters {
                    wheelbase: *wheelbase as f64,
                    max_steering_angle: max_steering_angle.to_radians(), // Convert to radians now
                    max_force: 10000.0, // These could come from config later
                    max_torque: 5000.0,
                });

                // Attach the tag component that requests a physical body.
                agent_entity_commands.insert(SpawnRequestAckermann);
            }
            Vehicle::Quadcopter { .. } => {
                // Future implementation would go here
                // agent_entity_commands.insert(QuadcopterParameters { ... });
                // agent_entity_commands.insert(SpawnRequestQuadcopter);
            }
        }

        let mut imu_suite = ImuSuite::default();
        for (name, imu_config) in &agent_config.sensors.imu {
            // Use a match statement to handle different IMU types from the config.
            match imu_config {
                ImuConfig::SixDof {
                    rate,
                    accel_noise_stddev,
                    gyro_noise_stddev,
                    ..
                } => {
                    let topic_name = format!("/agents/{}/imu/{}", agent_config.name, name);
                    imu_suite.imus.push(Imu {
                        name: name.clone(),
                        topic_to_publish: topic_name,
                        timer: Timer::new(
                            Duration::from_secs_f32(1.0 / *rate),
                            TimerMode::Repeating,
                        ),
                        model: ImuModel::SixDof {
                            accel_noise_stddev: accel_noise_stddev.map(|v| v as f64),
                            gyro_noise_stddev: gyro_noise_stddev.map(|v| v as f64),
                        },
                    });
                }
                ImuConfig::NineDof {
                    rate,
                    accel_noise_stddev,
                    gyro_noise_stddev,
                    mag_noise_stddev,
                    ..
                } => {
                    let topic_name = format!("/agents/{}/imu/{}", agent_config.name, name);
                    // Here you would define a different topic, e.g., /sensor/imu/data_with_mag
                    // and a new message type that includes the magnetometer reading.
                    imu_suite.imus.push(Imu {
                        name: name.clone(),
                        topic_to_publish: topic_name,
                        timer: Timer::new(
                            Duration::from_secs_f32(1.0 / *rate),
                            TimerMode::Repeating,
                        ),
                        model: ImuModel::NineDof {
                            accel_noise_stddev: accel_noise_stddev.map(|v| v as f64),
                            gyro_noise_stddev: gyro_noise_stddev.map(|v| v as f64),
                            mag_noise_stddev: mag_noise_stddev.map(|v| v as f64),
                        },
                    });
                }
            }
        }
        // Insert the single, fully-populated ImuSuite component onto the agent.
        agent_entity_commands.insert(imu_suite);

        // ... (You would repeat the suite pattern for GPS, Lidar, etc.) ...

        // --- Attach Autonomy Stack Components & Configure Subscriptions ---

        // Use a match statement to handle different estimator types.

        if let Some(estimator_config) = &agent_config.autonomy_stack.estimator {
            match &estimator_config {
                EstimatorConfig::Ekf { rate } => {
                    info!("    + Attaching EKF at {} Hz", rate);
                    // 1. Create the HashMap for measurement noise.
                    // 1. Create the HashMap for measurement noise.
                    // The key is the topic name (e.g., "/agents/MyCar/imu/main_imu"),
                    // and the value is the corresponding measurement noise covariance matrix (R).
                    let mut measurement_noise_map = HashMap::new();

                    // 2. Iterate through the configured IMU sensors for this agent to build the noise map.
                    for (name, imu_config) in &agent_config.sensors.imu {
                        // Construct the topic name exactly as we do for the publisher.
                        let topic_name = format!("/agents/{}/imu/{}", agent_config.name, name);

                        // The measurement for an IMU consists of [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z].
                        // Therefore, the R matrix is 6x6. We assume noise between axes is uncorrelated,
                        // so we can build a diagonal matrix from the variances (stddev^2).
                        let r_matrix = match imu_config {
                            // Handle both types of IMU config.
                            ImuConfig::SixDof {
                                accel_noise_stddev,
                                gyro_noise_stddev,
                                ..
                            }
                            | ImuConfig::NineDof {
                                accel_noise_stddev,
                                gyro_noise_stddev,
                                ..
                            } => {
                                // Combine the noise standard deviations into one array.
                                let all_stddevs = [
                                    accel_noise_stddev[0],
                                    accel_noise_stddev[1],
                                    accel_noise_stddev[2],
                                    gyro_noise_stddev[0],
                                    gyro_noise_stddev[1],
                                    gyro_noise_stddev[2],
                                ];

                                // Create a diagonal matrix of variances.
                                DMatrix::from_diagonal(&DVector::from_iterator(
                                    6,
                                    all_stddevs.iter().map(|&stddev| (stddev as f64).powi(2)),
                                ))
                            }
                        };

                        // Insert the calculated R matrix into the map for this sensor's topic.
                        measurement_noise_map.insert(topic_name, r_matrix);
                    }

                    // You would repeat the process here for GPS, etc., adding their R matrices to the map.

                    // Add the core EKF component to the agent entity.
                    agent_entity_commands.insert(EKF {
                        // In a real system, Q would be configured in the TOML.
                        process_noise_covariance: DMatrix::identity(9, 9) * 0.01,
                        // The fully populated measurement noise map.
                        measurement_noise_map,
                    });

                    // --- DYNAMIC SUBSCRIPTION for IMU ---
                    // This part remains the same.
                    let mut imu_subscriptions = ImuSubscriptions::default();
                    let imu_topic_names =
                        topic_bus.find_topics_by_tag(TopicTag::Imu, &agent_config.name);
                    for topic_name in imu_topic_names {
                        info!("    -> EKF subscribing to IMU topic: {}", topic_name);
                        imu_subscriptions
                            .readers
                            .push(TopicReader::<ImuData>::new(&topic_name));
                    }
                    agent_entity_commands.insert(imu_subscriptions);

                    // --- DYNAMIC SUBSCRIPTION for GPS (Example) ---
                    // let mut gps_subscriptions = GpsSubscriptions::default();
                    // let gps_topic_names = topic_bus.find_topics_by_tag(TopicTag::Gps, &agent_config.name);
                    // for topic_name in gps_topic_names {
                    //     gps_subscriptions.readers.push(TopicReader::<GpsData>::new(&topic_name));
                    // }
                    // commands.entity(agent_entity_id).insert(gps_subscriptions);
                } // Add other estimator types here in the future
                  // EstimatorConfig::OtherFilter { .. } => { /* ... */ }
            }
        }
    }
}

/// This simple system runs once at the end of the `OnEnter(SceneBuilding)` chain.
/// Its only job is to move the app into the main `Running` state.
fn transition_to_running_state(mut next_state: ResMut<NextState<AppState>>) {
    info!("Scene building complete. Transitioning to Running state.");
    next_state.set(AppState::Running);
}

fn validate_agent_pipelines() {
    // Placeholder for your manifest validation logic.
    info!("[VALIDATION] Checking agent pipeline configurations...");
    // In the future, this system will query for LocalManifests and check for inconsistencies.
}
