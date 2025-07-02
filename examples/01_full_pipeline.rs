use bevy::prelude::*;
use nalgebra::{DMatrix, Isometry3, Translation3, UnitQuaternion, Vector3};
use rust_robotics::{prelude::*, simulation::plugins::sensors::imu::Imu}; // Use our library's prelude for easy access
use std::{fs, time::Duration};

fn main() {
    // --- 1. Load Configuration ---
    // In a real app, you might get this path from command-line arguments.
    let scenario_path = "assets/scenarios/simple_car_scenario.toml";

    // Attempt to load from file, otherwise use a default config for easy testing.
    let config: SimulationConfig = match fs::read_to_string(scenario_path) {
        Ok(toml_string) => {
            println!("Successfully loaded scenario from '{}'", scenario_path);
            toml::from_str(&toml_string).expect("Failed to parse scenario.toml!")
        }
        Err(_) => {
            println!("Could not find scenario file, using default configuration.");
            SimulationConfig::default()
        }
    };

    // Set up the Bevy App
    let mut app = App::new();

    // --- 2. Add Core Plugins & Resources ---
    app.add_plugins(DefaultPlugins)
        .insert_resource(Time::<Fixed>::from_hz(200.0))
        // Initialize the TopicBus resource
        .init_resource::<TopicBus>()
        .insert_resource(config)
        .add_plugins((ImuPlugin, EkfPlugin))
        // Pass 1: Create topics. Pass 2: Spawn entities.
        .add_systems(
            Startup,
            (create_topics_from_config, spawn_agents_from_config).chain(),
        )
        .add_systems(FixedUpdate, move_ground_truth_system);

    // --- 3. Run the app ---
    app.run();
}

/// STARTUP PASS 1: Read the config and create all necessary topics on the bus.
fn create_topics_from_config(config: Res<SimulationConfig>, mut topic_bus: ResMut<TopicBus>) {
    for agent_config in &config.agents {
        // Create topics for IMUs
        if let imu_suite = &agent_config.sensors.imu {
            for name in imu_suite.keys() {
                let topic_name = format!("/agents/{}/imu/{}", agent_config.name, name);
                println!("[SETUP] Creating topic: {}", topic_name);
                topic_bus.create_topic::<ImuData>(
                    &topic_name,
                    1000,
                    TopicTag::Imu,
                    agent_config.name.clone(),
                );
            }
        }
        // ... create topics for GPS, Lidar, etc. here ...
    }
}

/// STARTUP PASS 2: Spawn entities and configure them to use the topics.
fn spawn_agents_from_config(
    mut commands: Commands,
    config: Res<SimulationConfig>,
    topic_bus: Res<TopicBus>, // Read-only access to find topics
) {
    for agent_config in &config.agents {
        let start_pos = agent_config.starting_pose.position;
        let start_rot_deg = agent_config.starting_pose.orientation_deg;
        let start_isometry = Isometry3::from_parts(
            Translation3::new(
                start_pos[0] as f64,
                start_pos[1] as f64,
                start_pos[2] as f64,
            ),
            UnitQuaternion::from_euler_angles(
                start_rot_deg[0].to_radians() as f64, // Roll
                start_rot_deg[1].to_radians() as f64, // Pitch
                start_rot_deg[2].to_radians() as f64, // Yaw
            ),
        );

        let mut agent_entity_commands = commands.spawn((
            Name::new(agent_config.name.clone()),
            GroundTruthState {
                pose: start_isometry,
                linear_velocity: Vector3::zeros(),
                angular_velocity: Vector3::new(0.0, 0.2, 0.0), // Give it a slight spin for testing
            },
            EstimatedPose {
                pose: start_isometry,
                timestamp: 0.0,
                covariance: DMatrix::zeros(6, 6),
            },
            // DynamicsModel(Box::new(SimpleCarDynamics)),
        ));

        // --- Attach Sensor Components ---
        for (name, imu_config) in &agent_config.sensors.imu {
            // --- THE FIX IS HERE: Use a `match` statement ---
            // This is robust. If you later add another IMU type to the enum
            // (e.g., a simple 6-DOF one), the compiler will force you to handle it here.
            match imu_config {
                ImuConfig::NineDof {
                    rate,
                    noise_stddev: _,
                    frame_id: _,
                } => {
                    println!("    + Attaching EKF at {} Hz", rate);

                    let topic_name = format!("/agents/{}/imu/{}", agent_config.name, name);

                    // The component now has the publisher topic name
                    agent_entity_commands.insert(Imu {
                        name: name.clone(),
                        topic_to_publish: topic_name,
                        timer: Timer::new(
                            Duration::from_secs_f32(1.0 / *rate),
                            TimerMode::Repeating,
                        ),
                        // You can use the other config values here
                        // noise_stddev_accel: noise_stddev[0] as f64,
                        // noise_stddev_gyro: noise_stddev[1] as f64,
                    });
                } // If you had other IMU types, you'd handle them here:
                  // ImuConfig::OtherType { ... } => { /* logic for other type */ }
            }
        }
        // --- Attach Autonomy Stack & Configure Subscriptions ---
        if let EstimatorConfig::Ekf { rate } = &agent_config.autonomy_stack.estimator {
            // First, add the core EKF components
            let ekf_entity_id = agent_entity_commands
                .insert((
                    EKF {
                        timer: Timer::new(
                            Duration::from_secs_f32(1.0 / rate),
                            TimerMode::Repeating,
                        ),
                    },
                    EkfState::default(),
                ))
                .id();
            match &agent_config.autonomy_stack.estimator {
                EstimatorConfig::Ekf { rate } => {
                    println!("    + Attaching EKF at {} Hz", rate);
                    // The `insert` call is now inside the match arm.
                    agent_entity_commands.insert((
                        EKF {
                            timer: Timer::new(
                                Duration::from_secs_f32(1.0 / *rate),
                                TimerMode::Repeating,
                            ),
                        },
                        EkfState::default(),
                    ));
                } // If you were to add another estimator to your config enum, like:
                  // Ukf { rate: f32 },
                  // The compiler would give you an error right here until you add a new arm:
                  // EstimatorConfig::Ukf { rate } => {
                  //     println!("    + Attaching UKF at {} Hz", rate);
                  //     // ... insert UKF components ...
                  // }
            }

            // Now, discover and add the reader components to that same entity.
            let imu_topic_names = topic_bus.find_topics_by_tag(TopicTag::Imu, &agent_config.name);
            for topic_name in imu_topic_names {
                println!(
                    "[SETUP] EKF for {} subscribing to {}",
                    agent_config.name, topic_name
                );
                commands
                    .entity(ekf_entity_id)
                    .insert(TopicReader::<ImuData>::new(&topic_name));
            }
        }
    }
}

/// This system runs ONCE at startup to build the scene from the configuration.
// fn setup_scene(mut commands: Commands, config: Res<SimulationConfig>) {
//     println!("[SETUP] Spawning agents from configuration...");

//     for (i, agent_config) in config.agents.iter().enumerate() {
//         println!("  -> Spawning agent: {}", agent_config.name);

//         // --- Calculate Initial Pose ---
//         let start_pos = agent_config.starting_pose.position;
//         let start_rot_deg = agent_config.starting_pose.orientation_deg;
//         let start_isometry = Isometry3::from_parts(
//             Translation3::new(
//                 start_pos[0] as f64,
//                 start_pos[1] as f64,
//                 start_pos[2] as f64,
//             ),
//             UnitQuaternion::from_euler_angles(
//                 start_rot_deg[0].to_radians() as f64, // Roll
//                 start_rot_deg[1].to_radians() as f64, // Pitch
//                 start_rot_deg[2].to_radians() as f64, // Yaw
//             ),
//         );

//         // --- Start building the entity with commands ---
//         let mut agent_entity_commands = commands.spawn((
//             Name::new(agent_config.name.clone()),
//             // --- Ground Truth & Estimated State ---
//             GroundTruthState {
//                 pose: start_isometry,
//                 // These will be updated by the dynamics system.
//                 linear_velocity: Vector3::zeros(),
//                 angular_velocity: Vector3::new(0.0, 0.2, 0.0), // Give it a slight spin for testing
//             },
//             EstimatedPose {
//                 pose: start_isometry,
//                 timestamp: 0.0,
//                 covariance: DMatrix::zeros(6, 6),
//             },
//             // --- Vehicle Dynamics Model ---
//             // This part will become more complex as you read the `vehicle` type from config.
//             // DynamicsModel(Box::new(SimpleCarDynamics)),
//         ));

//         // --- Add Sensor Compone7nts from Config ---
//         // Iterate over named IMUs in the config for this agent
//         for (name, imu_config) in &agent_config.sensors.imu {
//             // Use a `match` statement to handle all possible variants of ImuConfig.
//             match imu_config {
//                 // This is the arm for the NineDof variant.
//                 ImuConfig::NineDof {
//                     rate, noise_stddev, ..
//                 } => {
//                     println!("    + Attaching IMU: '{}' at {} Hz", name, rate);
//                     // The `insert` call is now inside the match arm.
//                     agent_entity_commands.insert(Imu {
//                         name: name.clone(),
//                         noise_stddev_accel: noise_stddev[0] as f64,
//                         noise_stddev_gyro: noise_stddev[1] as f64,
//                         timer: Timer::new(
//                             Duration::from_secs_f32(1.0 / *rate),
//                             TimerMode::Repeating,
//                         ),
//                     });
//                 } // If you add another variant to ImuConfig, like `ImuConfig::OtherType`,
//                   // the compiler will force you to add a new arm here:
//                   // ImuConfig::OtherType { ... } => { /* handle it */ }
//             }
//         }

//         // Use a `match` statement to handle the estimator configuration.
//         match &agent_config.autonomy_stack.estimator {
//             EstimatorConfig::Ekf { rate } => {
//                 println!("    + Attaching EKF at {} Hz", rate);
//                 // The `insert` call is now inside the match arm.
//                 agent_entity_commands.insert((
//                     EKF {
//                         timer: Timer::new(
//                             Duration::from_secs_f32(1.0 / *rate),
//                             TimerMode::Repeating,
//                         ),
//                     },
//                     EkfState::default(),
//                 ));
//             } // If you were to add another estimator to your config enum, like:
//               // Ukf { rate: f32 },
//               // The compiler would give you an error right here until you add a new arm:
//               // EstimatorConfig::Ukf { rate } => {
//               //     println!("    + Attaching UKF at {} Hz", rate);
//               //     // ... insert UKF components ...
//               // }
//         }
//     }
// }

/// A simple system to move the ground truth state, simulating vehicle motion.
/// This will eventually be replaced by a proper physics/dynamics system.
fn move_ground_truth_system(mut query: Query<&mut GroundTruthState>, time: Res<Time<Fixed>>) {
    let dt = time.delta_secs_f64();
    for mut state in query.iter_mut() {
        // Simple Euler integration for demonstration
        let rotation = state.pose.rotation
            * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), state.angular_velocity.y * dt);
        state.pose.rotation = rotation;
    }
}

// /// A simple debug system to show the final estimated pose.
// fn print_estimated_pose_system(query: Query<(&Name, &EstimatedPose)>, time: Res<Time>) {
//     // Use a local timer to only print every second to avoid spamming the console.
//     let mut timer = Timer::from_seconds(1.0, TimerMode::Repeating);
//     if timer.tick(time.delta()).just_finished() {
//         for (name, pose) in query.iter() {
//             let t = pose.pose.translation.vector;
//             println!(
//                 "[OUTPUT] Agent: '{}', Estimated Pos: ({:.2}, {:.2}, {:.2})",
//                 name.as_str(),
//                 t.x,
//                 t.y,
//                 t.z
//             );
//         }
//     }
// }
