use bevy::prelude::*;
use nalgebra::{DMatrix, Isometry3, Translation3, UnitQuaternion, Vector3};
use rust_robotics::{
    prelude::*,
    simulation::plugins::{
        estimation::ekf::ImuSubscriptions,
        sensors::imu::{Imu, ImuSuite},
    },
}; // Use our library's prelude for easy access
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
        println!("[SETUP] Spawning agent: {}", agent_config.name);

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

        let agent_entity_id = commands
            .spawn((
                Name::new(agent_config.name.clone()),
                GroundTruthState {
                    pose: start_isometry,
                    linear_velocity: Vector3::zeros(),
                    angular_velocity: Vector3::new(0.0, 0.2, 0.0),
                },
                // DynamicsModel(Box::new(SimpleCarDynamics)),
                // We can add the EkfState here since every EKF will need one.
                EkfState::default(),
            ))
            .id();

        let mut imu_suite = ImuSuite::default();
        for (name, imu_config) in &agent_config.sensors.imu {
            // Use a match statement to handle different IMU types from the config.
            match imu_config {
                ImuConfig::NineDof {
                    rate,
                    noise_stddev: _,
                    frame_id: _,
                } => {
                    let topic_name = format!("/agents/{}/imu/{}", agent_config.name, name);
                    println!(
                        "    + Attaching IMU: '{}' to publish to {}",
                        name, topic_name
                    );

                    // Create an ImuSpec and push it into our suite's vector.
                    imu_suite.imus.push(Imu {
                        name: name.clone(),
                        topic_to_publish: topic_name,
                        timer: Timer::new(
                            Duration::from_secs_f32(1.0 / *rate),
                            TimerMode::Repeating,
                        ),
                        // You would use noise_stddev here to configure the sensor model
                    });
                }
            }
        }
        // Insert the single, fully-populated ImuSuite component onto the agent.
        commands.entity(agent_entity_id).insert(imu_suite);

        // ... (You would repeat the suite pattern for GPS, Lidar, etc.) ...

        // --- Attach Autonomy Stack Components & Configure Subscriptions ---

        // Use a match statement to handle different estimator types.
        match &agent_config.autonomy_stack.estimator {
            EstimatorConfig::Ekf { rate } => {
                println!("    + Attaching EKF at {} Hz", rate);

                // Add the core EKF component.
                commands.entity(agent_entity_id).insert(EKF {
                    timer: Timer::new(Duration::from_secs_f32(1.0 / *rate), TimerMode::Repeating),
                    // In a real system, you would populate these from the config.
                    // process_noise_covariance: DMatrix::identity(6, 6) * 0.01,
                    // measurement_noise_map: HashMap::new(), // This would be populated based on sensor configs
                });

                // --- DYNAMIC SUBSCRIPTION for IMU ---
                let mut imu_subscriptions = ImuSubscriptions::default();
                let imu_topic_names =
                    topic_bus.find_topics_by_tag(TopicTag::Imu, &agent_config.name);
                for topic_name in imu_topic_names {
                    println!("    -> EKF subscribing to IMU topic: {}", topic_name);
                    imu_subscriptions
                        .readers
                        .push(TopicReader::<ImuData>::new(&topic_name));
                }
                commands.entity(agent_entity_id).insert(imu_subscriptions);

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
