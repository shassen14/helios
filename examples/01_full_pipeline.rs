use avian3d::{PhysicsPlugins, prelude::PhysicsDebugPlugin};
use bevy::log::LogPlugin;
use bevy::prelude::*;
use nalgebra::{DMatrix, DVector, Isometry3, Translation3, UnitQuaternion, Vector3};
// use rust_robotics::simulation::plugins::world::test_environment::TestEnvironmentPlugin;
use rust_robotics::{
    prelude::*,
    simulation::{
        core::dynamics::{DynamicsModel, SimpleCarDynamics},
        plugins::{
            estimation::ekf::ImuSubscriptions,
            sensors::imu::{Imu, ImuSuite},
            vehicles::ackermann::{AckermannCarPlugin, AckermannConfig, VehicleControllerInput},
        },
    },
}; // Use our library's prelude for easy access
use std::{collections::HashMap, fs, time::Duration};

fn main() {
    // --- 1. Load Configuration ---
    // In a real app, you might get this path from command-line arguments.
    let scenario_path = "assets/scenarios/simple_car_scenario.toml";

    // Attempt to load from file, otherwise use a default config for easy testing.
    let config: SimulationConfig = match fs::read_to_string(scenario_path) {
        Ok(toml_string) => {
            info!("Successfully loaded scenario from '{}'", scenario_path);
            toml::from_str(&toml_string).expect("Failed to parse scenario.toml!")
        }
        Err(_) => {
            error!("Could not find scenario file, using default configuration.");
            SimulationConfig::default()
        }
    };

    // Set up the Bevy App
    let mut app = App::new();

    // --- 2. Add Core Plugins & Resources ---
    app.add_plugins(DefaultPlugins.set(LogPlugin {
        // This is the minimum level of log that will be processed.
        // `Info` is a good default.
        level: bevy::log::Level::DEBUG,

        // This is a powerful filter string.
        // It says: "Default to info level, but for our crate `rust_robotics`,
        // show me `debug` level messages and higher. Also, quiet down the noisy
        // `wgpu` rendering logs to only show errors."
        filter: "info,rust_robotics=debug,wgpu_core=error,wgpu_hal=error".to_string(),
        ..Default::default()
    }))
    .add_plugins(PhysicsPlugins::default()) // Adds required physics systems and resources
    .insert_resource(Time::<Fixed>::from_hz(200.0))
    // Initialize the TopicBus resource
    .init_resource::<TopicBus>()
    .insert_resource(config)
    // .add_plugins((ImuPlugin, EkfPlugin))
    .add_plugins(PhysicsDebugPlugin::default())
    .add_plugins((WorldSpawnerPlugin, AckermannCarPlugin))
    .add_systems(
        Startup,
        (create_topics_from_config, spawn_agents_from_config).chain(),
    )
    .add_systems(FixedUpdate, move_ground_truth_system)
    .add_systems(Update, keyboard_controller)
    // .add_systems(Update, count_matching_vehicles)
    .add_systems(Update, log_vehicle_transform);

    // --- 3. Run the app ---
    app.run();
}

#[derive(Component)]
struct NeedsCollider;

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

/// STARTUP PASS 2: Spawn entities and configure them to use the topics.
fn spawn_agents_from_config(
    mut commands: Commands,
    config: Res<SimulationConfig>,
    topic_bus: Res<TopicBus>, // Read-only access to find topics
) {
    for agent_config in &config.agents {
        info!("[SETUP] Spawning agent: {}", agent_config.name);

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
                DynamicsModel(Box::new(SimpleCarDynamics)),
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
                    info!(
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
                info!("    + Attaching EKF at {} Hz", rate);
                // 1. Create the HashMap for measurement noise.
                let mut measurement_noise_map = HashMap::new();

                // 2. Iterate through the configured IMU sensors for this agent.
                for (name, imu_config) in &agent_config.sensors.imu {
                    match imu_config {
                        ImuConfig::NineDof { noise_stddev, .. } => {
                            // 3. Construct the topic name, just like we do for the publisher.
                            let topic_name = format!("/agents/{}/imu/{}", agent_config.name, name);

                            // 4. Create the noise matrix (R) for this specific sensor.
                            //    Assuming the measurement is 3D angular velocity.
                            let r_matrix = DMatrix::from_diagonal(&DVector::from_row_slice(&[
                                noise_stddev[0].powi(2) as f64, // Variance is stddev^2
                                noise_stddev[1].powi(2) as f64,
                                noise_stddev[2].powi(2) as f64,
                            ]));

                            // 5. Insert it into the map.
                            measurement_noise_map.insert(topic_name, r_matrix);
                        }
                    }
                }

                // Add the core EKF component.
                commands.entity(agent_entity_id).insert(EKF {
                    // timer: Timer::new(Duration::from_secs_f32(1.0 / *rate), TimerMode::Repeating),
                    // In a real system, you would populate these from the config.
                    process_noise_covariance: DMatrix::identity(9, 9) * 0.01,
                    measurement_noise_map: measurement_noise_map, // This would be populated based on sensor configs
                });

                // --- DYNAMIC SUBSCRIPTION for IMU ---
                let mut imu_subscriptions = ImuSubscriptions::default();
                let imu_topic_names =
                    topic_bus.find_topics_by_tag(TopicTag::Imu, &agent_config.name);
                for topic_name in imu_topic_names {
                    info!("    -> EKF subscribing to IMU topic: {}", topic_name);
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

/// A simple system for testing. Press arrow keys to drive the car.
fn keyboard_controller(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut VehicleControllerInput>,
) {
    for mut controller in query.iter_mut() {
        // Reset inputs
        controller.throttle = 0.0;
        controller.steering_angle = 0.0;

        if keyboard_input.pressed(KeyCode::ArrowUp) {
            // Use info! to log a significant event.
            info!("Up arrow pressed!");
            controller.throttle = 1.0;
        }
        if keyboard_input.pressed(KeyCode::ArrowDown) {
            info!("Down arrow pressed!");
            controller.throttle = -1.0;
        }
        if keyboard_input.pressed(KeyCode::ArrowLeft) {
            info!("Left arrow pressed!");
            controller.steering_angle = 0.7; // Increased for more visible turning
        }
        if keyboard_input.pressed(KeyCode::ArrowRight) {
            info!("Right arrow pressed!");
            controller.steering_angle = -0.7; // Increased for more visible turning
        }
    }
}

/// A debug system that prints the Transform of any entity with a VehicleControllerInput.
fn log_vehicle_transform(
    // We query for the vehicle's Transform and its controller input,
    // so we can see if the position changes when we press keys.
    query: Query<(&Transform, &VehicleControllerInput), With<AckermannConfig>>,
    // We use a local timer to only print every quarter of a second, to avoid spam.
) {
    let (transform, controller) = query
        .single()
        .expect("Query for vehicle failed, but counter found one!");

    // Check if there is any active input from the keyboard.
    if controller.throttle != 0.0 || controller.steering_angle != 0.0 {
        // This log should now appear every frame that you are holding down an arrow key.
        info!(
            "VEHICLE MOVING (theoretically): Transform={:?}, Throttle={}, Steering={}",
            transform.translation, controller.throttle, controller.steering_angle
        );
    }
}

/// A temporary debug system to diagnose why a query is failing.
fn count_matching_vehicles(
    // Use the EXACT same query as the system that is failing.
    query: Query<
        Entity,
        (
            With<Transform>,
            With<VehicleControllerInput>,
            With<AckermannConfig>,
        ),
    >,
) {
    // .iter().count() is a simple way to see how many entities match.
    let count = query.iter().count();

    // We will use a prominent WARN log so it's easy to see.
    warn!(
        "Found EXACTLY {} entities matching the log_vehicle_transform query.",
        count
    );
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
