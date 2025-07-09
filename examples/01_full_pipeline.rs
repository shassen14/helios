// examples/01_full_pipeline.rs

use avian3d::prelude::{AngularVelocity, LinearVelocity};
use avian3d::{PhysicsPlugins, prelude::PhysicsDebugPlugin};
use bevy::log::LogPlugin;
use bevy::prelude::*;
use nalgebra::Vector3;
// use rust_robotics::simulation::plugins::world::test_environment::TestEnvironmentPlugin;
use rust_robotics::simulation::core::app_state::AppState;
use rust_robotics::simulation::core::simulation_setup::SimulationSetupPlugin;
use rust_robotics::simulation::plugins::world::spawner::check_for_asset_load;
use rust_robotics::simulation::utils::transforms::bevy_transform_to_nalgebra_isometry;
use rust_robotics::{
    prelude::*,
    simulation::plugins::vehicles::ackermann::{AckermannCarPlugin, VehicleControllerInput},
}; // Use our library's prelude for easy access // Import our new state

use std::fs;

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
        level: bevy::log::Level::WARN,

        // This is a powerful filter string.
        // It says: "Default to info level, but for our crate `rust_robotics`,
        // show me `debug` level messages and higher. Also, quiet down the noisy
        // `wgpu` rendering logs to only show errors."
        filter: "info,rust_robotics=debug,wgpu_core=error,wgpu_hal=error".to_string(),
        ..Default::default()
    }))
    .add_plugins(PhysicsPlugins::default()) // Adds required physics systems and resources
    .add_plugins(PhysicsDebugPlugin::default())
    .insert_resource(Time::<Fixed>::from_hz(200.0))
    .insert_resource(config)
    .init_resource::<TopicBus>()
    .init_state::<AppState>();

    // --- 3. Add Simulation Plugins ---
    // Your plugins no longer add systems to `Startup` or `Update` directly.
    // They will now be configured below to run in specific states.
    app.add_plugins((
        SimulationSetupPlugin,
        WorldSpawnerPlugin,
        // TestEnvironmentPlugin,
        AckermannCarPlugin,
        ImuPlugin,
        EkfPlugin,
    ));

    // --- 4. SCHEDULE ALL SYSTEMS INTO STATES ---
    // This is where we orchestrate the entire loading and simulation flow.

    // --- Loading Phase ---
    // This system runs on the Update schedule, but ONLY IF the app is in the
    // AssetLoading state. This is the modern replacement for `OnUpdate`.
    app.add_systems(
        Update,
        check_for_asset_load.run_if(in_state(AppState::AssetLoading)),
    );

    app.add_systems(FixedUpdate, move_ground_truth_system)
        .add_systems(Update, keyboard_controller);

    // --- 6. Run the app ---
    app.run();
}

/// This system runs every fixed physics step to synchronize our custom `GroundTruthState`
/// component with the "source of truth" state from the Avian physics engine.
/// It also calculates derived values like linear acceleration.
pub fn move_ground_truth_system(
    // Query for all entities that have a ground truth state and the relevant physics components.
    mut query: Query<(
        &mut GroundTruthState,
        &Transform,
        &LinearVelocity,
        &AngularVelocity,
    )>,
    // The Fixed time resource is crucial for correct derivative calculations.
    time: Res<Time<Fixed>>,
) {
    let dt = time.delta_secs_f64();
    // A guard to prevent division by zero if the simulation is paused or on the first frame.
    if dt == 0.0 {
        return;
    }

    for (mut state, transform, lin_vel, ang_vel) in query.iter_mut() {
        // --- 1. Synchronize Pose ---
        // Convert the Bevy Transform to a nalgebra Isometry3 for use in our math/logic code.
        // This is the most direct way to mirror the entity's position and orientation.
        state.pose = bevy_transform_to_nalgebra_isometry(transform);

        // --- 2. Synchronize Velocities ---
        // Copy the velocities directly from the physics components, casting f32 to f64.
        let current_linear_velocity =
            Vector3::new(lin_vel.x as f64, lin_vel.y as f64, lin_vel.z as f64);
        state.linear_velocity = current_linear_velocity;
        state.angular_velocity = Vector3::new(ang_vel.x as f64, ang_vel.y as f64, ang_vel.z as f64);

        // --- 3. Calculate Linear Acceleration ---
        // Use the finite difference formula: a = (v_current - v_previous) / dt
        // This gives us the average acceleration over the last time step.
        state.linear_acceleration = (current_linear_velocity - state.last_linear_velocity) / dt;

        // --- 4. Update internal state for the next frame ---
        // This is CRITICAL. The current velocity becomes the next frame's previous velocity.
        state.last_linear_velocity = current_linear_velocity;

        // if state.linear_acceleration.norm() > 2.0 {
        //     println!("state: {:?}", state.pose);
        // }
    }
}

/// A simple system for testing. Press arrow keys to drive the car.
fn keyboard_controller(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut VehicleControllerInput>,
) {
    for mut controller in query.iter_mut() {
        controller.linear_acceleration = 0.0;
        controller.steering_angle = 0.0;

        if keyboard_input.pressed(KeyCode::ArrowUp) {
            controller.linear_acceleration = 5.0; // Apply 5 m/s^2 acceleration
        }
        if keyboard_input.pressed(KeyCode::ArrowDown) {
            controller.linear_acceleration = -5.0; // Apply -5 m/s^2 acceleration
        }
        if keyboard_input.pressed(KeyCode::ArrowLeft) {
            controller.steering_angle = 0.7; // Radians
        }
        if keyboard_input.pressed(KeyCode::ArrowRight) {
            controller.steering_angle = -0.7; // Radians
        }
        // println!("controller: {:?}", controller);
    }
}
