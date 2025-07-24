// helios_sim/examples/01_full_pipeline.rs

//! A full end-to-end simulation example for the Helios robotics platform.
//!
//! This example demonstrates how to:
//! 1. Load a simulation scenario from a TOML file.
//! 2. Set up the core Bevy application and plugins.
//! 3. Add the main `HeliosSimulationPlugin` which contains all the robotics logic.
//! 4. Include example-specific helpers like a debug camera and a keyboard controller.
//!
//! To run this example:
//! `cargo run --example 01_full_pipeline`
//! `cargo run --example 01_full_pipeline -- --scenario assets/scenarios/simple_car_scenario.toml --headless`

use clap::Parser;
use helios_sim::cli::Cli;

// --- Bevy Imports ---
use avian3d::prelude::*;
use bevy::{log::LogPlugin, prelude::*};
use helios_sim::prelude::AppState;
use helios_sim::simulation::config::ConfigPlugin;

// --- Project-Specific Imports ---
// Import the main plugin from our simulation library crate.
use helios_sim::HeliosSimulationPlugin;
// Import the controller input component so our keyboard controller can use it.
use helios_sim::simulation::plugins::vehicles::ackermann::VehicleControllerInput;

fn main() {
    // --- 1. Obtain Simulation Configuration ---
    let cli = Cli::parse();

    let mut app = App::new();

    if cli.headless {
        // Run in headless mode: only add the core plugins needed for logic and physics.
        app.add_plugins((MinimalPlugins, AssetPlugin::default()));
        info!("Running in headless mode.");
    } else {
        // Run in graphical mode: add the default plugins for rendering, windowing, etc.
        app.add_plugins(
            DefaultPlugins.set(LogPlugin {
                level: bevy::log::Level::INFO,
                // A good filter for focusing on our crate's logs during development.
                filter: "info,wgpu_core=error,wgpu_hal=error,helios_sim=debug,helios_core=debug"
                    .to_string(),
                ..default()
            }),
        );
    }
    // --- 2. Add Core Bevy Plugins & Resources ---
    app
        // Bevy's default plugins provide the core engine functionality (rendering, windowing, etc.).
        // The Avian3D physics plugins.
        .add_plugins(PhysicsPlugins::default())
        // An Avian3D plugin to visualize colliders for debugging.
        .add_plugins(PhysicsDebugPlugin::default())
        // Insert the loaded configuration as a Bevy resource so all systems can access it.
        .insert_resource(cli.clone());

    app.init_state::<AppState>();

    app.add_plugins(ConfigPlugin);

    // --- 3. Add the Main Helios Simulation Plugin ---
    // This single line brings in our entire simulation architecture:
    // core setup, all vehicle/sensor/estimator plugins, and their systems.
    app.add_plugins(HeliosSimulationPlugin);

    // --- 4. Add Example-Specific Systems & Setup ---
    // These are helpers for this specific example, not part of the core library.
    app.add_systems(Update, keyboard_controller); // A system to drive the car with arrow keys

    // --- 5. Run the App ---
    println!("Starting Helios Simulation...");
    app.run();
}

/// A simple system that reads keyboard input and populates the `VehicleControllerInput`
/// component on any controllable vehicle.
fn keyboard_controller(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut VehicleControllerInput>,
) {
    // This will apply the same input to ALL entities that have a VehicleControllerInput.
    for mut controller in &mut query {
        // Reset inputs from the previous frame.
        controller.throttle = 0.0;
        controller.steering_angle = 0.0;

        // Set throttle based on up/down arrow keys.
        if keyboard_input.pressed(KeyCode::ArrowUp) {
            controller.throttle = 1.0; // Full throttle forward
        }
        if keyboard_input.pressed(KeyCode::ArrowDown) {
            controller.throttle = -1.0; // Full throttle reverse
        }

        // Set steering based on left/right arrow keys.
        if keyboard_input.pressed(KeyCode::ArrowLeft) {
            controller.steering_angle = 0.7; // Steer left (radians)
        }
        if keyboard_input.pressed(KeyCode::ArrowRight) {
            controller.steering_angle = -0.7; // Steer right (radians)
        }
    }
}
