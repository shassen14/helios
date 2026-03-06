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
use helios_sim::HeliosSimulationPlugin;
// ControlOutputComponent + ControlOutput let the keyboard write typed intent.
use helios_core::control::ControlOutput;
use helios_sim::simulation::core::components::ControlOutputComponent;
// AckermannActuator is the marker that this entity is keyboard-drivable.
use helios_sim::simulation::plugins::vehicles::ackermann::AckermannActuator;

fn main() {
    let cli = Cli::parse();

    let mut app = App::new();

    if cli.headless {
        app.add_plugins((MinimalPlugins, AssetPlugin::default()));
        info!("Running in headless mode.");
    } else {
        app.add_plugins(
            DefaultPlugins.set(LogPlugin {
                level: bevy::log::Level::INFO,
                filter: "info,wgpu_core=error,wgpu_hal=error,helios_sim=debug,helios_core=debug"
                    .to_string(),
                ..default()
            }),
        );
    }
    app
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(PhysicsDebugPlugin::default())
        .insert_resource(cli.clone());

    app.init_state::<AppState>();
    app.add_plugins(ConfigPlugin);
    app.add_plugins(HeliosSimulationPlugin);

    // Keyboard controller writes RawActuators — no coupling to vehicle-specific types.
    app.add_systems(Update, keyboard_controller);

    println!("Starting Helios Simulation...");
    app.run();
}

/// Reads keyboard input and writes `ControlOutputComponent(RawActuators([throttle, steering]))`.
/// The Ackermann actuator system converts this to forces; no vehicle-specific logic here.
fn keyboard_controller(
    mut commands: Commands,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    query: Query<Entity, With<AckermannActuator>>,
) {
    let mut throttle = 0.0_f64;
    let mut steering = 0.0_f64;

    if keyboard_input.pressed(KeyCode::ArrowUp) {
        throttle = 1.0;
    }
    if keyboard_input.pressed(KeyCode::ArrowDown) {
        throttle = -1.0;
    }
    if keyboard_input.pressed(KeyCode::ArrowLeft) {
        steering = 0.7; // normalised torque demand [-1, 1], positive = left (FLU convention)
    }
    if keyboard_input.pressed(KeyCode::ArrowRight) {
        steering = -0.7;
    }

    for entity in &query {
        commands.entity(entity).insert(ControlOutputComponent(
            ControlOutput::RawActuators(vec![throttle, steering]),
        ));
    }
}
