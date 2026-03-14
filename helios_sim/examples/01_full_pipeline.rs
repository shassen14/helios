// helios_sim/examples/01_full_pipeline.rs
//
// Full end-to-end autonomy pipeline: sensors + estimation + mapping + planning + control.
//
// cargo run --example 01_full_pipeline
// cargo run --example 01_full_pipeline -- --scenario configs/scenarios/simple_car_scenario.toml --headless

use clap::Parser;
use helios_sim::cli::Cli;

use avian3d::prelude::*;
use bevy::{log::LogPlugin, prelude::*};
use helios_core::control::ControlOutput;
use helios_sim::prelude::AppState;
use helios_sim::simulation::config::ConfigPlugin;
use helios_sim::simulation::core::components::ControlOutputComponent;
use helios_sim::simulation::plugins::vehicles::ackermann::AckermannActuator;
use helios_sim::HeliosSimulationPlugin;

fn main() {
    let cli = Cli::parse();

    let mut app = App::new();

    if cli.headless {
        app.add_plugins(
            DefaultPlugins
                .set(bevy::window::WindowPlugin {
                    primary_window: None,
                    exit_condition: bevy::window::ExitCondition::DontExit,
                    ..default()
                })
                .disable::<bevy::winit::WinitPlugin>()
                .set(LogPlugin {
                    level: bevy::log::Level::INFO,
                    filter: "info,wgpu_core=error,wgpu_hal=error,helios_sim=debug,helios_core=debug"
                        .to_string(),
                    ..default()
                }),
        );
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
    app.add_plugins(PhysicsPlugins::default())
        .insert_resource(cli.clone());
    if !cli.headless {
        app.add_plugins(PhysicsDebugPlugin::default());
    }

    app.init_state::<AppState>();
    app.add_plugins(ConfigPlugin);
    app.add_plugins(HeliosSimulationPlugin);
    app.add_systems(Update, keyboard_controller);

    println!("Starting Helios Simulation — Full Pipeline");
    app.run();
}

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
        steering = 0.7;
    }
    if keyboard_input.pressed(KeyCode::ArrowRight) {
        steering = -0.7;
    }

    for entity in &query {
        commands
            .entity(entity)
            .insert(ControlOutputComponent(ControlOutput::RawActuators(vec![
                throttle, steering,
            ])));
    }
}
