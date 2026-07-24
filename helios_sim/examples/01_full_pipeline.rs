// helios_sim/examples/01_full_pipeline.rs
//
// Full end-to-end autonomy pipeline: sensors + estimation + mapping + planning + control.
//
// cargo run --example 01_full_pipeline
// cargo run --example 01_full_pipeline -- --scenario configs/scenarios/simple_car_scenario.toml --headless

use clap::Parser;
use helios_sim::agents::vehicles::ackermann::AckermannActuator;
use helios_sim::cli::Cli;
use helios_sim::core::components::ControlOutputComponent;
use helios_sim::core::host::{HeliosHost, Presentation, TimePolicy};

use helios_core::control::ControlOutput;

use avian3d::prelude::PhysicsDebugPlugin;
use bevy::prelude::*;

fn main() {
    let cli = Cli::parse();

    // The example only varies the two host inputs `HeliosHost` exposes; it no
    // longer hand-assembles the plugin stack, so it can't drift out of sync with
    // the canonical host (which is what left `TimePolicy` uninserted before).
    let presentation = if cli.headless {
        Presentation::Headless
    } else {
        Presentation::Windowed
    };
    let time_policy = TimePolicy::from_cli(cli.speed, presentation);

    let mut app = App::new();
    app.add_plugins(HeliosHost::new(cli, presentation, time_policy));

    // Collider gizmos are a watch-the-window aid; there is nothing to draw in a
    // headless run.
    if let Presentation::Windowed = presentation {
        app.add_plugins(PhysicsDebugPlugin);
    }

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
