// helios_sim/examples/05_control_only.rs
//
// Static path + ground-truth estimator + control + metrics.  No sensors or mapping.
// Requires [simulation] mock_path in the scenario TOML.
//
// On exit prints: rise time, settling time, overshoot, cross-track error, heading error.
// Press H to toggle legend (should show only F1/F4/F6/F8 — no F2/F3/F7/F9).
//
// cargo run --example 05_control_only
// cargo run --example 05_control_only -- --scenario configs/scenarios/isolation/control_only.toml

use clap::Parser;
use helios_sim::cli::Cli;

use avian3d::prelude::*;
use bevy::{log::LogPlugin, prelude::*};
use helios_sim::prelude::{AppState, ProfiledSimulationPlugin, SimulationProfile};
use helios_sim::simulation::config::ConfigPlugin;

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
                    filter:
                        "info,wgpu_core=error,wgpu_hal=error,helios_sim=debug,helios_core=debug"
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
        .insert_resource(cli);
    app.init_state::<AppState>();
    app.add_plugins(ConfigPlugin);
    app.add_plugins(ProfiledSimulationPlugin {
        profile: SimulationProfile::ControlOnly,
    });
    println!("Starting Helios Simulation — Control Only");
    app.run();
}
