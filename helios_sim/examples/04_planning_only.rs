// helios_sim/examples/04_planning_only.rs
//
// Static map + ground-truth estimator + planning.  No sensors or control.
// Requires [simulation] mock_map in the scenario TOML.
//
// cargo run --example 04_planning_only
// cargo run --example 04_planning_only -- --scenario configs/scenarios/isolation/planning_only.toml

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
        profile: SimulationProfile::PlanningOnly,
    });
    println!("Starting Helios Simulation — Planning Only");
    app.run();
}
