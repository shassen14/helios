// helios_sim/examples/06_path_tracking.rs
//
// Sensors + real estimation + control + static path + metrics.
// Requires [simulation] mock_path in the scenario TOML.
//
// On exit prints: cross-track error and heading error.
//
// cargo run --example 06_path_tracking
// cargo run --example 06_path_tracking -- --scenario configs/scenarios/isolation/path_tracking.toml

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
        app.add_plugins((MinimalPlugins, AssetPlugin::default()));
    } else {
        app.add_plugins(DefaultPlugins.set(LogPlugin {
            level: bevy::log::Level::INFO,
            filter: "info,wgpu_core=error,wgpu_hal=error,helios_sim=debug,helios_core=debug"
                .to_string(),
            ..default()
        }));
    }
    app.add_plugins(PhysicsPlugins::default())
        .insert_resource(cli);
    app.init_state::<AppState>();
    app.add_plugins(ConfigPlugin);
    app.add_plugins(ProfiledSimulationPlugin { profile: SimulationProfile::PathTracking });
    println!("Starting Helios Simulation — Path Tracking");
    app.run();
}
