// helios_sim/examples/02_estimation_only.rs
//
// Sensors + EKF/UKF estimation only.  No mapping, planning, or control.
// Press H to toggle legend (should show only F1/F2/F3/F4/F5/F6/F8).
//
// cargo run --example 02_estimation_only
// cargo run --example 02_estimation_only -- --scenario configs/scenarios/isolation/estimation_only.toml

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
        profile: SimulationProfile::EstimationOnly,
    });
    println!("Starting Helios Simulation — Estimation Only");
    app.run();
}
