// helios_sim/src/bin/helios_research.rs
//
// Primary research binary for Helios.
//
// Usage:
//   cargo run --bin helios_research -- --profile control_only --scenario configs/scenarios/...
//   cargo run --bin helios_research -- --monte-carlo 20 --profile control_only --scenario ...

use avian3d::prelude::*;
use bevy::{log::LogPlugin, prelude::*};
use clap::Parser;
use helios_core::control::ControlOutput;
use helios_sim::simulation::config::ConfigPlugin;
use helios_sim::simulation::core::components::ControlOutputComponent;
use helios_sim::simulation::plugins::vehicles::ackermann::AckermannActuator;
use helios_sim::{
    prelude::{AppState, ProfiledSimulationPlugin, SimulationProfile},
};
use helios_sim::cli::Cli as BaseCli;
use std::path::PathBuf;

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

#[derive(Parser, Debug, Clone)]
#[command(
    name = "helios_research",
    about = "Flexible multi-profile Helios research binary"
)]
struct ResearchCli {
    /// Path to the scenario TOML file.
    #[arg(short, long, default_value = "configs/scenarios/00_tutorial_showcase.toml")]
    pub scenario: PathBuf,

    /// Simulation profile override.
    /// One of: full_pipeline, estimation_only, mapping_only, planning_only,
    ///         control_only, path_tracking.
    /// If omitted, reads `[simulation] profile` from the scenario TOML.
    #[arg(long)]
    pub profile: Option<String>,

    /// Run N sequential Monte Carlo iterations and print aggregate stats.
    #[arg(long, value_name = "N")]
    pub monte_carlo: Option<u32>,

    /// Root directory for all Helios config files.
    #[arg(long, default_value = "configs")]
    pub config_root: PathBuf,

    /// Run without a graphical window.
    #[arg(long, default_value_t = false)]
    pub headless: bool,
}

impl From<&ResearchCli> for BaseCli {
    fn from(r: &ResearchCli) -> Self {
        Self {
            scenario: r.scenario.clone(),
            config_root: r.config_root.clone(),
            headless: r.headless,
        }
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

fn main() {
    let args = ResearchCli::parse();

    if let Some(n) = args.monte_carlo {
        run_monte_carlo(&args, n);
    } else {
        run_single(&args, 0);
    }
}

fn run_monte_carlo(args: &ResearchCli, n: u32) {
    info!("[MC] Starting Monte Carlo with {} runs", n);
    for i in 0..n {
        info!("[MC] Run {}/{}", i + 1, n);
        run_single(args, i);
    }
    info!("[MC] All {} runs complete.", n);
}

fn run_single(args: &ResearchCli, _run_index: u32) {
    let cli: BaseCli = BaseCli::from(args);
    let profile = resolve_profile(args);

    let mut app = App::new();

    if args.headless {
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
        .insert_resource(cli.clone());

    app.init_state::<AppState>();
    app.add_plugins(ConfigPlugin);
    app.add_plugins(ProfiledSimulationPlugin { profile });
    app.add_systems(Update, keyboard_controller);

    app.run();
}

fn resolve_profile(args: &ResearchCli) -> SimulationProfile {
    if let Some(ref p) = args.profile {
        if let Some(profile) = SimulationProfile::from_str_opt(p) {
            return profile;
        }
        eprintln!("[helios_research] Unknown profile '{}', using FullPipeline.", p);
    }
    SimulationProfile::FullPipeline
}

fn keyboard_controller(
    mut commands: Commands,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    query: Query<Entity, With<AckermannActuator>>,
) {
    let mut throttle = 0.0_f64;
    let mut steering = 0.0_f64;

    if keyboard_input.pressed(KeyCode::ArrowUp) { throttle = 1.0; }
    if keyboard_input.pressed(KeyCode::ArrowDown) { throttle = -1.0; }
    if keyboard_input.pressed(KeyCode::ArrowLeft) { steering = 0.7; }
    if keyboard_input.pressed(KeyCode::ArrowRight) { steering = -0.7; }

    for entity in &query {
        commands
            .entity(entity)
            .insert(ControlOutputComponent(ControlOutput::RawActuators(vec![throttle, steering])));
    }
}
