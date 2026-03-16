// helios_sim/src/bin/helios_research.rs
//
// Primary research binary for Helios.
//
// Usage:
//   cargo run --bin helios_research -- --profile control_only --scenario configs/scenarios/...
//   cargo run --bin helios_research -- --monte-carlo 20 --profile control_only --scenario ...
//
// Samply CPU profiler (no sudo required on macOS or Linux):
//   cargo install samply
//   NOTE: profile the binary directly — `samply record cargo run` profiles cargo, not the binary.
//   NOTE: BEVY_ASSET_ROOT=./helios_sim is required so Bevy finds assets/ relative to the workspace root,
//         not relative to the binary in target/profiling/.
//   cargo build --profile profiling --bin helios_research
//   BEVY_ASSET_ROOT=./helios_sim samply record --output --save-only samply-profile.json ./target/profiling/helios_research --duration-secs 60 --scenario configs/scenarios/00_tutorial_showcase.toml
//   (opens Firefox Profiler automatically in your browser)
//
// DHAT heap profiler:
//   cargo run --bin helios_research --profile profiling --features dhat-heap --headless --duration-secs 30 --scenario configs/scenarios/00_tutorial_showcase.toml
//   View output: open dhat-heap.json at https://nnethercote.github.io/dh_view/dh_view.html

#[cfg(feature = "dhat-heap")]
#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

use avian3d::prelude::*;
use bevy::{log::LogPlugin, prelude::*};
use clap::Parser;
use helios_core::control::ControlOutput;
use helios_sim::cli::Cli as BaseCli;
use helios_sim::prelude::{AppState, ProfiledSimulationPlugin, SimulationProfile};
use helios_sim::simulation::config::ConfigPlugin;
use helios_sim::simulation::core::components::ControlOutputComponent;
use helios_sim::simulation::plugins::vehicles::ackermann::AckermannActuator;
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
    #[arg(
        short,
        long,
        default_value = "configs/scenarios/00_tutorial_showcase.toml"
    )]
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

    /// Exit cleanly after N seconds. Required for DHAT to write output.
    #[arg(long, value_name = "SECONDS")]
    pub duration_secs: Option<f32>,
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
// Profiling resources and systems
// ---------------------------------------------------------------------------

#[derive(Resource)]
struct ProfilingDuration(Option<f32>);

fn timed_exit_system(
    time: Res<Time>,
    mut elapsed: Local<f32>,
    duration: Res<ProfilingDuration>,
    state: Res<State<AppState>>,
    mut next_state: ResMut<NextState<AppState>>,
) {
    if let Some(limit) = duration.0 {
        *elapsed += time.delta_secs();
        if *elapsed >= limit && *state.get() == AppState::Running {
            info!("[profiling] Duration limit reached ({limit:.0}s). Flushing and exiting.");
            next_state.set(AppState::Flushing);
        }
    }
}

fn flushing_exit_system(mut exit: EventWriter<AppExit>) {
    exit.write(AppExit::Success);
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

fn main() {
    #[cfg(feature = "dhat-heap")]
    let _profiler = dhat::Profiler::new_heap();

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
        .insert_resource(cli.clone());

    app.init_state::<AppState>();
    app.add_plugins(ConfigPlugin);
    app.add_plugins(ProfiledSimulationPlugin { profile });
    app.add_systems(Update, keyboard_controller);

    app.insert_resource(ProfilingDuration(args.duration_secs))
        .add_systems(Update, timed_exit_system)
        .add_systems(
            Update,
            flushing_exit_system.run_if(in_state(AppState::Flushing)),
        );

    app.run();
}

fn resolve_profile(args: &ResearchCli) -> SimulationProfile {
    if let Some(ref p) = args.profile {
        if let Some(profile) = SimulationProfile::from_str_opt(p) {
            return profile;
        }
        eprintln!(
            "[helios_research] Unknown profile '{}', using FullPipeline.",
            p
        );
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
