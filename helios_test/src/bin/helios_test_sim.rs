//! `helios_test_sim` — drives the pure `helios_test` runner against a live
//! `helios_sim` Bevy host and turns the result into a CI-friendly exit code.
//!
//! Exit codes are the contract:
//! - `0` — the run passed.
//! - `1` — the run ran but an assertion failed (or a target was unresolved).
//! - `2` — the run file is malformed: unreadable, unparseable, incoherent, or
//!   it names a scenario that doesn't exist. Detected *before* the sim starts.
//!
//! The bin owns the exit code; Bevy owns the loop. The verdict crosses that
//! boundary as the `AppExit` that `app.run()` returns — `finalize` encodes the
//! pass/fail into the exit code (`run()` empties the App, so the world can't be
//! read afterward).

use std::path::{Path, PathBuf};

use helios_sim::prelude::AppState;
use helios_test::{
    sim::{ActiveRunner, ReportOutputPath, RunOutcome, RunVerdict, WallClockStart},
    Runner,
};

use avian3d::prelude::*;
use bevy::prelude::*;
use clap::Parser;

#[derive(Parser, Debug, Clone)]
#[command(
    name = "helios_test_sim",
    about = "Run a Helios test against a sim host"
)]
struct SimCli {
    /// Path to the run file to execute.
    #[arg(long)]
    pub run: PathBuf,

    /// Root directory for all Helios config files; the run's scenario `from` is
    /// resolved relative to this.
    #[arg(long, default_value = "configs")]
    pub config_root: PathBuf,

    /// Where to write the machine-readable TOML report.
    #[arg(long, default_value = "test_report.toml")]
    pub out: PathBuf,
}

fn main() {
    let args = SimCli::parse();

    let run_name = run_name_from_path(&args.run);

    // --- Pre-flight: prove the run file is sane before spinning up a sim. ---
    // Any failure here is exit 2 ("the run file is wrong"), distinct from a
    // test that runs and fails (exit 1).
    let run = match helios_test::run::load(&args.run) {
        Ok(run) => run,
        Err(e) => {
            eprintln!("malformed run: {e}");
            std::process::exit(2);
        }
    };

    // `Runner::new` consumes `run`, so capture the scenario reference first.
    let scenario_ref = run.scenario().path().to_string();

    let runner = match Runner::new(run) {
        Ok(runner) => runner,
        Err(e) => {
            eprintln!("invalid run: {e}");
            std::process::exit(2);
        }
    };

    // Resolve the scenario ref under --config-root and confirm it exists, so a
    // bad reference fails here with a clear message rather than panicking deep
    // inside the sim's config loader.
    let scenario_path = resolve_scenario_path(&args.config_root, &scenario_ref);
    if !scenario_path.exists() {
        eprintln!("scenario not found: {}", scenario_path.display());
        std::process::exit(2);
    }

    // --- Assemble the host. Order mirrors helios_research::run_single. ---
    let mut app = App::new();
    app.add_plugins(helios_test::sim::headless())
        .add_plugins(PhysicsPlugins::default());

    // The Cli resource must exist BEFORE ConfigPlugin's startup system reads it.
    app.insert_resource(helios_sim::cli::Cli {
        scenario: scenario_path,
        config_root: args.config_root.clone(),
        headless: true,
    });
    app.init_state::<AppState>();
    app.add_plugins(helios_sim::simulation::config::ConfigPlugin);
    app.add_plugins(helios_sim::HeliosSimulationPlugin);

    // The test layer wraps the host: resources first, then the plugin that
    // registers the three runner systems against them.
    app.insert_resource(ActiveRunner::new(runner));
    app.insert_resource(WallClockStart::new(std::time::Instant::now()));
    app.insert_resource(RunOutcome::new(run_name));
    app.init_resource::<RunVerdict>(); // defaults to Failed (fail-closed)
    app.insert_resource(ReportOutputPath::new(args.out));
    app.add_plugins(helios_test::sim::TestRunnerPlugin);

    // `run()` returns the AppExit that `finalize` wrote (it carries the verdict
    // as its exit code). We can't read RunVerdict from the world here because
    // `run()` empties the App in Bevy 0.16 — hence the AppExit channel.
    let exit = app.run();
    std::process::exit(if exit.is_success() { 0 } else { 1 });
}

/// Derive a human-readable run name from the run file path (its file stem),
/// falling back to a fixed label if the path has no stem.
fn run_name_from_path(run_path: &Path) -> String {
    run_path
        .file_stem()
        .map(|stem| stem.to_string_lossy().into_owned())
        .unwrap_or_else(|| "unnamed_run".to_string())
}

/// Resolve a scenario reference to a concrete path under `config_root`. Pure
/// path joining — existence is checked by the caller.
fn resolve_scenario_path(config_root: &Path, scenario_ref: &str) -> PathBuf {
    config_root.join(scenario_ref)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn run_name_is_the_file_stem() {
        let name = run_name_from_path(Path::new("configs/test/runs/smoke.toml"));
        assert_eq!(name, "smoke");
    }

    #[test]
    fn run_name_falls_back_when_no_stem() {
        // A path ending in ".." has no file stem.
        let name = run_name_from_path(Path::new(".."));
        assert_eq!(name, "unnamed_run");
    }

    #[test]
    fn scenario_path_joins_under_config_root() {
        let path = resolve_scenario_path(
            Path::new("configs"),
            "sim/scenarios/00_tutorial_showcase.toml",
        );
        assert_eq!(
            path,
            PathBuf::from("configs/sim/scenarios/00_tutorial_showcase.toml")
        );
    }
}
