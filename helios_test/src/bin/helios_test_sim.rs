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

use helios_sim::host::{HeliosHost, Presentation};
use helios_test::{
    report::aggregate::{console::print, toml_writer::write},
    sim::{
        ActiveRunner, MetricsCollector, MetricsPlugin, ReportOutputPath, RunMetadata, RunOutcome,
        RunVerdict, WallClockStart,
    },
    statistics::{standard_statistics, StatId},
    AggregateReport, Runner,
};

use bevy::prelude::*;
use clap::Parser;
use std::path::{Path, PathBuf};

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

    #[arg(long)]
    pub monte_carlo: Option<usize>,
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

    let scenario_ref = run.scenario().path().to_string();

    // Validate up front so an incoherent run fails here (exit 2) rather than
    // partway through a sim. We rebuild a fresh `Runner` per MC iteration below,
    // so this one is discarded — it only proves the run is coherent. Validate on
    // a clone so `run` survives for the loop.
    if let Err(e) = Runner::new(run.clone()) {
        eprintln!("invalid run: {e}");
        std::process::exit(2);
    }

    // Resolve the scenario ref under --config-root and confirm it exists, so a
    // bad reference fails here with a clear message rather than panicking deep
    // inside the sim's config loader.
    let scenario_path = resolve_scenario_path(&args.config_root, &scenario_ref);
    if !scenario_path.exists() {
        eprintln!("scenario not found: {}", scenario_path.display());
        std::process::exit(2);
    }

    // Absent or `0` collapses to a single run; `max(1)` keeps `--monte-carlo 0`
    // from silently running zero apps and reporting a vacuous pass.
    let n = args.monte_carlo.unwrap_or(1).max(1);

    let mut runs = Vec::new();
    let mut all_passed = true;

    for i in 0..n {
        // --- Assemble the host body; the test layer wraps it below. ---
        let mut app = App::new();

        let cli = helios_sim::cli::Cli {
            scenario: scenario_path.clone(),
            config_root: args.config_root.clone(),
            headless: true,
            seed: None,
        };

        app.add_plugins(HeliosHost::new(cli, Presentation::Headless));

        // Created out here so the bin keeps its own clone: the world's clone (below)
        // is dropped when `run()` drains the App, but this one keeps the collected
        // metrics alive to be read afterward.
        let collector = MetricsCollector::new();

        // The test layer wraps the host: resources first, then the plugin that
        // registers the three runner systems against them.
        app.insert_resource(collector.clone()); // the world's clone of the collector
        app.insert_resource(RunMetadata::new(i as u32, 0)); // todo: stub seed
                                                            // needs to be replaced

        // Fresh runner per run: `Runner::new` re-validates and allocates fresh
        // per-run state (empty registry, pending latches). Pre-flight already
        // proved coherence, so an Err here is unexpected — treat it as exit 2.
        let runner = match Runner::new(run.clone()) {
            Ok(runner) => runner,
            Err(e) => {
                eprintln!("invalid run: {e}");
                std::process::exit(2);
            }
        };
        app.insert_resource(ActiveRunner::new(runner));
        app.insert_resource(WallClockStart::new(std::time::Instant::now()));
        app.insert_resource(RunOutcome::new(run_name.clone()));
        app.init_resource::<RunVerdict>(); // defaults to Failed (fail-closed)
                                           // Per-run report path: with one run it is `--out` unchanged; with many,
                                           // each run gets a `.run{i}` suffix so the N single-run reports don't
                                           // clobber each other (and `--out` stays free for the aggregate below).
        app.insert_resource(ReportOutputPath::new(per_run_report_path(&args.out, i, n)));
        app.add_plugins(helios_test::sim::TestRunnerPlugin);
        app.add_plugins(MetricsPlugin);

        // `run()` returns the AppExit that `finalize` wrote (it carries the verdict
        // as its exit code). We can't read RunVerdict from the world here because
        // `run()` empties the App in Bevy 0.16 — hence the AppExit channel.
        let exit = app.run();
        all_passed &= exit.is_success();

        // The world is drained now, but the bin's clone still points at the metrics
        // the summarizer stored before flush. `take` empties the slot; `None` means
        // the summarizer never ran (e.g. the run aborted before reaching flush).
        if let Some(m) = collector.take() {
            runs.push(m);
        }
    }

    // Only a batch produces an aggregate; a single run's artifact is its own
    // report, already written by `finalize`. Build the reducer set lazily so a
    // single run does no aggregation work.
    if n > 1 {
        let stat_table = standard_statistics();
        let stat_ids = ["mean", "std", "max", "min"].map(StatId::new);

        let aggregate_report = AggregateReport::from_runs(&runs, &stat_table, &stat_ids);

        // A failed write is an I/O problem, not a test failure: log it and carry
        // on, exactly as the single-run report path does. It must not flip the
        // exit code (reserved for assertion pass/fail) or panic.
        if let Err(e) = write(&aggregate_report, &args.out) {
            eprintln!("failed to write aggregate report: {e}");
        }

        print(&aggregate_report);
    }

    std::process::exit(if all_passed { 0 } else { 1 });
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

/// Where run `index` of a `total`-run batch writes its single-run report.
///
/// A single-run batch (`total <= 1`) writes to `base` unchanged — the common
/// case must not get a surprise suffix. A multi-run batch inserts a `.run{index}`
/// segment before the extension (`out.toml` → `out.run0.toml`) so the N reports
/// don't overwrite each other; `base` itself is left free for the aggregate.
/// A `base` with no extension just gets the suffix appended (`out` → `out.run0`).
fn per_run_report_path(base: &Path, index: usize, total: usize) -> PathBuf {
    if total <= 1 {
        return base.to_path_buf();
    }

    // Assemble the whole filename in one string rather than `set_extension`,
    // which would mistake the `.run{index}` segment for the extension and
    // overwrite it (`out.run0` + set "toml" → `out.toml`, collapsing the suffix).
    let stem = base.file_stem().map(|s| s.to_string_lossy().into_owned());
    let ext = base.extension().map(|e| e.to_string_lossy().into_owned());
    let file_name = match (stem, ext) {
        (Some(stem), Some(ext)) => format!("{stem}.run{index}.{ext}"),
        (Some(stem), None) => format!("{stem}.run{index}"),
        (None, Some(ext)) => format!("run{index}.{ext}"),
        (None, None) => format!("run{index}"),
    };

    base.with_file_name(file_name)
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

    #[test]
    fn single_run_keeps_the_base_report_path() {
        // One run must write exactly where `--out` points, with no suffix.
        let path = per_run_report_path(Path::new("test_report.toml"), 0, 1);
        assert_eq!(path, PathBuf::from("test_report.toml"));
    }

    #[test]
    fn batch_runs_get_distinct_suffixed_paths() {
        // Each run in a batch lands on its own file so the N reports never clobber.
        let base = Path::new("out/test_report.toml");
        let r0 = per_run_report_path(base, 0, 3);
        let r1 = per_run_report_path(base, 1, 3);

        assert_eq!(r0, PathBuf::from("out/test_report.run0.toml"));
        assert_eq!(r1, PathBuf::from("out/test_report.run1.toml"));
        assert_ne!(r0, r1);
    }

    #[test]
    fn suffix_is_appended_when_base_has_no_extension() {
        let path = per_run_report_path(Path::new("report"), 2, 5);
        assert_eq!(path, PathBuf::from("report.run2"));
    }
}
