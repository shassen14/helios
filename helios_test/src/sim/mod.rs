//! The sim bridge: the (Bevy-touching) adapter that lets the pure, host-agnostic
//! [`Runner`] drive a live `helios_sim` app. Everything here is gated behind the
//! `sim` feature so the base crate stays Bevy-free.
//!
//! The types below are the bridge's *vocabulary* — typed mailboxes that carry
//! state between the three runner systems (`plugin.rs`) and the bin, which
//! cannot share local variables and so must communicate through `Resource`s.
//! They are intentionally dumb: no verdict logic lives here — that stays in the
//! pure `Runner`.

mod metrics;
mod plugin;

pub use metrics::MetricsPlugin;
pub use plugin::TestRunnerPlugin;

use crate::report::ReportStatus;
use crate::run::termination::TerminationReason;
use crate::runner::Runner;
use crate::RunMetrics;

use bevy::prelude::Resource;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Instant;

/// Holds the live [`Runner`] across ticks. Wraps it because `Runner` lives in
/// the Bevy-free base crate and cannot derive `Resource` itself. Mutated by
/// systems 1–2 (`ResMut`), read by system 3 (`Res`). Inserted by the bin so a
/// `Runner::new` failure becomes a clean exit code *before* the app spins up.
#[derive(Resource)]
pub struct ActiveRunner(Runner);

impl ActiveRunner {
    pub fn new(runner: Runner) -> Self {
        Self(runner)
    }
}

/// Wall-clock start instant, captured by the bin right before `app.run()`. Read
/// once at `finalize` to compute `wall_duration_secs` — the runner deliberately
/// reads no wall clock, so the host supplies real time from here.
#[derive(Resource)]
pub struct WallClockStart(Instant);

impl WallClockStart {
    pub fn new(now: Instant) -> Self {
        Self(now)
    }
}

/// Carries *why* the run stopped from the `tick` system (which decides it, in
/// `FixedUpdate`) to the `finalize` system (which needs it, in a different
/// schedule and so cannot see `tick`'s locals).
///
/// `reason` is `Option` because system 1 can short-circuit straight to
/// `Flushing` on unresolved targets — *before any tick runs* — leaving no
/// `TerminationReason`. `finalize` must cope with that `None` rather than panic.
#[derive(Resource, Default)]
pub struct RunOutcome {
    reason: Option<TerminationReason>,
    run_name: String,
}

impl RunOutcome {
    pub fn new(run_name: String) -> Self {
        Self {
            run_name,
            reason: None,
        }
    }
}

/// The run's pass/fail, written at `finalize` and read by the bin *after*
/// `app.run()` returns — the one wire by which the verdict escapes the Bevy
/// loop and becomes the process exit code.
///
/// Defaults to `Failed` (fail-closed): if `finalize` never runs, the run must
/// report failure, never a silent pass.
#[derive(Resource)]
pub struct RunVerdict(ReportStatus);

impl Default for RunVerdict {
    fn default() -> Self {
        Self(ReportStatus::Failed)
    }
}

impl RunVerdict {
    /// Set the verdict from a finished report's status.
    pub fn set(&mut self, status: ReportStatus) {
        self.0 = status;
    }

    /// Whether the run passed. Read by the bin to choose exit 0 vs 1.
    pub fn passed(&self) -> bool {
        self.0 == ReportStatus::Passed
    }
}

/// The one wire by which a whole `RunMetrics` escapes the Bevy loop — the
/// struct-sized sibling of [`RunVerdict`], which can only carry a pass/fail bit
/// out via the exit code.
///
/// Read the type inside-out and it spells the design:
/// - `Option` — the slot starts empty and is filled exactly once, at flush.
///   `None` means the run produced no metrics (the summarizer never ran);
///   the bin treats that as "nothing to collect", never a silent zero.
/// - `Mutex` — the summarizer writes through a *shared* `&` reference (interior
///   mutability), which is why `store`/`take` take `&self`. Thread-safe because
///   Bevy resources are touched across threads (`Resource: Send + Sync`).
/// - `Arc` — shared ownership across the `app.run()` boundary. The bin keeps one
///   clone and inserts another into the world; when `run()` drains the world
///   (Bevy 0.16) the world drops *its* clone, but the bin's clone keeps the
///   value alive. Reading a `Res` from the world *after* `run()` would instead
///   panic on the emptied world — this clone is the documented escape hatch.
#[derive(Resource, Clone)]
pub struct MetricsCollector(Arc<Mutex<Option<RunMetrics>>>);

impl MetricsCollector {
    pub fn new() -> Self {
        Self(Arc::new(Mutex::new(None)))
    }

    /// Pull the stored metrics out, leaving the slot empty — the bin's read,
    /// called *after* `run()` returns. Destructive: a second `take` yields
    /// `None`. A poisoned lock (a thread panicked mid-write) also yields `None`
    /// rather than panicking, because runtime paths never unwrap.
    pub fn take(&self) -> Option<RunMetrics> {
        self.0.lock().ok().and_then(|mut guard| guard.take())
    }

    /// Store this run's metrics — the summarizer's write, at `OnEnter(Flushing)`
    /// before the world drains. A poisoned lock drops the metrics silently; we
    /// cannot safely recover and must not panic in a runtime path.
    pub fn store(&self, metrics: RunMetrics) {
        if let Ok(mut guard) = self.0.lock() {
            *guard = Some(metrics);
        }
    }
}

/// Where `finalize` writes the machine-readable TOML report. Supplied by the
/// bin's `--out` flag so the path is never hardcoded in source.
#[derive(Resource)]
pub struct ReportOutputPath(PathBuf);

impl ReportOutputPath {
    pub fn new(path: PathBuf) -> Self {
        Self(path)
    }
}

/// The per-run identity (`run_index` + `seed`) the summarizer stamps onto each
/// `RunMetrics`. Inserted by the bin; `Default` (both `0`) suits a single run.
/// A multi-run driver writes a distinct value per run — the per-run seed
/// override sets these, and the summarizer reads them back unchanged.
#[derive(Resource, Default)]
pub struct RunMetadata {
    run_index: u32,
    seed: u64,
}

impl RunMetadata {
    pub fn new(run_index: u32, seed: u64) -> Self {
        Self { run_index, seed }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn verdict_defaults_to_failed() {
        // Fail-closed: a runner that never finalizes must not look like a pass.
        assert!(!RunVerdict::default().passed());
    }

    #[test]
    fn verdict_reflects_set_status() {
        let mut v = RunVerdict::default();
        v.set(ReportStatus::Passed);
        assert!(v.passed());
        v.set(ReportStatus::Failed);
        assert!(!v.passed());
    }

    #[test]
    fn outcome_new_starts_with_no_reason() {
        // Before any tick terminates the run, the reason is absent and only the
        // name (which the bin supplies up front) is populated.
        let outcome = RunOutcome::new("smoke".to_string());
        assert_eq!(outcome.run_name, "smoke");
        assert!(outcome.reason.is_none());
    }

    #[test]
    fn collector_round_trips_metrics_and_take_is_destructive() {
        use crate::MetricId;

        let collector = MetricsCollector::new();
        // Nothing stored yet: the bin must see "no metrics", not a default.
        assert!(collector.take().is_none());

        let mut m = RunMetrics::new(3, 99);
        m.insert(MetricId::new("final_speed"), 1.5);
        collector.store(m);

        let got = collector
            .take()
            .expect("stored metrics must survive and come back out");
        assert_eq!(got.run_index(), 3);
        assert_eq!(got.seed(), 99);
        assert_eq!(got.get(&MetricId::new("final_speed")), Some(1.5));

        // `take` empties the slot — a second read finds nothing.
        assert!(collector.take().is_none());
    }

    #[test]
    fn collector_clones_share_one_slot() {
        // The bin keeps one clone and inserts another into the world; both must
        // point at the *same* value, or the metrics would not escape the drained
        // world. Storing through one clone must be visible through the other.
        use crate::MetricId;

        let bin_side = MetricsCollector::new();
        let world_side = bin_side.clone();

        let mut m = RunMetrics::new(0, 0);
        m.insert(MetricId::new("final_speed"), 2.0);
        world_side.store(m); // the summarizer would write through this clone

        let got = bin_side // the bin reads through its own clone
            .take()
            .expect("a store through one clone is visible through the other");
        assert_eq!(got.get(&MetricId::new("final_speed")), Some(2.0));
    }
}
