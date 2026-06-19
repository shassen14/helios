//! The sim bridge: the (Bevy-touching) adapter that lets the pure, host-agnostic
//! [`Runner`] drive a live `helios_sim` app. Everything here is gated behind the
//! `sim` feature so the base crate stays Bevy-free.
//!
//! The types below are the bridge's *vocabulary* — typed mailboxes that carry
//! state between the three runner systems (`plugin.rs`) and the bin, which
//! cannot share local variables and so must communicate through `Resource`s.
//! They are intentionally dumb: no verdict logic lives here — that stays in the
//! pure `Runner`.

mod headless;
mod plugin;

pub use headless::headless;
pub use plugin::TestRunnerPlugin;

use crate::report::ReportStatus;
use crate::run::termination::TerminationReason;
use crate::runner::Runner;

use bevy::prelude::Resource;
use std::path::PathBuf;
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

/// Where `finalize` writes the machine-readable TOML report. Supplied by the
/// bin's `--out` flag so the path is never hardcoded in source.
#[derive(Resource)]
pub struct ReportOutputPath(PathBuf);

impl ReportOutputPath {
    pub fn new(path: PathBuf) -> Self {
        Self(path)
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
}
