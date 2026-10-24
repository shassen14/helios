use crate::assertion::EntryName;

use helios_core::prelude::MonotonicTime;
use helios_runtime::observe::key::RecordedKey;

use serde::Deserialize;

/// _Pre-step._ When a run is allowed to stop. Every run carries a simulated-time
/// budget, so every run stops; `fail_fast` may stop it sooner.
///
/// The `deny_unknown_fields` fence is doing real work: it rejects not-yet-supported
/// triggers (e.g. `on_collision`) at parse time rather than silently ignoring
/// them, so no run file can depend on a stop condition that isn't implemented yet.
///
/// This is the *config* (when to stop). [`TerminationReason`] is the separate
/// *outcome* (why we actually stopped) — produced and consumed at different
/// times, so they stay distinct types.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Termination {
    max_simulated_seconds: f64,
    #[serde(default)]
    fail_fast: bool,
}

impl Termination {
    /// Simulated-time budget in seconds. The runner stops once the sim clock
    /// has run this long, regardless of assertion state.
    pub fn max_simulated_seconds(&self) -> f64 {
        self.max_simulated_seconds
    }

    /// Whether a run that started at `start` has used its whole budget by
    /// `now`. `>=` so the tick that reaches the budget is the one that stops,
    /// rather than overshooting by a tick.
    pub fn budget_spent(&self, start: MonotonicTime, now: MonotonicTime) -> bool {
        now.0 - start.0 >= self.max_simulated_seconds
    }

    /// Whether the run stops on the first every-sample assertion that fails.
    /// Off unless the run file sets it.
    pub fn fail_fast(&self) -> bool {
        self.fail_fast
    }
}

/// _Per-step._ Why a run actually stopped — the outcome paired with [`Termination`]'s
/// config.
#[derive(Debug, Clone, PartialEq)]
pub enum TerminationReason {
    /// The failed setup abort.
    Aborted,
    /// The simulated-time budget elapsed.
    MaxSimulatedSeconds,
    /// An every-sample assertion failed and the run is `fail_fast`. `name`
    /// is the assertion's, and `key` the series that failed it: one of
    /// several, for an assertion on each instance of a keyed channel.
    AssertionFailed { name: EntryName, key: RecordedKey },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_both_fields() {
        let parsed: Termination = toml::from_str(
            r#"
            max_simulated_seconds = 30.0
            fail_fast = true
            "#,
        )
        .unwrap();
        assert_eq!(parsed.max_simulated_seconds(), 30.0);
        assert!(parsed.fail_fast());
    }

    #[test]
    fn fail_fast_defaults_off() {
        let parsed: Termination = toml::from_str("max_simulated_seconds = 30.0").unwrap();
        assert!(!parsed.fail_fast());
    }

    #[test]
    fn a_missing_budget_is_a_parse_error() {
        // Without a budget a run may never stop, so the file is refused before
        // validation sees it.
        assert!(toml::from_str::<Termination>("fail_fast = true").is_err());
    }

    #[test]
    fn rejects_unsupported_trigger() {
        // The forward-compat fence: an unimplemented stop condition is a hard
        // parse error, not a silently-ignored field.
        assert!(toml::from_str::<Termination>(
            r#"
            max_simulated_seconds = 30.0
            on_collision = true
            "#
        )
        .is_err());
    }

    #[test]
    fn rejects_the_removed_on_assertion_trigger() {
        assert!(toml::from_str::<Termination>(
            r#"
            max_simulated_seconds = 30.0
            on_assertion = "any_failed"
            "#
        )
        .is_err());
    }
}
