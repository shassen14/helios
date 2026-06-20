use crate::assertion::AssertionResult;

use serde::Deserialize;

/// _Pre-step._ When a run is allowed to stop. Every run must declare at least
/// one trigger; a run with no stop condition would loop forever.
///
/// Both fields are optional and may be combined — a run can cap simulated time
/// *and* bail early on an assertion outcome, whichever fires first. The
/// `deny_unknown_fields` fence is doing real work: it rejects not-yet-supported
/// triggers (e.g. `on_collision`) at parse time rather than silently ignoring
/// them, so no run file can depend on a stop condition that isn't implemented yet.
///
/// This is the *config* (when to stop). [`TerminationReason`] is the separate
/// *outcome* (why we actually stopped) — produced and consumed at different
/// times, so they stay distinct types.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Termination {
    max_simulated_seconds: Option<f64>,
    on_assertion: Option<OnAssertion>,
}

impl Termination {
    /// Simulated-time budget in seconds, if the run set one. The runner stops
    /// once the sim clock reaches this, regardless of assertion state.
    pub fn max_simulated_seconds(&self) -> Option<f64> {
        self.max_simulated_seconds
    }

    /// The assertion-outcome trigger, if the run set one. Returned by reference
    /// because `OnAssertion` is borrowed for matching, not copied.
    pub fn on_assertion(&self) -> Option<&OnAssertion> {
        self.on_assertion.as_ref()
    }
}

/// _Pre-step._ Assertion-outcome trigger: stop the run as soon as the run's assertions
/// reach this collective state. `rename_all = "snake_case"` so the TOML reads
/// `on_assertion = "any_passed"`, matching the rest of the config vocabulary.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OnAssertion {
    /// Stop the moment any assertion passes (success-seeking runs).
    AnyPassed,
    /// Stop the moment any assertion fails (fail-fast runs).
    AnyFailed,
}

/// _Per-step._ Why a run actually stopped — the outcome paired with [`Termination`]'s
/// config. Carries the triggering [`AssertionResult`] for the assertion case so
/// the report can show what tripped the stop, not just that one did.
#[derive(Debug, Clone, PartialEq)]
pub enum TerminationReason {
    /// The failed setup abort.
    Aborted,
    /// The simulated-time budget elapsed.
    MaxSimulatedSeconds,
    /// An assertion reached the configured `on_assertion` state.
    OnAssertion { result: AssertionResult },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_both_triggers() {
        let parsed: Termination = toml::from_str(
            r#"
            max_simulated_seconds = 30.0
            on_assertion = "any_failed"
            "#,
        )
        .unwrap();
        assert_eq!(parsed.max_simulated_seconds(), Some(30.0));
        assert_eq!(parsed.on_assertion(), Some(&OnAssertion::AnyFailed));
    }

    #[test]
    fn omitted_triggers_are_none() {
        // Both fields are `Option`, so a missing key deserializes to `None`
        // without needing `#[serde(default)]`.
        let parsed: Termination = toml::from_str("").unwrap();
        assert_eq!(parsed.max_simulated_seconds(), None);
        assert_eq!(parsed.on_assertion(), None);
    }

    #[test]
    fn on_assertion_uses_snake_case() {
        let parsed: Termination = toml::from_str(r#"on_assertion = "any_passed""#).unwrap();
        assert_eq!(parsed.on_assertion(), Some(&OnAssertion::AnyPassed));
    }

    #[test]
    fn rejects_unsupported_trigger() {
        // The forward-compat fence: an unimplemented stop condition is a hard
        // parse error, not a silently-ignored field.
        assert!(toml::from_str::<Termination>(r#"on_collision = true"#).is_err());
    }

    #[test]
    fn rejects_unknown_on_assertion_variant() {
        // A misspelled or unsupported outcome name fails rather than defaulting.
        assert!(toml::from_str::<Termination>(r#"on_assertion = "any_pending""#).is_err());
    }
}
