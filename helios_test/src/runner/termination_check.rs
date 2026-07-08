use helios_core::data::MonotonicTime;

use crate::{
    assertion::AssertionResult,
    run::termination::{OnAssertion, Termination, TerminationReason},
};

/// Decide whether the run should stop this tick, and if so why. Pure and
/// read-only: it judges nothing itself, it only reads the verdicts the
/// evaluator already produced (`results`, one per assertion this tick) against
/// the run's stop config. `Some(reason)` means stop; `None` means keep going.
/// The caller acts on the decision — this never terminates anything.
///
/// Precedence: an assertion outcome is checked before the time budget, because
/// "an assertion tripped" is the meaningful test result, whereas "time ran out"
/// is the fallback that only matters when nothing tripped.
pub fn check(
    now: MonotonicTime,
    start: MonotonicTime,
    termination: &Termination,
    results: &[AssertionResult],
) -> Option<TerminationReason> {
    // Assertion trigger. Both variants do the same thing — find the first
    // result matching the watched state and report *that* result — so they
    // share one shape and differ only in the pattern. Reporting the matched
    // result (rather than a freshly built `Passed`) means the `Failed` case
    // carries its kind and observed value through to the report unchanged.
    if let Some(on_assertion) = termination.on_assertion() {
        let triggered = match on_assertion {
            OnAssertion::AnyPassed => results
                .iter()
                .find(|r| matches!(r, AssertionResult::Passed { .. })),
            OnAssertion::AnyFailed => results
                .iter()
                .find(|r| matches!(r, AssertionResult::Failed { .. })),
        };
        if let Some(result) = triggered {
            return Some(TerminationReason::OnAssertion {
                result: result.clone(),
            });
        }
    }

    // Time budget. `>=` so the tick that reaches the budget is the one that
    // stops, rather than overshooting by a tick.
    if let Some(max_time) = termination.max_simulated_seconds() {
        if now.0 - start.0 >= max_time {
            return Some(TerminationReason::MaxSimulatedSeconds);
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::{AssertionValue, FailureKind};

    // Build a `Termination` by parsing TOML, since its fields are private. The
    // caller supplies the trigger body (`max_simulated_seconds`, `on_assertion`,
    // or both).
    fn term(body: &str) -> Termination {
        toml::from_str(body).unwrap()
    }

    fn at(secs: f64) -> MonotonicTime {
        MonotonicTime(secs)
    }

    // A `Failed` result carrying a kind and observed value, to confirm the
    // trigger forwards the real failure rather than a synthetic one.
    fn failed() -> AssertionResult {
        AssertionResult::Failed {
            kind: FailureKind::ConditionFailed,
            actual: Some(AssertionValue::Float(9.0)),
        }
    }

    // A `Passed` result. It carries the observed value, so the trigger has a
    // concrete result to clone into `OnAssertion`.
    fn passed() -> AssertionResult {
        AssertionResult::Passed {
            actual: AssertionValue::Float(1.0),
        }
    }

    #[test]
    fn no_trigger_set_never_stops() {
        // Empty termination → both triggers `None`. Even with decisive results
        // present, there's nothing configured to act on them.
        let t = term("");
        let results = [passed(), failed()];
        assert_eq!(check(at(100.0), at(0.0), &t, &results), None);
    }

    #[test]
    fn time_budget_fires_once_elapsed() {
        let t = term("max_simulated_seconds = 5.0");
        assert_eq!(
            check(at(6.0), at(0.0), &t, &[]),
            Some(TerminationReason::MaxSimulatedSeconds)
        );
    }

    #[test]
    fn time_budget_fires_exactly_at_the_boundary() {
        // `>=`: reaching the budget on the nose stops here, not a tick later.
        let t = term("max_simulated_seconds = 5.0");
        assert_eq!(
            check(at(5.0), at(0.0), &t, &[]),
            Some(TerminationReason::MaxSimulatedSeconds)
        );
    }

    #[test]
    fn time_budget_holds_before_the_boundary() {
        let t = term("max_simulated_seconds = 5.0");
        assert_eq!(check(at(4.999), at(0.0), &t, &[]), None);
    }

    #[test]
    fn elapsed_is_measured_from_start_not_zero() {
        // 12 - 8 = 4 < 5, so a nonzero start must not read as already elapsed.
        let t = term("max_simulated_seconds = 5.0");
        assert_eq!(check(at(12.0), at(8.0), &t, &[]), None);
    }

    #[test]
    fn any_passed_fires_on_a_pass() {
        let t = term(r#"on_assertion = "any_passed""#);
        let results = [AssertionResult::Pending, passed()];
        assert_eq!(
            check(at(0.0), at(0.0), &t, &results),
            Some(TerminationReason::OnAssertion { result: passed() })
        );
    }

    #[test]
    fn any_passed_ignores_failures_and_pending() {
        let t = term(r#"on_assertion = "any_passed""#);
        let results = [AssertionResult::Pending, failed()];
        assert_eq!(check(at(0.0), at(0.0), &t, &results), None);
    }

    #[test]
    fn any_failed_fires_and_carries_the_failing_result() {
        // The reason must forward the real failure (kind + observed value), not
        // a placeholder, so the report can explain what tripped.
        let t = term(r#"on_assertion = "any_failed""#);
        let results = [passed(), failed()];
        assert_eq!(
            check(at(0.0), at(0.0), &t, &results),
            Some(TerminationReason::OnAssertion { result: failed() })
        );
    }

    #[test]
    fn any_failed_ignores_passes_and_pending() {
        let t = term(r#"on_assertion = "any_failed""#);
        let results = [passed(), AssertionResult::Pending];
        assert_eq!(check(at(0.0), at(0.0), &t, &results), None);
    }

    #[test]
    fn assertion_trigger_wins_over_an_elapsed_time_budget() {
        // Both triggers would fire this tick; the assertion outcome is the more
        // informative result, so it's the one reported.
        let t = term(
            r#"
            max_simulated_seconds = 5.0
            on_assertion = "any_failed"
            "#,
        );
        let results = [failed()];
        assert_eq!(
            check(at(10.0), at(0.0), &t, &results),
            Some(TerminationReason::OnAssertion { result: failed() })
        );
    }

    #[test]
    fn time_budget_still_fires_when_the_assertion_trigger_does_not_match() {
        // on_assertion set but nothing failed; the time budget is the fallback.
        let t = term(
            r#"
            max_simulated_seconds = 5.0
            on_assertion = "any_failed"
            "#,
        );
        let results = [passed()];
        assert_eq!(
            check(at(6.0), at(0.0), &t, &results),
            Some(TerminationReason::MaxSimulatedSeconds)
        );
    }
}
