use helios_core::prelude::MonotonicTime;

use crate::run::termination::{Termination, TerminationReason};

/// Decide whether the run should stop this tick, and if so why. Pure and
/// read-only: it judges nothing itself. `first_failed` is the number of the
/// first continuous assertion, in run-file order, whose latch has failed.
/// `Some(reason)` means stop; `None` means keep going. The caller acts on the
/// decision — this never terminates anything.
///
/// Precedence: a failure is checked before the time budget, because "an
/// assertion failed" is the meaningful test result, whereas "time ran out" is
/// the fallback that only matters when nothing failed.
pub fn check(
    now: MonotonicTime,
    start: MonotonicTime,
    termination: &Termination,
    first_failed: Option<usize>,
) -> Option<TerminationReason> {
    if termination.fail_fast() {
        if let Some(number) = first_failed {
            return Some(TerminationReason::AssertionFailed { number });
        }
    }

    if termination.budget_spent(start, now) {
        return Some(TerminationReason::MaxSimulatedSeconds);
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;

    // Build a `Termination` by parsing TOML, since its fields are private.
    fn term(body: &str) -> Termination {
        toml::from_str(body).unwrap()
    }

    fn at(secs: f64) -> MonotonicTime {
        MonotonicTime(secs)
    }

    const BUDGET: &str = "max_simulated_seconds = 5.0";

    const FAIL_FAST: &str = r#"
        max_simulated_seconds = 5.0
        fail_fast = true
    "#;

    #[test]
    fn time_budget_fires_once_elapsed() {
        assert_eq!(
            check(at(6.0), at(0.0), &term(BUDGET), None),
            Some(TerminationReason::MaxSimulatedSeconds)
        );
    }

    #[test]
    fn time_budget_fires_exactly_at_the_boundary() {
        // `>=`: reaching the budget on the nose stops here, not a tick later.
        assert_eq!(
            check(at(5.0), at(0.0), &term(BUDGET), None),
            Some(TerminationReason::MaxSimulatedSeconds)
        );
    }

    #[test]
    fn time_budget_holds_before_the_boundary() {
        assert_eq!(check(at(4.999), at(0.0), &term(BUDGET), None), None);
    }

    #[test]
    fn elapsed_is_measured_from_start_not_zero() {
        // 12 - 8 = 4 < 5, so a nonzero start must not read as already elapsed.
        assert_eq!(check(at(12.0), at(8.0), &term(BUDGET), None), None);
    }

    #[test]
    fn fail_fast_stops_on_a_failure_and_names_it() {
        assert_eq!(
            check(at(0.0), at(0.0), &term(FAIL_FAST), Some(2)),
            Some(TerminationReason::AssertionFailed { number: 2 })
        );
    }

    #[test]
    fn a_failure_without_fail_fast_runs_on() {
        assert_eq!(check(at(0.0), at(0.0), &term(BUDGET), Some(2)), None);
    }

    #[test]
    fn a_failure_wins_over_an_elapsed_time_budget() {
        // Both would stop this tick; the failure is the more informative result.
        assert_eq!(
            check(at(10.0), at(0.0), &term(FAIL_FAST), Some(1)),
            Some(TerminationReason::AssertionFailed { number: 1 })
        );
    }

    #[test]
    fn time_budget_still_fires_under_fail_fast_with_no_failure() {
        assert_eq!(
            check(at(6.0), at(0.0), &term(FAIL_FAST), None),
            Some(TerminationReason::MaxSimulatedSeconds)
        );
    }
}
