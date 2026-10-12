use crate::{
    assertion::{condition::Condition, Assertion, AssertionResult, AssertionValue, FailureKind},
    collector::{Rejection, Sample},
};

/// Judge one observed value against `assertion`. Pure: the caller hands in
/// the value, so the verdict depends on nothing else and is unit-testable
/// without a run.
///
/// Never `Pending`: there is a value to judge.
pub fn evaluate(assertion: &Assertion, actual: AssertionValue) -> AssertionResult {
    // `Ok(false)` is "compared and didn't hold"; `Err(kind)` is "couldn't
    // compare at all" (bad bounds or incompatible types). Either way the
    // observed value rides along so the report can show actual-vs-expected.
    match check(assertion, &actual) {
        Ok(true) => AssertionResult::Passed { actual },
        Ok(false) => AssertionResult::Failed {
            kind: FailureKind::ConditionFailed,
            actual: Some(actual),
        },
        Err(kind) => AssertionResult::Failed {
            kind,
            actual: Some(actual),
        },
    }
}

/// Judge `assertion` on the newest of `samples`, the last to arrive. A
/// series that turned a value away fails for that reason, whatever else it
/// holds; with no samples the verdict is `Pending`.
pub fn evaluate_latest(
    assertion: &Assertion,
    rejection: Option<Rejection>,
    samples: &[Sample],
) -> AssertionResult {
    if let Some(rejection) = rejection {
        return AssertionResult::Failed {
            kind: FailureKind::Rejected(rejection),
            actual: None,
        };
    }
    match samples.last() {
        Some(sample) => evaluate(assertion, sample.value.clone()),
        None => AssertionResult::Pending,
    }
}

/// Run the assertion's condition against the observed value. `Ok(held)` means
/// the comparison ran (held or not); `Err(kind)` means it couldn't run.
///
/// `WithinRange` is dispatched separately because its bounds live on the
/// assertion struct, not in the value pair that `apply` compares — and a range
/// missing a bound is `InvalidBounds`, a failure kind `ConditionError` can't
/// express. Both `ConditionError` variants fold to `TypeMismatch` for now;
/// split if a report ever needs to tell a type mismatch from a not-comparable.
fn check(assertion: &Assertion, actual: &AssertionValue) -> Result<bool, FailureKind> {
    if *assertion.condition() == Condition::WithinRange {
        if let Some((low, high)) = assertion.bounds() {
            Condition::apply_range(actual, low, high).map_err(|_| FailureKind::TypeMismatch)
        } else {
            Err(FailureKind::InvalidBounds)
        }
    } else {
        assertion
            .condition()
            .apply(actual, assertion.expected())
            .map_err(|_| FailureKind::TypeMismatch)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::{extract::ExtractError, terminal::TerminalAssertion, AssertionTarget};

    use helios_core::prelude::MonotonicTime;

    const TARGET: &str = "agent.car.estimator.x";

    fn terminal(condition: Condition, value: AssertionValue) -> Assertion {
        Assertion::Terminal(TerminalAssertion {
            target: AssertionTarget::parse(TARGET).expect("valid target"),
            condition,
            value,
            low: None,
            high: None,
            allow_nondeterministic: false,
        })
    }

    fn terminal_range(low: Option<f64>, high: Option<f64>) -> Assertion {
        Assertion::Terminal(TerminalAssertion {
            target: AssertionTarget::parse(TARGET).expect("valid target"),
            condition: Condition::WithinRange,
            value: AssertionValue::Float(0.0),
            low,
            high,
            allow_nondeterministic: false,
        })
    }

    fn sample(at: f64, value: f64) -> Sample {
        Sample {
            timestamp: MonotonicTime(at),
            value: AssertionValue::Float(value),
        }
    }

    #[test]
    fn passes_when_condition_holds() {
        let a = terminal(Condition::Equals, AssertionValue::Float(5.0));
        assert_eq!(
            evaluate(&a, AssertionValue::Float(5.0)),
            // `Passed` carries the observed value, just like the `Failed` cases.
            AssertionResult::Passed {
                actual: AssertionValue::Float(5.0),
            }
        );
    }

    #[test]
    fn condition_failed_carries_observed_value() {
        let a = terminal(Condition::Equals, AssertionValue::Float(9.0));
        assert_eq!(
            evaluate(&a, AssertionValue::Float(5.0)),
            AssertionResult::Failed {
                kind: FailureKind::ConditionFailed,
                actual: Some(AssertionValue::Float(5.0)),
            }
        );
    }

    #[test]
    fn within_range_passes_inside_bounds() {
        let a = terminal_range(Some(1.0), Some(3.0));
        assert_eq!(
            evaluate(&a, AssertionValue::Float(2.0)),
            AssertionResult::Passed {
                actual: AssertionValue::Float(2.0),
            }
        );
    }

    #[test]
    fn within_range_fails_outside_bounds() {
        let a = terminal_range(Some(1.0), Some(3.0));
        assert_eq!(
            evaluate(&a, AssertionValue::Float(5.0)),
            AssertionResult::Failed {
                kind: FailureKind::ConditionFailed,
                actual: Some(AssertionValue::Float(5.0)),
            }
        );
    }

    #[test]
    fn invalid_bounds_when_range_missing_a_bound() {
        // `within_range` with only one bound set → `bounds()` returns None.
        let a = terminal_range(Some(1.0), None);
        assert_eq!(
            evaluate(&a, AssertionValue::Float(2.0)),
            AssertionResult::Failed {
                kind: FailureKind::InvalidBounds,
                actual: Some(AssertionValue::Float(2.0)),
            }
        );
    }

    #[test]
    fn type_mismatch_when_condition_cannot_compare() {
        // `less_than` on a bool is not an ordering comparison; the resulting
        // ConditionError folds to TypeMismatch.
        let a = terminal(Condition::LessThan, AssertionValue::Bool(false));
        assert_eq!(
            evaluate(&a, AssertionValue::Bool(true)),
            AssertionResult::Failed {
                kind: FailureKind::TypeMismatch,
                actual: Some(AssertionValue::Bool(true)),
            }
        );
    }

    #[test]
    fn the_latest_sample_is_the_last_to_arrive() {
        // Arrival order, not time order: the later-stamped sample came first.
        let a = terminal(Condition::Equals, AssertionValue::Float(1.0));
        assert_eq!(
            evaluate_latest(&a, None, &[sample(2.0, 9.0), sample(1.0, 1.0)]),
            AssertionResult::Passed {
                actual: AssertionValue::Float(1.0),
            }
        );
    }

    #[test]
    fn no_samples_is_pending() {
        let a = terminal(Condition::Equals, AssertionValue::Float(1.0));
        assert_eq!(evaluate_latest(&a, None, &[]), AssertionResult::Pending);
    }

    #[test]
    fn a_rejected_value_fails_whatever_else_arrived() {
        let a = terminal(Condition::Equals, AssertionValue::Float(1.0));
        let rejection = Rejection::Extract(ExtractError::PayloadMismatch { payload: "f64" });
        assert_eq!(
            evaluate_latest(&a, Some(rejection), &[sample(1.0, 1.0)]),
            AssertionResult::Failed {
                kind: FailureKind::Rejected(rejection),
                actual: None,
            }
        );
    }
}
