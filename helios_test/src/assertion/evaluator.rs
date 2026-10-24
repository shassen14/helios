use crate::{
    assertion::{condition::Condition, AssertionResult, AssertionValue, FailureKind},
    collector::{Rejection, Sample},
    statistics::within_run::Reducer,
};

use helios_core::prelude::MonotonicTime;

/// Judge one value against `condition`: a sample, for an assertion that
/// judges every sample, or a series' reduced value. Pure: the caller hands
/// in the value, so the verdict depends on nothing else and is unit-testable
/// without a run.
///
/// Never `Pending`: there is a value to judge.
pub fn evaluate(condition: &Condition, actual: AssertionValue) -> AssertionResult {
    // `Ok(false)` is "compared and didn't hold"; `Err` is "couldn't compare
    // at all". Either way the observed value rides along so the report can
    // show actual-vs-expected. Both `ConditionError` variants fold to
    // `TypeMismatch`; split if a report ever needs to tell them apart.
    match condition.apply(&actual) {
        Ok(true) => AssertionResult::Passed { actual },
        Ok(false) => AssertionResult::Failed {
            kind: FailureKind::ConditionFailed,
            actual: Some(actual),
        },
        Err(_) => AssertionResult::Failed {
            kind: FailureKind::TypeMismatch,
            actual: Some(actual),
        },
    }
}

/// The sample of `samples` with the smallest stamp that fails `condition`,
/// and why; of several with that stamp, the first to arrive. Arrival order
/// isn't stamp order, so an every-sample assertion's failure is placed here
/// rather than at the first failure to arrive.
pub fn earliest_failure<'s>(
    condition: &Condition,
    samples: &'s [Sample],
) -> Option<(&'s Sample, FailureKind)> {
    samples
        .iter()
        .filter_map(|sample| match evaluate(condition, sample.value.clone()) {
            AssertionResult::Failed { kind, .. } => Some((sample, kind)),
            AssertionResult::Passed { .. } | AssertionResult::Pending => None,
        })
        .reduce(|earliest, failing| {
            if failing.0.timestamp.0 < earliest.0.timestamp.0 {
                failing
            } else {
                earliest
            }
        })
}

/// Judge `samples` reduced by `reducer` over the window from `start` to
/// `end`. A series that turned a value away fails for that reason, whatever
/// else it holds; one the reducer has no value for (an empty window) is
/// `Pending`; a sample the reducer can't read is a `TypeMismatch`.
pub fn evaluate_reduced(
    condition: &Condition,
    reducer: &Reducer,
    rejection: Option<Rejection>,
    samples: &[Sample],
    start: MonotonicTime,
    end: MonotonicTime,
) -> AssertionResult {
    if let Some(rejection) = rejection {
        return AssertionResult::Failed {
            kind: FailureKind::Rejected(rejection),
            actual: None,
        };
    }
    match reducer.reduce(samples, start, end) {
        Ok(Some(value)) => evaluate(condition, value),
        Ok(None) => AssertionResult::Pending,
        Err(_) => AssertionResult::Failed {
            kind: FailureKind::TypeMismatch,
            actual: None,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::extract::ExtractError;

    fn at(secs: f64) -> MonotonicTime {
        MonotonicTime(secs)
    }

    fn sample(stamp: f64, value: f64) -> Sample {
        Sample {
            timestamp: at(stamp),
            value: AssertionValue::Float(value),
        }
    }

    fn reduced(
        condition: &Condition,
        reducer: &Reducer,
        rejection: Option<Rejection>,
        samples: &[Sample],
    ) -> AssertionResult {
        evaluate_reduced(condition, reducer, rejection, samples, at(0.0), at(10.0))
    }

    #[test]
    fn passes_when_condition_holds() {
        assert_eq!(
            evaluate(&Condition::LessThan(5.0), AssertionValue::Float(4.0)),
            AssertionResult::Passed {
                actual: AssertionValue::Float(4.0),
            }
        );
    }

    #[test]
    fn condition_failed_carries_observed_value() {
        assert_eq!(
            evaluate(&Condition::LessThan(5.0), AssertionValue::Float(9.0)),
            AssertionResult::Failed {
                kind: FailureKind::ConditionFailed,
                actual: Some(AssertionValue::Float(9.0)),
            }
        );
    }

    #[test]
    fn within_range_judges_its_own_bounds() {
        let range = Condition::WithinRange([1.0, 3.0]);
        assert_eq!(
            evaluate(&range, AssertionValue::Float(2.0)),
            AssertionResult::Passed {
                actual: AssertionValue::Float(2.0),
            }
        );
        assert_eq!(
            evaluate(&range, AssertionValue::Float(5.0)),
            AssertionResult::Failed {
                kind: FailureKind::ConditionFailed,
                actual: Some(AssertionValue::Float(5.0)),
            }
        );
    }

    #[test]
    fn type_mismatch_when_condition_cannot_compare() {
        assert_eq!(
            evaluate(&Condition::LessThan(1.0), AssertionValue::Bool(true)),
            AssertionResult::Failed {
                kind: FailureKind::TypeMismatch,
                actual: Some(AssertionValue::Bool(true)),
            }
        );
    }

    #[test]
    fn final_judges_the_latest_stamp() {
        // The later-stamped sample arrived first.
        assert_eq!(
            reduced(
                &Condition::LessThan(5.0),
                &Reducer::Final,
                None,
                &[sample(2.0, 1.0), sample(1.0, 9.0)]
            ),
            AssertionResult::Passed {
                actual: AssertionValue::Float(1.0),
            }
        );
    }

    #[test]
    fn a_reducer_with_no_value_is_pending() {
        assert_eq!(
            reduced(&Condition::LessThan(5.0), &Reducer::Mean, None, &[]),
            AssertionResult::Pending
        );
    }

    #[test]
    fn count_on_an_empty_series_is_judged() {
        assert_eq!(
            reduced(
                &Condition::Equals(AssertionValue::Int(0)),
                &Reducer::Count,
                None,
                &[]
            ),
            AssertionResult::Passed {
                actual: AssertionValue::Int(0),
            }
        );
    }

    #[test]
    fn max_with_a_nan_sample_fails_its_bound() {
        // As judging each sample would: NaN < 25 is false.
        let result = reduced(
            &Condition::LessThan(25.0),
            &Reducer::Max,
            None,
            &[sample(1.0, 1.0), sample(2.0, f64::NAN)],
        );
        assert!(
            matches!(
                result,
                AssertionResult::Failed {
                    kind: FailureKind::ConditionFailed,
                    ..
                }
            ),
            "{result:?}"
        );
    }

    #[test]
    fn a_number_reducer_on_a_bool_is_a_type_mismatch() {
        let flags = [Sample {
            timestamp: at(1.0),
            value: AssertionValue::Bool(true),
        }];
        assert_eq!(
            reduced(&Condition::LessThan(1.0), &Reducer::Mean, None, &flags),
            AssertionResult::Failed {
                kind: FailureKind::TypeMismatch,
                actual: None,
            }
        );
    }

    #[test]
    fn a_rejected_value_fails_whatever_else_arrived() {
        let rejection = Rejection::Extract(ExtractError::PayloadMismatch { payload: "f64" });
        assert_eq!(
            reduced(
                &Condition::LessThan(5.0),
                &Reducer::Final,
                Some(rejection),
                &[sample(1.0, 1.0)]
            ),
            AssertionResult::Failed {
                kind: FailureKind::Rejected(rejection),
                actual: None,
            }
        );
    }

    #[test]
    fn the_earliest_failure_is_by_stamp_not_arrival() {
        let samples = [sample(3.0, 9.0), sample(1.0, 1.0), sample(2.0, 8.0)];
        let (failing, kind) =
            earliest_failure(&Condition::LessThan(5.0), &samples).expect("two samples fail");
        assert_eq!(failing, &sample(2.0, 8.0));
        assert_eq!(kind, FailureKind::ConditionFailed);
        assert_eq!(earliest_failure(&Condition::LessThan(10.0), &samples), None);
    }
}
