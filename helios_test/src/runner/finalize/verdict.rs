//! [`judge`]: one assertion's verdict, from its window's placement and the
//! samples the run reached in it.

use super::windowed::Windowed;
use crate::{
    assertion::{
        evaluator::{earliest_failure, evaluate_reduced},
        Assertion, AssertionResult,
    },
    collector::Rejection,
    report::{FailureReason, Verdict},
    runner::window::Placement,
    statistics::within_run::latest,
};

/// One assertion's report verdict. A value its series turned away fails it
/// whatever its window. Otherwise the window decides: unplaced is
/// `Unjudged`, an anchor that never occurred fails, and a placed window is
/// judged on its samples, a failure final either way and a pass or silence
/// only when the run covered it.
pub(super) fn judge(
    assertion: &Assertion,
    rejection: Option<Rejection>,
    windowed: &Windowed,
) -> Verdict {
    if rejection.is_some() {
        return Verdict::Failed {
            actual: None,
            reason: FailureReason::Rejected,
            failed_at_secs: None,
        };
    }
    let window = match windowed.placement {
        Placement::Placed { window, .. } => window,
        Placement::Unplaced => return Verdict::Unjudged { actual: None },
        Placement::AnchorNeverOccurred => {
            return Verdict::Failed {
                actual: None,
                reason: FailureReason::AnchorNeverOccurred,
                failed_at_secs: None,
            }
        }
    };
    let samples = &windowed.samples;

    let Some(reducer) = &assertion.reduce else {
        // The value that broke it, at its own stamp, not whatever arrived
        // after: a run that recovered would otherwise show a healthy number
        // beside FAILED.
        if let Some((failing, kind)) = earliest_failure(&assertion.condition, samples) {
            return Verdict::Failed {
                actual: Some(failing.value.clone()),
                reason: FailureReason::from(&kind),
                failed_at_secs: Some(windowed.secs(failing.timestamp)),
            };
        }
        let last = latest(samples).map(|sample| sample.value.clone());
        if !windowed.covered() {
            return Verdict::Unjudged { actual: last };
        }
        // Silent over its whole window: a drop.
        return match last {
            Some(actual) => Verdict::Passed { actual },
            None => no_samples(),
        };
    };

    // Any other verdict belongs to an end the run never reached. The value
    // reduced so far is still shown.
    if !windowed.covered() {
        return Verdict::Unjudged {
            actual: reducer
                .reduce(samples, window.start, window.end)
                .ok()
                .flatten(),
        };
    }
    match evaluate_reduced(
        &assertion.condition,
        reducer,
        None,
        samples,
        window.start,
        window.end,
    ) {
        AssertionResult::Pending => no_samples(),
        judged => Verdict::from(judged),
    }
}

/// The verdict of an assertion that saw no value over its whole window.
fn no_samples() -> Verdict {
    Verdict::Failed {
        actual: None,
        reason: FailureReason::NoSamples,
        failed_at_secs: None,
    }
}

#[cfg(test)]
mod tests {
    use crate::runner::finalize::fixtures::{
        budget, failed_first, finalize, finalize_stopped_by, run_out, statuses,
    };

    use crate::{
        assertion::AssertionValue,
        report::{AssertionStatus, FailureReason, ReportStatus, Verdict},
        runner::fixtures::{
            at, cont, ready, reduced_below, report, report_unreadable, run, term, CAR, TRUCK,
        },
    };

    #[test]
    fn a_final_assertion_is_judged_on_the_latest_sample_reached() {
        // The 9.0 arrives after the run's last tick, past its end, so the
        // verdict rests on the 5.0 the run reached.
        let (mut r, mut collector) = ready(run(&budget(), &[&term("x1", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        run_out(&mut r, &collector);
        report(&mut collector, CAR, 5.5, 9.0);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
        assert_eq!(
            report.assertions()[0].verdict().actual(),
            Some(&AssertionValue::Float(5.0))
        );
        assert_eq!(report.status(), &ReportStatus::Passed);
    }

    #[test]
    fn an_assertion_that_never_saw_a_sample_fails() {
        // "Never emitted" is how a silent drop looks, so it can't pass, for
        // either kind of assertion.
        let (mut r, collector) = ready(
            run(&budget(), &[&term("x2", CAR, 5.0), &cont("x3", CAR, 5.0)]),
            &[CAR],
        );
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(
            statuses(&report),
            [AssertionStatus::Failed, AssertionStatus::Failed]
        );
        for entry in report.assertions() {
            assert_eq!(
                entry.verdict(),
                &Verdict::Failed {
                    actual: None,
                    reason: FailureReason::NoSamples,
                    failed_at_secs: None,
                }
            );
        }
        assert_eq!(report.status(), &ReportStatus::Failed);
    }

    #[test]
    fn an_every_sample_failure_reports_the_value_that_broke_it() {
        // It breaks at 1.0 s and recovers at 1.5 s; the report shows the
        // breaking value and its time, not the healthy value that came last.
        let (mut r, mut collector) = ready(run(&budget(), &[&cont("x4", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        report(&mut collector, CAR, 1.0, 9.0);
        report(&mut collector, CAR, 1.5, 5.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);
        let entry = &report.assertions()[0];

        assert_eq!(
            entry.verdict(),
            &Verdict::Failed {
                actual: Some(AssertionValue::Float(9.0)),
                reason: FailureReason::ConditionFailed,
                failed_at_secs: Some(1.0),
            }
        );
    }

    #[test]
    fn an_early_stop_leaves_what_had_not_failed_unjudged() {
        // Assertion 1 failed and stopped the run. Assertion 2 held so far,
        // 3 never saw a sample, and 4 is reduced: none of them saw the end
        // of its window, so none is passed, or failed for silence.
        let assertions = [
            cont("first", CAR, 5.0),
            cont("x6", CAR, 9.0),
            cont("x7", TRUCK, 5.0),
            term("x8", CAR, 9.0),
        ];
        let blocks: Vec<&str> = assertions.iter().map(String::as_str).collect();
        let (mut r, mut collector) = ready(run(&budget(), &blocks), &[CAR, TRUCK]);
        report(&mut collector, CAR, 0.5, 9.0);
        r.tick(at(1.0), &collector);

        let report = finalize_stopped_by(&r, &collector, failed_first());

        assert_eq!(
            statuses(&report),
            [
                AssertionStatus::Failed,
                AssertionStatus::Unjudged,
                AssertionStatus::Unjudged,
                AssertionStatus::Unjudged,
            ]
        );
        assert_eq!(report.status(), &ReportStatus::Failed);
    }

    #[test]
    fn an_unjudged_reduced_assertion_shows_the_value_it_stopped_at() {
        let (mut r, mut collector) = ready(
            run(
                &budget(),
                &[&cont("first", CAR, 5.0), &term("x10", CAR, 5.0)],
            ),
            &[CAR],
        );
        report(&mut collector, CAR, 0.5, 7.0);
        r.tick(at(1.0), &collector);

        let report = finalize_stopped_by(&r, &collector, failed_first());

        assert_eq!(
            report.assertions()[1].verdict().actual(),
            Some(&AssertionValue::Float(7.0))
        );
    }

    #[test]
    fn an_every_sample_assertion_reports_its_latch() {
        let (mut r, mut collector) = ready(run(&budget(), &[&cont("x11", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
        assert_eq!(report.status(), &ReportStatus::Passed);
    }

    #[test]
    fn a_value_turned_away_before_the_end_fails_the_assertion() {
        let (mut r, mut collector) = ready(run(&budget(), &[&term("x15", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        report_unreadable(&mut collector, CAR, 1.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Failed]);
    }

    #[test]
    fn a_value_turned_away_after_the_last_tick_is_past_the_end() {
        // Like a late sample, a late rejection is outside the run.
        let (mut r, mut collector) = ready(run(&budget(), &[&term("x16", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        run_out(&mut r, &collector);
        report_unreadable(&mut collector, CAR, 5.5);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
    }

    #[test]
    fn a_reduced_assertion_is_judged_on_its_reduced_value() {
        // The mean of 1, 2 and 6 is 3: below 4 though one sample isn't.
        let (mut r, mut collector) = ready(
            run(
                &budget(),
                &[&reduced_below("mean_low", CAR, "\"mean\"", 4.0)],
            ),
            &[CAR],
        );
        for (at, value) in [(0.5, 1.0), (1.0, 2.0), (1.5, 6.0)] {
            report(&mut collector, CAR, at, value);
        }
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(
            report.assertions()[0].verdict(),
            &Verdict::Passed {
                actual: AssertionValue::Float(3.0)
            }
        );
    }

    #[test]
    fn max_with_a_nan_sample_fails_as_judging_each_sample_would() {
        let (mut r, mut collector) = ready(
            run(
                &budget(),
                &[
                    &reduced_below("max_low", CAR, "\"max\"", 25.0),
                    &format!(
                        "name = \"each_low\"\ntarget = \"agent.{CAR}.node.estimator.x\"\n\
                         condition = {{ less_than = 25.0 }}"
                    ),
                ],
            ),
            &[CAR],
        );
        report(&mut collector, CAR, 0.5, 1.0);
        report(&mut collector, CAR, 1.0, f64::NAN);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(
            statuses(&report),
            [AssertionStatus::Failed, AssertionStatus::Failed]
        );
    }

    #[test]
    fn count_equals_zero_passes_on_a_silent_series() {
        // "Expect no samples": `count` is defined on an empty window.
        let none = format!(
            "name = \"never\"\ntarget = \"agent.{CAR}.node.estimator.x\"\n\
             reduce = \"count\"\ncondition = {{ equals = 0 }}"
        );
        let (mut r, collector) = ready(run(&budget(), &[&none]), &[CAR]);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(
            report.assertions()[0].verdict(),
            &Verdict::Passed {
                actual: AssertionValue::Int(0)
            }
        );
        assert_eq!(report.status(), &ReportStatus::Passed);
    }

    #[test]
    fn an_every_sample_failure_is_placed_at_the_earliest_failing_stamp() {
        // The failure at 3 s arrives first; the one at 2 s is earlier.
        let (mut r, mut collector) = ready(run(&budget(), &[&cont("x30", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 3.0, 9.0);
        report(&mut collector, CAR, 2.0, 8.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(
            report.assertions()[0].verdict(),
            &Verdict::Failed {
                actual: Some(AssertionValue::Float(8.0)),
                reason: FailureReason::ConditionFailed,
                failed_at_secs: Some(2.0),
            }
        );
    }
}
