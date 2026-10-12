use super::{state::ContinuousStatus, Phase, Runner, FOREIGN_COLLECTOR, NEVER_SET_UP};
use crate::{
    assertion::{evaluator::evaluate_latest, Assertion, AssertionResult, FailureKind},
    collector::ObservationCollector,
    report::{AssertionReportEntry, AssertionStatus, Report, ReportStatus, TerminatedBy},
    run::termination::TerminationReason,
};
use helios_core::prelude::MonotonicTime;

impl Runner {
    /// Build the run's [`Report`] once the host stops ticking. Continuous
    /// assertions are read from their latches (already folded across every
    /// tick by `tick`); terminal assertions are judged here, once, on the
    /// latest sample the run reached. An assertion that never saw a sample
    /// fails. The run is `Passed` only if every assertion passed and setup
    /// didn't fail.
    ///
    /// A run that isn't ready judges nothing: its assertions are reported
    /// `Pending`, and the run fails on its setup failures.
    ///
    /// Takes `&self`, not `&mut`: the latches are only read, nothing is
    /// recorded. `run_name`, `wall_duration_secs`, the stop `reason`, and
    /// `master_seed` are supplied by the host — the runner reads no wall clock,
    /// stores no name, and does not own the seed. `master_seed` is `None` for an
    /// unseeded run, whose real seed is chosen inside the sim and unknown here.
    pub fn finalize(
        &self,
        sim_now: MonotonicTime,
        wall_duration_secs: f64,
        run_name: String,
        terminated_by_reason: TerminationReason,
        collector: &ObservationCollector,
        master_seed: Option<u64>,
    ) -> Report {
        // A host that finalizes without having set the run up, or with
        // another run's collector, fails it here as `tick` would have.
        let mut setup_failures = self.setup_failures.clone();
        let series = match &self.phase {
            Phase::Ready(plan) => {
                let series = plan.series_in(collector);
                if series.is_none() {
                    setup_failures.push(FOREIGN_COLLECTOR.to_string());
                }
                series
            }
            Phase::AwaitingPipelines => {
                setup_failures.push(NEVER_SET_UP.to_string());
                None
            }
            Phase::Aborted => None,
        };

        let mut entries = Vec::with_capacity(self.run.assertions().len());
        // Run-level verdict accumulator: flipped false the moment any assertion
        // is not Passed, so it survives the loop without a second pass.
        let mut all_passed = true;

        for (i, assertion) in self.run.assertions().iter().enumerate() {
            let state = &self.states[i];
            // Each branch produces the two things the entry needs — a verdict
            // and the observed value. Continuous reads its latch (its
            // `last_value` is already an `Option`); terminal is judged now and
            // the value pulled from the fresh result.
            let (status, actual) = match (&series, assertion) {
                (None, _) => (AssertionStatus::Pending, None),
                // Never saw a sample: fails as no samples.
                (Some(_), Assertion::Continuous(_))
                    if *state.status() == ContinuousStatus::Pending =>
                {
                    (AssertionStatus::Failed, None)
                }
                (Some(_), Assertion::Continuous(_)) => (
                    AssertionStatus::from(state.status()),
                    state.last_value().clone(),
                ),
                (Some(series), Assertion::Terminal(_)) => {
                    let result = match evaluate_latest(
                        assertion,
                        series[i].rejection(),
                        state.reached(series[i].samples()),
                    ) {
                        AssertionResult::Pending => AssertionResult::Failed {
                            kind: FailureKind::NoSamples,
                            actual: None,
                        },
                        judged => judged,
                    };
                    // `from(&result)` borrows for the verdict; `into_actual`
                    // then moves the value out — borrow ends before the move,
                    // so this tuple order is what keeps both legal.
                    (AssertionStatus::from(&result), result.into_actual())
                }
            };

            if !matches!(status, AssertionStatus::Passed) {
                all_passed = false;
            }

            entries.push(AssertionReportEntry::new(
                assertion.target().as_str().to_string(),
                *assertion.condition(),
                assertion.expected().clone(),
                actual,
                status,
            ));
        }

        // A setup failure outranks the assertion tally: a run whose scene never
        // became fit to test cannot pass, however few assertions were left to
        // judge it.
        let run_status = if all_passed && setup_failures.is_empty() {
            ReportStatus::Passed
        } else {
            ReportStatus::Failed
        };

        // Elapsed simulated time from the origin `tick` stamped on its first
        // call. `None` only if finalize is reached with no tick having run;
        // report zero rather than panic.
        let simulated_duration_secs = match self.start_time {
            Some(start) => sim_now.0 - start.0,
            None => 0.0f64,
        };

        Report::new(
            run_name,
            self.run.scenario().path().to_string(),
            master_seed,
            run_status,
            setup_failures,
            simulated_duration_secs,
            wall_duration_secs,
            TerminatedBy::from(&terminated_by_reason),
            entries,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        assertion::AssertionValue,
        runner::fixtures::{at, cont, ready, report, run, term, CAR},
    };

    const BUDGET: &str = "max_simulated_seconds = 5.0";

    // Build the report the way `finalize_system` does.
    fn finalize(r: &Runner, collector: &ObservationCollector) -> Report {
        r.finalize(
            at(0.0),
            0.0,
            "run".to_string(),
            TerminationReason::MaxSimulatedSeconds,
            collector,
            None,
        )
    }

    fn statuses(report: &Report) -> Vec<AssertionStatus> {
        report
            .assertions()
            .iter()
            .map(|entry| entry.status().clone())
            .collect()
    }

    #[test]
    fn a_terminal_assertion_is_judged_on_the_latest_sample_reached() {
        // The 9.0 arrives after the run's last tick, past its end, so the
        // verdict rests on the 5.0 the run reached.
        let (mut r, mut collector) = ready(run(BUDGET, &[&term(CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        r.tick(at(1.0), &collector);
        report(&mut collector, CAR, 1.5, 9.0);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
        assert_eq!(
            report.assertions()[0].actual(),
            Some(&AssertionValue::Float(5.0))
        );
        assert_eq!(report.status(), &ReportStatus::Passed);
    }

    #[test]
    fn an_assertion_that_never_saw_a_sample_fails() {
        // "Never emitted" is how a silent drop looks, so it can't pass, for
        // either kind of assertion.
        let (mut r, collector) = ready(run(BUDGET, &[&term(CAR, 5.0), &cont(CAR, 5.0)]), &[CAR]);
        r.tick(at(1.0), &collector);

        let report = finalize(&r, &collector);

        assert_eq!(
            statuses(&report),
            [AssertionStatus::Failed, AssertionStatus::Failed]
        );
        assert_eq!(report.status(), &ReportStatus::Failed);
    }

    #[test]
    fn a_continuous_assertion_reports_its_latch() {
        let (mut r, mut collector) = ready(run(BUDGET, &[&cont(CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        r.tick(at(1.0), &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
        assert_eq!(report.status(), &ReportStatus::Passed);
    }

    #[test]
    fn finalizing_with_another_runs_collector_fails_the_run() {
        let (mut r, mut own) = ready(run(BUDGET, &[&term(CAR, 5.0)]), &[CAR]);
        report(&mut own, CAR, 0.5, 5.0);
        r.tick(at(1.0), &own);
        let other = r.collector();

        let report = finalize(&r, &other);

        assert_eq!(report.status(), &ReportStatus::Failed);
        assert_eq!(report.setup_failures(), [FOREIGN_COLLECTOR]);
        assert_eq!(statuses(&report), [AssertionStatus::Pending]);
    }

    #[test]
    fn finalizing_a_run_never_set_up_fails_it() {
        // With no assertions there is nothing else to fail it: it would
        // otherwise report a pass for a run that never ran.
        let r = Runner::new(run(BUDGET, &[])).expect("valid run");
        let collector = r.collector();

        let report = finalize(&r, &collector);

        assert_eq!(report.status(), &ReportStatus::Failed);
        assert_eq!(report.setup_failures(), [NEVER_SET_UP]);
    }

    #[test]
    fn a_setup_failure_fails_a_run_with_no_assertions() {
        // The regression this guards: a scenario whose agent has no pipeline
        // usually has no assertion naming it either. Judged on assertions
        // alone, an empty set is a vacuous pass and the run reports success
        // for a scene that could never have been tested.
        let mut r = Runner::new(run(BUDGET, &[])).expect("valid run");
        let collector = r.collector();
        r.fail_setup("agent 'car' has no autonomy pipeline".to_string());

        let report = finalize(&r, &collector);

        assert_eq!(report.status(), &ReportStatus::Failed);
        assert!(report.assertions().is_empty());
    }

    #[test]
    fn a_setup_failure_is_reported_with_its_reason() {
        // status alone tells CI a run failed; without the reason there is
        // nothing in the report to say why, since no assertion failed.
        let mut r = Runner::new(run(BUDGET, &[])).expect("valid run");
        let collector = r.collector();
        r.fail_setup("first".to_string());
        r.fail_setup("second".to_string());

        let report = finalize(&r, &collector);

        assert_eq!(report.setup_failures(), ["first", "second"]);
    }

    #[test]
    fn a_clean_run_records_no_setup_failures() {
        let (r, collector) = ready(run(BUDGET, &[]), &[]);

        let report = finalize(&r, &collector);

        assert_eq!(report.status(), &ReportStatus::Passed);
        assert!(report.setup_failures().is_empty());
    }
}
