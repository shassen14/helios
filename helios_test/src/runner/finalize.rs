use super::{
    state::{AssertionState, ContinuousStatus},
    Phase, Runner, FOREIGN_COLLECTOR, NEVER_SET_UP,
};
use crate::{
    assertion::{evaluator::evaluate_latest, Assertion, AssertionResult},
    collector::{ObservationCollector, Series},
    report::{
        AssertionReportEntry, AssertionStatus, FailureReason, Report, ReportStatus, TerminatedBy,
        Verdict,
    },
    run::termination::TerminationReason,
};
use helios_core::prelude::{AgentId, MonotonicTime};
use helios_runtime::port::RefusedWrites;

impl Runner {
    /// Build the run's [`Report`] once the host stops ticking. Continuous
    /// assertions are read from their latches (already folded across every
    /// tick by `tick`); terminal assertions are judged here, once, on the
    /// latest sample the run reached.
    ///
    /// Only a run whose ticks reached its time budget covered every
    /// assertion's window, which today is the whole run. The runner reads
    /// that from its own clock, not from the stop reason the host passes,
    /// which only labels the report: a `fail_fast` failure on the tick the
    /// budget runs out still covered it. In one that stopped early (a
    /// `fail_fast` stop or an aborted setup), an assertion that hadn't failed
    /// by then is `Unjudged`: neither its pass nor its silence says anything
    /// about a window the run never finished. A failure is final however the
    /// run stopped. In a run that covered its windows, an assertion that never
    /// saw a sample fails, since that is how a dropped signal looks.
    ///
    /// The run is `Passed` only if every assertion passed, setup didn't fail,
    /// no agent's bus refused a write ([`HostFacts::refused_writes`]) and the
    /// collector dropped no requested value
    /// ([`ObservationCollector::unbound_arrivals`]): values the wiring or the
    /// recording dropped leave the assertions judging another run than the
    /// one configured. Only a run that was set up reports the collector's
    /// drops; in one aborted at setup, a refused assertion's key is unbound
    /// by design. An early stop is always a failure or an abort, so an
    /// `Unjudged` assertion only ever sits in a run that failed for another
    /// reason.
    ///
    /// Takes `&self`, not `&mut`: the latches are only read, nothing is
    /// recorded. Everything else the report needs comes from the host as
    /// [`HostFacts`].
    pub fn finalize(&self, host: HostFacts, collector: &ObservationCollector) -> Report {
        let HostFacts {
            sim_now,
            wall_duration_secs,
            run_name,
            terminated_by_reason,
            master_seed,
            refused_writes,
        } = host;

        // A host that finalizes without having set the run up, or with
        // another run's collector, fails it here as `tick` would have.
        let mut setup_failures = self.setup_failures.clone();
        let judged = match &self.phase {
            Phase::Ready(plan) => {
                let judged = plan.with_series(collector);
                if judged.is_none() {
                    setup_failures.push(FOREIGN_COLLECTOR.to_string());
                }
                judged
            }
            Phase::AwaitingPipelines => {
                setup_failures.push(NEVER_SET_UP.to_string());
                None
            }
            Phase::Aborted => None,
        };
        let covered = match (self.start_time, self.latest_tick) {
            (Some(start), Some(latest)) => self.run.termination().budget_spent(start, latest),
            _ => false,
        };

        let mut entries = Vec::with_capacity(self.run.assertions().len());
        // Run-level verdict accumulator: flipped false the moment any assertion
        // is not Passed, so it survives the loop without a second pass.
        let mut all_passed = true;

        for (i, assertion) in self.run.assertions().iter().enumerate() {
            // This assertion's series and state; `None` when the run isn't
            // ready to judge. A pair of references is `Copy`, so `copied`
            // lifts it out of the borrowed list.
            let verdict = match judged.as_ref().and_then(|j| j.get(i)).copied() {
                Some((series, state)) => judge(assertion, series, state, covered, self.start_time),
                None => Verdict::Unjudged { actual: None },
            };

            if verdict.status() != AssertionStatus::Passed {
                all_passed = false;
            }

            entries.push(AssertionReportEntry::new(
                assertion.target().as_str().to_string(),
                *assertion.condition(),
                assertion.expected().clone(),
                verdict,
            ));
        }

        let refused_writes: Vec<String> = refused_writes
            .into_iter()
            .map(|(agent, refused)| format!("agent '{agent}': {refused}"))
            .collect();

        let unbound_arrivals: Vec<String> = match &self.phase {
            Phase::Ready(_) => collector
                .unbound_arrivals()
                .iter()
                .map(ToString::to_string)
                .collect(),
            Phase::AwaitingPipelines | Phase::Aborted => Vec::new(),
        };

        // A setup failure, a refused write or a dropped value outranks the
        // assertion tally: a run whose scene never became fit to test, or
        // whose wiring or recording dropped values, cannot pass however its
        // assertions came out.
        let run_status = if all_passed
            && setup_failures.is_empty()
            && refused_writes.is_empty()
            && unbound_arrivals.is_empty()
        {
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
            refused_writes,
            unbound_arrivals,
            simulated_duration_secs,
            wall_duration_secs,
            TerminatedBy::from(&terminated_by_reason),
            entries,
        )
    }
}

/// What a finished run's report needs that only the host knows: the runner
/// reads no clock, stores no name, does not own the seed and holds no
/// pipeline.
pub struct HostFacts {
    /// The tick that ended the run, not the clock as it stands at finalize.
    pub sim_now: MonotonicTime,
    pub wall_duration_secs: f64,
    pub run_name: String,
    /// Why the host stopped ticking. It only labels the report; whether the
    /// run covered its windows is read from the runner's own clock.
    pub terminated_by_reason: TerminationReason,
    /// `None` for an unseeded run, whose real seed is chosen inside the sim
    /// and unknown here.
    pub master_seed: Option<u64>,
    /// Every write each agent's bus refused, read from its pipeline.
    pub refused_writes: Vec<(AgentId, RefusedWrites)>,
}

/// One assertion's report verdict. `covered` is whether the run reached the
/// end of the assertion's window: a pass, or a failure for silence, needs it;
/// a failure the assertion already met is final either way. `start` is the
/// run's first tick, which a continuous failure's time is measured from.
fn judge(
    assertion: &Assertion,
    series: &Series,
    state: &AssertionState,
    covered: bool,
    start: Option<MonotonicTime>,
) -> Verdict {
    match assertion {
        Assertion::Continuous(_) => match state.status() {
            // The value that broke it, not whatever arrived after: a run that
            // recovered would otherwise show a healthy number beside FAILED.
            ContinuousStatus::FailedAt { t, value, kind } => Verdict::Failed {
                actual: value.clone(),
                reason: FailureReason::from(kind),
                failed_at_secs: start.map(|start| t.0 - start.0),
            },
            _ if !covered => Verdict::Unjudged {
                actual: state.last_value().clone(),
            },
            // Holding means at least one value held, so there is one to show.
            ContinuousStatus::Holding => match state.last_value() {
                Some(actual) => Verdict::Passed {
                    actual: actual.clone(),
                },
                None => no_samples(),
            },
            // Silent over its whole window: a drop.
            ContinuousStatus::Pending => no_samples(),
        },
        Assertion::Terminal(_) => {
            let reached = state.reached(series.samples());
            // A value turned away before the run stopped is a fault however
            // it stopped; any other verdict belongs to an end the run never
            // reached. The value it stopped at is still shown.
            if !covered && state.rejection().is_none() {
                return Verdict::Unjudged {
                    actual: reached.last().map(|sample| sample.value.clone()),
                };
            }
            match evaluate_latest(assertion, state.rejection(), reached) {
                AssertionResult::Pending => no_samples(),
                judged => Verdict::from(judged),
            }
        }
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
    use super::*;

    use crate::{
        assertion::AssertionValue,
        runner::{
            fixtures::{
                at, cont, pipeline, ready, refused_write, report, report_speed, report_unbound,
                report_unreadable, run, speed_term, term, CAR, TRUCK,
            },
            TickAction,
        },
    };

    const BUDGET_SECS: f64 = 5.0;

    // `{:?}` forces a decimal point so TOML reads it as a float.
    fn budget() -> String {
        format!("max_simulated_seconds = {BUDGET_SECS:?}")
    }

    // Tick from the run's start to its budget, so it covers its window. Each
    // tick reaches whatever has arrived by then.
    fn run_out(r: &mut Runner, collector: &ObservationCollector) {
        r.tick(at(0.0), collector);
        r.tick(at(BUDGET_SECS), collector);
    }

    // Build the report the way `finalize_system` does, for a run the host
    // says ran out its budget.
    fn finalize(r: &Runner, collector: &ObservationCollector) -> Report {
        finalize_stopped_by(r, collector, TerminationReason::MaxSimulatedSeconds)
    }

    fn finalize_stopped_by(
        r: &Runner,
        collector: &ObservationCollector,
        reason: TerminationReason,
    ) -> Report {
        r.finalize(host_facts(reason, Vec::new()), collector)
    }

    // What the host hands finalize: a run named "run", stopped by `reason`,
    // whose buses refused `refused_writes`.
    fn host_facts(
        reason: TerminationReason,
        refused_writes: Vec<(AgentId, RefusedWrites)>,
    ) -> HostFacts {
        HostFacts {
            sim_now: at(0.0),
            wall_duration_secs: 0.0,
            run_name: "run".to_string(),
            terminated_by_reason: reason,
            master_seed: None,
            refused_writes,
        }
    }

    const FAILED_FIRST: TerminationReason = TerminationReason::AssertionFailed { number: 1 };

    fn statuses(report: &Report) -> Vec<AssertionStatus> {
        report
            .assertions()
            .iter()
            .map(|entry| entry.status())
            .collect()
    }

    #[test]
    fn a_terminal_assertion_is_judged_on_the_latest_sample_reached() {
        // The 9.0 arrives after the run's last tick, past its end, so the
        // verdict rests on the 5.0 the run reached.
        let (mut r, mut collector) = ready(run(&budget(), &[&term(CAR, 5.0)]), &[CAR]);
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
        let (mut r, collector) = ready(run(&budget(), &[&term(CAR, 5.0), &cont(CAR, 5.0)]), &[CAR]);
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
    fn a_continuous_failure_reports_the_value_that_broke_it() {
        // It breaks at 1.0 s and recovers at 1.5 s; the report shows the
        // breaking value and its time, not the healthy value that came last.
        let (mut r, mut collector) = ready(run(&budget(), &[&cont(CAR, 5.0)]), &[CAR]);
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
        // 3 never saw a sample, and 4 is terminal: none of them saw the end
        // of its window, so none is passed, or failed for silence.
        let assertions = [
            cont(CAR, 5.0),
            cont(CAR, 9.0),
            cont(TRUCK, 5.0),
            term(CAR, 9.0),
        ];
        let blocks: Vec<&str> = assertions.iter().map(String::as_str).collect();
        let (mut r, mut collector) = ready(run(&budget(), &blocks), &[CAR, TRUCK]);
        report(&mut collector, CAR, 0.5, 9.0);
        r.tick(at(1.0), &collector);

        let report = finalize_stopped_by(&r, &collector, FAILED_FIRST);

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
    fn an_unjudged_terminal_assertion_shows_the_value_it_stopped_at() {
        let (mut r, mut collector) =
            ready(run(&budget(), &[&cont(CAR, 5.0), &term(CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 7.0);
        r.tick(at(1.0), &collector);

        let report = finalize_stopped_by(&r, &collector, FAILED_FIRST);

        assert_eq!(
            report.assertions()[1].verdict().actual(),
            Some(&AssertionValue::Float(7.0))
        );
    }

    #[test]
    fn a_continuous_assertion_reports_its_latch() {
        let (mut r, mut collector) = ready(run(&budget(), &[&cont(CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
        assert_eq!(report.status(), &ReportStatus::Passed);
    }

    #[test]
    fn a_failure_on_the_tick_the_budget_runs_out_leaves_the_rest_judged() {
        // The failure names the stop, but the run still reached its end, so
        // the truck's assertion passed rather than going unjudged.
        let termination = format!("{}, fail_fast = true", budget());
        let (mut r, mut collector) = ready(
            run(&termination, &[&cont(CAR, 5.0), &cont(TRUCK, 5.0)]),
            &[CAR, TRUCK],
        );
        r.tick(at(0.0), &collector);
        report(&mut collector, CAR, 4.5, 9.0);
        report(&mut collector, TRUCK, 4.5, 5.0);
        assert_eq!(
            r.tick(at(BUDGET_SECS), &collector),
            TickAction::Terminate {
                reason: FAILED_FIRST,
            }
        );

        let report = finalize_stopped_by(&r, &collector, FAILED_FIRST);

        assert_eq!(
            statuses(&report),
            [AssertionStatus::Failed, AssertionStatus::Passed]
        );
    }

    #[test]
    fn the_stop_reason_the_host_passes_does_not_decide_coverage() {
        // The host says the budget ran out, but the runner's own ticks only
        // reached 1 s of 5: a pass would be claimed for time never run.
        let (mut r, mut collector) = ready(run(&budget(), &[&cont(CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        r.tick(at(1.0), &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Unjudged]);
    }

    #[test]
    fn a_value_turned_away_before_the_end_fails_the_assertion() {
        let (mut r, mut collector) = ready(run(&budget(), &[&term(CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        report_unreadable(&mut collector, CAR, 1.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Failed]);
    }

    #[test]
    fn a_value_turned_away_after_the_last_tick_is_past_the_end() {
        // Like a late sample, a late rejection is outside the run.
        let (mut r, mut collector) = ready(run(&budget(), &[&term(CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        run_out(&mut r, &collector);
        report_unreadable(&mut collector, CAR, 5.5);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
    }

    #[test]
    fn finalizing_with_another_runs_collector_fails_the_run() {
        let (mut r, mut own) = ready(run(&budget(), &[&term(CAR, 5.0)]), &[CAR]);
        report(&mut own, CAR, 0.5, 5.0);
        r.tick(at(1.0), &own);
        let other = r.collector();

        let report = finalize(&r, &other);

        assert_eq!(report.status(), &ReportStatus::Failed);
        assert_eq!(report.setup_failures(), [FOREIGN_COLLECTOR]);
        assert_eq!(statuses(&report), [AssertionStatus::Unjudged]);
    }

    #[test]
    fn finalizing_a_run_never_set_up_fails_it() {
        // With no assertions there is nothing else to fail it: it would
        // otherwise report a pass for a run that never ran.
        let r = Runner::new(run(&budget(), &[])).expect("valid run");
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
        let mut r = Runner::new(run(&budget(), &[])).expect("valid run");
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
        let mut r = Runner::new(run(&budget(), &[])).expect("valid run");
        let collector = r.collector();
        r.fail_setup("first".to_string());
        r.fail_setup("second".to_string());

        let report = finalize(&r, &collector);

        assert_eq!(report.setup_failures(), ["first", "second"]);
    }

    #[test]
    fn a_clean_run_records_no_setup_failures() {
        let (r, collector) = ready(run(&budget(), &[]), &[]);

        let report = finalize(&r, &collector);

        assert_eq!(report.status(), &ReportStatus::Passed);
        assert!(report.setup_failures().is_empty());
    }

    #[test]
    fn a_refused_write_fails_a_run_whose_assertions_passed() {
        let (mut r, mut collector) = ready(run(&budget(), &[&cont(CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        run_out(&mut r, &collector);

        let report = r.finalize(
            host_facts(
                TerminationReason::MaxSimulatedSeconds,
                vec![refused_write(CAR)],
            ),
            &collector,
        );

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
        assert_eq!(report.status(), &ReportStatus::Failed);
        let [line] = report.refused_writes() else {
            panic!(
                "one refused channel, one line: {:?}",
                report.refused_writes()
            );
        };
        assert!(line.starts_with("agent 'car': "), "{line}");
        assert!(line.contains("\"undeclared\""), "{line}");
    }

    #[test]
    fn a_value_under_no_bound_series_fails_a_run_whose_assertions_passed() {
        // The speed channel's path is requested and bound as the pipeline
        // declares it, carrying a float; a value under it carrying a bool
        // lands in no series.
        let (mut r, mut collector) = ready(run(&budget(), &[&speed_term(CAR, 5.0)]), &[CAR]);
        report_speed(&mut collector, CAR, 0.5, 5.0);
        report_unbound(&mut collector, CAR, 0.5);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
        assert_eq!(report.status(), &ReportStatus::Failed);
        let [line] = report.unbound_arrivals() else {
            panic!("one unbound key, one line: {:?}", report.unbound_arrivals());
        };
        assert!(line.starts_with("agent.car.channel.speed ("), "{line}");
        assert!(line.contains("bool"), "{line}");
        assert!(
            line.ends_with(": 1 value(s) arrived under no bound series"),
            "{line}"
        );
    }

    #[test]
    fn a_run_aborted_at_setup_reports_no_unbound_values() {
        // The truck's assertion is refused, so its key is never bound; values
        // under its path say nothing beyond the setup failure.
        let mut r = Runner::new(run(&budget(), &[&term(TRUCK, 5.0)])).expect("valid run");
        let mut collector = r.collector();
        let pipeline = pipeline();
        assert!(r
            .on_pipeline_built(&[(AgentId::new(CAR), &pipeline)], &mut collector)
            .is_err());
        report(&mut collector, TRUCK, 0.5, 5.0);

        let report = finalize(&r, &collector);

        assert_eq!(report.status(), &ReportStatus::Failed);
        assert!(report.unbound_arrivals().is_empty());
    }
}
