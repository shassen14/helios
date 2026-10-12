use crate::{
    assertion::{
        evaluator::{evaluate, evaluate_latest},
        Assertion, AssertionResult, FailureKind,
    },
    collector::{ObservationCollector, Series},
    run::termination::TerminationReason,
    runner::state::AssertionState,
};

use super::{termination_check, Phase, Runner, FOREIGN_COLLECTOR, NEVER_SET_UP};

use helios_core::prelude::MonotonicTime;

impl Runner {
    /// Advance the run by one observation at simulated time `t`, judging every
    /// assertion on what arrived in `collector` and reporting whether to stop.
    /// It only reads the collector, so the host stays the sole source of what
    /// was observed.
    ///
    /// Only a run whose setup succeeded judges. One whose setup failed stops
    /// at once: a host may tick again before it acts on the failure, and a
    /// verdict from those ticks would be judged against a run already
    /// declared unfit. A host that ticks without setting the run up, or hands
    /// it another run's collector, fails setup here and stops.
    pub fn tick(&mut self, t: MonotonicTime, collector: &ObservationCollector) -> TickAction {
        let aborted = TickAction::Terminate {
            reason: TerminationReason::Aborted,
        };
        let plan = match &self.phase {
            Phase::Ready(plan) => plan,
            Phase::Aborted => return aborted,
            Phase::AwaitingPipelines => {
                self.fail_setup(NEVER_SET_UP.to_string());
                return aborted;
            }
        };
        let Some(series) = plan.series_in(collector) else {
            self.fail_setup(FOREIGN_COLLECTOR.to_string());
            return aborted;
        };

        // The run's clock origin: stamped on the first tick and fixed thereafter,
        // so the time budget is measured from when the run actually began, not
        // from zero. Copied out so the borrow of `start_time` ends here.
        let start = *self.start_time.get_or_insert(t);

        // Judge every assertion on the samples that arrived since the last
        // tick. Terminal assertions are judged too, because the stop-decision
        // reads them for `on_assertion` triggers; only continuous ones latch.
        // `self.run` and `self.states` are disjoint fields, so the immutable
        // borrow of one and the mutable borrow of the other coexist.
        let results: Vec<AssertionResult> = self
            .run
            .assertions()
            .iter()
            .zip(self.states.iter_mut())
            .zip(series)
            .map(|((assertion, state), series)| match assertion {
                Assertion::Continuous(_) => check_new(assertion, state, series, t),
                Assertion::Terminal(_) => {
                    state.reach_new(series.samples());
                    evaluate_latest(
                        assertion,
                        series.rejection(),
                        state.reached(series.samples()),
                    )
                }
            })
            .collect();

        // Decide. `check` applies the precedence (assertion outcome before time
        // budget) and returns the reason, if any.
        match termination_check::check(t, start, self.run.termination(), &results) {
            Some(reason) => TickAction::Terminate { reason },
            None => TickAction::Continue,
        }
    }
}

/// _Per-step._ The outcome of one `tick`: keep going, or stop with a reason.
#[derive(Debug, Clone, PartialEq)]
pub enum TickAction {
    Continue,
    Terminate { reason: TerminationReason },
}

/// Judge a continuous assertion on each sample that arrived since the last
/// tick, once each, latching a failure at the sample's own time. The verdict
/// for the stop-decision is the first failure among them, else the latest
/// pass, else `Pending` when none arrived. A series that turned a value away
/// fails, latched at `t` since the value it rejected isn't kept.
fn check_new(
    assertion: &Assertion,
    state: &mut AssertionState,
    series: &Series,
    t: MonotonicTime,
) -> AssertionResult {
    if let Some(rejection) = series.rejection() {
        let kind = FailureKind::Rejected(rejection);
        state.record(None, Err(kind.to_string()), t);
        return AssertionResult::Failed { kind, actual: None };
    }

    let mut verdict = AssertionResult::Pending;
    for sample in state.reach_new(series.samples()) {
        let result = evaluate(assertion, sample.value.clone());
        match &result {
            AssertionResult::Passed { actual } => {
                state.record(Some(actual.clone()), Ok(()), sample.timestamp);
            }
            AssertionResult::Failed { kind, actual } => {
                state.record(actual.clone(), Err(kind.to_string()), sample.timestamp);
            }
            AssertionResult::Pending => {}
        }
        if !matches!(verdict, AssertionResult::Failed { .. }) {
            verdict = result;
        }
    }
    verdict
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        assertion::AssertionValue,
        runner::{
            fixtures::{at, cont, ready, report, run, term, CAR, TRUCK},
            state::ContinuousStatus,
        },
    };

    #[test]
    fn a_failed_setup_stops_the_run_without_judging() {
        // The value would pass, so a latch that moved would show the tick
        // judged it.
        let a = cont(CAR, 5.0);
        let (mut r, mut collector) = ready(run("max_simulated_seconds = 10.0", &[&a]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        r.fail_setup("unusable".to_string());

        assert_eq!(
            r.tick(at(1.0), &collector),
            TickAction::Terminate {
                reason: TerminationReason::Aborted,
            }
        );
        assert_eq!(r.states[0].status(), &ContinuousStatus::Pending);
        assert_eq!(r.start_time, None);
    }

    #[test]
    fn a_tick_before_setup_fails_setup_and_stops() {
        // A host that never called `on_pipeline_built`: the report names that
        // instead of every assertion failing for want of a series.
        let a = cont(CAR, 5.0);
        let mut r = Runner::new(run("max_simulated_seconds = 10.0", &[&a])).expect("valid run");
        let collector = r.collector();

        assert_eq!(
            r.tick(at(1.0), &collector),
            TickAction::Terminate {
                reason: TerminationReason::Aborted,
            }
        );
        assert_eq!(r.setup_failures, [NEVER_SET_UP]);
        assert_eq!(r.phase, Phase::Aborted);
    }

    #[test]
    fn another_runs_collector_fails_setup_and_stops() {
        // Built over the same targets, so its series look the same; reading
        // them would judge the wrong run without a word.
        let a = cont(CAR, 5.0);
        let (mut r, _own) = ready(run("max_simulated_seconds = 10.0", &[&a]), &[CAR]);
        let mut other = r.collector();
        report(&mut other, CAR, 0.5, 5.0);

        assert_eq!(
            r.tick(at(1.0), &other),
            TickAction::Terminate {
                reason: TerminationReason::Aborted,
            }
        );
        assert_eq!(r.setup_failures, [FOREIGN_COLLECTOR]);
        assert_eq!(r.states[0].status(), &ContinuousStatus::Pending);
    }

    #[test]
    fn continuous_pass_holds_and_continues() {
        // Value matches: no trigger set and time budget unmet, so the run goes on
        // and the latch advances Pending -> Holding.
        let a = cont(CAR, 5.0);
        let (mut r, mut collector) = ready(run("max_simulated_seconds = 10.0", &[&a]), &[CAR]);
        report(&mut collector, CAR, 0.0, 5.0);

        assert_eq!(r.tick(at(0.0), &collector), TickAction::Continue);
        assert_eq!(r.states[0].status(), &ContinuousStatus::Holding);
    }

    #[test]
    fn continuous_failure_terminates_and_latches_at_the_samples_time() {
        // The latch records the time the failing value describes, not the
        // tick that judged it.
        let a = cont(CAR, 5.0);
        let (mut r, mut collector) = ready(run("on_assertion = \"any_failed\"", &[&a]), &[CAR]);
        report(&mut collector, CAR, 1.5, 9.0);

        assert_eq!(
            r.tick(at(2.0), &collector),
            TickAction::Terminate {
                reason: TerminationReason::OnAssertion {
                    result: AssertionResult::Failed {
                        kind: FailureKind::ConditionFailed,
                        actual: Some(AssertionValue::Float(9.0)),
                    },
                },
            }
        );
        assert_eq!(
            r.states[0].status(),
            &ContinuousStatus::FailedAt {
                t: at(1.5),
                reason: "condition failed".into(),
            }
        );
    }

    #[test]
    fn every_sample_between_ticks_is_checked() {
        // A failing value followed by a passing one before the next tick:
        // reading only the latest would miss the failure.
        let a = cont(CAR, 5.0);
        let (mut r, mut collector) = ready(run("on_assertion = \"any_failed\"", &[&a]), &[CAR]);
        report(&mut collector, CAR, 0.5, 9.0);
        report(&mut collector, CAR, 0.75, 5.0);

        assert!(matches!(
            r.tick(at(1.0), &collector),
            TickAction::Terminate {
                reason: TerminationReason::OnAssertion { .. }
            }
        ));
        assert!(matches!(
            r.states[0].status(),
            ContinuousStatus::FailedAt { .. }
        ));
    }

    #[test]
    fn a_sample_is_checked_once() {
        // The second tick has nothing new, so it judges nothing: `Pending`,
        // which no trigger reads, and the latch is left as it was.
        let a = cont(CAR, 5.0);
        let (mut r, mut collector) = ready(
            run(
                "max_simulated_seconds = 10.0, on_assertion = \"any_passed\"",
                &[&a],
            ),
            &[CAR],
        );
        r.tick(at(0.0), &collector);
        report(&mut collector, CAR, 0.5, 5.0);

        assert!(matches!(
            r.tick(at(1.0), &collector),
            TickAction::Terminate { .. }
        ));
        assert_eq!(r.tick(at(2.0), &collector), TickAction::Continue);
    }

    #[test]
    fn terminal_pass_terminates_without_latching() {
        // A terminal assertion drives the stop-decision (`any_passed` fires on
        // its pass) but is never folded into the latch — terminals are judged at
        // finalize, so the status stays Pending.
        let a = term(CAR, 5.0);
        let (mut r, mut collector) = ready(run("on_assertion = \"any_passed\"", &[&a]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);

        assert_eq!(
            r.tick(at(1.0), &collector),
            TickAction::Terminate {
                reason: TerminationReason::OnAssertion {
                    result: AssertionResult::Passed {
                        actual: AssertionValue::Float(5.0),
                    },
                },
            }
        );
        assert_eq!(r.states[0].status(), &ContinuousStatus::Pending);
    }

    #[test]
    fn no_samples_yet_continues_without_latching() {
        // Nothing has arrived — `Pending`, not a failure mid-run. Even with
        // `any_failed` armed, the run continues and the latch is left alone.
        let a = cont(CAR, 5.0);
        let (mut r, collector) = ready(
            run(
                "max_simulated_seconds = 10.0, on_assertion = \"any_failed\"",
                &[&a],
            ),
            &[CAR],
        );

        assert_eq!(r.tick(at(0.0), &collector), TickAction::Continue);
        assert_eq!(r.states[0].status(), &ContinuousStatus::Pending);
    }

    #[test]
    fn time_budget_measured_from_first_tick() {
        // A pure smoke run (no assertions). The clock origin is stamped on the
        // first tick at t=10, so elapsed is measured from there, not zero: the
        // budget fires at t=15, not immediately.
        let (mut r, collector) = ready(run("max_simulated_seconds = 5.0", &[]), &[]);

        assert_eq!(r.tick(at(10.0), &collector), TickAction::Continue);
        assert_eq!(r.tick(at(14.0), &collector), TickAction::Continue);
        assert_eq!(
            r.tick(at(15.0), &collector),
            TickAction::Terminate {
                reason: TerminationReason::MaxSimulatedSeconds,
            }
        );
    }

    #[test]
    fn states_track_each_continuous_assertion_by_index() {
        // Two agents, one series each: each assertion must read its own. No
        // trigger and the budget unmet, so the run continues and both latches
        // update independently: index 0 holds, index 1 fails.
        let a = cont(CAR, 5.0);
        let b = cont(TRUCK, 5.0);
        let (mut r, mut collector) = ready(
            run("max_simulated_seconds = 100.0", &[&a, &b]),
            &[CAR, TRUCK],
        );
        report(&mut collector, CAR, 0.5, 5.0);
        report(&mut collector, TRUCK, 0.5, 1.0);

        assert_eq!(r.tick(at(1.0), &collector), TickAction::Continue);
        assert_eq!(r.states[0].status(), &ContinuousStatus::Holding);
        assert_eq!(
            r.states[1].status(),
            &ContinuousStatus::FailedAt {
                t: at(0.5),
                reason: "condition failed".into(),
            }
        );
    }
}
