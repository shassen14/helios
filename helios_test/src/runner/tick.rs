use crate::{
    assertion::{evaluator::evaluate, AssertionResult, Condition, EntryName, FailureKind},
    collector::{ObservationCollector, Sample, Series},
    run::termination::TerminationReason,
    runner::{
        resolve::{EntryOwner, EntryView},
        state::{AssertionState, ContinuousStatus},
        window::{place, PlacedWindow, Placement},
    },
};

use super::{termination_check, Phase, Runner, TickSpan, FOREIGN_COLLECTOR, NEVER_SET_UP};

use helios_core::prelude::MonotonicTime;
use helios_runtime::observe::key::RecordedKey;

impl Runner {
    /// Advance the run by one observation at simulated time `t`, judging each
    /// every-sample assertion on a fixed window on what arrived in
    /// `collector`, and reporting whether to stop. It only reads the collector, so the host stays the
    /// sole source of what was observed.
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
        let plan = match &mut self.phase {
            Phase::Ready(plan) => plan,
            Phase::Aborted => return aborted,
            Phase::AwaitingPipelines => {
                self.fail_setup(NEVER_SET_UP.to_string());
                return aborted;
            }
        };
        let Some(entries) = plan.with_series_mut(collector) else {
            self.fail_setup(FOREIGN_COLLECTOR.to_string());
            return aborted;
        };

        // The run's clock: its origin stamped on the first tick and fixed
        // thereafter, so the time budget is measured from when the run
        // actually began, not from zero.
        let span = match &mut self.ticks {
            Some(span) => {
                span.latest = t;
                *span
            }
            None => *self.ticks.insert(TickSpan::starting_at(t)),
        };

        // Latch every every-sample assertion on a fixed window on the samples
        // that arrived since the last tick. Any other entry is evaluated only
        // at the end, a reduced one or a measure because it has one value,
        // and an every-sample one on a window from `run.end` or an anchor
        // because its window isn't known yet; here it just reaches its new
        // samples. The states are borrowed mutably out of `self.phase` and
        // the assertions immutably out of `self.run`: disjoint fields, so the
        // two borrows coexist. Every entry is visited rather than stopping at
        // the first failure, so each reaches this tick's samples.
        let assertions = self.run.assertions();
        let mut first_failed: Option<(&EntryName, &RecordedKey)> = None;
        for EntryView {
            owner,
            key,
            held,
            series,
            state,
            ..
        } in entries
        {
            let on_arrival = match owner {
                EntryOwner::Assertion(index) => assertions.get(index).filter(|assertion| {
                    assertion.judges_every_sample() && assertion.window.fixed()
                }),
                EntryOwner::Measure(_) => None,
            };
            // A fixed window is placed from the first tick alone.
            let placed =
                on_arrival.map(|assertion| place(&assertion.window, span, false, |_| None));
            let (Some(assertion), Some(Placement::Placed { window, .. })) = (on_arrival, placed)
            else {
                state.reach_new(series.samples(), series.rejection());
                continue;
            };
            check_new(&assertion.condition, &window, held, state, series, t);
            // Every entry a tick sees is recorded, so it has a key.
            if let (None, Some(key)) = (first_failed, key) {
                if matches!(state.status(), ContinuousStatus::FailedAt { .. }) {
                    first_failed = Some((&assertion.name, key));
                }
            }
        }

        // Decide. `check` applies the precedence (failure before time budget)
        // and returns the reason, if any.
        match termination_check::check(t, span.first, self.run.termination(), first_failed) {
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

/// Judge an every-sample assertion's `condition` on each sample that arrived
/// since the last tick stamped in `window`, once each, latching a failure at
/// the sample's own time. A `held` series' value carried into the window is
/// judged once, as of the window's start, when the tick reaches it. A series
/// that turned a value away fails, latched at `t` since the value it
/// rejected isn't kept.
fn check_new(
    condition: &Condition,
    window: &PlacedWindow,
    held: bool,
    state: &mut AssertionState,
    series: &Series,
    t: MonotonicTime,
) {
    let new = state.reach_new(series.samples(), series.rejection());
    if let Some(rejection) = state.rejection() {
        let kind = FailureKind::Rejected(rejection);
        state.record(None, Err(kind), t);
        return;
    }

    for sample in new
        .iter()
        .filter(|sample| window.contains(sample.timestamp))
    {
        judge(condition, state, sample);
    }
    let started = window.start_cut().is_some_and(|from| t.0 >= from.0);
    if held && started && state.carry_in_once() {
        if let Some(carried) = window.carried_in(state.reached(series.samples())) {
            judge(condition, state, &carried);
        }
    }
}

/// Fold `condition` judged on `sample` into `state`'s latch.
fn judge(condition: &Condition, state: &mut AssertionState, sample: &Sample) {
    match evaluate(condition, sample.value.clone()) {
        AssertionResult::Passed { .. } => state.record(None, Ok(()), sample.timestamp),
        AssertionResult::Failed { kind, actual } => {
            state.record(actual, Err(kind), sample.timestamp)
        }
        AssertionResult::Pending => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        assertion::AssertionValue,
        runner::fixtures::{
            at, cont, each_wheel, leaf_key, measure, ready, report, report_speed, report_wheel,
            run, run_measuring, speed_key, speed_target, states, target, term, wheel_key, windowed,
            CAR, TRUCK,
        },
    };

    const BUDGET: &str = "max_simulated_seconds = 10.0";

    const FAIL_FAST: &str = "max_simulated_seconds = 10.0, fail_fast = true";

    /// The stop for the assertion named `name` failing on its series `key`.
    fn failed(name: &str, key: RecordedKey) -> TerminationReason {
        TerminationReason::AssertionFailed {
            name: EntryName::parse(name).expect("valid name"),
            key,
        }
    }

    #[test]
    fn a_failed_setup_stops_the_run_without_judging() {
        // The plan, and with it every state, is gone once setup fails; an
        // unstamped clock shows the tick returned before judging anything.
        let a = cont("a", CAR, 5.0);
        let (mut r, mut collector) = ready(run("max_simulated_seconds = 10.0", &[&a]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        r.fail_setup("unusable".to_string());

        assert_eq!(
            r.tick(at(1.0), &collector),
            TickAction::Terminate {
                reason: TerminationReason::Aborted,
            }
        );
        assert!(states(&r).is_empty());
        assert_eq!(r.ticks, None);
    }

    #[test]
    fn a_tick_before_setup_fails_setup_and_stops() {
        // A host that never called `on_pipeline_built`: the report names that
        // instead of every assertion failing for want of a series.
        let a = cont("a", CAR, 5.0);
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
        let a = cont("a", CAR, 5.0);
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
        assert_eq!(r.ticks, None);
    }

    #[test]
    fn an_every_sample_pass_holds_and_continues() {
        // Value matches: no trigger set and time budget unmet, so the run goes on
        // and the latch advances Pending -> Holding.
        let a = cont("a", CAR, 5.0);
        let (mut r, mut collector) = ready(run("max_simulated_seconds = 10.0", &[&a]), &[CAR]);
        report(&mut collector, CAR, 0.0, 5.0);

        assert_eq!(r.tick(at(0.0), &collector), TickAction::Continue);
        assert_eq!(states(&r)[0].status(), &ContinuousStatus::Holding);
    }

    #[test]
    fn an_every_sample_failure_terminates_and_latches_at_the_samples_time() {
        // The latch records the time the failing value describes, not the
        // tick that judged it.
        let a = cont("a", CAR, 5.0);
        let (mut r, mut collector) = ready(run(FAIL_FAST, &[&a]), &[CAR]);
        report(&mut collector, CAR, 1.5, 9.0);

        assert_eq!(
            r.tick(at(2.0), &collector),
            TickAction::Terminate {
                reason: failed("a", leaf_key(CAR)),
            }
        );
        assert_eq!(
            states(&r)[0].status(),
            &ContinuousStatus::FailedAt {
                t: at(1.5),
                value: Some(AssertionValue::Float(9.0)),
                kind: FailureKind::ConditionFailed,
            }
        );
    }

    #[test]
    fn every_sample_between_ticks_is_checked() {
        // A failing value followed by a passing one before the next tick:
        // reading only the latest would miss the failure.
        let a = cont("a", CAR, 5.0);
        let (mut r, mut collector) = ready(run(FAIL_FAST, &[&a]), &[CAR]);
        report(&mut collector, CAR, 0.5, 9.0);
        report(&mut collector, CAR, 0.75, 5.0);

        assert_eq!(
            r.tick(at(1.0), &collector),
            TickAction::Terminate {
                reason: failed("a", leaf_key(CAR)),
            }
        );
        assert!(matches!(
            states(&r)[0].status(),
            ContinuousStatus::FailedAt { .. }
        ));
    }

    #[test]
    fn a_fail_fast_stop_names_the_assertion_that_failed() {
        // Both assertions are on the car; only the second fails, so the stop
        // must name it, not merely say that one failed.
        let a = cont("a", CAR, 5.0);
        let b = cont("b", CAR, 6.0);
        let (mut r, mut collector) = ready(run(FAIL_FAST, &[&a, &b]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);

        assert_eq!(
            r.tick(at(1.0), &collector),
            TickAction::Terminate {
                reason: failed("b", leaf_key(CAR)),
            }
        );
    }

    #[test]
    fn a_failing_reduced_value_mid_run_does_not_stop_a_fail_fast_run() {
        // A reduced assertion is judged at the end: a value that misses it
        // mid-run (the car not at its goal yet) is no failure, and it is
        // never latched.
        let a = cont("a", CAR, 5.0);
        let b = term("b", CAR, 6.0);
        let (mut r, mut collector) = ready(run(FAIL_FAST, &[&a, &b]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);

        assert_eq!(r.tick(at(1.0), &collector), TickAction::Continue);
        assert_eq!(states(&r)[1].status(), &ContinuousStatus::Pending);
    }

    #[test]
    fn a_failure_without_fail_fast_runs_to_the_budget() {
        let a = cont("a", CAR, 5.0);
        let (mut r, mut collector) = ready(run(BUDGET, &[&a]), &[CAR]);
        report(&mut collector, CAR, 0.5, 9.0);

        assert_eq!(r.tick(at(1.0), &collector), TickAction::Continue);
        assert!(matches!(
            states(&r)[0].status(),
            ContinuousStatus::FailedAt { .. }
        ));
    }

    #[test]
    fn no_samples_yet_continues_without_latching() {
        // Nothing has arrived — `Pending`, not a failure mid-run. Even with
        // `fail_fast` set, the run continues and the latch is left alone.
        let a = cont("a", CAR, 5.0);
        let (mut r, collector) = ready(run(FAIL_FAST, &[&a]), &[CAR]);

        assert_eq!(r.tick(at(0.0), &collector), TickAction::Continue);
        assert_eq!(states(&r)[0].status(), &ContinuousStatus::Pending);
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
    fn states_track_each_every_sample_assertion_by_index() {
        // Two agents, one series each: each assertion must read its own. No
        // trigger and the budget unmet, so the run continues and both latches
        // update independently: index 0 holds, index 1 fails.
        let a = cont("a", CAR, 5.0);
        let b = cont("b", TRUCK, 5.0);
        let (mut r, mut collector) = ready(
            run("max_simulated_seconds = 100.0", &[&a, &b]),
            &[CAR, TRUCK],
        );
        report(&mut collector, CAR, 0.5, 5.0);
        report(&mut collector, TRUCK, 0.5, 1.0);

        assert_eq!(r.tick(at(1.0), &collector), TickAction::Continue);
        assert_eq!(states(&r)[0].status(), &ContinuousStatus::Holding);
        assert_eq!(
            states(&r)[1].status(),
            &ContinuousStatus::FailedAt {
                t: at(0.5),
                value: Some(AssertionValue::Float(1.0)),
                kind: FailureKind::ConditionFailed,
            }
        );
    }

    #[test]
    fn a_measure_reaches_its_samples_and_never_stops_the_run() {
        // The measure reads the car's leaf, which would fail the every-sample
        // assertion's bound; only the assertion's state latches.
        let a = cont("a", CAR, 5.0);
        let m = measure("m", CAR, "\"max\"");
        let (mut r, mut collector) = ready(run_measuring(FAIL_FAST, &[&a], &[&m]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);

        assert_eq!(r.tick(at(1.0), &collector), TickAction::Continue);
        assert_eq!(states(&r)[1].status(), &ContinuousStatus::Pending);
        assert_eq!(
            states(&r)[1]
                .reached(collector_samples(&r, &collector))
                .len(),
            1
        );
    }

    #[test]
    fn the_tick_span_runs_from_the_first_tick_to_the_latest() {
        let (mut r, collector) = ready(run(BUDGET, &[]), &[]);

        r.tick(at(2.0), &collector);
        r.tick(at(3.5), &collector);

        assert_eq!(
            r.ticks,
            Some(TickSpan {
                first: at(2.0),
                latest: at(3.5),
            })
        );
    }

    /// Every sample is 5.
    const IS_FIVE: &str = "condition = { within_range = [5.0, 5.0] }";

    #[test]
    fn a_fixed_window_judges_only_samples_stamped_in_it() {
        let a = windowed("a", &target(CAR), "{ start = 1.0, end = 2.0 }", IS_FIVE);
        let (mut r, mut collector) = ready(run(FAIL_FAST, &[&a]), &[CAR]);
        // The first tick is run.start.
        assert_eq!(r.tick(at(0.0), &collector), TickAction::Continue);
        report(&mut collector, CAR, 0.5, 9.0);

        assert_eq!(r.tick(at(1.0), &collector), TickAction::Continue);

        report(&mut collector, CAR, 1.5, 9.0);
        assert_eq!(
            r.tick(at(2.0), &collector),
            TickAction::Terminate {
                reason: failed("a", leaf_key(CAR)),
            }
        );
    }

    #[test]
    fn a_held_value_is_judged_when_the_tick_reaches_the_window() {
        // Written at 0.5 s and in force at 2 s, when the window opens.
        let a = windowed("a", &speed_target(CAR), "{ start = 2.0 }", IS_FIVE);
        let (mut r, mut collector) = ready(run(FAIL_FAST, &[&a]), &[CAR]);
        assert_eq!(r.tick(at(0.0), &collector), TickAction::Continue);
        report_speed(&mut collector, CAR, 0.5, 9.0);

        assert_eq!(r.tick(at(1.0), &collector), TickAction::Continue);
        assert_eq!(
            r.tick(at(2.0), &collector),
            TickAction::Terminate {
                reason: failed("a", speed_key(CAR)),
            }
        );
        assert_eq!(
            states(&r)[0].status(),
            &ContinuousStatus::FailedAt {
                t: at(2.0),
                value: Some(AssertionValue::Float(9.0)),
                kind: FailureKind::ConditionFailed,
            }
        );
    }

    #[test]
    fn an_anchored_window_is_never_judged_on_arrival() {
        // Its window isn't known until the event arrives, so the failing
        // sample waits for finalize and never stops the run.
        let window = format!("{{ start = {{ after = \"{}\" }} }}", speed_target(CAR));
        let anchored = windowed("anchored", &target(CAR), &window, IS_FIVE);
        let (mut r, mut collector) = ready(
            run(FAIL_FAST, &[&cont("truck", TRUCK, 5.0), &anchored]),
            &[CAR, TRUCK],
        );
        report_speed(&mut collector, CAR, 0.5, 1.0);
        report(&mut collector, CAR, 1.0, 9.0);

        assert_eq!(r.tick(at(2.0), &collector), TickAction::Continue);
        assert_eq!(states(&r)[1].status(), &ContinuousStatus::Pending);
    }

    /// The samples of the series `r`'s first entry reads.
    fn collector_samples<'c>(
        r: &Runner,
        collector: &'c ObservationCollector,
    ) -> &'c [crate::collector::Sample] {
        match &r.phase {
            Phase::Ready(plan) => plan.with_series(collector, &[]).expect("its own collector")[0]
                .series
                .samples(),
            Phase::AwaitingPipelines | Phase::Aborted => &[],
        }
    }

    #[test]
    fn a_fail_fast_stop_names_the_instance_that_failed() {
        let each = format!(
            "name = \"slow\"\ntarget = {}\ncondition = {{ less_than = 5.0 }}",
            each_wheel(CAR)
        );
        let (mut r, mut collector) = ready(run(FAIL_FAST, &[&each]), &[CAR]);
        assert_eq!(r.tick(at(0.0), &collector), TickAction::Continue);
        report_wheel(&mut collector, CAR, "front", 0.5, 1.0);
        assert_eq!(r.tick(at(1.0), &collector), TickAction::Continue);

        report_wheel(&mut collector, CAR, "rear", 1.5, 9.0);
        assert_eq!(
            r.tick(at(2.0), &collector),
            TickAction::Terminate {
                reason: failed("slow", wheel_key(CAR, "rear")),
            }
        );
    }
}
