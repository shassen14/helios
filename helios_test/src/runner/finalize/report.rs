//! [`Runner::finalize`]: the run's [`Report`], built once the host stops
//! ticking from what the run reached and the [`HostFacts`] only the host
//! knows.

use super::reached::Reached;
use crate::{
    collector::ObservationCollector,
    evaluation::derived::Derived,
    report::{Report, TerminatedBy},
    run::termination::TerminationReason,
    runner::{Phase, Runner},
};

use helios_core::prelude::{AgentId, MonotonicTime};
use helios_runtime::port::RefusedWrites;

impl Runner {
    /// Build the run's [`Report`] once the host stops ticking. Each
    /// evaluation derives its series here, from the values stamped up to the
    /// run's last tick. Every entry is evaluated here, once, on the samples
    /// the run reached that are stamped in its window: an every-sample
    /// assertion on each of them, placing a failure at the earliest failing
    /// stamp (the latch `tick` folded only decides `fail_fast`); a reduced
    /// assertion and a measure on their one reduced value.
    ///
    /// Each window is placed on the run's clock and covered once the run's
    /// latest tick reached its end. A window bounded from `run.end`, and the
    /// whole run, are covered only by a run that reached its time budget.
    /// The runner reads that from its own clock, not from the stop reason
    /// the host passes, which only labels the report: a `fail_fast` failure
    /// on the tick the budget runs out still covered it. An assertion whose
    /// window wasn't covered, or can't be placed (a `run.end` bound after an
    /// early stop, or an anchor that hadn't occurred yet), and that hadn't
    /// failed in it, is `Unjudged`: neither its pass nor its silence says
    /// anything about a window the run never finished, and a measure is
    /// marked not covered. A failure is final however the run stopped. In a
    /// covered window, an assertion with no samples to judge fails, since
    /// that is how a dropped signal looks; in a run that ran out, one whose
    /// anchor never occurred fails too.
    ///
    /// The run is `Passed` only if every assertion passed, setup didn't fail,
    /// no agent's bus refused a write ([`HostFacts::refused_writes`]) and the
    /// recording dropped nothing: no requested value arrived under an
    /// unbound key ([`ObservationCollector::unbound_arrivals`]), no measure's
    /// series turned a value away and no evaluation refused an input's
    /// value or a sample. Values the wiring or the recording dropped leave
    /// the entries describing another run than the one configured. Only a
    /// run that was set up reports the recording's drops; in one aborted at
    /// setup, a refused entry's key is unbound by design.
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

        // Each evaluation's series, from the values stamped up to the last
        // tick; with no tick, from none.
        let until = self
            .ticks
            .map_or(MonotonicTime(f64::NEG_INFINITY), |span| span.latest);
        let derived: Option<Vec<Derived>> = match &self.phase {
            Phase::Ready(plan) => plan.derive(collector, until),
            Phase::AwaitingPipelines | Phase::Aborted => None,
        };
        let (reached, failure) = Reached::new(self, collector, derived.as_deref());
        let mut setup_failures = self.setup_failures.clone();
        setup_failures.extend(failure.map(str::to_owned));

        let assertions = reached.assertions();
        let recording = reached.recording();
        let measures = reached.measures();
        let evaluations = reached.evaluations();

        let refused_writes: Vec<String> = refused_writes
            .into_iter()
            .map(|(agent, refused)| format!("agent '{agent}': {refused}"))
            .collect();

        // Elapsed simulated time from the origin `tick` stamped on its first
        // call. `None` only if finalize is reached with no tick having run;
        // report zero rather than panic.
        let simulated_duration_secs = match self.ticks {
            Some(span) => sim_now.0 - span.first.0,
            None => 0.0f64,
        };

        Report::new(
            run_name,
            self.run.scenario().path().to_string(),
            master_seed,
            setup_failures,
            refused_writes,
            recording,
            simulated_duration_secs,
            wall_duration_secs,
            TerminatedBy::from(&terminated_by_reason),
            assertions,
            measures,
            evaluations,
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

#[cfg(test)]
mod tests {
    use crate::{
        assertion::AssertionValue,
        report::{AssertionStatus, ReportStatus, ReportedUnit},
        run::termination::TerminationReason,
        runner::{
            finalize::fixtures::{
                budget, by_instance, failed_first, finalize, finalize_stopped_by, host_facts,
                run_out, statuses, BUDGET_SECS,
            },
            fixtures::{
                at, cont, each_wheel, measure, pipeline, ready, reduced_below, refused_write,
                report, report_speed, report_unbound, report_wheel, run, run_measuring, speed_term,
                term, CAR, TRUCK,
            },
            Runner, TickAction, FOREIGN_COLLECTOR, NEVER_SET_UP,
        },
    };

    use helios_core::kernel::si_unit::SiUnit;
    use helios_core::prelude::AgentId;

    #[test]
    fn finalizing_with_another_runs_collector_fails_the_run() {
        let (mut r, mut own) = ready(run(&budget(), &[&term("x17", CAR, 5.0)]), &[CAR]);
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
        let (mut r, mut collector) = ready(run(&budget(), &[&cont("x18", CAR, 5.0)]), &[CAR]);
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
        let (mut r, mut collector) = ready(run(&budget(), &[&speed_term("x19", CAR, 5.0)]), &[CAR]);
        report_speed(&mut collector, CAR, 0.5, 5.0);
        report_unbound(&mut collector, CAR, 0.5);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Passed]);
        assert_eq!(report.status(), &ReportStatus::Failed);
        let [line] = report.recording() else {
            panic!("one unbound key, one line: {:?}", report.recording());
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
        let mut r = Runner::new(run(&budget(), &[&term("x20", TRUCK, 5.0)])).expect("valid run");
        let mut collector = r.collector();
        let pipeline = pipeline();
        assert!(r
            .on_pipeline_built(&[(AgentId::new(CAR), &pipeline)], &mut collector)
            .is_err());
        report(&mut collector, TRUCK, 0.5, 5.0);

        let report = finalize(&r, &collector);

        assert_eq!(report.status(), &ReportStatus::Failed);
        assert!(report.recording().is_empty());
    }

    #[test]
    fn each_instance_is_judged_and_reported_apart() {
        let each = format!(
            "name = \"slow\"\ntarget = {}\ncondition = {{ less_than = 5.0 }}",
            each_wheel(CAR)
        );
        let count = format!(
            "name = \"updates\"\ntarget = {}\nreduce = \"count\"",
            each_wheel(CAR)
        );
        let (mut r, mut collector) = ready(
            run_measuring(&budget(), &[&each, &cont("first", CAR, 5.0)], &[&count]),
            &[CAR],
        );
        report(&mut collector, CAR, 0.5, 5.0);
        report_wheel(&mut collector, CAR, "front", 0.5, 1.0);
        report_wheel(&mut collector, CAR, "front", 1.0, 2.0);
        report_wheel(&mut collector, CAR, "rear", 0.5, 9.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        // In run-file order, each instance in the catalog's order.
        assert_eq!(
            by_instance(&report),
            [
                (Some("front"), AssertionStatus::Passed),
                (Some("rear"), AssertionStatus::Failed),
                (None, AssertionStatus::Passed),
            ]
        );
        assert!(report.assertions()[..2]
            .iter()
            .all(|entry| entry.name() == "slow"));
        let counts: Vec<(Option<&str>, Option<&AssertionValue>)> = report
            .measures()
            .iter()
            .map(|entry| (entry.instance(), entry.value()))
            .collect();
        assert_eq!(
            counts,
            [
                (Some("front"), Some(&AssertionValue::Int(2))),
                (Some("rear"), Some(&AssertionValue::Int(1))),
            ]
        );
        assert_eq!(report.status(), &ReportStatus::Failed);
    }

    #[test]
    fn an_assertion_on_each_instance_in_a_run_never_set_up_has_one_line() {
        let each = format!(
            "name = \"slow\"\ntarget = {}\ncondition = {{ less_than = 5.0 }}",
            each_wheel(CAR)
        );
        let r = Runner::new(run(&budget(), &[&each])).expect("valid run");
        let collector = r.collector();

        let report = finalize(&r, &collector);

        assert_eq!(by_instance(&report), [(None, AssertionStatus::Unjudged)]);
    }

    #[test]
    fn the_stop_reason_the_host_passes_does_not_decide_coverage() {
        // The host says the budget ran out, but the runner's own ticks only
        // reached 1 s of 5: a pass would be claimed for time never run.
        let (mut r, mut collector) = ready(run(&budget(), &[&cont("x14", CAR, 5.0)]), &[CAR]);
        report(&mut collector, CAR, 0.5, 5.0);
        r.tick(at(1.0), &collector);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Unjudged]);
    }

    #[test]
    fn a_failure_on_the_tick_the_budget_runs_out_leaves_the_rest_judged() {
        // The failure names the stop, but the run still reached its end, so
        // the truck's assertion passed rather than going unjudged.
        let termination = format!("{}, fail_fast = true", budget());
        let (mut r, mut collector) = ready(
            run(
                &termination,
                &[&cont("first", CAR, 5.0), &cont("x13", TRUCK, 5.0)],
            ),
            &[CAR, TRUCK],
        );
        r.tick(at(0.0), &collector);
        report(&mut collector, CAR, 4.5, 9.0);
        report(&mut collector, TRUCK, 4.5, 5.0);
        assert_eq!(
            r.tick(at(BUDGET_SECS), &collector),
            TickAction::Terminate {
                reason: failed_first(),
            }
        );

        let report = finalize_stopped_by(&r, &collector, failed_first());

        assert_eq!(
            statuses(&report),
            [AssertionStatus::Failed, AssertionStatus::Passed]
        );
    }

    #[test]
    fn each_entry_reports_the_unit_its_value_is_read_in() {
        // The leaf's meters, the reducer's seconds, an f64 channel's none;
        // a count of the leaf, and each wheel's speed in the extractor's m/s.
        let fastest = format!(
            "name = \"fastest\"\ntarget = {}\nreduce = \"max\"",
            each_wheel(CAR)
        );
        let (mut r, collector) = ready(
            run_measuring(
                &budget(),
                &[
                    &cont("at_one", CAR, 1.0),
                    &reduced_below("gaps", CAR, "\"max_gap\"", 10.0),
                    &speed_term("speed", CAR, 1.0),
                ],
                &[&measure("updates", CAR, "\"count\""), &fastest],
            ),
            &[CAR],
        );
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        let declared = |si_unit| Some(ReportedUnit::Declared(si_unit));
        let assertions: Vec<_> = report
            .assertions()
            .iter()
            .map(|entry| entry.unit())
            .collect();
        assert_eq!(
            assertions,
            [
                declared(SiUnit::Meters),
                declared(SiUnit::Seconds),
                Some(ReportedUnit::Undeclared),
            ]
        );
        let measures: Vec<_> = report.measures().iter().map(|entry| entry.unit()).collect();
        assert_eq!(
            measures,
            [
                declared(SiUnit::Dimensionless),
                declared(SiUnit::MetersPerSecond),
                declared(SiUnit::MetersPerSecond),
            ]
        );
    }
}
