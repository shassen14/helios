//! Test fixtures for finalizing: a run ticked to its budget, the facts its
//! host hands finalize, and readings of the report.

use super::report::HostFacts;
use crate::{
    assertion::EntryName,
    collector::ObservationCollector,
    report::{AssertionStatus, DerivedEvaluation, Report},
    run::{termination::TerminationReason, Run},
    runner::{
        fixtures::{at, lag, lag_target, leaf_key, run_evaluating, speed_target, CAR},
        Runner,
    },
};

use helios_core::prelude::AgentId;
use helios_runtime::port::RefusedWrites;

pub(super) const BUDGET_SECS: f64 = 5.0;

// `{:?}` forces a decimal point so TOML reads it as a float.
pub(super) fn budget() -> String {
    format!("max_simulated_seconds = {BUDGET_SECS:?}")
}

// Tick from the run's start to its budget, so it covers its window. Each
// tick reaches whatever has arrived by then.
pub(super) fn run_out(r: &mut Runner, collector: &ObservationCollector) {
    r.tick(at(0.0), collector);
    r.tick(at(BUDGET_SECS), collector);
}

// Build the report the way `finalize_system` does, for a run the host
// says ran out its budget.
pub(super) fn finalize(r: &Runner, collector: &ObservationCollector) -> Report {
    finalize_stopped_by(r, collector, TerminationReason::MaxSimulatedSeconds)
}

pub(super) fn finalize_stopped_by(
    r: &Runner,
    collector: &ObservationCollector,
    reason: TerminationReason,
) -> Report {
    r.finalize(host_facts(reason, Vec::new()), collector)
}

// Build the report for a run the host stopped with no reason from the
// runner: one it aborted, or one it never told to stop.
pub(super) fn finalize_unstopped(r: &Runner, collector: &ObservationCollector) -> Report {
    let host = HostFacts {
        terminated_by_reason: None,
        ..host_facts(TerminationReason::MaxSimulatedSeconds, Vec::new())
    };
    r.finalize(host, collector)
}

// What the host hands finalize: a run named "run", stopped by `reason`,
// whose buses refused `refused_writes`.
pub(super) fn host_facts(
    reason: TerminationReason,
    refused_writes: Vec<(AgentId, RefusedWrites)>,
) -> HostFacts {
    HostFacts {
        sim_now: at(0.0),
        wall_duration_secs: 0.0,
        run_name: "run".to_string(),
        terminated_by_reason: Some(reason),
        master_seed: None,
        refused_writes,
    }
}

/// The stop for the assertion named `first` failing on the car's leaf.
pub(super) fn failed_first() -> TerminationReason {
    TerminationReason::AssertionFailed {
        name: EntryName::parse("first").expect("valid name"),
        key: leaf_key(CAR),
    }
}

/// The report's first evaluation, which derived.
pub(super) fn derived(report: &Report) -> &DerivedEvaluation {
    report.evaluations()[0]
        .derived()
        .expect("the first evaluation derived")
}

pub(super) fn statuses(report: &Report) -> Vec<AssertionStatus> {
    report
        .assertions()
        .iter()
        .map(|entry| entry.status())
        .collect()
}

/// Every sample, or the reduced value, is 5.
pub(super) const IS_FIVE: &str = "condition = { within_range = [5.0, 5.0] }";

/// A window starting `plus` seconds after the car's speed is first
/// written.
pub(super) fn after_speed(plus: f64) -> String {
    format!(
        "{{ start = {{ after = \"{}\", plus = {plus:?} }} }}",
        speed_target(CAR)
    )
}

/// The instance each entry of `entries` judged, with its status.
pub(super) fn by_instance(report: &Report) -> Vec<(Option<&str>, AssertionStatus)> {
    report
        .assertions()
        .iter()
        .map(|entry| (entry.instance(), entry.status()))
        .collect()
}

/// A run evaluating `lag` as "slow" on the car, asserting its max
/// shortfall is below 1 m/s and measuring its mean.
pub(super) fn lagging() -> Run {
    let keeps_up = format!(
        "name = \"keeps_up\"\ntarget = \"{}\"\nreduce = \"max\"\n\
         condition = {{ less_than = 1.0 }}",
        lag_target("slow")
    );
    let mean = format!(
        "name = \"mean_shortfall\"\ntarget = \"{}\"\nreduce = \"mean\"",
        lag_target("slow")
    );
    run_evaluating(&budget(), &[&keeps_up], &[&mean], &[&lag("slow", CAR)])
}
