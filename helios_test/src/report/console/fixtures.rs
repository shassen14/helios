//! What the console's tests render: a finished report with one passing and
//! one failing assertion, stopped on the time budget.

use super::super::{
    AssertionReportEntry, DroppedValues, FailureReason, Report, ReportedUnit, RunEnd, RunEntries,
    RunIdentity, RunStop, SeriesSummary, Verdict,
};
use crate::{
    assertion::{AssertionTarget, AssertionValue, Condition, Window},
    run::termination::TerminationReason,
    statistics::within_run::Reducer,
};

use helios_core::kernel::si_unit::SiUnit;

/// A series of `samples` samples with a 1 s largest gap.
pub(super) fn series(samples: usize) -> SeriesSummary {
    SeriesSummary {
        samples,
        max_gap_secs: Some(1.0),
        final_age_secs: None,
        window_secs: None,
    }
}

fn passing_state() -> AssertionReportEntry {
    AssertionReportEntry::new(
        "mission_succeeded".to_string(),
        AssertionTarget::parse("agent.car.node.mission.state").expect("valid target"),
        None,
        None,
        Window::default(),
        Some(Reducer::Final),
        None,
        Condition::Equals(AssertionValue::String("Succeeded".to_string())),
        SeriesSummary {
            final_age_secs: Some(0.5),
            ..series(3)
        },
        Verdict::Passed {
            actual: AssertionValue::String("Succeeded".to_string()),
        },
    )
}

pub(super) fn failing_speed(verdict: Verdict) -> AssertionReportEntry {
    AssertionReportEntry::new(
        "speed_capped".to_string(),
        AssertionTarget::parse("agent.car.node.estimator.speed").expect("valid target"),
        None,
        None,
        Window::default(),
        None,
        Some(ReportedUnit::Declared(SiUnit::MetersPerSecond)),
        Condition::LessThan(5.0),
        series(40),
        verdict,
    )
}

// Enough to exercise every rendering branch.
pub(super) fn sample_report() -> Report {
    Report::new(
        RunIdentity {
            run_name: "smoke".to_string(),
            scenario: "sim.scenarios.parking_lot".to_string(),
            master_seed: Some(42),
        },
        RunEnd {
            simulated_duration_secs: 47.3,
            wall_duration_secs: 4.1,
            stop: RunStop::Ended(TerminationReason::MaxSimulatedSeconds),
        },
        DroppedValues::default(),
        RunEntries {
            assertions: vec![
                passing_state(),
                failing_speed(Verdict::Failed {
                    actual: Some(AssertionValue::Float(7.0)),
                    reason: FailureReason::ConditionFailed,
                    failed_at_secs: None,
                }),
            ],
            ..RunEntries::default()
        },
    )
}
