//! The report entries of a measure and of an evaluation, and how a recorded
//! key is spelled in them.

use super::windowed::Windowed;
use crate::{
    assertion::Measure,
    collector::instance_of,
    evaluation::{
        derived::{Derived, Refused},
        path::DerivedName,
    },
    report::{
        EvaluationReportEntry, FailureReason, InputReport, MeasureReportEntry, MetricReport,
        ReportedUnit, SeriesSummary,
    },
    runner::{
        resolve::{evaluations::PlannedEvaluation, EntryView},
        state::AssertionState,
        window::Placement,
    },
};

use helios_runtime::observe::key::RecordedKey;

/// One measure's report line: its value reduced over its window, or why it
/// has none. `None` for both when the run never read it, or never placed its
/// window.
pub(super) fn report_measure(
    measure: &Measure,
    judged: Option<&EntryView<'_, '_, &AssertionState>>,
    windowed: Option<&Windowed>,
) -> MeasureReportEntry {
    let rejected = judged.and_then(|entry| entry.rejection()).is_some();
    let value = match (rejected, windowed.map(|windowed| windowed.placement)) {
        (true, _) => Some(Err(FailureReason::Rejected)),
        (false, None | Some(Placement::Unplaced)) => None,
        (false, Some(Placement::AnchorNeverOccurred)) => {
            Some(Err(FailureReason::AnchorNeverOccurred))
        }
        (false, Some(Placement::Placed { window, .. })) => {
            let samples = windowed.map_or(&[][..], |windowed| &windowed.samples);
            Some(
                match measure.reduce.reduce(samples, window.start, window.end) {
                    Ok(Some(value)) => Ok(value),
                    Ok(None) => Err(FailureReason::NoSamples),
                    Err(_) => Err(FailureReason::TypeMismatch),
                },
            )
        }
    };
    MeasureReportEntry::new(
        measure.name.to_string(),
        measure.target.clone(),
        judged.and_then(|entry| entry.key.and_then(instance)),
        measure.window.clone(),
        measure.reduce.clone(),
        windowed.is_some_and(Windowed::covered),
        value,
        judged.and_then(|entry| entry.unit),
        windowed.map_or_else(unread, |windowed| {
            windowed.summary(&Some(measure.reduce.clone()))
        }),
    )
}

/// One evaluation's report line: what it read, from whom, how much it
/// derived, and every metric it offers.
pub(super) fn report_evaluation(
    evaluation: &PlannedEvaluation,
    derived: &Derived,
    refused: &[Refused],
) -> EvaluationReportEntry {
    let inputs = evaluation
        .inputs
        .iter()
        .enumerate()
        .map(|(role, input)| {
            let (writer, provenance) = InputReport::writer_of(input.writer.as_ref());
            InputReport {
                role: input.role.to_owned(),
                target: input.target.clone(),
                instance: instance(&input.key),
                writer,
                provenance,
                refused: refused.get(role).map_or(0, |refused| refused.count),
            }
        })
        .collect();
    let metrics = evaluation
        .metrics
        .iter()
        .zip(&derived.metrics)
        .map(|(metric, series)| MetricReport {
            path: DerivedName::new(evaluation.name.clone(), metric.name).to_string(),
            unit: ReportedUnit::Declared(metric.si_unit),
            samples: series.as_ref().map(|series| series.samples().len()),
        })
        .collect();
    EvaluationReportEntry {
        name: evaluation.name.to_string(),
        kind: evaluation.kind.to_owned(),
        samples: derived.base_samples,
        uncovered: derived.uncovered,
        refused: derived.refused.count,
        params: evaluation.params.clone(),
        inputs,
        metrics,
    }
}

/// The instance `key` records, spelled for the report; `None` on a series
/// that isn't one instance of a keyed channel.
pub(super) fn instance(key: &RecordedKey) -> Option<String> {
    instance_of(&key.subject).map(str::to_owned)
}

/// `key`'s path, with its instance where it has one.
pub(super) fn spelled(key: &RecordedKey) -> String {
    match instance(key) {
        Some(instance) => format!("{}[instance={instance}]", key.path()),
        None => key.path(),
    }
}

/// The summary of an entry the run never read: not set up, set up against
/// another collector, or never ticked.
pub(super) fn unread() -> SeriesSummary {
    SeriesSummary {
        samples: 0,
        max_gap_secs: None,
        final_age_secs: None,
        window_secs: None,
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        assertion::AssertionValue,
        report::{AssertionStatus, FailureReason, ReportStatus, ReportedUnit, Verdict},
        runner::{
            finalize::fixtures::{
                after_speed, budget, finalize, lagging, run_out, statuses, BUDGET_SECS,
            },
            fixtures::{
                at, cont, measure, ready, report, report_speed, report_unreadable, report_wheel,
                run_measuring, target, windowed, CAR, TRUCK,
            },
        },
    };

    use helios_core::kernel::si_unit::SiUnit;

    #[test]
    fn a_measure_reports_its_value_and_never_fails_the_run() {
        let (mut r, mut collector) = ready(
            run_measuring(
                &budget(),
                &[],
                &[
                    &measure("peak", CAR, "\"max\""),
                    &measure("mean_of_nothing", TRUCK, "\"mean\""),
                ],
            ),
            &[CAR, TRUCK],
        );
        report(&mut collector, CAR, 0.5, 2.0);
        report(&mut collector, CAR, 1.0, 7.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        let [peak, nothing] = report.measures() else {
            panic!("two measures: {:?}", report.measures());
        };
        assert_eq!(peak.value(), Some(&AssertionValue::Float(7.0)));
        assert!(peak.covered());
        assert_eq!(nothing.value(), None);
        assert_eq!(nothing.reason(), Some(FailureReason::NoSamples));
        assert_eq!(report.status(), &ReportStatus::Passed);
    }

    #[test]
    fn a_measure_the_run_did_not_cover_reports_the_value_it_stopped_at() {
        let (mut r, mut collector) = ready(
            run_measuring(&budget(), &[], &[&measure("peak", CAR, "\"max\"")]),
            &[CAR],
        );
        report(&mut collector, CAR, 0.5, 2.0);
        r.tick(at(1.0), &collector);

        let report = finalize(&r, &collector);

        let [peak] = report.measures() else {
            panic!("one measure: {:?}", report.measures());
        };
        assert_eq!(peak.value(), Some(&AssertionValue::Float(2.0)));
        assert!(!peak.covered());
    }

    #[test]
    fn a_value_a_measures_series_turned_away_fails_the_run_under_recording() {
        // The assertion passes; the measure's value is no fault of the run's,
        // but the recording dropping one is.
        let (mut r, mut collector) = ready(
            run_measuring(
                &budget(),
                &[&cont("held", CAR, 5.0)],
                &[&measure("peak", CAR, "\"max\"")],
            ),
            &[CAR],
        );
        report(&mut collector, CAR, 0.5, 5.0);
        report_unreadable(&mut collector, CAR, 1.0);
        r.tick(at(0.0), &collector);
        r.tick(at(BUDGET_SECS), &collector);

        let report = finalize(&r, &collector);

        assert_eq!(report.measures()[0].reason(), Some(FailureReason::Rejected));
        assert_eq!(report.status(), &ReportStatus::Failed);
        let [line] = report.recording() else {
            panic!("one recording line: {:?}", report.recording());
        };
        assert!(
            line.starts_with("measure 'peak' on 'agent.car.node.estimator.x': value rejected"),
            "{line}"
        );
    }

    #[test]
    fn a_measure_on_an_anchor_that_never_occurred_says_so() {
        let peak = windowed("peak", &target(CAR), &after_speed(0.0), "reduce = \"max\"");
        let (mut r, mut collector) = ready(run_measuring(&budget(), &[], &[&peak]), &[CAR]);
        report(&mut collector, CAR, 1.0, 5.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        let entry = &report.measures()[0];
        assert_eq!(entry.value(), None);
        assert_eq!(entry.reason(), Some(FailureReason::AnchorNeverOccurred));
        assert_eq!(report.status(), &ReportStatus::Passed);
    }

    #[test]
    fn a_derived_series_is_judged_at_finalize_in_its_metrics_unit() {
        let (mut r, mut collector) = ready(lagging(), &[CAR]);
        // A speed before the first wheel value is uncovered; the rear
        // wheel is a sibling the evaluation never selected.
        report_speed(&mut collector, CAR, 0.5, 3.0);
        report_wheel(&mut collector, CAR, "front", 1.0, 5.0);
        report_wheel(&mut collector, CAR, "rear", 1.0, 9.0);
        report_speed(&mut collector, CAR, 1.0, 4.5);
        report_speed(&mut collector, CAR, 2.0, 4.75);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(report.status(), &ReportStatus::Passed, "{report:?}");
        let keeps_up = &report.assertions()[0];
        assert_eq!(
            keeps_up.verdict().actual(),
            Some(&AssertionValue::Float(0.5))
        );
        assert_eq!(
            keeps_up.unit(),
            Some(ReportedUnit::Declared(SiUnit::MetersPerSecond))
        );
        assert_eq!(keeps_up.series().samples, 2);
        assert_eq!(
            report.measures()[0].value(),
            Some(&AssertionValue::Float(0.375))
        );
        let [slow] = report.evaluations() else {
            panic!("one evaluation: {report:?}");
        };
        assert_eq!((slow.samples, slow.uncovered), (3, 1));
        assert_eq!(slow.metrics[0].path, "eval.inline.slow.shortfall");
        assert_eq!(slow.metrics[0].samples, Some(2));
        assert_eq!(slow.inputs[1].instance.as_deref(), Some("front"));
        assert_eq!(slow.inputs[0].writer, "node estimator");
        assert!(report.recording().is_empty(), "{:?}", report.recording());
    }

    #[test]
    fn a_value_after_the_last_tick_is_not_evaluated() {
        let (mut r, mut collector) = ready(lagging(), &[CAR]);
        report_wheel(&mut collector, CAR, "front", 1.0, 5.0);
        report_speed(&mut collector, CAR, 1.0, 4.5);
        report_speed(&mut collector, CAR, BUDGET_SECS + 1.0, 0.0);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(report.assertions()[0].series().samples, 1);
        assert_eq!(report.status(), &ReportStatus::Passed);
    }

    #[test]
    fn a_refused_input_value_fails_the_run_and_every_entry_on_its_metrics() {
        let (mut r, mut collector) = ready(lagging(), &[CAR]);
        report_wheel(&mut collector, CAR, "front", 1.0, 5.0);
        report_speed(&mut collector, CAR, 1.0, 4.5);
        report_speed(&mut collector, CAR, 2.0, f64::NAN);
        run_out(&mut r, &collector);

        let report = finalize(&r, &collector);

        assert_eq!(report.status(), &ReportStatus::Failed);
        assert_eq!(
            report.recording(),
            [
                "evaluation 'slow' input 'actual': 1 value(s) refused, the first a number that \
              isn't finite"
            ]
        );
        assert!(matches!(
            report.assertions()[0].verdict(),
            Verdict::Failed {
                reason: FailureReason::Rejected,
                ..
            }
        ));
        assert_eq!(report.measures()[0].reason(), Some(FailureReason::Rejected));
        assert_eq!(report.evaluations()[0].inputs[0].refused, 1);
    }

    #[test]
    fn a_derived_series_never_ticked_is_unjudged() {
        let (r, mut collector) = ready(lagging(), &[CAR]);
        report_wheel(&mut collector, CAR, "front", 1.0, 5.0);
        report_speed(&mut collector, CAR, 1.0, 4.5);

        let report = finalize(&r, &collector);

        assert_eq!(statuses(&report), [AssertionStatus::Unjudged]);
        assert_eq!(report.evaluations()[0].samples, 0);
    }
}
