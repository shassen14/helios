//! An assertion's line and a measure's: what each judged or measured, the
//! value it read in its unit, and what its series held.

use super::{
    super::{
        phrase::{entry_name, reducer_phrase, with_unit},
        AssertionReportEntry, FailureReason, MeasureReportEntry, ReportedUnit, SeriesSummary,
        Verdict,
    },
    status_label,
};
use crate::{
    assertion::{AssertionValue, Condition},
    statistics::within_run::Reducer,
};

/// Width the `[assert] …` text is padded to before the status label, so the
/// `PASSED` / `FAILED` column lines up regardless of target length.
const STATUS_COLUMN: usize = 48;

/// "[assert] <name>: [<reducer> of ]<target> <condition>", padded to the
/// status column, then its label, what failed where it failed, and what its
/// series held.
pub(super) fn assertion_line(entry: &AssertionReportEntry) -> String {
    let head = format!(
        "[assert] {}: {}{} {}",
        entry_name(&entry.name, entry.instance()),
        reduced_of(entry.reduce.as_ref()),
        entry.target,
        condition_phrase(&entry.condition, entry.unit()),
    );

    // Pad the left half to a fixed column, then append the status label so
    // the labels align down the page.
    let mut line = format!(
        "{head:<width$} {label}",
        width = STATUS_COLUMN,
        label = status_label(&entry.status()),
    );

    // On a failure, show what was observed, when it broke and why.
    if let Verdict::Failed {
        actual,
        reason,
        failed_at_secs,
    } = entry.verdict()
    {
        let detail = failure_detail(actual.as_ref(), entry.unit(), *reason, *failed_at_secs);
        line.push_str(&format!(" ({detail})"));
    }

    // What the series held, so a silent signal shows beside a pass.
    line.push_str(&format!(
        " [{}]",
        series_phrase(&entry.series, entry.unit())
    ));

    line
}

/// "[measure] <name>: <reducer> of <target> = <value> [<series>]", the value
/// or why it has none, and whether its window was covered.
pub(super) fn measure_line(entry: &MeasureReportEntry) -> String {
    let value = match (entry.value(), entry.reason()) {
        (Some(value), _) => with_unit(value_string(value), entry.unit()),
        (None, Some(reason)) => format!("none ({})", reason_phrase(reason)),
        (None, None) => "none".to_string(),
    };
    let mut line = format!(
        "[measure] {}: {} of {} = {value} [{}]",
        entry_name(&entry.name, entry.instance()),
        reducer_phrase(&entry.reduce),
        entry.target,
        series_phrase(entry.series(), entry.unit()),
    );
    if !entry.covered() {
        line.push_str(" (not covered: the run stopped before its end)");
    }
    line
}

/// Phrase a failure as "got <value> at t=<secs>s: <reason>", leaving out the
/// value and the time when the failure has none.
fn failure_detail(
    actual: Option<&AssertionValue>,
    unit: Option<ReportedUnit>,
    reason: FailureReason,
    failed_at_secs: Option<f64>,
) -> String {
    let mut observed = Vec::new();
    if let Some(actual) = actual {
        observed.push(format!("got {}", with_unit(value_string(actual), unit)));
    }
    if let Some(at) = failed_at_secs {
        observed.push(format!("at t={at:.2}s"));
    }
    let reason = reason_phrase(reason);
    if observed.is_empty() {
        reason.to_string()
    } else {
        format!("{}: {reason}", observed.join(" "))
    }
}

/// The human phrase for why an assertion failed — a phrasebook, like
/// [`condition_phrase`]; the TOML carries the snake_case name instead.
fn reason_phrase(reason: FailureReason) -> &'static str {
    match reason {
        FailureReason::Rejected => "value rejected",
        FailureReason::NoSamples => "no samples",
        FailureReason::TypeMismatch => "type mismatch",
        FailureReason::ConditionFailed => "condition failed",
        FailureReason::AnchorNeverOccurred => "anchor never occurred",
    }
}

/// The human phrase for a condition with its operand in the unit it is
/// read in — a phrasebook, not logic: `< 25`, `== 0`, `in [0.3, 0.7] m/s`.
fn condition_phrase(condition: &Condition, unit: Option<ReportedUnit>) -> String {
    let phrase = match condition {
        Condition::Equals(value) => format!("== {}", value_string(value)),
        Condition::NotEquals(value) => format!("!= {}", value_string(value)),
        Condition::LessThan(bound) => format!("< {bound}"),
        Condition::GreaterThan(bound) => format!("> {bound}"),
        Condition::WithinRange([low, high]) => format!("in [{low}, {high}]"),
    };
    with_unit(phrase, unit)
}

/// `"<reducer> of "` before a reduced assertion's target; nothing before an
/// every-sample one's.
fn reduced_of(reduce: Option<&Reducer>) -> String {
    reduce
        .map(|reducer| format!("{} of ", reducer_phrase(reducer)))
        .unwrap_or_default()
}

/// What a series held: `12 samples, max gap 1.00s`, for `final` the judged
/// sample's age, where a bounded window fell, and whether the number judged
/// has no declared unit.
fn series_phrase(series: &SeriesSummary, unit: Option<ReportedUnit>) -> String {
    let mut parts = vec![format!("{} samples", series.samples)];
    if unit == Some(ReportedUnit::Undeclared) {
        parts.push(format!("unit {}", ReportedUnit::Undeclared));
    }
    if let Some([start, end]) = series.window_secs {
        parts.push(format!("window {start:.2}s to {end:.2}s"));
    }
    if let Some(gap) = series.max_gap_secs {
        parts.push(format!("max gap {gap:.2}s"));
    }
    if let Some(age) = series.final_age_secs {
        parts.push(format!("final {age:.2}s before the end"));
    }
    parts.join(", ")
}

/// Render a value the way a human expects to read it: strings quoted (matching
/// the report spec's `"Succeeded"`), everything else bare.
fn value_string(value: &AssertionValue) -> String {
    match value {
        AssertionValue::Bool(b) => b.to_string(),
        AssertionValue::Int(i) => i.to_string(),
        AssertionValue::Float(f) => f.to_string(),
        AssertionValue::String(s) => format!("\"{s}\""),
    }
}

#[cfg(test)]
mod tests {
    use super::{
        super::{fixtures::*, render},
        *,
    };
    use crate::assertion::{AssertionTarget, Window};

    use helios_core::kernel::si_unit::SiUnit;

    #[test]
    fn renders_each_assertion_line() {
        let text = render(&sample_report());
        // Passing line: name, reducer, quoted string and the `==` glyph,
        // labelled PASSED, with what the series held.
        assert!(text.contains(
            "[assert] mission_succeeded: final of agent.car.node.mission.state == \"Succeeded\""
        ));
        assert!(text.contains("PASSED [3 samples, max gap 1.00s, final 0.50s before the end]"));
        // Failing line: no reducer, the `<` glyph, FAILED, and the observed value.
        // Its bound and value in the unit they are read in.
        assert!(text.contains("[assert] speed_capped: agent.car.node.estimator.speed < 5 m/s"));
        assert!(text.contains("FAILED (got 7 m/s: condition failed) [40 samples, max gap 1.00s]"));
    }

    #[test]
    fn a_dimensionless_or_undeclared_number_prints_bare() {
        let bound = Condition::WithinRange([0.3, 0.7]);
        assert_eq!(
            condition_phrase(&bound, Some(ReportedUnit::Declared(SiUnit::Radians))),
            "in [0.3, 0.7] rad"
        );
        for bare in [
            Some(ReportedUnit::Declared(SiUnit::Dimensionless)),
            Some(ReportedUnit::Undeclared),
        ] {
            assert_eq!(condition_phrase(&bound, bare), "in [0.3, 0.7]");
        }
        assert_eq!(
            series_phrase(&series(2), Some(ReportedUnit::Undeclared)),
            "2 samples, unit undeclared, max gap 1.00s"
        );
        assert_eq!(
            series_phrase(
                &series(2),
                Some(ReportedUnit::Declared(SiUnit::Dimensionless))
            ),
            "2 samples, max gap 1.00s"
        );
    }

    #[test]
    fn a_range_prints_its_bounds() {
        assert_eq!(
            condition_phrase(&Condition::WithinRange([0.3, 0.7]), None),
            "in [0.3, 0.7]"
        );
    }

    #[test]
    fn a_failure_shows_its_time_and_reason() {
        let mut report = sample_report();
        report.assertions[1] = failing_speed(Verdict::Failed {
            actual: Some(AssertionValue::Float(7.0)),
            reason: FailureReason::ConditionFailed,
            failed_at_secs: Some(1.25),
        });
        assert!(render(&report).contains("FAILED (got 7 m/s at t=1.25s: condition failed)"));
    }

    #[test]
    fn a_failure_with_no_value_shows_only_its_reason() {
        let mut report = sample_report();
        report.assertions[1] = failing_speed(Verdict::Failed {
            actual: None,
            reason: FailureReason::NoSamples,
            failed_at_secs: None,
        });
        assert!(render(&report).contains("FAILED (no samples)"));
    }

    #[test]
    fn unjudged_entry_without_actual_does_not_panic() {
        let mut report = sample_report();
        report.assertions[1] = failing_speed(Verdict::Unjudged { actual: None });
        assert!(render(&report).contains("UNJUDGED"));
    }

    #[test]
    fn a_measure_shows_its_value_or_why_it_has_none() {
        let mut report = sample_report();
        report.measures = vec![
            MeasureReportEntry::new(
                "goal_distance".to_string(),
                AssertionTarget::parse("agent.car.node.eval.goal_distance").expect("valid target"),
                None,
                Window::default(),
                Reducer::Final,
                true,
                Some(Ok(AssertionValue::Float(0.25))),
                Some(ReportedUnit::Declared(SiUnit::Meters)),
                series(1),
            ),
            MeasureReportEntry::new(
                "peak_speed".to_string(),
                AssertionTarget::parse("agent.car.node.estimator.speed").expect("valid target"),
                None,
                Window::default(),
                Reducer::Max,
                false,
                Some(Err(FailureReason::NoSamples)),
                Some(ReportedUnit::Undeclared),
                series(0),
            ),
        ];
        let text = render(&report);
        assert!(text.contains(
            "[measure] goal_distance: final of agent.car.node.eval.goal_distance = 0.25 m \
             [1 samples, max gap 1.00s]"
        ));
        assert!(text.contains(
            "[measure] peak_speed: max of agent.car.node.estimator.speed = none (no samples) \
             [0 samples, unit undeclared, max gap 1.00s] (not covered: the run stopped before its end)"
        ));
    }

    #[test]
    fn a_windowed_entry_shows_where_its_window_fell() {
        let mut report = sample_report();
        report.assertions = vec![failing_speed(Verdict::Failed {
            actual: None,
            reason: FailureReason::AnchorNeverOccurred,
            failed_at_secs: None,
        })];
        report.assertions[0].series.window_secs = Some([2.5, 7.0]);
        let text = render(&report);
        assert!(
            text.contains(
                "FAILED (anchor never occurred) [40 samples, window 2.50s to 7.00s, max gap 1.00s]"
            ),
            "{text}"
        );
    }
}
