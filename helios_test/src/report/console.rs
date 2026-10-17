//! _Post-step._ The report's human surface: render a finished [`Report`] to a
//! block of text and print it. Pure presentation over the same struct the TOML
//! writer serializes, so the console and `result.toml` can never disagree.
//!
//! Every "how do I show this value" decision is a small private helper in this
//! file — not an `impl Display` on the data types. Keeping the presentation here
//! means the report types in `mod.rs` stay pure data, and there's exactly one
//! place to look when the output format changes.
//!
//! `console` is a submodule of `report`, so it reads `Report`'s and
//! `AssertionReportEntry`'s private fields directly rather than widening their
//! public accessor set for what is purely presentation glue.

use super::{
    AssertionStatus, FailureReason, Report, ReportStatus, SeriesSummary, TerminatedBy, Verdict,
};

use crate::{
    assertion::{AssertionValue, Condition},
    statistics::within_run::Reducer,
};

/// Width the `[assert] …` text is padded to before the status label, so the
/// `PASSED` / `FAILED` column lines up regardless of target length.
const STATUS_COLUMN: usize = 48;

/// Print a finished report to stdout. This is the one sanctioned `println!` in
/// the crate: it renders an already-decided result for a human, it does not emit
/// metrics or telemetry (which must go through the observability hook).
pub fn print(report: &Report) {
    println!("{}", render(report));
}

/// Build the full report text, line by line. Split out from [`print`] so it can
/// be unit-tested by asserting on the returned `String` without capturing
/// stdout. Does no judging — every pass/fail fact was already decided upstream;
/// this only arranges those facts as text.
fn render(report: &Report) -> String {
    // Collect each line into a vec, then join with newlines at the end. Simpler
    // than pushing `\n` by hand, and it can't accidentally produce stray blank
    // lines or trailing whitespace.
    let mut lines: Vec<String> = Vec::new();

    // Header: the run's name.
    lines.push(format!("[run] {}", report.run_name));

    // Why the run stopped, plus the simulated time it stopped at.
    lines.push(format!("[run] terminated: {}", termination_line(report)));

    // Setup failures come before the assertion lines: they explain why there
    // may be no assertion lines worth reading.
    for reason in &report.setup_failures {
        lines.push(format!("[setup] {reason}"));
    }

    // Refused writes too: values the wiring dropped make the assertion lines
    // judge another run than the one configured.
    for refused in &report.refused_writes {
        lines.push(format!("[wiring] {refused}"));
    }

    // And values the recording dropped: a series that missed them judges
    // less than the run recorded.
    for dropped in &report.recording {
        lines.push(format!("[recording] {dropped}"));
    }

    // One line per assertion, in run-file order.
    for entry in report.assertions() {
        // The left half: "[assert] <name>: [<reducer> of ]<target> <condition>".
        let head = format!(
            "[assert] {}: {}{} {}",
            entry.name,
            reduced_of(entry.reduce.as_ref()),
            entry.target,
            condition_phrase(&entry.condition),
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
            let detail = failure_detail(actual.as_ref(), *reason, *failed_at_secs);
            line.push_str(&format!(" ({detail})"));
        }

        // What the series held, so a silent signal shows beside a pass.
        line.push_str(&format!(" [{}]", series_phrase(&entry.series)));

        lines.push(line);
    }

    // One line per measure, after the verdicts: a value, never a verdict.
    for entry in report.measures() {
        let value = match (entry.value(), entry.reason()) {
            (Some(value), _) => value_string(value),
            (None, Some(reason)) => format!("none ({})", reason_phrase(reason)),
            (None, None) => "none".to_string(),
        };
        let mut line = format!(
            "[measure] {}: {} of {} = {value} [{}]",
            entry.name,
            reducer_phrase(&entry.reduce),
            entry.target,
            series_phrase(entry.series()),
        );
        if !entry.covered() {
            line.push_str(" (not covered: the run stopped before its end)");
        }
        lines.push(line);
    }

    // Footer: the run-level verdict and how many assertions passed. The verdict
    // comes from `report.status`, not recomputed from the tally — the runner
    // already decided it, and console must not second-guess that.
    let total = report.assertions().len();
    let passed = report
        .assertions()
        .iter()
        .filter(|entry| matches!(entry.status(), AssertionStatus::Passed))
        .count();
    lines.push(format!(
        "[run] result: {verdict}  ({passed} of {total} assertions)",
        verdict = run_status_label(&report.status),
    ));

    lines.join("\n")
}

/// Phrase the termination reason as "<cause> at t=<sim-seconds>s". The sim time
/// is the report's `simulated_duration_secs`; for a `fail_fast` stop we also
/// say which assertion failed.
fn termination_line(report: &Report) -> String {
    let at = format!("t={:.1}s", report.simulated_duration_secs);
    match &report.terminated_by {
        TerminatedBy::Aborted => "aborted during setup, before the first tick".to_string(),
        TerminatedBy::TimeBudget => format!("time budget reached at {at}"),
        TerminatedBy::AssertionFailed { name } => format!(
            "assertion '{name}' {} at {at}",
            status_label(&AssertionStatus::Failed)
        ),
    }
}

/// Phrase a failure as "got <value> at t=<secs>s: <reason>", leaving out the
/// value and the time when the failure has none.
fn failure_detail(
    actual: Option<&AssertionValue>,
    reason: FailureReason,
    failed_at_secs: Option<f64>,
) -> String {
    let mut observed = Vec::new();
    if let Some(actual) = actual {
        observed.push(format!("got {}", value_string(actual)));
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
    }
}

/// The human phrase for a condition with its operand — a phrasebook, not
/// logic: `< 25`, `== 0`, `in [0.3, 0.7]`.
fn condition_phrase(condition: &Condition) -> String {
    match condition {
        Condition::Equals(value) => format!("== {}", value_string(value)),
        Condition::NotEquals(value) => format!("!= {}", value_string(value)),
        Condition::LessThan(bound) => format!("< {bound}"),
        Condition::GreaterThan(bound) => format!("> {bound}"),
        Condition::WithinRange([low, high]) => format!("in [{low}, {high}]"),
    }
}

/// `"<reducer> of "` before a reduced assertion's target; nothing before an
/// every-sample one's.
fn reduced_of(reduce: Option<&Reducer>) -> String {
    reduce
        .map(|reducer| format!("{} of ", reducer_phrase(reducer)))
        .unwrap_or_default()
}

/// A reducer as the run file names it, with its parameter: `mean`,
/// `time_above(1.5)`.
fn reducer_phrase(reducer: &Reducer) -> String {
    match reducer {
        Reducer::TimeAbove(threshold) => format!("{}({threshold})", reducer.name()),
        other => other.name().to_string(),
    }
}

/// What a series held: `12 samples, max gap 1.00s`, and for `final` the
/// judged sample's age.
fn series_phrase(series: &SeriesSummary) -> String {
    let mut parts = vec![format!("{} samples", series.samples)];
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

/// Label for a per-assertion verdict. A plain `&'static str`, no allocation.
fn status_label(status: &AssertionStatus) -> &'static str {
    match status {
        AssertionStatus::Passed => "PASSED",
        AssertionStatus::Failed => "FAILED",
        AssertionStatus::Unjudged => "UNJUDGED",
    }
}

/// Label for the run-level verdict. Separate from [`status_label`] because
/// `ReportStatus` has no `Unjudged` — a finished run is only ever passed or failed.
fn run_status_label(status: &ReportStatus) -> &'static str {
    match status {
        ReportStatus::Passed => "PASSED",
        ReportStatus::Failed => "FAILED",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::report::{AssertionReportEntry, MeasureReportEntry};

    /// A series of `samples` samples with a 1 s largest gap.
    fn series(samples: usize) -> SeriesSummary {
        SeriesSummary {
            samples,
            max_gap_secs: Some(1.0),
            final_age_secs: None,
        }
    }

    fn passing_state() -> AssertionReportEntry {
        AssertionReportEntry::new(
            "mission_succeeded".to_string(),
            "agent.car.node.mission.state".to_string(),
            Some(Reducer::Final),
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

    fn failing_speed(verdict: Verdict) -> AssertionReportEntry {
        AssertionReportEntry::new(
            "speed_capped".to_string(),
            "agent.car.node.estimator.speed".to_string(),
            None,
            Condition::LessThan(5.0),
            series(40),
            verdict,
        )
    }

    // Build a finished report with one passing and one failing assertion, stopped
    // on the time budget — enough to exercise every rendering branch.
    fn sample_report() -> Report {
        Report::new(
            "smoke".to_string(),
            "sim.scenarios.parking_lot".to_string(),
            Some(42),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            47.3,
            4.1,
            TerminatedBy::TimeBudget,
            vec![
                passing_state(),
                failing_speed(Verdict::Failed {
                    actual: Some(AssertionValue::Float(7.0)),
                    reason: FailureReason::ConditionFailed,
                    failed_at_secs: None,
                }),
            ],
            Vec::new(),
        )
    }

    #[test]
    fn renders_header_termination_and_result() {
        let text = render(&sample_report());
        assert!(text.contains("[run] smoke"));
        assert!(text.contains("[run] terminated: time budget reached at t=47.3s"));
        assert!(text.contains("[run] result: FAILED  (1 of 2 assertions)"));
    }

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
        assert!(text.contains("[assert] speed_capped: agent.car.node.estimator.speed < 5"));
        assert!(text.contains("FAILED (got 7: condition failed) [40 samples, max gap 1.00s]"));
    }

    #[test]
    fn a_range_prints_its_bounds() {
        assert_eq!(
            condition_phrase(&Condition::WithinRange([0.3, 0.7])),
            "in [0.3, 0.7]"
        );
        assert_eq!(reducer_phrase(&Reducer::TimeAbove(1.5)), "time_above(1.5)");
    }

    #[test]
    fn a_failure_shows_its_time_and_reason() {
        let mut report = sample_report();
        report.assertions[1] = failing_speed(Verdict::Failed {
            actual: Some(AssertionValue::Float(7.0)),
            reason: FailureReason::ConditionFailed,
            failed_at_secs: Some(1.25),
        });
        assert!(render(&report).contains("FAILED (got 7 at t=1.25s: condition failed)"));
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
    fn a_fail_fast_stop_names_the_failed_assertion() {
        let mut report = sample_report();
        report.terminated_by = TerminatedBy::AssertionFailed {
            name: "speed_capped".to_string(),
        };
        assert!(render(&report)
            .contains("[run] terminated: assertion 'speed_capped' FAILED at t=47.3s"));
    }

    #[test]
    fn a_refused_write_is_shown_as_a_wiring_line() {
        let mut report = sample_report();
        report.refused_writes = vec!["agent 'car': truth.tf refused 3 write(s)".to_string()];
        assert!(render(&report).contains("[wiring] agent 'car': truth.tf refused 3 write(s)"));
    }

    #[test]
    fn a_recording_fault_is_shown_as_a_recording_line() {
        let mut report = sample_report();
        report.recording = vec!["agent.car.node.estimator.x: 1 value(s)".to_string()];
        assert!(render(&report).contains("[recording] agent.car.node.estimator.x: 1 value(s)"));
    }

    #[test]
    fn a_measure_shows_its_value_or_why_it_has_none() {
        let mut report = sample_report();
        report.measures = vec![
            MeasureReportEntry::new(
                "goal_distance".to_string(),
                "agent.car.node.eval.goal_distance".to_string(),
                Reducer::Final,
                true,
                Ok(AssertionValue::Float(0.25)),
                series(1),
            ),
            MeasureReportEntry::new(
                "peak_speed".to_string(),
                "agent.car.node.estimator.speed".to_string(),
                Reducer::Max,
                false,
                Err(FailureReason::NoSamples),
                series(0),
            ),
        ];
        let text = render(&report);
        assert!(text.contains(
            "[measure] goal_distance: final of agent.car.node.eval.goal_distance = 0.25 \
             [1 samples, max gap 1.00s]"
        ));
        assert!(text.contains(
            "[measure] peak_speed: max of agent.car.node.estimator.speed = none (no samples) \
             [0 samples, max gap 1.00s] (not covered: the run stopped before its end)"
        ));
    }
}
