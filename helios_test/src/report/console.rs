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

use super::{AssertionStatus, FailureReason, Report, ReportStatus, TerminatedBy, Verdict};

use crate::assertion::{AssertionValue, Condition};

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

    // One line per assertion, in run-file order.
    for entry in report.assertions() {
        // The left half: "[assert] <target> <op> <expected>".
        let head = format!(
            "[assert] {} {} {}",
            entry.target,
            condition_glyph(entry.condition),
            value_string(&entry.expected),
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
        TerminatedBy::AssertionFailed { number } => format!(
            "assertion {number} {} at {at}",
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
/// [`condition_glyph`]; the TOML carries the snake_case name instead.
fn reason_phrase(reason: FailureReason) -> &'static str {
    match reason {
        FailureReason::Rejected => "value rejected",
        FailureReason::NoSamples => "no samples",
        FailureReason::TypeMismatch => "type mismatch",
        FailureReason::ConditionFailed => "condition failed",
        FailureReason::InvalidBounds => "invalid bounds",
    }
}

/// The human glyph for a comparison verb — a phrasebook, not logic. `Condition`
/// is `Copy`, so it's taken by value. `WithinRange` renders as the word "within"
/// because the report entry carries only `expected`, not the low/high band.
fn condition_glyph(condition: Condition) -> &'static str {
    match condition {
        Condition::Equals => "==",
        Condition::NotEquals => "!=",
        Condition::LessThan => "<",
        Condition::GreaterThan => ">",
        Condition::WithinRange => "within",
    }
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

    use crate::report::AssertionReportEntry;

    // Build a finished report with one passing and one failing assertion, stopped
    // on the time budget — enough to exercise every rendering branch.
    fn sample_report() -> Report {
        Report::new(
            "smoke".to_string(),
            "sim.scenarios.parking_lot".to_string(),
            Some(42),
            ReportStatus::Failed,
            Vec::new(),
            47.3,
            4.1,
            TerminatedBy::TimeBudget,
            vec![
                AssertionReportEntry::new(
                    "agent.car.mission_state".to_string(),
                    Condition::Equals,
                    AssertionValue::String("Succeeded".to_string()),
                    Verdict::Passed {
                        actual: AssertionValue::String("Succeeded".to_string()),
                    },
                ),
                AssertionReportEntry::new(
                    "agent.car.speed".to_string(),
                    Condition::LessThan,
                    AssertionValue::Float(5.0),
                    Verdict::Failed {
                        actual: Some(AssertionValue::Float(7.0)),
                        reason: FailureReason::ConditionFailed,
                        failed_at_secs: None,
                    },
                ),
            ],
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
        // Passing line: quoted string and the `==` glyph, labelled PASSED.
        assert!(text.contains("[assert] agent.car.mission_state == \"Succeeded\""));
        assert!(text.contains("PASSED"));
        // Failing line: the `<` glyph, FAILED, and the observed value.
        assert!(text.contains("[assert] agent.car.speed < 5"));
        assert!(text.contains("FAILED (got 7: condition failed)"));
    }

    #[test]
    fn a_failure_shows_its_time_and_reason() {
        let mut report = sample_report();
        report.assertions[1] = AssertionReportEntry::new(
            "agent.car.speed".to_string(),
            Condition::LessThan,
            AssertionValue::Float(5.0),
            Verdict::Failed {
                actual: Some(AssertionValue::Float(7.0)),
                reason: FailureReason::ConditionFailed,
                failed_at_secs: Some(1.25),
            },
        );
        assert!(render(&report).contains("FAILED (got 7 at t=1.25s: condition failed)"));
    }

    #[test]
    fn a_failure_with_no_value_shows_only_its_reason() {
        let mut report = sample_report();
        report.assertions[1] = AssertionReportEntry::new(
            "agent.car.speed".to_string(),
            Condition::LessThan,
            AssertionValue::Float(5.0),
            Verdict::Failed {
                actual: None,
                reason: FailureReason::NoSamples,
                failed_at_secs: None,
            },
        );
        assert!(render(&report).contains("FAILED (no samples)"));
    }

    #[test]
    fn unjudged_entry_without_actual_does_not_panic() {
        let report = Report::new(
            "unjudged".to_string(),
            "s".to_string(),
            None,
            ReportStatus::Failed,
            Vec::new(),
            1.0,
            0.1,
            TerminatedBy::TimeBudget,
            vec![AssertionReportEntry::new(
                "agent.car.position".to_string(),
                Condition::Equals,
                AssertionValue::Float(0.0),
                Verdict::Unjudged { actual: None },
            )],
        );
        let text = render(&report);
        assert!(text.contains("UNJUDGED"));
    }

    #[test]
    fn a_fail_fast_stop_names_the_failed_assertion() {
        let mut report = sample_report();
        report.terminated_by = TerminatedBy::AssertionFailed { number: 2 };
        assert!(render(&report).contains("[run] terminated: assertion 2 FAILED at t=47.3s"));
    }
}
