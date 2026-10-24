//! _Post-step._ The report's human surface: render a finished [`Report`] to a
//! block of text and print it. Pure presentation over the same struct the TOML
//! writer serializes, so the console and `result.toml` can never disagree.
//!
//! Every "how do I show this value" decision is a small private helper in
//! this module — not an `impl Display` on the data types. Keeping the
//! presentation here means the report types in `mod.rs` stay pure data, and
//! there's exactly one place to look when the output format changes.
//!
//! `console` is a submodule of `report`, so it reads `Report`'s and
//! `AssertionReportEntry`'s private fields directly rather than widening their
//! public accessor set for what is purely presentation glue.

mod entries;
mod evaluation;

#[cfg(test)]
mod fixtures;

use self::{
    entries::{assertion_line, measure_line},
    evaluation::evaluation_lines,
};
use super::{phrase::entry_name, AssertionStatus, Report, ReportStatus, TerminatedBy};

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

    // One block per evaluation, before the entries judging its metrics.
    for evaluation in report.evaluations() {
        lines.push(evaluation_lines(evaluation));
    }

    // One line per assertion, in run-file order.
    for entry in report.assertions() {
        lines.push(assertion_line(entry));
    }

    // One line per measure, after the verdicts: a value, never a verdict.
    for entry in report.measures() {
        lines.push(measure_line(entry));
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
        TerminatedBy::AssertionFailed { name, instance, .. } => format!(
            "assertion '{}' {} at {at}",
            entry_name(name, None, instance.as_deref()),
            status_label(&AssertionStatus::Failed)
        ),
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
    use super::{fixtures::sample_report, *};

    #[test]
    fn renders_header_termination_and_result() {
        let text = render(&sample_report());
        assert!(text.contains("[run] smoke"));
        assert!(text.contains("[run] terminated: time budget reached at t=47.3s"));
        assert!(text.contains("[run] result: FAILED  (1 of 2 assertions)"));
    }

    #[test]
    fn a_fail_fast_stop_names_the_failed_assertion() {
        let mut report = sample_report();
        report.terminated_by = TerminatedBy::AssertionFailed {
            name: "speed_capped".to_string(),
            path: "agent.car.node.estimator.speed".to_string(),
            instance: None,
        };
        assert!(render(&report)
            .contains("[run] terminated: assertion 'speed_capped' FAILED at t=47.3s"));
    }

    #[test]
    fn an_entry_on_one_instance_is_named_with_it() {
        let mut report = sample_report();
        report.assertions[1].instance = Some("rear".to_string());
        report.terminated_by = TerminatedBy::AssertionFailed {
            name: "speed_capped".to_string(),
            path: "agent.car.channel.wheels".to_string(),
            instance: Some("rear".to_string()),
        };

        let text = render(&report);

        assert!(
            text.contains("[assert] speed_capped[instance=rear]: "),
            "{text}"
        );
        assert!(
            text.contains("assertion 'speed_capped[instance=rear]' FAILED"),
            "{text}"
        );
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
}
