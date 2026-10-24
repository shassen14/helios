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
    evaluation::evaluation_report_lines,
};
use super::{
    phrase::entry_name, AbortCause, AssertionStatus, FaultStep, HostFault, Report, ReportStatus,
    TerminatedBy,
};

/// Between the problems of one refused entry or evaluation, and between the
/// causes of one abort.
const LIST_SEPARATOR: &str = "; ";

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

    // Each unbuilt pipeline's errors come before the entry lines: they
    // explain why there may be no entry lines worth reading. Every other
    // cause is told in full by the termination line or its entries.
    if let TerminatedBy::Aborted { causes } = &report.terminated_by {
        for cause in causes {
            if let AbortCause::PipelineNotBuilt { agent, errors } = cause {
                lines.extend(
                    errors
                        .iter()
                        .map(|error| format!("[setup] agent '{agent}' has no pipeline: {error}")),
                );
            }
        }
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
        lines.push(evaluation_report_lines(evaluation));
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
/// say which assertion failed, and for an abort every cause.
fn termination_line(report: &Report) -> String {
    let at = format!("t={:.1}s", report.simulated_duration_secs);
    match &report.terminated_by {
        TerminatedBy::Aborted { causes } => {
            let causes: Vec<String> = causes.iter().map(cause_phrase).collect();
            format!("aborted untested: {}", causes.join(LIST_SEPARATOR))
        }
        TerminatedBy::TimeBudget => format!("time budget reached at {at}"),
        TerminatedBy::AssertionFailed { name, instance, .. } => format!(
            "assertion '{}' {} at {at}",
            entry_name(name, None, instance.as_deref()),
            status_label(&AssertionStatus::Failed)
        ),
    }
}

/// One cause of an abort, phrased for the termination line.
fn cause_phrase(cause: &AbortCause) -> String {
    match cause {
        AbortCause::PipelineNotBuilt { agent, .. } => {
            format!("the pipeline of agent '{agent}' wasn't built")
        }
        AbortCause::UnusableEntries => "the run refused what it can't use".to_string(),
        AbortCause::HostFault(fault) => match fault {
            HostFault::AskedBeforeSetup => {
                "the host asked the collector what to record before setup".to_string()
            }
            HostFault::NeverSetUp => "the host never set the run up".to_string(),
            HostFault::NeverTicked => "the host finalized the run without a tick".to_string(),
            HostFault::StoppedUnasked => {
                "the host stopped the run without being told to".to_string()
            }
            HostFault::ForeignCollector { at } => format!(
                "the host handed over another run's collector {}",
                match at {
                    FaultStep::Tick { at_secs } => format!("at t={at_secs:.1}s"),
                    FaultStep::Finalize => "at finalize".to_string(),
                }
            ),
        },
    }
}

/// " (unusable: <problem>; <problem>)", after a refused line.
fn unusable_detail(problems: &[String]) -> String {
    format!(" (unusable: {})", problems.join(LIST_SEPARATOR))
}

/// Label for a per-assertion verdict. A plain `&'static str`, no allocation.
fn status_label(status: &AssertionStatus) -> &'static str {
    match status {
        AssertionStatus::Passed => "PASSED",
        AssertionStatus::Failed => "FAILED",
        AssertionStatus::Unjudged => "UNJUDGED",
        AssertionStatus::Unusable => "UNUSABLE",
    }
}

/// Label for the run-level verdict. Separate from [`status_label`] because
/// a run's statuses aren't an assertion's.
fn run_status_label(status: &ReportStatus) -> &'static str {
    match status {
        ReportStatus::Passed => "PASSED",
        ReportStatus::Failed => "FAILED",
        ReportStatus::Aborted => "ABORTED",
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
    fn an_aborted_run_names_every_cause_and_each_unbuilt_pipelines_errors() {
        let mut report = sample_report();
        report.status = ReportStatus::Aborted;
        report.terminated_by = TerminatedBy::Aborted {
            causes: vec![
                AbortCause::PipelineNotBuilt {
                    agent: "truck".to_string(),
                    errors: vec!["nothing supplies the map".to_string()],
                },
                AbortCause::UnusableEntries,
                AbortCause::HostFault(HostFault::ForeignCollector {
                    at: FaultStep::Finalize,
                }),
            ],
        };
        report.assertions[1] = fixtures::failing_speed(super::super::Verdict::Unusable {
            problems: vec!["its agent's pipeline wasn't built".to_string()],
        });

        let text = render(&report);

        assert!(
            text.contains(
                "[run] terminated: aborted untested: the pipeline of agent 'truck' wasn't \
                 built; the run refused what it can't use; the host handed over another \
                 run's collector at finalize"
            ),
            "{text}"
        );
        assert!(
            text.contains("[setup] agent 'truck' has no pipeline: nothing supplies the map"),
            "{text}"
        );
        assert!(
            text.contains("UNUSABLE (unusable: its agent's pipeline wasn't built)"),
            "{text}"
        );
        assert!(text.contains("[run] result: ABORTED"), "{text}");
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
