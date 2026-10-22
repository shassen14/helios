//! _Post-step._ The aggregate's human surface: render an [`AggregateReport`]
//! to text and print it. Pure presentation over the struct `toml_writer`
//! serializes, so the console and the `.toml` can never disagree. A
//! submodule of `aggregate`, so it reads the rows' private fields directly,
//! and phrases entries as the single-run console does.

use super::{AggregateReport, AssertionAggregate, MeasureAggregate};
use crate::report::phrase::{entry_name, reducer_phrase, with_unit};

/// Print a finished aggregate report to stdout. The one sanctioned
/// `println!` for this surface: it renders an already-decided result for a
/// human, it does not emit metrics or telemetry.
pub fn print(report: &AggregateReport) {
    println!("{}", render(report));
}

/// The full text, line by line: the batch, then one line per measure and
/// one per assertion, in run-file order. Does no computation; every number
/// was reduced by the aggregator.
fn render(report: &AggregateReport) -> String {
    let mut lines = vec![format!(
        "[mc] {} runs{}{}",
        report.run_count,
        seeds_phrase(&report.seeds),
        no_report_phrase(&report.no_report)
    )];
    lines.extend(report.measures.iter().map(|row| measure_line(report, row)));
    lines.extend(report.assertions.iter().map(assertion_line));
    lines.join("\n")
}

/// `, seeds [1, 2]`, or nothing for an unseeded batch.
fn seeds_phrase(seeds: &[u64]) -> String {
    if seeds.is_empty() {
        return String::new();
    }
    let seeds: Vec<String> = seeds.iter().map(u64::to_string).collect();
    format!(", seeds [{}]", seeds.join(", "))
}

/// `; runs 1, 4 produced no report`, or nothing when every run did.
fn no_report_phrase(no_report: &[usize]) -> String {
    match no_report {
        [] => String::new(),
        [run] => format!("; run {run} produced no report"),
        runs => {
            let runs: Vec<String> = runs.iter().map(usize::to_string).collect();
            format!("; runs {} produced no report", runs.join(", "))
        }
    }
}

/// `[mc measure] <name>: <reducer> of <target> over N runs (S samples):
/// mean … m, std … m, …`, then the runs it had no number in.
fn measure_line(report: &AggregateReport, row: &MeasureAggregate) -> String {
    let statistics: Vec<String> = report
        .statistics
        .iter()
        .filter_map(|statistic| {
            let value = row.statistics.get(statistic)?;
            Some(format!(
                "{statistic} {}",
                with_unit(value.to_string(), row.unit)
            ))
        })
        .collect();
    let mut line = format!(
        "[mc measure] {}: {} of {} over {} runs ({} samples)",
        entry_name(&row.name, row.instance.as_deref()),
        reducer_phrase(&row.reduce),
        row.target,
        row.runs,
        row.samples,
    );
    if !statistics.is_empty() {
        line.push_str(&format!(": {}", statistics.join(", ")));
    }
    let missing: Vec<String> = [
        (row.not_covered, "not covered"),
        (row.no_value, "no value"),
        (row.not_numbers, "not a number"),
        (row.absent, "absent"),
        (row.no_report, "no report"),
    ]
    .iter()
    .filter(|(count, _)| *count > 0)
    .map(|(count, why)| format!("{count} {why}"))
    .collect();
    if !missing.is_empty() {
        line.push_str(&format!(" [{}]", missing.join(", ")));
    }
    line
}

/// `[mc assert] <name>: <target> passed P of N runs (pass_rate …)`, then
/// each count of the runs it didn't pass in.
fn assertion_line(row: &AssertionAggregate) -> String {
    let mut line = format!(
        "[mc assert] {}: {} passed {} of {} runs",
        entry_name(&row.name, row.instance.as_deref()),
        row.target,
        row.passed,
        row.runs,
    );
    if let Some(pass_rate) = row.pass_rate {
        line.push_str(&format!(" (pass_rate {pass_rate})"));
    }
    for (count, why) in [
        (row.failed, "failed"),
        (row.unjudged, "unjudged"),
        (row.absent, "absent"),
        (row.no_report, "no report"),
    ] {
        if count > 0 {
            line.push_str(&format!(", {count} {why}"));
        }
    }
    line
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::AssertionValue;
    use crate::report::aggregate::tests::{assertion, batch, batch_with_gaps, measure};
    use crate::report::Verdict;

    #[test]
    fn a_measure_line_names_its_runs_samples_and_units() {
        let report = batch(&[
            (vec![measure("final_x", Some(1.0), 10, true)], vec![]),
            (vec![measure("final_x", Some(3.0), 12, true)], vec![]),
        ]);

        let text = render(&report);

        assert!(text.starts_with("[mc] 2 runs, seeds [0, 1]"), "{text}");
        assert!(
            text.contains(
                "[mc measure] final_x: final of eval.inline.pose.x over 2 runs (22 samples): \
                 mean 2 m, std 1.4142135623730951 m, min 1 m, max 3 m"
            ),
            "{text}"
        );
    }

    #[test]
    fn runs_without_a_number_are_counted_by_why() {
        let report = batch(&[
            (vec![measure("final_x", Some(1.0), 10, false)], vec![]),
            (vec![measure("final_x", None, 0, true)], vec![]),
        ]);

        let text = render(&report);

        assert!(text.contains("[1 not covered, 1 no value]"), "{text}");
    }

    #[test]
    fn an_assertion_line_counts_its_verdicts() {
        let passed = Verdict::Passed {
            actual: AssertionValue::Float(1.0),
        };
        let unjudged = Verdict::Unjudged { actual: None };
        let report = batch(&[
            (vec![], vec![assertion("nis_final", passed.clone())]),
            (vec![], vec![assertion("nis_final", passed)]),
            (vec![], vec![assertion("nis_final", unjudged)]),
        ]);

        let text = render(&report);

        assert!(
            text.contains(
                "[mc assert] nis_final: agent.car.node.primary.aiding.gps.nis passed 2 of 3 \
                 runs (pass_rate 0.6666666666666666), 1 unjudged"
            ),
            "{text}"
        );
    }

    #[test]
    fn runs_with_no_report_are_named_and_counted_on_each_line() {
        let passed = Verdict::Passed {
            actual: AssertionValue::Float(1.0),
        };
        let report = batch_with_gaps(&[
            Some((
                vec![measure("final_x", Some(1.0), 10, true)],
                vec![assertion("nis_final", passed)],
            )),
            None,
            Some((vec![], vec![])),
        ]);

        let text = render(&report);

        assert!(
            text.starts_with("[mc] 3 runs, seeds [0, 1, 2]; run 1 produced no report"),
            "{text}"
        );
        assert!(text.contains("[1 absent, 1 no report]"), "{text}");
        assert!(
            text.contains(
                "passed 1 of 3 runs (pass_rate 0.3333333333333333), 1 absent, 1 no report"
            ),
            "{text}"
        );
    }

    #[test]
    fn an_unseeded_empty_batch_renders_its_header_only() {
        let report = AggregateReport::from_runs(&[]);

        assert_eq!(render(&report), "[mc] 0 runs");
    }
}
