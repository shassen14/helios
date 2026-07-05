//! _Post-step._ The aggregate's human surface: render an [`AggregateReport`] to a
//! block of text and print it. Pure presentation over the same struct
//! `toml_writer` serializes, so the console table and the `.toml` can never
//! disagree.
//!
//! Every "how do I show this value" decision is a small private helper in this
//! file — not an `impl Display` on the data types. `console` is a submodule of
//! `aggregate`, so it reads `AggregateReport`'s and `MetricRow`'s private fields
//! directly rather than widening their public surface for what is purely
//! presentation glue.
//!
//! The table is a header row and one row per metric, *all* formatted through the
//! same [`format_row`] helper — so the columns cannot drift out of alignment.
//! The stat columns are driven by `AggregateReport::stat_ids` (the requested
//! order), looked up in each metric's map; a metric missing a stat shows
//! [`MISSING`], never a fabricated zero.

use super::{AggregateReport, MetricRow};

/// Width the metric-name column is padded to before the value columns begin.
const METRIC_COLUMN: usize = 16;

/// Width each value column (the `present` count and every statistic) is padded
/// to, so the columns line up down the page.
const STAT_COLUMN: usize = 10;

/// Decimal places shown for a statistic value.
const FLOAT_PRECISION: usize = 3;

/// Shown in a stat column when a metric has no value for that statistic (the
/// statistic was undefined for that metric's samples). A placeholder, never `0` —
/// a zero would read as a real measurement.
const MISSING: &str = "—";

/// Print a finished aggregate report to stdout. The one sanctioned `println!` for
/// this surface: it renders an already-decided result for a human, it does not
/// emit metrics or telemetry (those go through the observability hook).
pub fn print(report: &AggregateReport) {
    println!("{}", render(report));
}

/// Build the full table text, line by line. Split out from [`print`] so it can be
/// unit-tested by asserting on the returned `String` without capturing stdout.
/// Does no computation — every number was reduced upstream by the aggregator;
/// this only arranges those numbers as aligned text.
fn render(report: &AggregateReport) -> String {
    // Collect each line, then join with newlines at the end — simpler than
    // pushing `\n` by hand and it can't leave stray blank or trailing-space lines.
    let mut lines: Vec<String> = Vec::new();

    // Summary header: how many runs the table aggregates.
    lines.push(format!("[mc] {} runs", report.run_count));

    // Column header: the `present` column, then one column per requested
    // statistic. Built from `stat_ids` so this row fixes the column order that
    // every data row below must follow.
    let mut header_cells: Vec<String> = Vec::with_capacity(report.stat_ids.len() + 1);
    header_cells.push("present".to_string());
    header_cells.extend(report.stat_ids.iter().cloned());
    lines.push(format_row("metric", &header_cells));

    // One row per metric, in the map's sorted key order (deterministic output).
    for (name, row) in &report.metrics {
        let mut cells: Vec<String> = Vec::with_capacity(report.stat_ids.len() + 1);
        cells.push(row.present.to_string());

        // Walk `stat_ids`, not the row's map: the columns must appear in the
        // requested order for every row, with a placeholder where this metric
        // never produced that statistic.
        for stat_id in &report.stat_ids {
            cells.push(stat_cell(row, stat_id));
        }

        lines.push(format_row(name, &cells));
    }

    lines.join("\n")
}

/// Lay out one table line: the left label padded to the metric column, then each
/// value cell padded to a value column. Header and data rows both go through
/// here, which is what guarantees their columns align. Trailing padding on the
/// last cell is trimmed so no line carries dangling whitespace.
fn format_row(label: &str, cells: &[String]) -> String {
    let mut line = format!("{:<width$}", label, width = METRIC_COLUMN);
    for cell in cells {
        line.push_str(&format!("{:<width$}", cell, width = STAT_COLUMN));
    }
    line.trim_end().to_string()
}

/// The text for one metric's value under one statistic: the formatted number when
/// the metric recorded that statistic, or [`MISSING`] when it didn't.
fn stat_cell(row: &MetricRow, stat_id: &str) -> String {
    match row.stats.get(stat_id) {
        Some(value) => format_value(*value),
        None => MISSING.to_string(),
    }
}

/// Render a statistic value to a fixed number of decimals, so the columns share a
/// consistent shape.
fn format_value(value: f64) -> String {
    format!("{:.prec$}", value, prec = FLOAT_PRECISION)
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::statistics::{standard_statistics, StatId, StatisticTable};
    use crate::{MetricId, RunMetrics};

    fn run_with(run_index: u32, seed: u64, recorded: &[(&str, f64)]) -> RunMetrics {
        let mut run = RunMetrics::new(run_index, Some(seed));
        for (id, value) in recorded {
            run.insert(MetricId::new(*id), *value);
        }
        run
    }

    fn stat_ids(ids: &[&str]) -> Vec<StatId> {
        ids.iter().map(|id| StatId::new(*id)).collect()
    }

    #[test]
    fn renders_summary_header_and_known_rows() {
        let runs = vec![
            run_with(0, 0, &[("cte.mean", 5.0)]),
            run_with(1, 1, &[("cte.mean", 5.0)]),
        ];
        let table = standard_statistics();
        let report = AggregateReport::from_runs(&runs, &table, &stat_ids(&["mean", "min", "max"]));

        let text = render(&report);

        assert!(text.contains("[mc] 2 runs"), "missing summary header:\n{text}");
        assert!(text.contains("metric"), "missing column header:\n{text}");
        assert!(text.contains("present"));
        // The metric row, with its present count and a value at the chosen precision.
        assert!(text.contains("cte.mean"), "missing metric row:\n{text}");
        assert!(text.contains("5.000"), "value not at 3 decimals:\n{text}");
    }

    #[test]
    fn columns_follow_requested_order_not_alphabetical() {
        // Deliberately request `min` before `mean` — alphabetical order would
        // flip them, so the header proves the column order tracks `stat_ids`.
        let runs = vec![run_with(0, 0, &[("cte.mean", 1.0)])];
        let table = standard_statistics();
        let report = AggregateReport::from_runs(&runs, &table, &stat_ids(&["min", "mean"]));

        let header = render(&report)
            .lines()
            .find(|line| line.starts_with("metric"))
            .expect("a header row")
            .to_string();

        let min_at = header.find("min").expect("min column");
        let mean_at = header.find("mean").expect("mean column");
        assert!(min_at < mean_at, "columns not in requested order: {header}");
    }

    #[test]
    fn absent_statistic_renders_the_placeholder() {
        let runs = vec![run_with(0, 0, &[("cte.mean", 1.0)])];

        // A reducer that is always undefined, so its column has no value to show.
        let mut table: StatisticTable = standard_statistics();
        table.register(StatId::new("always_undefined"), |_| None);

        let report =
            AggregateReport::from_runs(&runs, &table, &stat_ids(&["mean", "always_undefined"]));

        let text = render(&report);
        assert!(
            text.contains(MISSING),
            "an undefined statistic should show the placeholder:\n{text}"
        );
    }

    #[test]
    fn empty_batch_still_renders_a_header() {
        let table = standard_statistics();
        let report = AggregateReport::from_runs(&[], &table, &stat_ids(&["mean"]));

        let text = render(&report);
        assert!(text.contains("[mc] 0 runs"));
        assert!(text.contains("metric"));
    }

    #[test]
    fn no_line_carries_trailing_whitespace() {
        let runs = vec![run_with(0, 0, &[("cte.mean", 1.0)])];
        let table = standard_statistics();
        let report = AggregateReport::from_runs(&runs, &table, &stat_ids(&["mean", "min", "max"]));

        for line in render(&report).lines() {
            assert_eq!(line.trim_end(), line, "trailing whitespace on: {line:?}");
        }
    }
}
