//! _Post-step._ The Monte Carlo batch's report: reduce a slice of per-run
//! [`RunMetrics`] into one cross-run summary, keyed by metric and statistic.
//!
//! This is the cross-run half of a two-level reduction. `crate::statistics`
//! reduces a within-run series to a per-run scalar; here, once every run is a
//! scalar, each metric is just a bag of N numbers — and the *same*
//! [`StatisticTable`] reduces that bag to `mean`/`std`/`min`/`max`. One `mean`,
//! two levels. The aggregator is [`AggregateReport`]'s constructor, so the report
//! and the function that builds it never drift apart.

pub mod console;
pub mod toml_writer;

use crate::{
    statistics::{StatId, StatisticTable},
    MetricId, RunMetrics,
};

use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};

/// _Post-step._ The serialized summary of an N-run batch: how many runs ran,
/// which seed produced each, and — per metric — the selected statistics reduced
/// across the runs that recorded it.
///
/// Built once after the batch and serialized to TOML / rendered to the console,
/// mirroring the single-run [`crate::Report`]: it holds only report-owned types,
/// so metric and statistic ids are flattened to `String` here. That keeps this
/// output schema stable while the internal `MetricId` / `StatId` types churn.
///
/// `seeds` is carried metadata, never itself reduced — there is no "mean of the
/// seeds", so it lives beside the metric map, not inside it. It holds only the
/// seeds that are known: an unseeded batch reports an empty list rather than
/// placeholder zeros. The `BTreeMap` (and the `BTreeSet` the constructor builds
/// from) keep the rows in a stable, sorted order, so the serialized output is
/// deterministic across runs of the tool.
#[derive(Debug, Clone, Serialize)]
pub struct AggregateReport {
    run_count: usize,
    seeds: Vec<u64>,
    /// The selected statistics, in the order they were requested — this is the
    /// console's column order. The per-row `stats` maps are sorted by id for
    /// stable TOML; the console reorders them back to this selection order.
    stat_ids: Vec<String>,
    /// One row per metric any run recorded, keyed by the metric's name and
    /// sorted by it.
    metrics: BTreeMap<String, MetricRow>,
}

impl AggregateReport {
    /// Reduce `runs` to a cross-run summary. For every metric *any* run recorded,
    /// gather that metric's values from the runs that recorded it and apply each
    /// statistic named in `stat_ids`, looked up in `table`.
    ///
    /// `table` and `stat_ids` are injected, not built here, so the reducer set is
    /// config-chosen and this stays a pure, testable function over its inputs.
    pub fn from_runs(runs: &[RunMetrics], table: &StatisticTable, stat_ids: &[StatId]) -> Self {
        let run_count: usize = runs.len();
        // Only known seeds are carried: an unseeded run has no seed to report, so
        // it contributes nothing rather than a fabricated `0`. `--seed` is
        // all-or-nothing across a batch, so this is either every run's seed (in
        // run order) or empty — never a subset that would misalign with runs.
        let seeds: Vec<u64> = runs.iter().filter_map(|r| r.seed()).collect();
        let ids: Vec<String> = stat_ids.iter().map(|s| s.as_str().to_string()).collect();

        // Every metric any run recorded, deduplicated and sorted. The `flat_map`
        // splices each run's metric ids into one stream; collecting into a
        // `BTreeSet` collapses the duplicates and fixes the row order below.
        let metric_set: BTreeSet<&MetricId> =
            runs.iter().flat_map(|r| r.iter().map(|(m, _)| m)).collect();

        let mut metrics: BTreeMap<String, MetricRow> = BTreeMap::new();

        for metric_id in metric_set {
            // Only the runs that recorded this metric contribute: a run missing
            // it yields `None`, which `filter_map` drops. `present` is how many
            // did record it, so a mean over 3 of 10 runs is never read as if all
            // 10 measured.
            let present_values: Vec<f64> = runs.iter().filter_map(|r| r.get(metric_id)).collect();

            // Apply each selected statistic to the gathered values. A statistic
            // undefined for this input returns `None` and is simply omitted from
            // the row — an absent key, never a null (TOML has no null).
            let stats: BTreeMap<String, f64> = stat_ids
                .iter()
                .filter_map(|stat_id| {
                    table
                        .apply(stat_id, &present_values)
                        .map(|v| (stat_id.as_str().to_string(), v))
                })
                .collect();

            let metric_row = MetricRow::new(present_values.len(), stats);

            metrics.insert(metric_id.as_str().to_string(), metric_row);
        }

        Self {
            run_count,
            seeds,
            stat_ids: ids,
            metrics,
        }
    }
}

/// _Post-step._ One metric's row in the aggregate: how many runs recorded it, and
/// the selected statistics reduced over those runs. The metric's own name is the
/// key in [`AggregateReport::metrics`], so it is not repeated here. Statistics
/// that came out undefined are absent from `stats`, never stored as a null.
#[derive(Debug, Clone, Serialize)]
pub struct MetricRow {
    present: usize,
    stats: BTreeMap<String, f64>,
}

impl MetricRow {
    /// Pair a present-run count with the already-reduced statistics for one
    /// metric. Both come from the aggregator's per-metric loop.
    pub(crate) fn new(present: usize, stats: BTreeMap<String, f64>) -> Self {
        Self { present, stats }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::statistics::standard_statistics;

    // `std` goes through a `sqrt`, so compare it with a tolerance; the other
    // reducers in these fixtures land on exactly representable values.
    const EPS: f64 = 1e-9;

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < EPS
    }

    // Build one seeded run that recorded the given `(metric, value)` pairs.
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
    fn reduces_a_metric_across_runs_to_known_statistics() {
        // The classic worked example: these eight values have mean 5, population
        // std 2, min 2, max 9 — one value per run, all recording `cte.mean`.
        let samples = [2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0];
        let runs: Vec<RunMetrics> = samples
            .iter()
            .enumerate()
            .map(|(i, v)| run_with(i as u32, i as u64, &[("cte.mean", *v)]))
            .collect();

        let table = standard_statistics();
        let report =
            AggregateReport::from_runs(&runs, &table, &stat_ids(&["mean", "std", "min", "max"]));

        assert_eq!(report.run_count, 8);

        let row = &report.metrics["cte.mean"];
        assert_eq!(row.present, 8);
        assert_eq!(row.stats["mean"], 5.0);
        assert_eq!(row.stats["min"], 2.0);
        assert_eq!(row.stats["max"], 9.0);
        assert!(
            approx(row.stats["std"], 2.0),
            "std was {}",
            row.stats["std"]
        );
    }

    #[test]
    fn a_metric_missing_from_some_runs_reduces_over_the_present_subset() {
        // `rise_time` is recorded by two of the three runs; `cte.mean` by all.
        let runs = vec![
            run_with(0, 0, &[("cte.mean", 0.5), ("rise_time", 1.0)]),
            run_with(1, 1, &[("cte.mean", 0.5)]),
            run_with(2, 2, &[("cte.mean", 0.5), ("rise_time", 3.0)]),
        ];

        let table = standard_statistics();
        let report = AggregateReport::from_runs(&runs, &table, &stat_ids(&["mean"]));

        let rise = &report.metrics["rise_time"];
        assert_eq!(rise.present, 2, "only two runs recorded rise_time");
        assert_eq!(rise.stats["mean"], 2.0, "mean of [1.0, 3.0]");

        assert_eq!(report.metrics["cte.mean"].present, 3);
    }

    #[test]
    fn carries_the_per_run_seeds_in_order() {
        let runs = vec![
            run_with(0, 10, &[("cte.mean", 0.0)]),
            run_with(1, 20, &[("cte.mean", 0.0)]),
            run_with(2, 30, &[("cte.mean", 0.0)]),
        ];

        let table = standard_statistics();
        let report = AggregateReport::from_runs(&runs, &table, &stat_ids(&["mean"]));

        assert_eq!(report.seeds, vec![10, 20, 30]);
    }

    #[test]
    fn an_unseeded_batch_carries_no_seeds() {
        // Runs the harness left unseeded (`None`) must not surface as placeholder
        // zeros: the seed list is empty, not `[0, 0]`. This is the fix for the
        // fabricated-seed smell.
        let runs = vec![
            {
                let mut r = RunMetrics::new(0, None);
                r.insert(MetricId::new("cte.mean"), 0.0);
                r
            },
            {
                let mut r = RunMetrics::new(1, None);
                r.insert(MetricId::new("cte.mean"), 0.0);
                r
            },
        ];

        let table = standard_statistics();
        let report = AggregateReport::from_runs(&runs, &table, &stat_ids(&["mean"]));

        assert!(
            report.seeds.is_empty(),
            "unseeded batch must report no seeds"
        );
        // The metrics still aggregate normally — only the seed metadata is absent.
        assert_eq!(report.metrics["cte.mean"].present, 2);
    }

    #[test]
    fn metric_rows_are_in_sorted_key_order() {
        // Inserted out of order; the BTreeSet/BTreeMap must hand them back sorted.
        let runs = vec![run_with(
            0,
            0,
            &[("rise_time", 1.0), ("cte.mean", 0.4), ("abs_error", 0.1)],
        )];

        let table = standard_statistics();
        let report = AggregateReport::from_runs(&runs, &table, &stat_ids(&["mean"]));

        let keys: Vec<&str> = report.metrics.keys().map(|k| k.as_str()).collect();
        assert_eq!(keys, ["abs_error", "cte.mean", "rise_time"]);
    }

    #[test]
    fn an_undefined_statistic_is_omitted_not_stored_as_null() {
        let runs = vec![run_with(0, 0, &[("cte.mean", 1.0)])];

        // A reducer that is always undefined; its key must never reach the row.
        let mut table = standard_statistics();
        table.register(StatId::new("always_undefined"), |_| None);

        let report =
            AggregateReport::from_runs(&runs, &table, &stat_ids(&["mean", "always_undefined"]));

        let row = &report.metrics["cte.mean"];
        assert_eq!(row.stats["mean"], 1.0);
        assert!(
            !row.stats.contains_key("always_undefined"),
            "an undefined statistic must be absent, not null"
        );
    }

    #[test]
    fn empty_batch_yields_an_empty_report() {
        let table = standard_statistics();
        let report = AggregateReport::from_runs(&[], &table, &stat_ids(&["mean", "std"]));

        assert_eq!(report.run_count, 0);
        assert!(report.seeds.is_empty());
        assert!(report.metrics.is_empty());
        // The selected columns are still recorded — an empty batch still knows
        // which statistics it would have produced.
        assert_eq!(report.stat_ids, vec!["mean".to_string(), "std".to_string()]);
    }
}
