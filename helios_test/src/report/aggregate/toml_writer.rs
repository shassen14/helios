//! _Post-step._ The aggregate's machine surface: serialize a finished
//! [`AggregateReport`] to TOML and write it to disk, so a batch's summary can be
//! parsed alongside the per-run reports. Deliberately thin — serialize and write,
//! nothing else. Human-facing formatting lives in `console.rs`.
//!
//! Reuses [`ReportWriteError`] from the single-run writer: its `Io` / `Serialize`
//! variants are not `Report`-specific, so there is no reason to define a parallel
//! error here.

use super::AggregateReport;
use crate::report::toml_writer::ReportWriteError;

use std::fs;
use std::path::Path;

/// Serialize `aggregate_report` to pretty TOML and write it to `path`,
/// overwriting any existing file. Correctness of the serialize step depends on
/// the report's field order keeping scalars (`run_count`, `seeds`, `stat_ids`)
/// ahead of the `metrics` table, and `present` ahead of `stats` within each row —
/// TOML rejects a scalar emitted after a sub-table.
pub fn write(aggregate_report: &AggregateReport, path: &Path) -> Result<(), ReportWriteError> {
    let text = toml::to_string_pretty(aggregate_report)?;

    fs::write(path, text)?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::statistics::{standard_statistics, StatId};
    use crate::{MetricId, RunMetrics};

    use std::sync::atomic::{AtomicU64, Ordering};

    fn run_with(run_index: u32, seed: u64, recorded: &[(&str, f64)]) -> RunMetrics {
        let mut run = RunMetrics::new(run_index, Some(seed));
        for (id, value) in recorded {
            run.insert(MetricId::new(*id), *value);
        }
        run
    }

    // A representative batch: two runs, one metric every run recorded and one
    // recorded by only the first run, reduced over `mean / min / max`. Exercises
    // the scalars, the `metrics` table, and a row's `stats` sub-table.
    fn sample_report() -> AggregateReport {
        let runs = vec![
            run_with(0, 10, &[("cte.mean", 5.0), ("rise_time", 1.0)]),
            run_with(1, 20, &[("cte.mean", 5.0)]),
        ];
        let table = standard_statistics();
        let stat_ids = ["mean", "min", "max"].map(StatId::new);
        AggregateReport::from_runs(&runs, &table, &stat_ids)
    }

    // Write `report` to a unique temp file, returning the path. The atomic counter
    // keeps parallel test threads from colliding on a filename — same std-only
    // pattern the single-run writer's tests use, to avoid a temp-file dep.
    fn write_to_temp(report: &AggregateReport) -> std::path::PathBuf {
        static SEQ: AtomicU64 = AtomicU64::new(0);
        let n = SEQ.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!("helios_aggregate_test_{n}.toml"));
        write(report, &path).unwrap();
        path
    }

    #[test]
    fn round_trips_through_toml() {
        let report = sample_report();
        let path = write_to_temp(&report);

        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);

        // Re-parse as a generic TOML document and spot-check the scalars, the
        // dotted metric key, a value inside its `stats` sub-table, and the
        // present count for the partially-recorded metric.
        let value: toml::Value = toml::from_str(&text).unwrap();
        assert_eq!(value["run_count"].as_integer(), Some(2));
        assert_eq!(value["seeds"].as_array().unwrap().len(), 2);

        let cte = &value["metrics"]["cte.mean"];
        assert_eq!(cte["present"].as_integer(), Some(2));
        assert_eq!(cte["stats"]["mean"].as_float(), Some(5.0));

        // `rise_time` was recorded by only the first run.
        assert_eq!(value["metrics"]["rise_time"]["present"].as_integer(), Some(1));

        // A statistic that wasn't requested is absent, not nulled.
        assert!(cte["stats"].get("std").is_none());
    }

    #[test]
    fn missing_parent_dir_is_io_error() {
        let report = sample_report();
        let result = write(&report, Path::new("/no/such/dir/aggregate.toml"));
        assert!(matches!(result, Err(ReportWriteError::Io(_))));
    }
}
