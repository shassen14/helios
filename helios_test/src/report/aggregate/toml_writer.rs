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
/// overwriting any existing file.
pub fn write(aggregate_report: &AggregateReport, path: &Path) -> Result<(), ReportWriteError> {
    let text = toml::to_string_pretty(aggregate_report)?;

    fs::write(path, text)?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::AssertionValue;
    use crate::report::aggregate::tests::{assertion, batch, measure};
    use crate::report::{FailureReason, Verdict};

    use std::sync::atomic::{AtomicU64, Ordering};

    // Two runs: a measure both gave a number and an assertion one passed.
    // Exercises the scalars, both arrays of tables and a row's `statistics`
    // table.
    fn sample_report() -> AggregateReport {
        let failed = Verdict::Failed {
            actual: Some(AssertionValue::Float(9.0)),
            reason: FailureReason::ConditionFailed,
            failed_at_secs: None,
        };
        let passed = Verdict::Passed {
            actual: AssertionValue::Float(1.0),
        };
        batch(&[
            (
                vec![measure("final_x", Some(1.0), 10, true)],
                vec![assertion("nis_final", passed)],
            ),
            (
                vec![measure("final_x", Some(3.0), 10, true)],
                vec![assertion("nis_final", failed)],
            ),
        ])
    }

    // Write `report` to a unique temp file, returning the path. The process id
    // and the atomic counter keep parallel tests, as threads or as processes,
    // from colliding on a filename — same std-only pattern the single-run
    // writer's tests use, to avoid a temp-file dep.
    fn write_to_temp(report: &AggregateReport) -> std::path::PathBuf {
        static SEQ: AtomicU64 = AtomicU64::new(0);
        let n = SEQ.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!(
            "helios_aggregate_test_{}_{n}.toml",
            std::process::id()
        ));
        write(report, &path).unwrap();
        path
    }

    #[test]
    fn round_trips_through_toml() {
        let report = sample_report();
        let path = write_to_temp(&report);

        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);

        let value: toml::Value = toml::from_str(&text).unwrap();
        assert_eq!(value["run_count"].as_integer(), Some(2));
        assert_eq!(value["seeds"].as_array().map(Vec::len), Some(2));
        assert_eq!(value["statistics"][0].as_str(), Some("mean"));

        let measure = &value["measures"][0];
        assert_eq!(measure["name"].as_str(), Some("final_x"));
        assert_eq!(measure["unit"].as_str(), Some("m"));
        assert_eq!(measure["runs"].as_integer(), Some(2));
        assert_eq!(measure["samples"].as_integer(), Some(20));
        assert_eq!(measure["statistics"]["mean"].as_float(), Some(2.0));
        assert!(measure["statistics"]["std"].as_float().is_some());
        // Zero counts are left out, never written as 0.
        assert!(measure.get("not_covered").is_none());
        assert!(measure.get("no_report").is_none());
        assert!(value.get("no_report").is_none());

        let assertion = &value["assertions"][0];
        assert_eq!(assertion["passed"].as_integer(), Some(1));
        assert_eq!(assertion["failed"].as_integer(), Some(1));
        assert_eq!(assertion["pass_rate"].as_float(), Some(0.5));
    }

    #[test]
    fn missing_parent_dir_is_io_error() {
        let report = sample_report();
        let result = write(&report, Path::new("/no/such/dir/aggregate.toml"));
        assert!(matches!(result, Err(ReportWriteError::Io(_))));
    }
}
