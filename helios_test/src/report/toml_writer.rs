//! _Post-step._ The report's machine surface: serialize a finished [`Report`] to
//! TOML and write it to disk, so CI can parse the verdict alongside the process
//! exit code. Deliberately thin — serialize and write, nothing else. Human-facing
//! formatting lives in `console.rs`; keeping the two apart is what stops
//! readability concerns from leaking into the machine schema.

use super::Report;

use std::fs;
use std::path::Path;
use thiserror::Error;

/// _Post-step._ Why a report couldn't be written. The two variants stay distinct
/// so a failure is diagnosable: `Io` is a disk/path problem; `Serialize` is a
/// report whose shape the TOML serializer rejected (e.g. a scalar field ordered
/// after a sub-table). Each carries its source via `#[from]` so `?` converts
/// cleanly and the message keeps the underlying detail.
///
/// Not `Clone`: neither `std::io::Error` nor `toml::ser::Error` is `Clone`, and a
/// write error is consumed once at the call site.
#[derive(Debug, Error)]
pub enum ReportWriteError {
    #[error("failed to output toml: {0}")]
    Io(#[from] std::io::Error),
    #[error("failed to serialize toml: {0}")]
    Serialize(#[from] toml::ser::Error),
}

/// Serialize `report` to pretty TOML and write it to `path`, overwriting any
/// existing file. The `?` on each step converts the underlying error into the
/// matching [`ReportWriteError`] variant via `#[from]`. Correctness of the
/// serialize step depends on `Report`'s field order keeping scalars ahead of its
/// `terminated_by` table and `assertions` array-of-tables.
pub fn write(report: &Report, path: &Path) -> Result<(), ReportWriteError> {
    let text = toml::to_string_pretty(report)?;

    fs::write(path, text)?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::{AssertionValue, Condition};
    use crate::report::{AssertionReportEntry, AssertionStatus, ReportStatus, TerminatedBy};

    use std::sync::atomic::{AtomicU64, Ordering};

    // A representative finished report: a failing run that stopped on the time
    // budget, with one passing and one failing assertion line. Exercises every
    // serialized shape the writer touches — scalars, the `terminated_by` table,
    // and the `assertions` array-of-tables.
    fn sample_report() -> Report {
        Report::new(
            "smoke".to_string(),
            "sim.scenarios.parking_lot".to_string(),
            Some(42),
            ReportStatus::Failed,
            Vec::new(),
            12.5,
            0.3,
            TerminatedBy::TimeBudget,
            vec![
                AssertionReportEntry::new(
                    "agent.car.estimator.position".to_string(),
                    Condition::Equals,
                    AssertionValue::Float(0.0),
                    Some(AssertionValue::Float(0.0)),
                    AssertionStatus::Passed,
                ),
                AssertionReportEntry::new(
                    "agent.car.speed".to_string(),
                    Condition::LessThan,
                    AssertionValue::Float(5.0),
                    Some(AssertionValue::Float(7.0)),
                    AssertionStatus::Failed,
                ),
            ],
        )
    }

    // Write `report` to a unique temp file, returning the path. The atomic counter
    // keeps parallel test threads from colliding on a filename — same std-only
    // pattern as `run::mod`'s tests, to avoid a temp-file dev-dependency.
    fn write_to_temp(report: &Report) -> std::path::PathBuf {
        static SEQ: AtomicU64 = AtomicU64::new(0);
        let n = SEQ.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!("helios_report_test_{n}.toml"));
        write(report, &path).unwrap();
        path
    }

    #[test]
    fn round_trips_through_toml() {
        let report = sample_report();
        let path = write_to_temp(&report);

        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);

        // Re-parse the file as a generic TOML document and spot-check that the
        // scalars, the nested `terminated_by` table, and the assertion array all
        // landed in their expected shapes.
        let value: toml::Value = toml::from_str(&text).unwrap();
        assert_eq!(value["run_name"].as_str(), Some("smoke"));
        assert_eq!(value["status"].as_str(), Some("failed"));
        assert_eq!(value["master_seed"].as_integer(), Some(42));
        assert_eq!(value["terminated_by"]["kind"].as_str(), Some("time_budget"));

        let assertions = value["assertions"].as_array().unwrap();
        assert_eq!(assertions.len(), 2);
        assert_eq!(assertions[0]["status"].as_str(), Some("passed"));
        assert_eq!(assertions[1]["status"].as_str(), Some("failed"));
    }

    #[test]
    fn missing_parent_dir_is_io_error() {
        let report = sample_report();
        let result = write(&report, Path::new("/no/such/dir/report.toml"));
        assert!(matches!(result, Err(ReportWriteError::Io(_))));
    }
}
