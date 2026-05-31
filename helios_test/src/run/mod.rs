pub mod observation_ref;
pub mod scenario_ref;
pub mod termination;
pub mod validation;

use std::path::Path;

use observation_ref::ObservationRef;
use scenario_ref::ScenarioRef;
use termination::Termination;

use crate::assertion::Assertion;

use serde::Deserialize;
use thiserror::Error;

/// The run-file vocabulary: the single top-level deserialize target that ties
/// scenario + observation + assertions + termination into one file.
///
/// `schema_version` is carried so the format can evolve with explicit migration
/// instead of silent breakage. `deny_unknown_fields` rejects stray or misspelled
/// top-level keys. `assert` is the author-facing key for the assertion list
/// (`[[assert]]`); `default` lets a run declare zero assertions — a pure smoke
/// run that only exercises the scenario is valid.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Run {
    schema_version: String,
    scenario: ScenarioRef,
    observation: ObservationRef,
    #[serde(default, rename = "assert")]
    assertions: Vec<Assertion>,
    termination: Termination,
}

impl Run {
    /// Declared format version. Compatibility is checked during validation,
    /// not at parse time.
    pub fn schema_version(&self) -> &str {
        &self.schema_version
    }

    pub fn scenario(&self) -> &ScenarioRef {
        &self.scenario
    }

    pub fn observation(&self) -> &ObservationRef {
        &self.observation
    }

    pub fn assertions(&self) -> &[Assertion] {
        &self.assertions
    }

    pub fn termination(&self) -> &Termination {
        &self.termination
    }
}

/// Why a run file couldn't be loaded. The three variants stay distinct so a
/// caller can tell *unreadable* from *malformed* from *incoherent*: `Io` and
/// `Parse` come from [`load`]; `Validation` is produced by the separate
/// `validate` pass. `Io`/`Parse` carry their source so the report can show the
/// OS or parser detail rather than a bare "load failed".
///
/// Not `Clone`: `std::io::Error` and `toml::de::Error` aren't `Clone`, and a
/// load error is consumed once at the call site, so cloning isn't needed.
#[derive(Debug, Error)]
pub enum RunLoadError {
    #[error("failed to read run file: {0}")]
    Io(#[from] std::io::Error),
    #[error("failed to parse run file: {0}")]
    Parse(#[from] toml::de::Error),
    #[error("run file failed validation")]
    Validation,
}

/// Read and parse a run file. Parse-only: this answers "can I read this?", not
/// "is this a sane run?" — cross-field coherence is `validate`'s job (step 12),
/// which keeps the two concerns and their error variants separate.
///
/// The `#[from]` attributes on `RunLoadError::Io`/`Parse` generate the `From`
/// impls that let `?` convert each underlying error into the right variant.
pub fn load(path: &Path) -> Result<Run, RunLoadError> {
    let text = std::fs::read_to_string(path)?;
    let run = toml::from_str::<Run>(&text)?;
    Ok(run)
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::sync::atomic::{AtomicU64, Ordering};

    // A complete, valid run file: schema version, both references, one terminal
    // assertion, and a termination budget.
    const VALID: &str = r#"
        schema_version = "1.0"
        scenario = { from = "sim.scenarios.parking_lot" }
        observation = { from = "observation.presets.verbose" }
        termination = { max_simulated_seconds = 30.0 }

        [[assert]]
        when = "at_completion"
        target = "agent.car.estimator.position"
        condition = "equals"
        value = 0.0
    "#;

    // Write `contents` to a unique file under the temp dir, load it through the
    // real `load` path (exercising read + parse together), then clean up. Uses
    // only `std` to avoid a dev-dependency on a temp-file crate; the atomic
    // counter keeps parallel test threads from colliding on a filename.
    fn load_str(contents: &str) -> Result<Run, RunLoadError> {
        static SEQ: AtomicU64 = AtomicU64::new(0);
        let n = SEQ.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!("helios_run_test_{n}.toml"));
        std::fs::write(&path, contents).unwrap();
        let result = load(&path);
        let _ = std::fs::remove_file(&path);
        result
    }

    #[test]
    fn loads_complete_run() {
        let run = load_str(VALID).unwrap();
        assert_eq!(run.schema_version(), "1.0");
        assert_eq!(run.scenario().path(), "sim.scenarios.parking_lot");
        assert_eq!(run.observation().path(), "observation.presets.verbose");
        assert_eq!(run.assertions().len(), 1);
        assert_eq!(run.termination().max_simulated_seconds(), Some(30.0));
    }

    #[test]
    fn assertions_default_to_empty() {
        // `[[assert]]` omitted entirely — a pure smoke run is valid.
        let toml = r#"
            schema_version = "1.0"
            scenario = { from = "s" }
            observation = { from = "o" }
            termination = { max_simulated_seconds = 5.0 }
        "#;
        let run = load_str(toml).unwrap();
        assert!(run.assertions().is_empty());
    }

    #[test]
    fn malformed_toml_is_parse_error() {
        assert!(matches!(
            load_str("schema_version = "),
            Err(RunLoadError::Parse(_))
        ));
    }

    #[test]
    fn missing_file_is_io_error() {
        let run = load(Path::new("/no/such/run/file.toml"));
        assert!(matches!(run, Err(RunLoadError::Io(_))));
    }

    #[test]
    fn rejects_unknown_top_level_key() {
        let toml = r#"
            schema_version = "1.0"
            scenario = { from = "s" }
            observation = { from = "o" }
            termination = { max_simulated_seconds = 5.0 }
            extra_key = true
        "#;
        assert!(matches!(load_str(toml), Err(RunLoadError::Parse(_))));
    }
}
