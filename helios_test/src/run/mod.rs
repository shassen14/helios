pub mod observation_ref;
pub mod scenario_ref;
pub mod termination;
pub mod validation;

use std::path::Path;

use observation_ref::ObservationRef;
use scenario_ref::ScenarioRef;
use termination::Termination;

use crate::{
    assertion::{Assertion, Measure},
    evaluation::Evaluation,
};

use serde::Deserialize;
use thiserror::Error;

/// The one run-file version this harness reads.
const CURRENT_VERSION: &str = "2.0";

/// The key every run file carries its version under.
const VERSION_KEY: &str = "schema_version";

/// _Pre-step._ The run-file vocabulary: the single top-level deserialize target
/// that ties scenario + observation + assertions + measures + evaluations +
/// termination into one file.
///
/// `deny_unknown_fields` rejects stray or misspelled top-level keys.
/// `assert`, `measure` and `evaluate` are the author-facing keys for the
/// three lists (`[[assert]]`, `[[measure]]`, `[[evaluate]]`); `default` lets
/// a run declare none — a pure smoke run that only exercises the scenario is
/// valid.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Run {
    schema_version: SchemaVersion,
    scenario: ScenarioRef,
    observation: ObservationRef,
    #[serde(default, rename = "assert")]
    assertions: Vec<Assertion>,
    #[serde(default, rename = "measure")]
    measures: Vec<Measure>,
    #[serde(default, rename = "evaluate")]
    evaluations: Vec<Evaluation>,
    termination: Termination,
}

impl Run {
    /// Declared format version: always the one this harness reads, since no
    /// other parses.
    pub fn schema_version(&self) -> &'static str {
        self.schema_version.as_str()
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

    pub fn measures(&self) -> &[Measure] {
        &self.measures
    }

    pub fn evaluations(&self) -> &[Evaluation] {
        &self.evaluations
    }

    pub fn termination(&self) -> &Termination {
        &self.termination
    }
}

/// _Pre-step._ A run file's format version. Only the current one
/// deserializes, so no `Run` of another version exists, however it was
/// parsed. There is no migration code: an older file is refused with what
/// changed ([`UnsupportedVersion`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(try_from = "String")]
pub struct SchemaVersion;

impl SchemaVersion {
    pub fn as_str(&self) -> &'static str {
        CURRENT_VERSION
    }
}

impl TryFrom<String> for SchemaVersion {
    type Error = UnsupportedVersion;

    fn try_from(version: String) -> Result<Self, Self::Error> {
        if version == CURRENT_VERSION {
            Ok(SchemaVersion)
        } else {
            Err(UnsupportedVersion { found: version })
        }
    }
}

/// _Pre-step._ A run file of a version this harness doesn't read, with what
/// changed since 1.0 so the author can migrate it.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
#[error(
    "run file schema_version \"{found}\" is not supported; this harness reads \"{}\". \
     Since 1.0: every [[assert]] needs a `name`; `when` is gone (no `reduce` judges every \
     sample, `when = \"at_completion\"` is `reduce = \"final\"`); and the condition carries \
     its operand (`condition = {{ less_than = 25.0 }}`, `{{ within_range = [low, high] }}`) \
     in place of `value`, `low` and `high`",
    CURRENT_VERSION
)]
pub struct UnsupportedVersion {
    found: String,
}

/// _Pre-step._ Why a run file couldn't be loaded. The variants stay distinct so a
/// caller can tell *unreadable* from *outdated* from *malformed* from
/// *incoherent*: `Io`, `UnsupportedVersion` and `Parse` come from [`load`];
/// `Validation` is produced by the separate `validate` pass. Each carries its
/// source so the report can show the detail rather than a bare "load failed".
///
/// Not `Clone`: `std::io::Error` and `toml::de::Error` aren't `Clone`, and a
/// load error is consumed once at the call site, so cloning isn't needed.
#[derive(Debug, Error)]
pub enum RunLoadError {
    #[error("failed to read run file: {0}")]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    UnsupportedVersion(#[from] UnsupportedVersion),
    #[error("failed to parse run file: {0}")]
    Parse(#[from] toml::de::Error),
    #[error("run file failed validation")]
    Validation,
}

/// Read and parse a run file. Parse-only: this answers "can I read this?", not
/// "is this a sane run?" — cross-field coherence is `validate`'s job, which
/// keeps the two concerns and their error variants separate.
///
/// The version is read first, from the file as a plain TOML table: parsed
/// straight into [`Run`], an older file fails on its first changed key, and
/// the author sees that serde error instead of what changed. A file with no
/// string version falls through to the typed parse, which names the problem.
pub fn load(path: &Path) -> Result<Run, RunLoadError> {
    let text = std::fs::read_to_string(path)?;
    let table = toml::from_str::<toml::Table>(&text)?;
    if let Some(version) = table.get(VERSION_KEY).and_then(toml::Value::as_str) {
        SchemaVersion::try_from(version.to_string())?;
    }
    let run = toml::from_str::<Run>(&text)?;
    Ok(run)
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::sync::atomic::{AtomicU64, Ordering};

    // A complete, valid run file: schema version, both references, one
    // assertion, one measure, and a termination budget.
    const VALID: &str = r#"
        schema_version = "2.0"
        scenario = { from = "sim.scenarios.parking_lot" }
        observation = { from = "observation.presets.verbose" }
        termination = { max_simulated_seconds = 30.0 }

        [[assert]]
        name = "parked"
        target = "agent.car.node.estimator.position"
        reduce = "final"
        condition = { within_range = [-0.5, 0.5] }

        [[measure]]
        name = "final_position"
        target = "agent.car.node.estimator.position"
        reduce = "final"
    "#;

    // The same run in the 1.0 format.
    const OLD: &str = r#"
        schema_version = "1.0"
        scenario = { from = "sim.scenarios.parking_lot" }
        observation = { from = "observation.presets.verbose" }
        termination = { max_simulated_seconds = 30.0 }

        [[assert]]
        when = "at_completion"
        target = "agent.car.node.estimator.position"
        condition = "within_range"
        value = 0.0
        low = -0.5
        high = 0.5
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
        assert_eq!(run.schema_version(), "2.0");
        assert_eq!(run.scenario().path(), "sim.scenarios.parking_lot");
        assert_eq!(run.observation().path(), "observation.presets.verbose");
        assert_eq!(run.assertions().len(), 1);
        assert_eq!(run.measures().len(), 1);
        assert_eq!(run.termination().max_simulated_seconds(), 30.0);
    }

    #[test]
    fn assertions_default_to_empty() {
        // `[[assert]]` and `[[measure]]` omitted entirely — a pure smoke run
        // is valid.
        let toml = r#"
            schema_version = "2.0"
            scenario = { from = "s" }
            observation = { from = "o" }
            termination = { max_simulated_seconds = 5.0 }
        "#;
        let run = load_str(toml).unwrap();
        assert!(run.assertions().is_empty());
        assert!(run.measures().is_empty());
    }

    #[test]
    fn a_1_0_file_is_refused_naming_what_changed() {
        let error = load_str(OLD).expect_err("1.0 is not read");
        assert!(
            matches!(error, RunLoadError::UnsupportedVersion(_)),
            "{error:?}"
        );
        let message = error.to_string();
        for change in ["\"1.0\"", "name", "reduce = \"final\"", "less_than"] {
            assert!(message.contains(change), "{change}: {message}");
        }
    }

    #[test]
    fn no_run_of_another_version_parses_even_bypassing_load() {
        let text = VALID.replace("\"2.0\"", "\"1.0\"");
        assert!(toml::from_str::<Run>(&text).is_err());
    }

    #[test]
    fn a_missing_version_is_a_parse_error() {
        let text = VALID.replace("schema_version = \"2.0\"", "");
        assert!(matches!(load_str(&text), Err(RunLoadError::Parse(_))));
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
            schema_version = "2.0"
            scenario = { from = "s" }
            observation = { from = "o" }
            termination = { max_simulated_seconds = 5.0 }
            extra_key = true
        "#;
        assert!(matches!(load_str(toml), Err(RunLoadError::Parse(_))));
    }
}
