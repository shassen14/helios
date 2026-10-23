pub mod observation_ref;
pub mod scenario_ref;
pub mod termination;
pub mod validation;

use std::path::Path;

use observation_ref::ObservationRef;
use scenario_ref::ScenarioRef;
use termination::Termination;

use crate::{
    assertion::{Assertion, EntryName, Measure},
    evaluation::{
        suite::{Suite, SuiteApplication},
        Evaluation, EvaluationDefaults,
    },
};

use serde::Deserialize;
use thiserror::Error;

/// The one run-file version this harness reads.
const CURRENT_VERSION: &str = "2.0";

/// The key every run file carries its version under.
const VERSION_KEY: &str = "schema_version";

/// _Pre-step._ A run: scenario + observation + assertions + measures +
/// evaluations + suites + termination, as [`load`] read it, with each suite
/// file its `[[suite]]` entries name.
#[derive(Debug, Clone)]
pub struct Run {
    schema_version: SchemaVersion,
    scenario: ScenarioRef,
    observation: ObservationRef,
    assertions: Vec<Assertion>,
    measures: Vec<Measure>,
    evaluations: Vec<Evaluation>,
    evaluation_defaults: Option<EvaluationDefaults>,
    suites: Vec<Suite>,
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

    pub fn evaluation_defaults(&self) -> Option<&EvaluationDefaults> {
        self.evaluation_defaults.as_ref()
    }

    /// Each `[[suite]]`, in run-file order, with its suite file.
    pub fn suites(&self) -> &[Suite] {
        &self.suites
    }

    pub fn termination(&self) -> &Termination {
        &self.termination
    }
}

/// The run-file vocabulary: the single top-level deserialize target, with
/// each `[[suite]]` naming its suite file, not yet read.
///
/// `deny_unknown_fields` rejects stray or misspelled top-level keys.
/// `assert`, `measure`, `evaluate` and `suite` are the author-facing keys for
/// the four lists (`[[assert]]`, `[[measure]]`, `[[evaluate]]`,
/// `[[suite]]`); `default` lets a run declare none — a pure smoke run that
/// only exercises the scenario is valid.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RunFile {
    schema_version: SchemaVersion,
    scenario: ScenarioRef,
    observation: ObservationRef,
    #[serde(default, rename = "assert")]
    assertions: Vec<Assertion>,
    #[serde(default, rename = "measure")]
    measures: Vec<Measure>,
    #[serde(default, rename = "evaluate")]
    evaluations: Vec<Evaluation>,
    /// `[evaluation]`, what every evaluation shares unless it gives its own.
    #[serde(default, rename = "evaluation")]
    evaluation_defaults: Option<EvaluationDefaults>,
    #[serde(default, rename = "suite")]
    suites: Vec<SuiteApplication>,
    termination: Termination,
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
/// *incoherent*: `Io`, `UnsupportedVersion`, `Parse` and the two suite
/// variants come from [`load`];
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
    #[error("suite '{name}': failed to read its suite file '{from}': {source}")]
    SuiteIo {
        name: EntryName,
        from: String,
        source: std::io::Error,
    },
    #[error("suite '{name}': failed to parse its suite file '{from}': {source}")]
    SuiteParse {
        name: EntryName,
        from: String,
        source: Box<toml::de::Error>,
    },
    #[error("run file failed validation")]
    Validation,
}

/// Read and parse the run file at `path`, and each suite file it names,
/// resolved against `config_root` as the scenario's `from` is.
/// Parse-only: this answers "can I read this?", not "is this a sane run?" —
/// cross-field coherence is `validate`'s job, which keeps the two concerns
/// and their error variants separate.
pub fn load(path: &Path, config_root: &Path) -> Result<Run, RunLoadError> {
    let text = std::fs::read_to_string(path)?;
    parse(&text, |from| {
        std::fs::read_to_string(config_root.join(from))
    })
}

/// Parse the run file `text`, reading each suite file it names through
/// `read_suite`, given the `from` its `[[suite]]` writes.
///
/// The version is read first, from the file as a plain TOML table: parsed
/// straight into the run file's type, an older file fails on its first
/// changed key, and the author sees that serde error instead of what
/// changed. A file with no string version falls through to the typed parse,
/// which names the problem.
pub fn parse(
    text: &str,
    read_suite: impl Fn(&str) -> std::io::Result<String>,
) -> Result<Run, RunLoadError> {
    let table = toml::from_str::<toml::Table>(text)?;
    if let Some(version) = table.get(VERSION_KEY).and_then(toml::Value::as_str) {
        SchemaVersion::try_from(version.to_string())?;
    }
    let file = toml::from_str::<RunFile>(text)?;
    let suites = file
        .suites
        .into_iter()
        .map(|application| {
            let text = read_suite(&application.from).map_err(|source| RunLoadError::SuiteIo {
                name: application.name.clone(),
                from: application.from.clone(),
                source,
            })?;
            let config = toml::from_str(&text).map_err(|source| RunLoadError::SuiteParse {
                name: application.name.clone(),
                from: application.from.clone(),
                source: Box::new(source),
            })?;
            Ok(Suite {
                application,
                config,
            })
        })
        .collect::<Result<_, RunLoadError>>()?;
    Ok(Run {
        schema_version: file.schema_version,
        scenario: file.scenario,
        observation: file.observation,
        assertions: file.assertions,
        measures: file.measures,
        evaluations: file.evaluations,
        evaluation_defaults: file.evaluation_defaults,
        suites,
        termination: file.termination,
    })
}

/// A suite reader for a run that names no suite file: each is refused, so a
/// test run naming one fails to parse.
#[cfg(test)]
pub(crate) fn no_suite_files(from: &str) -> std::io::Result<String> {
    Err(std::io::Error::new(
        std::io::ErrorKind::NotFound,
        format!("no suite file '{from}' in this test"),
    ))
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
    // only `std` to avoid a dev-dependency on a temp-file crate. The process
    // id and the atomic counter together keep parallel tests from colliding
    // on a filename, whether they run as threads of one process or as one
    // process each (nextest).
    fn load_str(contents: &str) -> Result<Run, RunLoadError> {
        static SEQ: AtomicU64 = AtomicU64::new(0);
        let n = SEQ.fetch_add(1, Ordering::Relaxed);
        let path =
            std::env::temp_dir().join(format!("helios_run_test_{}_{n}.toml", std::process::id()));
        std::fs::write(&path, contents).unwrap();
        let result = load(&path, Path::new(""));
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
        assert!(parse(&text, no_suite_files).is_err());
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
        let run = load(Path::new("/no/such/run/file.toml"), Path::new(""));
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

    const APPLYING: &str = r#"
        schema_version = "2.0"
        scenario = { from = "s" }
        observation = { from = "o" }
        termination = { max_simulated_seconds = 5.0 }

        [[suite]]
        name = "tracking"
        from = "test/evaluations/twist.toml"
        agents = ["car"]
    "#;

    const TWIST_SUITE: &str = r#"
        [[evaluate]]
        name = "twist"
        kind = "twist_tracking"
        [evaluate.inputs]
        truth = "truth.twist"
        reference = "reference"
    "#;

    #[test]
    fn a_suite_is_read_from_the_file_its_application_names() {
        let run = parse(APPLYING, |from| {
            assert_eq!(from, "test/evaluations/twist.toml");
            Ok(TWIST_SUITE.to_owned())
        })
        .expect("the run and its suite parse");

        let [suite] = run.suites() else {
            panic!("one suite");
        };
        assert_eq!(suite.application.name.as_str(), "tracking");
        assert_eq!(suite.config.evaluations.len(), 1);
    }

    #[test]
    fn a_suite_file_resolves_against_the_config_root() {
        let root = std::env::temp_dir().join(format!("helios_suite_root_{}", std::process::id()));
        let suite = root.join("test/evaluations/twist.toml");
        std::fs::create_dir_all(suite.parent().unwrap()).unwrap();
        std::fs::write(&suite, TWIST_SUITE).unwrap();
        let run_path = root.join("run.toml");
        std::fs::write(&run_path, APPLYING).unwrap();

        let run = load(&run_path, &root);
        let _ = std::fs::remove_dir_all(&root);

        assert_eq!(run.expect("both files read").suites().len(), 1);
    }

    #[test]
    fn a_suite_file_unread_or_malformed_is_refused_naming_its_suite() {
        let missing = parse(APPLYING, no_suite_files).expect_err("no suite file");
        assert!(
            matches!(missing, RunLoadError::SuiteIo { .. }),
            "{missing:?}"
        );
        assert!(
            missing
                .to_string()
                .starts_with("suite 'tracking': failed to read its suite file"),
            "{missing}"
        );

        let malformed =
            parse(APPLYING, |_| Ok("[[evaluate]]\nname = 3".to_owned())).expect_err("malformed");
        assert!(
            matches!(malformed, RunLoadError::SuiteParse { .. }),
            "{malformed:?}"
        );
    }
}
