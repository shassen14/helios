pub mod aggregate;
pub mod console;
pub mod toml_writer;

use crate::{
    assertion::{AssertionResult, AssertionValue, Condition},
    run::termination::TerminationReason,
};

use serde::Serialize;

/// _Post-step._ The machine- and human-readable result of one run. Serialized to
/// TOML for CI and rendered to the console for humans — both from this one
/// struct, so the two surfaces can't disagree. Built entirely from report-owned
/// types (never the pre-step input vocab or the per-step evaluator results), so
/// this output schema stays stable while those internal types churn.
#[derive(Debug, Clone, Serialize)]
pub struct Report {
    run_name: String,
    scenario: String,
    master_seed: Option<u64>,
    status: ReportStatus,
    /// Why the run was unfit to test, if it was — an agent whose autonomy
    /// pipeline never assembled, an assertion naming a target no agent
    /// produces. Present only on a run that failed before it could tick, and
    /// omitted from the TOML entirely otherwise, so a healthy report keeps its
    /// current shape. Without it a CI reader sees `status = "failed"` with an
    /// empty `assertions` list and no way to tell why.
    ///
    /// Sits ahead of `terminated_by` deliberately: TOML puts every scalar and
    /// inline array before the first table, and the writer depends on this
    /// struct's field order matching that.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    setup_failures: Vec<String>,
    simulated_duration_secs: f64,
    wall_duration_secs: f64,
    terminated_by: TerminatedBy,
    assertions: Vec<AssertionReportEntry>,
}

impl Report {
    /// Assemble a finished report. Lives here, not as a struct literal in
    /// `finalize`, because these fields are private to the `report` module —
    /// keeping construction of the output schema in one place, next to the
    /// projections that feed it.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        run_name: String,
        scenario: String,
        master_seed: Option<u64>,
        status: ReportStatus,
        setup_failures: Vec<String>,
        simulated_duration_secs: f64,
        wall_duration_secs: f64,
        terminated_by: TerminatedBy,
        assertions: Vec<AssertionReportEntry>,
    ) -> Self {
        Self {
            run_name,
            scenario,
            master_seed,
            status,
            setup_failures,
            simulated_duration_secs,
            wall_duration_secs,
            terminated_by,
            assertions,
        }
    }

    /// Run-level verdict. Read by the console renderer and the TOML writer.
    pub fn status(&self) -> &ReportStatus {
        &self.status
    }

    /// The per-assertion lines, in run-file order.
    pub fn assertions(&self) -> &[AssertionReportEntry] {
        &self.assertions
    }

    /// Why the run was unfit to test, if it was. Empty on a run that got far
    /// enough for its assertions to mean something.
    pub fn setup_failures(&self) -> &[String] {
        &self.setup_failures
    }

    /// Why the run stopped.
    pub fn terminated_by(&self) -> &TerminatedBy {
        &self.terminated_by
    }

    /// Elapsed simulated time, in seconds, measured from the run's clock origin.
    pub fn simulated_duration_secs(&self) -> f64 {
        self.simulated_duration_secs
    }
}

/// _Post-step._ Run-level outcome. A `Report` only ever describes a run that
/// actually ran to a verdict; a run that never started (bad config, unresolved
/// target) is surfaced by the host as an error + non-zero exit, not a report.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ReportStatus {
    Passed,
    Failed,
}

/// _Post-step._ Why the run stopped — the report's own projection of the
/// per-step `TerminationReason`, kept separate so the live result types
/// (`AssertionResult`, `FailureKind`) never leak into the serialized schema.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "snake_case", tag = "kind")]
pub enum TerminatedBy {
    /// The failed setup abort.
    Aborted,
    /// The simulated-time budget elapsed.
    TimeBudget,
    /// A continuous assertion failed in a `fail_fast` run; `number` is its
    /// position in the run file, counting from 1.
    AssertionFailed { number: usize },
}

/// Project the per-step stop reason into the report's stable vocabulary.
/// `MaxSimulatedSeconds` becomes `TimeBudget`.
impl From<&TerminationReason> for TerminatedBy {
    fn from(value: &TerminationReason) -> Self {
        match value {
            TerminationReason::Aborted => TerminatedBy::Aborted,
            TerminationReason::MaxSimulatedSeconds => TerminatedBy::TimeBudget,
            TerminationReason::AssertionFailed { number } => {
                TerminatedBy::AssertionFailed { number: *number }
            }
        }
    }
}

/// _Post-step._ One assertion's line in the report. Its own type — rather than
/// serializing the per-step `AssertionResult` — so the output schema is
/// decoupled from the evaluator.
#[derive(Debug, Clone, Serialize)]
pub struct AssertionReportEntry {
    target: String,
    condition: Condition,
    expected: AssertionValue,
    /// `None` when no value was observed (a silent or unresolved assertion,
    /// or one whose values were turned away).
    actual: Option<AssertionValue>,
    status: AssertionStatus,
}

impl AssertionReportEntry {
    /// Build one report line. The caller supplies an already-projected
    /// `status` and `actual`; this type never sees the live evaluator types.
    pub fn new(
        target: String,
        condition: Condition,
        expected: AssertionValue,
        actual: Option<AssertionValue>,
        status: AssertionStatus,
    ) -> Self {
        Self {
            target,
            condition,
            expected,
            actual,
            status,
        }
    }

    /// This assertion's verdict.
    pub fn status(&self) -> &AssertionStatus {
        &self.status
    }

    /// The observed value, if one was captured.
    pub fn actual(&self) -> Option<&AssertionValue> {
        self.actual.as_ref()
    }
}

/// _Post-step._ Per-assertion verdict. Distinct from `ReportStatus`, which is
/// the run-level outcome rather than any single assertion's.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AssertionStatus {
    Passed,
    Failed,
    /// No verdict: the run stopped before the assertion's window ended (a
    /// `fail_fast` stop or an aborted setup) and it hadn't failed by then.
    /// Neither passes nor fails a run on its own; a run with one has already
    /// failed for another reason.
    Unjudged,
}

/// A freshly evaluated result reduced to a report verdict, for a terminal
/// assertion judged at finalize. The observed value is pulled out separately
/// via `AssertionResult::into_actual`; this only projects the verdict.
impl From<&AssertionResult> for AssertionStatus {
    fn from(value: &AssertionResult) -> Self {
        match value {
            AssertionResult::Passed { .. } => AssertionStatus::Passed,
            AssertionResult::Failed { .. } => AssertionStatus::Failed,
            AssertionResult::Pending => AssertionStatus::Unjudged,
        }
    }
}
