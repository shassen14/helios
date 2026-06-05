pub mod console;
pub mod toml_writer;

use crate::{
    assertion::{AssertionResult, AssertionValue, Condition},
    run::termination::TerminationReason,
    runner::state::ContinuousStatus,
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
    pub fn new(
        run_name: String,
        scenario: String,
        master_seed: Option<u64>,
        status: ReportStatus,
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
    /// The simulated-time budget elapsed.
    TimeBudget,
    /// An assertion reached the configured `on_assertion` state; `outcome` is
    /// the triggering assertion's verdict.
    Assertion { outcome: AssertionStatus },
}

/// Project the per-step stop reason into the report's stable vocabulary.
/// `MaxSimulatedSeconds` becomes `TimeBudget`; an assertion-triggered stop
/// carries the triggering result reduced to a bare `AssertionStatus`, so the
/// live `AssertionResult` / `FailureKind` types never reach the serialized form.
impl From<&TerminationReason> for TerminatedBy {
    fn from(value: &TerminationReason) -> Self {
        match value {
            TerminationReason::MaxSimulatedSeconds => TerminatedBy::TimeBudget,
            TerminationReason::OnAssertion { result } => TerminatedBy::Assertion {
                outcome: AssertionStatus::from(result),
            },
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
    /// `None` when no value was observed (pending, or an unresolved / no-extractor failure).
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
    Pending,
}

/// A continuous assertion's latch reduced to a report verdict: `Holding` held
/// on every observed tick (`Passed`), `FailedAt` latched a violation
/// (`Failed`), `Pending` never observed a value. The latch's `t` / `reason`
/// detail is dropped — the entry schema doesn't carry it.
impl From<&ContinuousStatus> for AssertionStatus {
    fn from(value: &ContinuousStatus) -> Self {
        match value {
            ContinuousStatus::Holding => AssertionStatus::Passed,
            ContinuousStatus::FailedAt { .. } => AssertionStatus::Failed,
            ContinuousStatus::Pending => AssertionStatus::Pending,
        }
    }
}

/// A freshly evaluated result reduced to a report verdict. Used for terminal
/// assertions (judged once at finalize) and for the `on_assertion` outcome
/// inside [`TerminatedBy`]. The observed value is pulled out separately via
/// `AssertionResult::into_actual`; this only projects the verdict.
impl From<&AssertionResult> for AssertionStatus {
    fn from(value: &AssertionResult) -> Self {
        match value {
            AssertionResult::Passed { .. } => AssertionStatus::Passed,
            AssertionResult::Failed { .. } => AssertionStatus::Failed,
            AssertionResult::Pending => AssertionStatus::Pending,
        }
    }
}
