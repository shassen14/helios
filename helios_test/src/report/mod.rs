pub mod aggregate;
pub mod console;
pub mod toml_writer;

use crate::{
    assertion::{AssertionResult, AssertionValue, Condition, FailureKind},
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
    /// The writes an agent's bus refused, one line per channel: a host or
    /// node wrote a channel no writer declared, a value of another type, or
    /// a frame or instance its writers never stated. Any one fails the run,
    /// since values the wiring dropped leave the assertions judging a run
    /// other than the one configured. Omitted from the TOML when empty, and
    /// ahead of `terminated_by` for the same reason as `setup_failures`.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    refused_writes: Vec<String>,
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
        refused_writes: Vec<String>,
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
            refused_writes,
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

    /// The writes an agent's bus refused, one line per channel. Empty on a
    /// run whose wiring dropped nothing.
    pub fn refused_writes(&self) -> &[String] {
        &self.refused_writes
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

/// _Post-step._ One assertion's line in the report: what was asserted and
/// its verdict. Its own type — rather than serializing the per-step
/// `AssertionResult` — so the output schema is decoupled from the evaluator.
#[derive(Debug, Clone, Serialize)]
pub struct AssertionReportEntry {
    target: String,
    condition: Condition,
    expected: AssertionValue,
    /// Written inline: `status` and the verdict's own fields sit beside
    /// `target` in the entry's table.
    #[serde(flatten)]
    verdict: Verdict,
}

impl AssertionReportEntry {
    /// Build one report line. The caller supplies an already-projected
    /// verdict; this type never sees the live evaluator types.
    pub fn new(
        target: String,
        condition: Condition,
        expected: AssertionValue,
        verdict: Verdict,
    ) -> Self {
        Self {
            target,
            condition,
            expected,
            verdict,
        }
    }

    /// This assertion's verdict, with what it observed.
    pub fn verdict(&self) -> &Verdict {
        &self.verdict
    }

    /// Which verdict, without its fields.
    pub fn status(&self) -> AssertionStatus {
        self.verdict.status()
    }
}

/// _Post-step._ One assertion's verdict and what it rests on. The variant
/// decides which fields exist, so a pass can't carry a failure reason and a
/// failure can't lack one.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum Verdict {
    Passed {
        actual: AssertionValue,
    },
    Failed {
        /// The value that failed it: for a continuous assertion the one that
        /// broke it, not the last one seen. `None` when no value was observed
        /// (none arrived, or the ones that did were turned away).
        #[serde(skip_serializing_if = "Option::is_none")]
        actual: Option<AssertionValue>,
        reason: FailureReason,
        /// When a continuous assertion broke: simulated seconds from the
        /// run's first tick to the reading that broke it. `None` for a
        /// terminal assertion and a failure with no reading to place.
        #[serde(skip_serializing_if = "Option::is_none")]
        failed_at_secs: Option<f64>,
    },
    /// No verdict: the run stopped before the assertion's window ended (a
    /// `fail_fast` stop or an aborted setup) and it hadn't failed by then.
    /// Neither passes nor fails a run on its own; a run with one has already
    /// failed for another reason. `actual` is the value it stopped at.
    Unjudged {
        #[serde(skip_serializing_if = "Option::is_none")]
        actual: Option<AssertionValue>,
    },
}

impl Verdict {
    /// Which verdict, without its fields.
    pub fn status(&self) -> AssertionStatus {
        match self {
            Verdict::Passed { .. } => AssertionStatus::Passed,
            Verdict::Failed { .. } => AssertionStatus::Failed,
            Verdict::Unjudged { .. } => AssertionStatus::Unjudged,
        }
    }

    /// The observed value shown with the verdict, if any.
    pub fn actual(&self) -> Option<&AssertionValue> {
        match self {
            Verdict::Passed { actual } => Some(actual),
            Verdict::Failed { actual, .. } | Verdict::Unjudged { actual } => actual.as_ref(),
        }
    }
}

/// A freshly evaluated result as a report verdict, for a terminal assertion
/// judged at finalize. A terminal failure has no time of breaking: it is
/// judged once, at the end.
impl From<AssertionResult> for Verdict {
    fn from(value: AssertionResult) -> Self {
        match value {
            AssertionResult::Passed { actual } => Verdict::Passed { actual },
            AssertionResult::Failed { kind, actual } => Verdict::Failed {
                actual,
                reason: FailureReason::from(&kind),
                failed_at_secs: None,
            },
            AssertionResult::Pending => Verdict::Unjudged { actual: None },
        }
    }
}

/// _Post-step._ Which verdict an assertion reached, without what it rests
/// on. Distinct from `ReportStatus`, which is the run-level outcome rather
/// than any single assertion's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AssertionStatus {
    Passed,
    Failed,
    Unjudged,
}

/// _Post-step._ Why an assertion failed — the report's own projection of the
/// per-step `FailureKind`, so rewording an error message can't change the
/// schema a CI reader matches on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FailureReason {
    /// A value arrived for the target but couldn't be read.
    Rejected,
    /// No value arrived for the target in the whole run.
    NoSamples,
    /// The observed value and the expected one are types the condition
    /// can't compare.
    TypeMismatch,
    /// The comparison ran and didn't hold.
    ConditionFailed,
    /// A `within_range` assertion lacked usable bounds.
    InvalidBounds,
}

impl From<&FailureKind> for FailureReason {
    fn from(value: &FailureKind) -> Self {
        match value {
            FailureKind::Rejected(_) => FailureReason::Rejected,
            FailureKind::NoSamples => FailureReason::NoSamples,
            FailureKind::TypeMismatch => FailureReason::TypeMismatch,
            FailureKind::ConditionFailed => FailureReason::ConditionFailed,
            FailureKind::InvalidBounds => FailureReason::InvalidBounds,
        }
    }
}
