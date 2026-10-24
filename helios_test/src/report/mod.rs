pub mod aggregate;
pub mod console;
pub mod evaluation;
mod phrase;
pub mod run;
pub mod toml_writer;
pub mod unit;

pub use evaluation::{
    BoundEvaluation, DerivedEvaluation, EvaluationOutcome, EvaluationReport, InputReport,
    MetricReport,
};
pub use run::{DroppedValues, RunEnd, RunEntries, RunIdentity, RunStop, UnbuiltAgent};
pub use unit::ReportedUnit;

use crate::{
    assertion::{AssertionResult, AssertionTarget, AssertionValue, Condition, FailureKind, Window},
    collector::instance_of,
    run::termination::TerminationReason,
    statistics::within_run::Reducer,
};

use serde::Serialize;

/// The report format's version. A report with none is the shape before
/// entries were named, so this starts at the second. The third moved why a
/// run aborted into `terminated_by` and added the `unusable` and `aborted`
/// statuses.
const REPORT_SCHEMA_VERSION: &str = "3.0";

/// _Post-step._ The machine- and human-readable result of one run. Serialized to
/// TOML for CI and rendered to the console for humans — both from this one
/// struct, so the two surfaces can't disagree. Built entirely from report-owned
/// types (never the pre-step input vocab or the per-step evaluator results), so
/// this output schema stays stable while those internal types churn.
#[derive(Debug, Clone, Serialize)]
pub struct Report {
    /// So a reader (CI, a run comparison, the Monte Carlo aggregate) can
    /// tell this shape from another rather than misread it.
    schema_version: &'static str,
    run_name: String,
    scenario: String,
    master_seed: Option<u64>,
    status: ReportStatus,
    /// The writes an agent's bus refused, one line per channel: a host or
    /// node wrote a channel no writer declared, a value of another type, or
    /// a frame or instance its writers never stated. Any one fails the run,
    /// since values the wiring dropped leave the assertions judging a run
    /// other than the one configured. Omitted from the TOML when empty.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    refused_writes: Vec<String>,
    /// What the recording dropped, one line each: values that arrived under
    /// a requested path but a key no series was bound to, and values a
    /// measure's series turned away. A judged series may have missed the
    /// first, and a measure aggregated across runs would average around the
    /// second. Any one fails the run. Omitted from the TOML when empty.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    recording: Vec<String>,
    simulated_duration_secs: f64,
    wall_duration_secs: f64,
    terminated_by: TerminatedBy,
    assertions: Vec<AssertionReportEntry>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    measures: Vec<MeasureReportEntry>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    evaluations: Vec<EvaluationReport>,
}

impl Report {
    /// Assemble a finished report. Lives here, not as a struct literal in
    /// `finalize`, because these fields are private to the `report` module —
    /// keeping construction of the output schema in one place, next to the
    /// projections that feed it.
    ///
    /// The run's status is derived here, not passed in, so it can't disagree
    /// with the entries: `Aborted` when the run stopped untested, else
    /// `Passed` only when every assertion passed and nothing was refused by a
    /// bus or dropped by the recording. A refused write or a dropped value
    /// outranks the assertion tally: a run whose wiring or recording dropped
    /// values cannot pass however its assertions came out. Measures never
    /// decide it.
    ///
    /// So are the abort's causes, in their fixed order: each pipeline not
    /// built, then [`AbortCause::UnusableEntries`] when an entry or
    /// evaluation the run refused is among `entries`, then each host fault.
    /// No caller states the refused entries' cause, so it can't disagree with
    /// the entries that carry the detail.
    pub fn new(
        identity: RunIdentity,
        end: RunEnd,
        dropped: DroppedValues,
        entries: RunEntries,
    ) -> Self {
        let RunIdentity {
            run_name,
            scenario,
            master_seed,
        } = identity;
        let RunEnd {
            simulated_duration_secs,
            wall_duration_secs,
            stop,
        } = end;
        let unusable = entries.any_unusable();
        let terminated_by = match stop {
            RunStop::Ended(reason) if !unusable => TerminatedBy::from(&reason),
            RunStop::Ended(_) => TerminatedBy::aborted(Vec::new(), true, Vec::new()),
            RunStop::Aborted { unbuilt, faults } => {
                // The runner aborts a run only for a pipeline not built, a
                // refused entry or a host fault.
                debug_assert!(
                    unusable || !unbuilt.is_empty() || !faults.is_empty(),
                    "an abort with no cause"
                );
                TerminatedBy::aborted(unbuilt, unusable, faults)
            }
        };
        let status = if matches!(terminated_by, TerminatedBy::Aborted { .. }) {
            ReportStatus::Aborted
        } else if entries.all_passed() && dropped.is_empty() {
            ReportStatus::Passed
        } else {
            ReportStatus::Failed
        };
        let DroppedValues {
            refused_writes,
            recording,
        } = dropped;
        let RunEntries {
            assertions,
            measures,
            evaluations,
        } = entries;
        Self {
            schema_version: REPORT_SCHEMA_VERSION,
            run_name,
            scenario,
            master_seed,
            status,
            refused_writes,
            recording,
            simulated_duration_secs,
            wall_duration_secs,
            terminated_by,
            assertions,
            measures,
            evaluations,
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

    /// The per-measure lines, in run-file order.
    pub fn measures(&self) -> &[MeasureReportEntry] {
        &self.measures
    }

    /// The per-evaluation lines, in run-file order.
    pub fn evaluations(&self) -> &[EvaluationReport] {
        &self.evaluations
    }

    /// The writes an agent's bus refused, one line per channel. Empty on a
    /// run whose wiring dropped nothing.
    pub fn refused_writes(&self) -> &[String] {
        &self.refused_writes
    }

    /// What the recording dropped, one line each. Empty on a run whose
    /// every requested value landed in a series.
    pub fn recording(&self) -> &[String] {
        &self.recording
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

/// _Post-step._ Run-level outcome. A run file refused before the run starts
/// (bad config) is surfaced by the host as an error + non-zero exit, not a
/// report.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ReportStatus {
    Passed,
    /// The run was tested and didn't pass: the robot, or its wiring or
    /// recording, failed.
    Failed,
    /// The run was never tested: `terminated_by` says why. Neither a pass nor
    /// a failure of the robot.
    Aborted,
}

/// _Post-step._ Why the run stopped — the report's own projection of the
/// per-step `TerminationReason`, kept separate so the live result types
/// (`AssertionResult`, `FailureKind`) never leak into the serialized schema.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "snake_case", tag = "kind")]
pub enum TerminatedBy {
    /// The runner stopped the run untested, for every one of `causes`.
    Aborted { causes: Vec<AbortCause> },
    /// The simulated-time budget elapsed.
    TimeBudget,
    /// An every-sample assertion failed in a `fail_fast` run; `name` is
    /// the assertion's, `path` and `instance` the series that failed it.
    AssertionFailed {
        name: String,
        path: String,
        #[serde(skip_serializing_if = "Option::is_none")]
        instance: Option<String>,
    },
}

impl TerminatedBy {
    /// An abort for each of `unbuilt`, for refused entries when `unusable`,
    /// and for each of `faults`, in that order.
    fn aborted(unbuilt: Vec<UnbuiltAgent>, unusable: bool, faults: Vec<HostFault>) -> Self {
        let causes = unbuilt
            .into_iter()
            .map(|UnbuiltAgent { agent, errors }| AbortCause::PipelineNotBuilt { agent, errors })
            .chain(unusable.then_some(AbortCause::UnusableEntries))
            .chain(faults.into_iter().map(AbortCause::HostFault))
            .collect();
        TerminatedBy::Aborted { causes }
    }
}

/// Project the per-step stop reason into the report's stable vocabulary.
/// `MaxSimulatedSeconds` becomes `TimeBudget`.
impl From<&TerminationReason> for TerminatedBy {
    fn from(value: &TerminationReason) -> Self {
        match value {
            TerminationReason::MaxSimulatedSeconds => TerminatedBy::TimeBudget,
            TerminationReason::AssertionFailed { name, key } => TerminatedBy::AssertionFailed {
                name: name.to_string(),
                path: key.path(),
                instance: instance_of(&key.subject).map(str::to_owned),
            },
        }
    }
}

/// _Post-step._ One reason a run was aborted untested.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "snake_case", tag = "kind")]
pub enum AbortCause {
    /// `agent`'s pipeline wasn't built, for `errors`, as the host gave them.
    PipelineNotBuilt { agent: String, errors: Vec<String> },
    /// Some entry or evaluation is `unusable`; each says why.
    UnusableEntries,
    /// The host broke its contract with the runner.
    HostFault(HostFault),
}

/// _Post-step._ How a host broke its contract with the runner.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "snake_case", tag = "fault")]
pub enum HostFault {
    /// It asked the collector what to record before setup bound anything.
    AskedBeforeSetup,
    /// It ticked or finalized the run without setting it up.
    NeverSetUp,
    /// It set the run up and finalized it without a tick.
    NeverTicked,
    /// It finalized, after ticking, a run the runner never told it to stop.
    StoppedUnasked,
    /// It handed the runner a collector the run wasn't set up against. Found
    /// at finalize, the stop reason the host gave is lost with it.
    ForeignCollector { at: FaultStep },
}

/// _Post-step._ Which of the runner's steps found a host fault.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "snake_case", tag = "step")]
pub enum FaultStep {
    /// A tick, `at_secs` simulated seconds from the run's first tick.
    Tick {
        at_secs: f64,
    },
    Finalize,
}

/// _Post-step._ A measure the run refused at setup: written as
/// `status = "unusable"` beside every reason why.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case", tag = "status")]
pub enum Refused {
    Unusable { problems: Vec<String> },
}

/// _Post-step._ One assertion's line in the report: what was asserted, the
/// series it was judged on, and its verdict. Its own type — rather than
/// serializing the per-step `AssertionResult` — so the output schema is
/// decoupled from the evaluator.
#[derive(Debug, Clone, Serialize)]
pub struct AssertionReportEntry {
    name: String,
    /// The selector as the run file wrote it.
    target: AssertionTarget,
    /// The agent whose evaluation this entry judged, where its target
    /// selects every agent's; one entry per agent under `eval.agent.*`.
    #[serde(skip_serializing_if = "Option::is_none")]
    agent: Option<String>,
    /// The instance this entry judged, on a keyed channel; one entry per
    /// instance under `instance = "*"`. So an entry is identified by its
    /// name, agent and instance.
    #[serde(skip_serializing_if = "Option::is_none")]
    instance: Option<String>,
    /// As the run file wrote it; absent on one over the whole run. Where it
    /// fell in this run is the series summary's `window_secs`.
    #[serde(skip_serializing_if = "Window::is_whole_run")]
    window: Window,
    /// Absent on an assertion that judged every sample.
    #[serde(skip_serializing_if = "Option::is_none")]
    reduce: Option<Reducer>,
    /// The unit the judged value and the condition's operand are read in.
    /// Absent on a bool or string, and on an entry the run never set up.
    #[serde(skip_serializing_if = "Option::is_none")]
    unit: Option<ReportedUnit>,
    /// With its operand, as the run file wrote it.
    condition: Condition,
    #[serde(flatten)]
    series: SeriesSummary,
    /// Written inline: `status` and the verdict's own fields sit beside
    /// `target` in the entry's table.
    #[serde(flatten)]
    verdict: Verdict,
}

impl AssertionReportEntry {
    /// Build one report line. The caller supplies an already-projected
    /// verdict; this type never sees the live evaluator types.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        name: String,
        target: AssertionTarget,
        agent: Option<String>,
        instance: Option<String>,
        window: Window,
        reduce: Option<Reducer>,
        unit: Option<ReportedUnit>,
        condition: Condition,
        series: SeriesSummary,
        verdict: Verdict,
    ) -> Self {
        Self {
            name,
            target,
            agent,
            instance,
            window,
            reduce,
            unit,
            condition,
            series,
            verdict,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    /// The agent judged, under `eval.agent.*`.
    pub fn agent(&self) -> Option<&str> {
        self.agent.as_deref()
    }

    /// The instance judged, on a keyed channel.
    pub fn instance(&self) -> Option<&str> {
        self.instance.as_deref()
    }

    /// The unit the judged value is read in.
    pub fn unit(&self) -> Option<ReportedUnit> {
        self.unit
    }

    /// What the judged series held.
    pub fn series(&self) -> &SeriesSummary {
        &self.series
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

/// _Post-step._ One measure's line in the report: a value reported without
/// a bound. Never fails the run.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MeasureReportEntry {
    name: String,
    /// As on [`AssertionReportEntry`].
    target: AssertionTarget,
    /// As on [`AssertionReportEntry`].
    #[serde(skip_serializing_if = "Option::is_none")]
    agent: Option<String>,
    /// As on [`AssertionReportEntry`].
    #[serde(skip_serializing_if = "Option::is_none")]
    instance: Option<String>,
    /// As on [`AssertionReportEntry`].
    #[serde(skip_serializing_if = "Window::is_whole_run")]
    window: Window,
    reduce: Reducer,
    /// Whether the run reached the end of the measure's window. A value from
    /// one it didn't is the value it stopped at.
    covered: bool,
    /// `None` when the reducer had none: no samples, a sample it couldn't
    /// read, a value turned away, or an anchor that never occurred, which
    /// `reason` says; or a window the run never placed, with no `reason`.
    #[serde(skip_serializing_if = "Option::is_none")]
    value: Option<AssertionValue>,
    /// As on [`AssertionReportEntry`]: the measured value's.
    #[serde(skip_serializing_if = "Option::is_none")]
    unit: Option<ReportedUnit>,
    #[serde(skip_serializing_if = "Option::is_none")]
    reason: Option<FailureReason>,
    #[serde(flatten)]
    series: SeriesSummary,
    /// Absent on a measure the run accepted.
    #[serde(flatten, skip_serializing_if = "Option::is_none")]
    refused: Option<Refused>,
}

impl MeasureReportEntry {
    /// Build one measure line. A `value` and a `reason` are exclusive: a
    /// measure has a value, a reason it has none, or, as `None`, neither,
    /// when the run never read it or never placed its window.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        name: String,
        target: AssertionTarget,
        agent: Option<String>,
        instance: Option<String>,
        window: Window,
        reduce: Reducer,
        covered: bool,
        value: Option<Result<AssertionValue, FailureReason>>,
        unit: Option<ReportedUnit>,
        series: SeriesSummary,
    ) -> Self {
        let (value, reason) = match value {
            Some(Ok(value)) => (Some(value), None),
            Some(Err(reason)) => (None, Some(reason)),
            None => (None, None),
        };
        Self {
            name,
            target,
            agent,
            instance,
            window,
            reduce,
            covered,
            value,
            unit,
            reason,
            series,
            refused: None,
        }
    }

    /// This line as one the run refused, for `problems`.
    pub fn refused(self, problems: Vec<String>) -> Self {
        Self {
            refused: Some(Refused::Unusable { problems }),
            ..self
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    /// Whether the run refused it.
    pub fn is_unusable(&self) -> bool {
        self.refused.is_some()
    }

    /// Why the run refused it; empty when it didn't.
    pub fn problems(&self) -> &[String] {
        match &self.refused {
            Some(Refused::Unusable { problems }) => problems,
            None => &[],
        }
    }

    pub fn covered(&self) -> bool {
        self.covered
    }

    /// As [`AssertionReportEntry::agent`].
    pub fn agent(&self) -> Option<&str> {
        self.agent.as_deref()
    }

    /// As [`AssertionReportEntry::instance`].
    pub fn instance(&self) -> Option<&str> {
        self.instance.as_deref()
    }

    /// The measured value, if the reducer had one.
    pub fn value(&self) -> Option<&AssertionValue> {
        self.value.as_ref()
    }

    /// As [`AssertionReportEntry::unit`].
    pub fn unit(&self) -> Option<ReportedUnit> {
        self.unit
    }

    /// Why there is no value, if there is none.
    pub fn reason(&self) -> Option<FailureReason> {
        self.reason
    }

    /// What the measured series held.
    pub fn series(&self) -> &SeriesSummary {
        &self.series
    }
}

/// _Post-step._ What a judged or measured series held over its window,
/// printed beside every verdict so a silent signal shows even where nothing
/// bounds it. Over the run's span instead when the window was never placed.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct SeriesSummary {
    /// How many samples the run reached in the window, a channel's
    /// carried-in value included.
    pub samples: usize,
    /// The longest span with no sample, the window's edges included. `None`
    /// when the run never read the series.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_gap_secs: Option<f64>,
    /// For `reduce = "final"`: how long before the window's end the judged
    /// sample was stamped. `final` passes on a signal that died long before
    /// the end, and this shows it.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub final_age_secs: Option<f64>,
    /// Where a bounded window fell in this run: its edges, in seconds from
    /// the run's first tick, the end never past the latest tick. `None` for
    /// a whole-run entry and a window never placed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub window_secs: Option<[f64; 2]>,
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
        /// The value that failed it: for an every-sample assertion the one
        /// that broke it, not the last one seen. `None` when no value was observed
        /// (none arrived, or the ones that did were turned away).
        #[serde(skip_serializing_if = "Option::is_none")]
        actual: Option<AssertionValue>,
        reason: FailureReason,
        /// When an every-sample assertion broke: simulated seconds from the
        /// run's first tick to the reading that broke it. `None` for a
        /// reduced assertion and a failure with no reading to place.
        #[serde(skip_serializing_if = "Option::is_none")]
        failed_at_secs: Option<f64>,
    },
    /// No verdict: the run's latest tick never reached the end of the
    /// assertion's window, or never placed it (a `run.end` bound after an
    /// early stop, an anchor that hadn't occurred yet), and it hadn't failed
    /// by then. Usually the run stopped early, by a `fail_fast` stop or an
    /// aborted run; a window anchored late enough to end past a run that
    /// ran out is the other case. Either way the run didn't test what was
    /// asked, so a run with one is never `Passed`. `actual` is the value it
    /// stopped at.
    Unjudged {
        #[serde(skip_serializing_if = "Option::is_none")]
        actual: Option<AssertionValue>,
    },
    /// The run refused it at setup, for `problems`, so it never ran.
    Unusable {
        problems: Vec<String>,
    },
}

impl Verdict {
    /// Which verdict, without its fields.
    pub fn status(&self) -> AssertionStatus {
        match self {
            Verdict::Passed { .. } => AssertionStatus::Passed,
            Verdict::Failed { .. } => AssertionStatus::Failed,
            Verdict::Unjudged { .. } => AssertionStatus::Unjudged,
            Verdict::Unusable { .. } => AssertionStatus::Unusable,
        }
    }

    /// The observed value shown with the verdict, if any.
    pub fn actual(&self) -> Option<&AssertionValue> {
        match self {
            Verdict::Passed { actual } => Some(actual),
            Verdict::Failed { actual, .. } | Verdict::Unjudged { actual } => actual.as_ref(),
            Verdict::Unusable { .. } => None,
        }
    }
}

/// A freshly evaluated result as a report verdict, for a reduced assertion
/// judged at finalize. A reduced failure has no time of breaking: it is
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
    Unusable,
}

/// _Post-step._ Why an assertion failed — the report's own projection of the
/// per-step `FailureKind`, so rewording an error message can't change the
/// schema a CI reader matches on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FailureReason {
    /// A value arrived for the target but couldn't be read.
    Rejected,
    /// No value arrived for the target in its window.
    NoSamples,
    /// The observed value and the expected one are types the condition
    /// can't compare.
    TypeMismatch,
    /// The comparison ran and didn't hold.
    ConditionFailed,
    /// The run ran out without the event the window is anchored `after`.
    AnchorNeverOccurred,
}

impl From<&FailureKind> for FailureReason {
    fn from(value: &FailureKind) -> Self {
        match value {
            FailureKind::Rejected(_) => FailureReason::Rejected,
            FailureKind::NoSamples => FailureReason::NoSamples,
            FailureKind::TypeMismatch => FailureReason::TypeMismatch,
            FailureKind::ConditionFailed => FailureReason::ConditionFailed,
            FailureKind::AnchorNeverOccurred => FailureReason::AnchorNeverOccurred,
        }
    }
}
