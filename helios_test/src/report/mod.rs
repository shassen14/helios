pub mod aggregate;
pub mod console;
pub mod toml_writer;

use crate::{
    assertion::{AssertionResult, AssertionValue, Condition, FailureKind, Window},
    run::termination::TerminationReason,
    statistics::within_run::Reducer,
};

use serde::Serialize;

/// The report format's version. A report with none is the shape before
/// entries were named, so this starts at the second.
const REPORT_SCHEMA_VERSION: &str = "2.0";

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
    /// Why the run was unfit to test, if it was — an agent whose autonomy
    /// pipeline never assembled, an assertion naming a target no agent
    /// produces. Present only on a run that failed before it could tick, and
    /// omitted from the TOML entirely otherwise, so a healthy report keeps its
    /// current shape. Without it a CI reader sees `status = "failed"` with an
    /// empty `assertions` list and no way to tell why.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    setup_failures: Vec<String>,
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
}

impl Report {
    /// Assemble a finished report. Lives here, not as a struct literal in
    /// `finalize`, because these fields are private to the `report` module —
    /// keeping construction of the output schema in one place, next to the
    /// projections that feed it.
    ///
    /// The run's status is derived here, not passed in, so it can't disagree
    /// with the entries: `Passed` only when every assertion passed and
    /// nothing failed setup, was refused by a bus or was dropped by the
    /// recording. A setup failure, a refused write or a dropped value
    /// outranks the assertion tally: a run whose scene never became fit to
    /// test, or whose wiring or recording dropped values, cannot pass however
    /// its assertions came out. Measures never decide it.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        run_name: String,
        scenario: String,
        master_seed: Option<u64>,
        setup_failures: Vec<String>,
        refused_writes: Vec<String>,
        recording: Vec<String>,
        simulated_duration_secs: f64,
        wall_duration_secs: f64,
        terminated_by: TerminatedBy,
        assertions: Vec<AssertionReportEntry>,
        measures: Vec<MeasureReportEntry>,
    ) -> Self {
        let all_passed = assertions
            .iter()
            .all(|entry| entry.status() == AssertionStatus::Passed);
        let status = if all_passed
            && setup_failures.is_empty()
            && refused_writes.is_empty()
            && recording.is_empty()
        {
            ReportStatus::Passed
        } else {
            ReportStatus::Failed
        };
        Self {
            schema_version: REPORT_SCHEMA_VERSION,
            run_name,
            scenario,
            master_seed,
            status,
            setup_failures,
            refused_writes,
            recording,
            simulated_duration_secs,
            wall_duration_secs,
            terminated_by,
            assertions,
            measures,
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
    /// An every-sample assertion failed in a `fail_fast` run; `name` is
    /// the assertion's.
    AssertionFailed { name: String },
}

/// Project the per-step stop reason into the report's stable vocabulary.
/// `MaxSimulatedSeconds` becomes `TimeBudget`.
impl From<&TerminationReason> for TerminatedBy {
    fn from(value: &TerminationReason) -> Self {
        match value {
            TerminationReason::Aborted => TerminatedBy::Aborted,
            TerminationReason::MaxSimulatedSeconds => TerminatedBy::TimeBudget,
            TerminationReason::AssertionFailed { name } => TerminatedBy::AssertionFailed {
                name: name.to_string(),
            },
        }
    }
}

/// _Post-step._ One assertion's line in the report: what was asserted, the
/// series it was judged on, and its verdict. Its own type — rather than
/// serializing the per-step `AssertionResult` — so the output schema is
/// decoupled from the evaluator.
#[derive(Debug, Clone, Serialize)]
pub struct AssertionReportEntry {
    name: String,
    target: String,
    /// As the run file wrote it; absent on one over the whole run. Where it
    /// fell in this run is the series summary's `window_secs`.
    #[serde(skip_serializing_if = "Window::is_whole_run")]
    window: Window,
    /// Absent on an assertion that judged every sample.
    #[serde(skip_serializing_if = "Option::is_none")]
    reduce: Option<Reducer>,
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
    pub fn new(
        name: String,
        target: String,
        window: Window,
        reduce: Option<Reducer>,
        condition: Condition,
        series: SeriesSummary,
        verdict: Verdict,
    ) -> Self {
        Self {
            name,
            target,
            window,
            reduce,
            condition,
            series,
            verdict,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
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
    target: String,
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
    #[serde(skip_serializing_if = "Option::is_none")]
    reason: Option<FailureReason>,
    #[serde(flatten)]
    series: SeriesSummary,
}

impl MeasureReportEntry {
    /// Build one measure line. A `value` and a `reason` are exclusive: a
    /// measure has a value, a reason it has none, or, as `None`, neither,
    /// when the run never read it or never placed its window.
    pub fn new(
        name: String,
        target: String,
        window: Window,
        reduce: Reducer,
        covered: bool,
        value: Option<Result<AssertionValue, FailureReason>>,
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
            window,
            reduce,
            covered,
            value,
            reason,
            series,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn covered(&self) -> bool {
        self.covered
    }

    /// The measured value, if the reducer had one.
    pub fn value(&self) -> Option<&AssertionValue> {
        self.value.as_ref()
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
    /// aborted setup; a window anchored late enough to end past a run that
    /// ran out is the other case. Either way the run didn't test what was
    /// asked, so a run with one is never `Passed`. `actual` is the value it
    /// stopped at.
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
