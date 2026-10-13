//! Helios test harness: load a run file, drive a brain's pipeline, and report.
//!
//! # The three lives of data in this crate
//!
//! Every type here belongs to one phase of a run's lifecycle. The *phase* — not
//! the feature module it happens to live in — is what tells you when the type
//! exists and what it's for. Each public type's doc comment is tagged with its
//! phase, so you can tell at a glance (and `grep` for a whole phase):
//!
//! - **`_Pre-step._`** — settled *before* the first tick: parsed from the run
//!   file, or checked against the pipelines once they are built. Its job is
//!   to be *strict*: reject unknown keys, fail loudly on a bad file or on an
//!   assertion the run could never judge. `Run`, `RunLoadError`,
//!   `ScenarioRef`, `ObservationRef`, `Termination`, `ValidationError`, `Assertion`, `TerminalAssertion`,
//!   `ContinuousAssertion`, `MalformedTarget`, `UnusableAssertion`,
//!   `TargetProblem`.
//! - **`_Per-step._`** — live state and verdicts produced *while* ticking; no
//!   serde, never serialized. `Runner`, `ResolvedPlan`, `AssertionState`,
//!   `ContinuousStatus`, `TickAction`, `AssertionResult`, `FailureKind`,
//!   `ConditionError`, `TerminationReason`, `ExtractorTable`, `Extractor`,
//!   `ObservationCollector`, `SeriesId`, `Series`, `Sample`, `Rejection`.
//! - **`_Per/Post-step._`** — measurement data filled *during* a run, then
//!   read back *after* it to build the serialized aggregate. `RunMetrics`.
//! - **`_Post-step._`** — built *after* the run and serialized out. Its job is
//!   to be *stable*: never break the consumer that parses it. `Report`,
//!   `ReportStatus`, `TerminatedBy`, `AssertionReportEntry`, `Verdict`,
//!   `AssertionStatus`, `FailureReason`, `AggregateReport`, `MetricRow`.
//! - **`_Shared._`** — leaf values that cross phases, so they carry both serde
//!   directions. `AssertionValue`, `Condition` (pre + post); `AssertionTarget`
//!   (pre + per).
//!
//! The leaves are shared; only the *containers* are phase-specific. An input
//! container's goal (strict) and an output container's goal (stable) pull in
//! opposite directions, which is why a single type can't serve both — and why
//! `Assertion` (pre) and `AssertionReportEntry` (post) are distinct types even
//! though they share the same value leaves.

pub mod assertion;
pub mod collector;
pub mod metrics;
pub mod monte_carlo;
pub mod report;
pub mod run;
pub mod runner;
pub mod statistics;

// The crate's public face. Host drivers (sim bin, fake-body tests,
// eventually the hw bin) import from the crate root, so internal module
// reorganization never breaks them. Re-export only what a driver needs to load a
// run, build and pump a `Runner`, and read the report; everything else stays
// reachable through its module path but isn't surfaced here.
pub use assertion::extract::{standard_extractors, ExtractorTable};
pub use assertion::{Assertion, AssertionResult, AssertionTarget, AssertionValue};
pub use collector::ObservationCollector;
pub use metrics::{MetricId, RunMetrics};
pub use report::aggregate::AggregateReport;
pub use report::{Report, ReportStatus};
pub use run::{Run, RunLoadError};
pub use runner::{Runner, TickAction};

// features
#[cfg(feature = "sim")]
pub mod sim;
