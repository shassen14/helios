//! Helios test harness: load a run file, drive a brain's pipeline, and report.
//!
//! # The three lives of data in this crate
//!
//! Every type here belongs to one phase of a run's lifecycle. The *phase* — not
//! the feature module it happens to live in — is what tells you when the type
//! exists and what it's for. Each public type's doc comment is tagged with its
//! phase, so you can tell at a glance (and `grep` for a whole phase):
//!
//! - **`_Pre-step._`** — parsed from the run file *before* the run starts. Its
//!   job is to be *strict*: reject unknown keys, fail loudly on a bad file.
//!   `Run`, `RunLoadError`, `ScenarioRef`, `ObservationRef`, `Termination`,
//!   `OnAssertion`, `ValidationError`, `Assertion`, `TerminalAssertion`,
//!   `ContinuousAssertion`.
//! - **`_Per-step._`** — live state and verdicts produced *while* ticking; no
//!   serde, never serialized. `Runner`, `AssertionState`, `ContinuousStatus`,
//!   `TickAction`, `AssertionResult`, `FailureKind`, `ConditionError`,
//!   `TerminationReason`, `TargetRegistry`, `ExtractorTable`, `Extractor`,
//!   `AgentId`.
//! - **`_Post-step._`** — built *after* the run and serialized out. Its job is
//!   to be *stable*: never break the consumer that parses it. `Report`,
//!   `ReportStatus`, `TerminatedBy`, `AssertionStatus`, `AssertionReportEntry`.
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
pub mod report;
pub mod run;
pub mod runner;
