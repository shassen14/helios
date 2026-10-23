//! Why a parsed run file can't run: the coherence problems a structurally
//! valid one can still have.

use crate::{
    assertion::{condition::OperandProblem, config::WindowProblem, EntryName},
    evaluation::path::DerivedName,
};

use thiserror::Error;

/// _Pre-step._ Coherence problems a structurally-valid run file can still have. Parsing
/// proves the *shape* is right; these are the cross-field checks serde can't
/// express. Caught at load time so a nonsensical run fails fast instead of
/// being discovered partway through a simulation.
#[derive(Debug, PartialEq, Error)]
pub enum ValidationError {
    #[error(
        "`max_simulated_seconds` must be positive and finite, got {0}: the run would never \
         terminate"
    )]
    UnreachableBudget(f64),
    #[error(
        "`fail_fast` is set but the run declares no assertion judging every sample (one \
         without `reduce`) of a recorded series on a window known from the first tick (bounds \
         in seconds after run.start) that could fail early; a derived series exists only at \
         finalize"
    )]
    FailFastWithoutEverySample,
    /// Names identify verdicts in the report and across runs, so two
    /// entries can't share one, whether assertions, measures, evaluations
    /// or suites.
    #[error(
        "two entries are named '{0}'; names are unique across [[assert]], [[measure]], \
         [[evaluate]] and [[suite]]"
    )]
    DuplicateName(EntryName),
    #[error("assertion '{name}': {problem}")]
    UnusableOperand {
        name: EntryName,
        problem: OperandProblem,
    },
    #[error("'{0}': the time_above threshold is not a finite number")]
    NonFiniteThreshold(EntryName),
    #[error("'{name}': {problem}")]
    UnusableWindow {
        name: EntryName,
        problem: WindowProblem,
    },
    /// A window anchored after a derived series. An anchor is the first
    /// stamp of a recorded series; a derived one exists only at finalize.
    #[error("'{name}': its window is anchored after '{anchor}', a derived series; anchor it on a recorded one")]
    AnchorOnDerived {
        name: EntryName,
        anchor: DerivedName,
    },
    #[error("evaluation '{name}': no evaluator kind '{kind}' (there are: {known})")]
    UnknownKind {
        name: EntryName,
        kind: String,
        known: String,
    },
    #[error("evaluation '{name}': its kind reads role '{role}', and [evaluate.inputs] names no series for it")]
    MissingInput { name: EntryName, role: &'static str },
    #[error("evaluation '{name}': its kind has no role '{role}' (it reads {roles})")]
    UnknownInput {
        name: EntryName,
        role: String,
        roles: String,
    },
    #[error("evaluation '{name}': its input '{role}' names a derived series; an evaluation reads recorded series")]
    DerivedInput { name: EntryName, role: String },
    #[error("evaluation '{name}': {problem}")]
    UnusableParams { name: EntryName, problem: String },
    #[error("evaluation '{name}': its [evaluate.gap] names '{role}', which is not one of its roles ({roles})")]
    UnknownGapRole {
        name: EntryName,
        role: String,
        roles: String,
    },
    /// A problem in a suite file, named with the suite.
    #[error("suite '{suite}' ({from}): {error}")]
    Suite {
        suite: EntryName,
        from: String,
        error: Box<ValidationError>,
    },
    #[error("two evaluations of the suite are named '{0}'")]
    DuplicateSuiteEntry(EntryName),
    /// A suite's role reading an agent series of another type than it reads.
    #[error("evaluation '{name}': {problem}")]
    UnfitSeries { name: EntryName, problem: String },
    #[error("'{name}': '{target}' names no [[evaluate]] entry")]
    UnknownEvaluation {
        name: EntryName,
        target: DerivedName,
    },
    /// A derived series of a suite's evaluation the run doesn't apply: no
    /// suite of that name, no entry of that name in it, or an agent it isn't
    /// applied to.
    #[error("'{name}': '{target}' {why}")]
    UnappliedSuiteTarget {
        name: EntryName,
        target: DerivedName,
        why: String,
    },
    #[error(
        "'{name}': '{target}' names a metric its evaluation doesn't offer (it offers {metrics})"
    )]
    UnknownMetric {
        name: EntryName,
        target: DerivedName,
        metrics: String,
    },
}
