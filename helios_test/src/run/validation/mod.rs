//! A parsed run file checked for coherence before anything runs: the
//! cross-field checks serde can't express.

mod entries;
mod error;
mod evaluations;
mod termination;

#[cfg(test)]
mod fixtures;

pub use error::ValidationError;

use self::{
    entries::check_entries,
    evaluations::{check_derived_targets, check_evaluations},
    termination::check_termination,
};
use crate::{evaluation::EvaluatorRegistry, run::Run};

/// Check a parsed [`Run`] for coherence. A flat checklist of guard clauses:
/// each looks for one specific incoherence and returns early with its matching
/// variant, so the report can name *which* thing was wrong. Falling through to
/// the end means every check passed.
///
/// Deliberately separate from parsing (`load`) — "can I read this?" and "is
/// this a sane run?" are different questions with different failures.
/// Evaluations are checked against the kinds in `evaluators`, which are
/// code, so a misspelled kind, role or parameter stops the run before a sim
/// starts.
pub fn validate(run: &Run, evaluators: &EvaluatorRegistry) -> Result<(), ValidationError> {
    let budget = check_termination(run)?;
    check_entries(run, budget)?;
    let kinds = check_evaluations(run, evaluators)?;
    check_derived_targets(run, &kinds)
}
