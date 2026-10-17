//! Checks, once the pipelines exist, that the run can evaluate every
//! assertion and measure: its target names one series its agent records, or
//! each instance of a keyed channel it asks for, or a metric of an
//! evaluation the run could build, and each series its window is anchored
//! `after` names one; a channel value there can be extracted, a
//! nondeterministic value is read only with permission, and the collector
//! requests it; a reducer that holds values between samples isn't given an
//! emission's events; and the reducer and the condition can apply to the
//! kind of value the series holds. Each entry that can be evaluated binds
//! its series, one per instance, and its anchors', to the exact keys they
//! name, and carries the unit its value is read in: the leaf's or the
//! extractor's, through the reducer.
//!
//! Done against what each agent declares it reports, before the first tick,
//! so a run that could never pass stops before it starts and names every
//! problem at once. What the checks find is kept as the run's plan, so a
//! tick never looks a target up again, and the plan carries each entry's
//! state, so only a run that can judge has any.

pub mod entries;
pub mod evaluations;
pub mod locate;
pub mod plan;
pub mod problems;

#[cfg(test)]
mod fixtures;

pub use plan::{EntryOwner, ResolvedPlan};
pub use problems::{TargetProblem, Unusable, UnusableEntry};

pub(super) use plan::{resolve, EntryView};
