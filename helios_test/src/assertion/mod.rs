pub mod condition;
pub mod config;
pub mod evaluator;
pub mod extract;
pub mod name;
pub mod target;

use crate::collector::Rejection;
pub use condition::{AssertionValue, Condition, ValueKind};
pub use config::{Assertion, Bound, Measure, RunInstant, Window};
pub use name::EntryName;
pub use target::{AssertionTarget, InstanceSelector, TargetRoot, TargetSubject};

/// _Per-step._ The evaluator's verdict for one assertion, produced each tick.
// Three outcomes, not two: `Pending` is distinct from `Failed` because a target
// with no value yet is not a failure mid-run — it just has nothing to judge. At
// finalize an assertion that never saw a value fails as `NoSamples` instead.
// Derives are required by the evaluator's tests — `assert_eq!` against a result
// needs `PartialEq`, and failure output needs `Debug`.
#[derive(Clone, Debug, PartialEq)]
pub enum AssertionResult {
    Passed {
        actual: AssertionValue,
    },
    Failed {
        kind: FailureKind,
        actual: Option<AssertionValue>,
    },
    Pending,
}

/// _Per-step._ Why an assertion failed.
// Each variant names a distinct cause, so the report can phrase it precisely
// instead of a generic "failed": the target's values were turned away on
// arrival, none arrived, a value of the wrong type, or the condition
// returned false.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FailureKind {
    /// A value arrived for the target but couldn't be read.
    Rejected(Rejection),
    /// No value arrived for the target in its window.
    NoSamples,
    TypeMismatch,
    ConditionFailed,
    /// The run ran out without the event a window bound is anchored
    /// `after`, so the window never existed. An empty window never passes.
    AnchorNeverOccurred,
}

impl std::fmt::Display for FailureKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FailureKind::Rejected(rejection) => write!(f, "value rejected: {rejection}"),
            FailureKind::NoSamples => write!(f, "no samples"),
            FailureKind::TypeMismatch => write!(f, "type mismatch"),
            FailureKind::ConditionFailed => write!(f, "condition failed"),
            FailureKind::AnchorNeverOccurred => write!(f, "anchor never occurred"),
        }
    }
}
