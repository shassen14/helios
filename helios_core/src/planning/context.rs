// helios_core/src/planning/context.rs
//
// PlannerContext: lightweight context passed to every Planner::plan() call.

use crate::types::TfProvider;

/// Context available to every `Planner::plan()` and `should_replan()` call.
pub struct PlannerContext<'a> {
    /// Transform tree provider, if available (None in unit tests).
    pub tf: Option<&'a dyn TfProvider>,
    /// Current simulation time in seconds.
    pub now: f64,
}
