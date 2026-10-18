// Core planning-internal data types: PlannerResult, PlannerStatus. The
// cross-domain nouns Path and PlannerGoal live in interchange/path.rs.
use crate::interchange::path::Path;

/// The result returned by `Planner::plan()` on each invocation.
pub enum PlannerResult {
    /// A new path was computed; callers should cache and use it.
    Path(Path),
    /// Goal was outside the map; returned a partial path to the map boundary.
    GoalOutsideMap(Path),
    /// A* (or other search) found no valid path.
    Unreachable,
    /// Robot is within `arrival_tolerance_m` of the goal.
    GoalReached,
    /// The existing path is still valid; no replan was performed.
    PathStillValid,
    /// No goal has been set yet.
    NoGoal,
    /// A recoverable internal error; the planner logs and returns a description.
    Error(String),
}

/// High-level lifecycle status of a planner.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PlannerStatus {
    Idle,
    Active,
    GoalReached,
    Failed,
}
