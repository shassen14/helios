//! GeometricPlanner trait and path-planning submodules.
//!
//! Defines the [`GeometricPlanner`] trait (set goal, plan, replan decision, current path).
//! Submodules: `astar` (concrete A* implementation), `context`, `search_space`, `types`.

pub mod astar;
pub mod search_space;
pub mod types;

use crate::frames::RobotState;
use crate::mapping::MapData;
use types::{Path, PlannerGoal, PlannerResult, PlannerStatus};

/// A stateful geometric planner that generates a `Path` given the current state and map.
///
/// The planner owns the replan decision via `should_replan()`. The pipeline
/// calls `plan()` every tick; the planner is free to return `PathStillValid`
/// when its internal rate gate has not elapsed.
pub trait GeometricPlanner: Send + Sync {
    /// Compute or update the planned path.
    ///
    /// Implementations must:
    /// - Return `PathStillValid` when no replan is needed.
    /// - Return `GoalReached` when within `arrival_tolerance_m`.
    /// - Return `GoalOutsideMap(partial_path)` for local planners whose goal is outside the map.
    /// - Never panic; return `Error(reason)` for recoverable failures.
    fn plan(&mut self, now: f64, inputs: &GeometricPlannerInputs) -> PlannerResult;

    /// Returns true if a replan should be performed on this tick.
    /// Each implementation decides based on rate_hz, deviation, goal change, etc.
    fn should_replan(&self, now: f64, inputs: &GeometricPlannerInputs) -> bool;

    /// Current lifecycle status.
    fn status(&self) -> PlannerStatus;

    /// The most recently computed path, if any.
    fn current_path(&self) -> Option<&Path>;
}

pub struct GeometricPlannerInputs {
    pub state: RobotState,
    pub map: MapData,
    pub goal: Option<PlannerGoal>,
}
