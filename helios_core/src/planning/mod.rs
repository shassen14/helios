// helios_core/src/planning/mod.rs
//
// Planner trait + sub-modules. Concrete implementations: astar, rrt.

pub mod astar;
pub mod context;
pub mod rrt;
pub mod search_space;
pub mod types;

use crate::frames::FrameAwareState;
use crate::mapping::MapData;
use context::PlannerContext;
use types::{Path, PlannerGoal, PlannerResult, PlannerStatus};

/// A stateful planner that generates a `Path` given the current state and map.
///
/// The planner owns the replan decision via `should_replan()`. The pipeline
/// calls `plan()` every tick; the planner is free to return `PathStillValid`
/// when its internal rate gate has not elapsed.
pub trait Planner: Send + Sync {
    /// Set (or update) the navigation goal. Triggers an immediate replan on the next `plan()` call.
    fn set_goal(&mut self, goal: PlannerGoal);

    /// Compute or update the planned path.
    ///
    /// Implementations must:
    /// - Return `PathStillValid` when no replan is needed.
    /// - Return `GoalReached` when within `arrival_tolerance_m`.
    /// - Return `GoalOutsideMap(partial_path)` for local planners whose goal is outside the map.
    /// - Never panic; return `Error(reason)` for recoverable failures.
    fn plan(
        &mut self,
        state: &FrameAwareState,
        map: &MapData,
        ctx: &PlannerContext,
    ) -> PlannerResult;

    /// Returns true if a replan should be performed on this tick.
    /// Each implementation decides based on rate_hz, deviation, goal change, etc.
    fn should_replan(&self, state: &FrameAwareState, ctx: &PlannerContext) -> bool;

    /// Current lifecycle status.
    fn status(&self) -> PlannerStatus;

    /// The most recently computed path, if any.
    fn current_path(&self) -> Option<&Path>;
}
