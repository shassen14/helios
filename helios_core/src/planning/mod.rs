//! [`SearchPlanner`] trait and path-planning submodules.
//!
//! Family-prefixed per `docs/algorithm_family_traits.md` §3.1: search-family
//! planners (A*, Dijkstra, D*) operate on a discrete graph/grid and find an
//! optimal path under a cost function. Sampling-family planners (RRT*, PRM,
//! BIT*) live behind a separate trait that does not exist yet — when they
//! land, they will not share `SearchPlanner`.
//!
//! Today the only impl is [`astar::AStarPlanner`].

pub mod astar;
pub mod search_space;
pub mod types;

use crate::frames::RobotState;
use crate::mapping::MapData;
use types::{PlannerGoal, PlannerResult};

/// A stateful search-family path planner.
///
/// The planner owns its own replan decision via [`should_replan`]
/// — a node-level rate timer cannot express deviation-based replanning, so
/// the trait keeps it. The pipeline calls [`plan`] every tick; the planner is
/// free to return [`PlannerResult::PathStillValid`] when its internal rate
/// gate has not elapsed.
///
/// [`plan`]: SearchPlanner::plan
/// [`should_replan`]: SearchPlanner::should_replan
pub trait SearchPlanner: Send + Sync {
    /// Compute or update the planned path.
    ///
    /// Implementations must:
    /// - Return `PathStillValid` when no replan is needed.
    /// - Return `GoalReached` when within `arrival_tolerance_m`.
    /// - Return `GoalOutsideMap(partial_path)` for local planners whose goal is outside the map.
    /// - Never panic; return `Error(reason)` for recoverable failures.
    fn plan(&mut self, now: f64, inputs: &SearchPlannerInputs) -> PlannerResult;

    /// Returns true if a replan should be performed on this tick.
    /// Each implementation decides based on rate_hz, deviation, goal change, etc.
    fn should_replan(&self, now: f64, inputs: &SearchPlannerInputs) -> bool;
}

pub struct SearchPlannerInputs {
    pub state: RobotState,
    pub map: MapData,
    pub goal: Option<PlannerGoal>,
}
