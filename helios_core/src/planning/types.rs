// helios_core/src/planning/types.rs
//
// Core planning data types: Path, PlannerGoal, PlannerResult, PlannerStatus.

use nalgebra::{Isometry3, Vector2};

use crate::control::TrajectoryPoint;

/// A planned path: an ordered sequence of trajectory waypoints.
#[derive(Clone)]
pub struct Path {
    pub waypoints: Vec<TrajectoryPoint>,
    pub timestamp: f64,
    /// "global" | "local" | custom string matching PipelineLevel.
    pub level_key: String,
}

impl Path {
    pub fn is_empty(&self) -> bool {
        self.waypoints.is_empty()
    }

    pub fn len(&self) -> usize {
        self.waypoints.len()
    }

    pub fn get(&self, idx: usize) -> Option<&TrajectoryPoint> {
        self.waypoints.get(idx)
    }
}

/// The goal a planner should drive toward.
#[derive(Clone)]
pub enum PlannerGoal {
    /// Full 6-DOF pose in ENU world frame.
    WorldPose(Isometry3<f64>),
    /// 2D position in ENU world frame (Z ignored).
    WorldPosition2D(Vector2<f64>),
    /// Track a waypoint index from the global planner's cached path.
    GlobalPathWaypoint { waypoint_index: usize },
}

impl PlannerGoal {
    /// Extract the 2D ENU position of this goal.
    pub fn position_2d(&self) -> Vector2<f64> {
        match self {
            PlannerGoal::WorldPose(iso) => Vector2::new(iso.translation.x, iso.translation.y),
            PlannerGoal::WorldPosition2D(v) => *v,
            PlannerGoal::GlobalPathWaypoint { .. } => Vector2::zeros(),
        }
    }
}

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
