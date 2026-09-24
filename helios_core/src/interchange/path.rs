// Cross-domain planner output: Path (planning produces, following consumes) and
// PlannerGoal (host/guidance sets, planning reads). PlannerResult and
// PlannerStatus stay in planning/ — only planning names them.
use crate::spatial::{conventions::Enu, quantities::Point};

use nalgebra::{Isometry3, Vector2};

/// A planned path: an ordered sequence of trajectory waypoints.
#[derive(Clone)]
pub struct Path {
    pub waypoints: Vec<Point<Enu>>,
    pub timestamp: f64,
    /// "global" | "local" | custom string matching PipelineLevel.
    pub level_key: String,
}

/// The goal a planner should drive toward.
#[derive(Clone, PartialEq)]
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
    pub(crate) fn position_2d(&self) -> Vector2<f64> {
        match self {
            PlannerGoal::WorldPose(iso) => Vector2::new(iso.translation.x, iso.translation.y),
            PlannerGoal::WorldPosition2D(v) => *v,
            PlannerGoal::GlobalPathWaypoint { .. } => Vector2::zeros(),
        }
    }
}
