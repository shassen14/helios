use std::error::Error;
use std::fmt;

#[derive(Debug)] // Make sure it derives Debug
pub enum PlanningError {
    StartOutsideGrid,
    GoalOutsideGrid,
    StartOccupied,
    GoalOccupied,
    NoPathFound,
    GridNotInitialized,
    // Add other potential errors (e.g., CollisionDetected for RRT*)
    ObstacleQueryFailed,   // Example
    InternalError(String), // Generic fallback
}

impl fmt::Display for PlanningError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PlanningError::StartOutsideGrid => {
                write!(f, "Start point is outside the planning grid.")
            }
            PlanningError::GoalOutsideGrid => write!(f, "Goal point is outside the planning grid."),
            PlanningError::StartOccupied => write!(f, "Start point is occupied by an obstacle."),
            PlanningError::GoalOccupied => write!(f, "Goal point is occupied by an obstacle."),
            PlanningError::NoPathFound => write!(f, "No valid path found between start and goal."),
            PlanningError::GridNotInitialized => {
                write!(f, "The planning grid has not been initialized.")
            }
            PlanningError::ObstacleQueryFailed => write!(f, "Failed to query obstacles."),
            PlanningError::InternalError(msg) => write!(f, "Internal planner error: {}", msg),
        }
    }
}

impl Error for PlanningError {} // Allow using it with `?` etc.
