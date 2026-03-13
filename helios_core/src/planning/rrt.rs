// helios_core/src/planning/rrt.rs
//
// RRT* planner stub — validates extensibility of the Planner trait.
// Returns Error("not yet implemented") to keep the pipeline stable.

use crate::frames::FrameAwareState;
use crate::mapping::MapData;

use super::context::PlannerContext;
use super::types::{Path, PlannerGoal, PlannerResult, PlannerStatus};
use super::Planner;

pub struct RrtStarConfig {
    pub rate_hz: f64,
    pub max_iterations: u32,
    pub step_size_m: f64,
    pub goal_tolerance_m: f64,
}

pub struct RrtStarPlanner {
    #[allow(dead_code)]
    config: RrtStarConfig,
    goal: Option<PlannerGoal>,
    status: PlannerStatus,
}

impl RrtStarPlanner {
    pub fn new(config: RrtStarConfig) -> Self {
        Self { config, goal: None, status: PlannerStatus::Idle }
    }
}

impl Planner for RrtStarPlanner {
    fn set_goal(&mut self, goal: PlannerGoal) {
        self.goal = Some(goal);
        self.status = PlannerStatus::Idle;
    }

    fn plan(
        &mut self,
        _state: &FrameAwareState,
        _map: &MapData,
        _ctx: &PlannerContext,
    ) -> PlannerResult {
        self.status = PlannerStatus::Failed;
        PlannerResult::Error("RRT* not yet implemented".into())
    }

    fn should_replan(&self, _state: &FrameAwareState, _ctx: &PlannerContext) -> bool {
        false // Never triggers until implemented.
    }

    fn status(&self) -> PlannerStatus {
        self.status.clone()
    }

    fn current_path(&self) -> Option<&Path> {
        None
    }
}
