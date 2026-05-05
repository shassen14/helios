// helios_runtime/src/pipeline/control_core.rs
//
// ControlCore struct + impl.

use std::collections::HashMap;

use helios_core::{
    control::{ControlInputs, ControlOutput},
    frames::FrameAwareState,
    mapping::MapData,
    planning::{
        types::{Path, PlannerGoal},
        PlannerInputs,
    },
    types::TrajectoryPoint,
};

use crate::{
    runtime::AgentRuntime,
    stage::{LeveledController, LeveledPlanner, PipelineLevel},
};

/// Control stage: planners + controllers.
pub struct ControlCore {
    pub planners: Vec<LeveledPlanner>,
    pub controllers: Vec<LeveledController>,
    /// Current navigation goal, passed to planners each tick via `PlannerInputs`.
    pub current_goal: Option<PlannerGoal>,
}
impl ControlCore {
    /// Set (or replace) the navigation goal. Planners will receive it on the next `step_planners` call.
    pub fn set_goal(&mut self, goal: PlannerGoal) {
        self.current_goal = Some(goal);
    }

    /// Run all planners in level order.
    ///
    /// `maps` must be keyed by `PipelineLevel`; each planner is matched against its own level.
    pub fn step_planners(
        &mut self,
        state: &FrameAwareState,
        maps: &HashMap<PipelineLevel, &MapData>,
        now: f64,
    ) -> Vec<(PipelineLevel, Path)> {
        let mut new_paths: Vec<(PipelineLevel, Path)> = vec![];

        for lp in &mut self.planners {
            let map = match maps.get(&lp.level) {
                Some(m) => *m,
                None => continue,
            };

            let inputs = PlannerInputs {
                state: state.state.clone(),
                map: map.clone(),
                goal: self.current_goal.clone(),
            };

            use helios_core::planning::types::PlannerResult;
            match lp.planner.plan(now, &inputs) {
                PlannerResult::Path(path) | PlannerResult::GoalOutsideMap(path) => {
                    new_paths.push((lp.level.clone(), path));
                }
                PlannerResult::GoalReached => {
                    // Keep last path; stop advancing.
                }
                PlannerResult::Unreachable => {
                    log::warn!("[ControlCore] Planner {:?}: no path found", lp.level);
                }
                PlannerResult::Error(msg) => {
                    log::warn!("[ControlCore] Planner {:?} error: {}", lp.level, msg);
                }
                PlannerResult::PathStillValid | PlannerResult::NoGoal => {}
            }
        }

        new_paths
    }

    /// Run the highest-priority controller given the current ego state.
    /// Advances look-ahead indices and passes the reference waypoint to the controller.
    /// State is passed in from EstimationCore to keep stages independent.
    pub fn step_controllers(
        &mut self,
        state: &FrameAwareState,
        reference_waypoint: Option<&TrajectoryPoint>,
        dt: f64,
        _runtime: &dyn AgentRuntime,
    ) -> Option<ControlOutput> {
        let inputs = ControlInputs {
            state: state.clone(),
            reference: reference_waypoint.cloned(),
        };
        self.controllers
            .first_mut()
            .map(|lc| lc.controller.compute(dt, &inputs))
    }
}
