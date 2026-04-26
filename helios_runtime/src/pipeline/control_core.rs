// helios_runtime/src/pipeline/control_core.rs
//
// ControlCore struct + impl.

use std::collections::HashMap;

use helios_core::{
    control::{ControlContext, ControlOutput},
    frames::FrameAwareState,
    mapping::MapData,
    planning::{
        context::PlannerContext,
        types::{Path, PlannerGoal},
    },
    types::TrajectoryPoint,
};

use crate::{
    runtime::{AgentRuntime, TfProviderAdapter},
    stage::{LeveledController, LeveledPlanner, PipelineLevel},
};

/// Control stage: planners + controllers.
pub struct ControlCore {
    pub planners: Vec<LeveledPlanner>,
    pub controllers: Vec<LeveledController>,
}

impl ControlCore {
    /// Set a navigation goal on all planners and trigger an immediate replan.
    pub fn set_goal(&mut self, goal: PlannerGoal) {
        for lp in &mut self.planners {
            lp.planner.set_goal(goal.clone());
        }
    }

    /// Run all planners in level order. Updates `cached_paths` and `lookahead_indices`.
    ///
    /// `maps` must be keyed by `PipelineLevel`; each planner is matched against its own level.
    pub fn step_planners(
        &mut self,
        state: &FrameAwareState,
        maps: &HashMap<PipelineLevel, &MapData>,
        now: f64,
        runtime: &dyn AgentRuntime,
    ) -> Vec<(PipelineLevel, Path)> {
        let adapter = TfProviderAdapter(runtime);
        let ctx = PlannerContext {
            tf: Some(&adapter),
            now,
        };

        let mut new_paths: Vec<(PipelineLevel, Path)> = vec![];

        for lp in &mut self.planners {
            let map = match maps.get(&lp.level) {
                Some(m) => *m,
                None => continue,
            };

            use helios_core::planning::types::PlannerResult;
            match lp.planner.plan(state, map, &ctx) {
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
        runtime: &dyn AgentRuntime,
    ) -> Option<ControlOutput> {
        let adapter = TfProviderAdapter(runtime);
        let ctx = ControlContext {
            tf: Some(&adapter),
            reference: reference_waypoint,
        };
        self.controllers
            .first_mut()
            .map(|lc| lc.controller.compute(state, dt, &ctx))
    }
}
