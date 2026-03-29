// helios_runtime/src/pipeline/control_core.rs
//
// ControlCore struct + impl.

use std::collections::HashMap;

use helios_core::{
    control::{ControlContext, ControlOutput},
    frames::FrameAwareState,
    planning::{context::PlannerContext, types::PlannerGoal},
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
    /// Most recently accepted path per level.
    pub cached_paths: HashMap<PipelineLevel, helios_core::planning::types::Path>,
    /// Index of the current look-ahead waypoint per level.
    pub lookahead_indices: HashMap<PipelineLevel, usize>,
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
        maps: &HashMap<PipelineLevel, &helios_core::mapping::MapData>,
        now: f64,
        runtime: &dyn AgentRuntime,
    ) {
        let adapter = TfProviderAdapter(runtime);
        let ctx = PlannerContext {
            tf: Some(&adapter),
            now,
        };

        for lp in &mut self.planners {
            let map = match maps.get(&lp.level) {
                Some(m) => *m,
                None => continue,
            };

            use helios_core::planning::types::PlannerResult;
            match lp.planner.plan(state, map, &ctx) {
                PlannerResult::Path(path) | PlannerResult::GoalOutsideMap(path) => {
                    self.lookahead_indices.insert(lp.level.clone(), 0);
                    self.cached_paths.insert(lp.level.clone(), path);
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
    }

    /// Advance the look-ahead index for `level` while the robot is within 2 m of the current waypoint.
    fn advance_lookahead(&mut self, state: &FrameAwareState, level: &PipelineLevel) {
        use helios_core::frames::FrameId;
        use helios_core::frames::StateVariable;
        use nalgebra::Vector2;

        let robot_pos = match state.get_vector3(&StateVariable::Px(FrameId::World)) {
            Some(p) => Vector2::new(p.x, p.y),
            None => return,
        };

        let path = match self.cached_paths.get(level) {
            Some(p) => p,
            None => return,
        };

        let idx = self.lookahead_indices.entry(level.clone()).or_insert(0);

        while *idx + 1 < path.waypoints.len() {
            let wp = &path.waypoints[*idx];
            let wp_pos = Vector2::new(wp.state.vector[0], wp.state.vector[1]);
            if (robot_pos - wp_pos).norm() < 2.0 {
                *idx += 1;
            } else {
                break;
            }
        }
    }

    /// Returns the active look-ahead waypoint for `level`.
    pub fn get_active_lookahead_waypoint(&self, level: &PipelineLevel) -> Option<&TrajectoryPoint> {
        let path = self.cached_paths.get(level)?;
        let idx = *self.lookahead_indices.get(level).unwrap_or(&0);
        path.waypoints.get(idx)
    }

    /// Returns the cached path for `level`.
    pub fn get_cached_path(
        &self,
        level: &PipelineLevel,
    ) -> Option<&helios_core::planning::types::Path> {
        self.cached_paths.get(level)
    }

    /// Inject a path for `level`, resetting the look-ahead index to 0.
    /// Use this instead of direct field access when setting a path from outside the pipeline
    /// (e.g. mock path injection, test fixtures).
    pub fn set_path(&mut self, level: PipelineLevel, path: helios_core::planning::types::Path) {
        self.lookahead_indices.insert(level.clone(), 0);
        self.cached_paths.insert(level, path);
    }

    /// Run the highest-priority controller given the current ego state.
    /// Advances look-ahead indices and passes the reference waypoint to the controller.
    /// State is passed in from EstimationCore to keep stages independent.
    pub fn step_controllers(
        &mut self,
        state: &FrameAwareState,
        dt: f64,
        runtime: &dyn AgentRuntime,
    ) -> Option<ControlOutput> {
        // Advance look-ahead for known levels.
        self.advance_lookahead(state, &PipelineLevel::Local);
        self.advance_lookahead(state, &PipelineLevel::Global);

        // Prefer Local, fall back to Global.
        let reference_waypoint: Option<TrajectoryPoint> = self
            .get_active_lookahead_waypoint(&PipelineLevel::Local)
            .or_else(|| self.get_active_lookahead_waypoint(&PipelineLevel::Global))
            .cloned();

        let adapter = TfProviderAdapter(runtime);
        let ctx = ControlContext {
            tf: Some(&adapter),
            reference: reference_waypoint.as_ref(),
        };
        self.controllers
            .first_mut()
            .map(|lc| lc.controller.compute(state, dt, &ctx))
    }
}
