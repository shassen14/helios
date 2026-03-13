// helios_runtime/src/pipeline.rs
//
// Three independent pipeline stages plus AutonomyPipeline (their composite).
//
// helios_sim decomposes AutonomyPipeline into ECS components via into_parts()
// so Bevy can schedule EstimationCore and MappingCore in parallel.
// helios_hw uses AutonomyPipeline as a whole unit.

use std::collections::HashMap;

use helios_core::{
    control::{ControlContext, ControlOutput, TrajectoryPoint},
    estimation::{FilterContext, StateEstimator},
    frames::FrameAwareState,
    mapping::{MapData, Mapper},
    messages::{MeasurementMessage, ModuleInput},
    planning::{context::PlannerContext, types::PlannerGoal, Planner},
    slam::SlamSystem,
    tracking::Tracker,
    types::Control,
};
use nalgebra::{DVector, Isometry3};

use crate::{
    runtime::{AgentRuntime, TfProviderAdapter},
    stage::{LeveledController, LeveledMapper, LeveledPlanner, PipelineLevel},
};

/// Snapshot of all stage outputs for a given tick.
pub struct PipelineOutputs {
    pub ego_state: Option<FrameAwareState>,
    pub global_map: Option<MapData>,
    pub local_map: Option<MapData>,
    pub control_output: Option<ControlOutput>,
}

// =========================================================================
// == Three independent pipeline stages ==
// =========================================================================

/// Estimation stage: trackers + estimator/SLAM + shared control input state.
/// Owns all logic for predict + update sequencing.
pub struct EstimationCore {
    pub trackers: Vec<Box<dyn Tracker>>,
    pub estimator: Option<Box<dyn StateEstimator>>,
    pub slam: Option<Box<dyn SlamSystem>>,
    pub(crate) last_u: Control,
    #[allow(dead_code)]
    pub(crate) control_dim: usize,
    /// Ground-truth state override. When `Some`, `get_state()` returns this value
    /// instead of the real estimator output. Used by mock-estimator profiles.
    pub injected_state: Option<helios_core::frames::FrameAwareState>,
}

impl EstimationCore {
    /// Process one incoming sensor measurement through trackers + estimator/SLAM.
    /// Called once per measurement event.
    pub fn process_measurement(&mut self, msg: &MeasurementMessage, runtime: &dyn AgentRuntime) {
        let adapter = TfProviderAdapter(runtime);
        let context = FilterContext { tf: Some(&adapter) };

        for tracker in &mut self.trackers {
            tracker.update(msg, &context);
        }

        if let Some(estimator) = &mut self.estimator {
            let dt = msg.timestamp - estimator.get_state().last_update_timestamp;
            if dt > 1e-9 {
                estimator.predict(dt, &self.last_u, &context);
            }
            let dynamics = estimator.get_dynamics_model();
            if let Some(new_u) = dynamics.get_control_from_measurement(&msg.data) {
                self.last_u = new_u;
            } else {
                estimator.update(msg, &context);
            }
        } else if let Some(slam) = &mut self.slam {
            let dt = msg.timestamp - slam.get_state().last_update_timestamp;
            if dt > 1e-9 {
                slam.predict(dt, &self.last_u, &context);
            }
            let dynamics = slam.get_dynamics_model();
            if let Some(new_u) = dynamics.get_control_from_measurement(&msg.data) {
                self.last_u = new_u;
            } else {
                slam.update(msg, &context);
            }
        }
    }

    /// Current best estimate of the ego state (from estimator or SLAM).
    /// If `injected_state` is set (mock-GT profiles), returns that instead.
    pub fn get_state(&self) -> Option<&FrameAwareState> {
        if let Some(ref s) = self.injected_state {
            return Some(s);
        }
        if let Some(est) = &self.estimator {
            Some(est.get_state())
        } else {
            self.slam.as_ref().map(|s| s.get_state())
        }
    }

    /// Override the estimated state with ground-truth physics data.
    /// Builds a minimal `FrameAwareState` (position + velocity + orientation) and
    /// stores it so `get_state()` returns it on the next call.
    ///
    /// Used by `MockGroundTruthEstimatorPlugin` — Bevy system calls this; no Bevy types here.
    pub fn inject_ground_truth(
        &mut self,
        pose: &Isometry3<f64>,
        velocity: nalgebra::Vector3<f64>,
        timestamp: f64,
    ) {
        use helios_core::frames::{FrameAwareState, FrameId, StateVariable};

        let layout = vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Py(FrameId::World),
            StateVariable::Pz(FrameId::World),
            StateVariable::Vx(FrameId::World),
            StateVariable::Vy(FrameId::World),
            StateVariable::Vz(FrameId::World),
            StateVariable::Qx(FrameId::World, FrameId::World),
            StateVariable::Qy(FrameId::World, FrameId::World),
            StateVariable::Qz(FrameId::World, FrameId::World),
            StateVariable::Qw(FrameId::World, FrameId::World),
        ];
        let mut state = FrameAwareState::new(layout, 1e-6, timestamp);
        state.vector[0] = pose.translation.x;
        state.vector[1] = pose.translation.y;
        state.vector[2] = pose.translation.z;
        state.vector[3] = velocity.x;
        state.vector[4] = velocity.y;
        state.vector[5] = velocity.z;
        let q = pose.rotation.quaternion();
        state.vector[6] = q.i;
        state.vector[7] = q.j;
        state.vector[8] = q.k;
        state.vector[9] = q.w;
        self.injected_state = Some(state);
    }

    /// SLAM global map, if a SLAM system is active.
    pub fn get_slam_map(&self) -> Option<&MapData> {
        self.slam.as_ref().map(|s| s.get_map())
    }
}

/// Mapping stage: leveled mappers.
/// When `slam_active` is true, global-level mappers are skipped (SLAM owns that slot).
pub struct MappingCore {
    pub mappers: Vec<LeveledMapper>,
    pub slam_active: bool,
}

impl MappingCore {
    /// Feed new sensor messages to all mappers (cheap log-odds update).
    pub fn process_messages(&mut self, inputs: &[MeasurementMessage], runtime: &dyn AgentRuntime) {
        let adapter = TfProviderAdapter(runtime);
        let context = FilterContext { tf: Some(&adapter) };

        for leveled_mapper in &mut self.mappers {
            if self.slam_active && leveled_mapper.level == PipelineLevel::Global {
                continue;
            }
            for msg in inputs {
                leveled_mapper
                    .mapper
                    .process(&ModuleInput::Measurement { message: msg }, &context);
            }
        }
    }

    /// Push an odom pose update into all mappers so the grid can recenter.
    pub fn process_pose_update(&mut self, pose: Isometry3<f64>) {
        let context = FilterContext { tf: None };
        for leveled_mapper in &mut self.mappers {
            leveled_mapper
                .mapper
                .process(&ModuleInput::PoseUpdate { pose }, &context);
        }
    }

    /// Current map at the given level.
    pub fn get_map(&self, level: &PipelineLevel) -> Option<&MapData> {
        self.mappers
            .iter()
            .find(|lm| &lm.level == level)
            .map(|lm| lm.mapper.get_map())
    }
}

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
        maps: &HashMap<PipelineLevel, &MapData>,
        now: f64,
        runtime: &dyn AgentRuntime,
    ) {
        let adapter = TfProviderAdapter(runtime);
        let ctx = PlannerContext { tf: Some(&adapter), now };

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
    pub fn get_active_lookahead_waypoint(
        &self,
        level: &PipelineLevel,
    ) -> Option<&TrajectoryPoint> {
        let path = self.cached_paths.get(level)?;
        let idx = *self.lookahead_indices.get(level).unwrap_or(&0);
        path.waypoints.get(idx)
    }

    /// Returns the cached path for `level`.
    pub fn get_cached_path(&self, level: &PipelineLevel) -> Option<&helios_core::planning::types::Path> {
        self.cached_paths.get(level)
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

// =========================================================================
// == AutonomyPipeline: composite of all three stages ==
// =========================================================================

/// The full autonomy pipeline. Constructed via `PipelineBuilder`.
///
/// - **helios_hw**: use as a whole unit — call `process_measurement()`, `step_controllers()`, etc.
/// - **helios_sim**: call `into_parts()` to decompose into three independent ECS components
///   so Bevy can schedule `EstimationCore` and `MappingCore` systems in parallel.
pub struct AutonomyPipeline {
    pub estimation: EstimationCore,
    pub mapping: MappingCore,
    pub control: ControlCore,
}

impl AutonomyPipeline {
    pub fn builder() -> PipelineBuilder {
        PipelineBuilder::new()
    }

    /// Decompose into three independent stage values for ECS component insertion.
    /// After this call, `self` is consumed. Use `into_parts()` in helios_sim spawn systems.
    pub fn into_parts(self) -> (EstimationCore, MappingCore, ControlCore) {
        (self.estimation, self.mapping, self.control)
    }

    // =========================================================================
    // == Delegate methods (for helios_hw convenience) ==
    // =========================================================================

    /// Process one incoming sensor measurement. Delegates to `EstimationCore`.
    pub fn process_measurement(
        &mut self,
        msg: &MeasurementMessage,
        runtime: &dyn AgentRuntime,
    ) -> PipelineOutputs {
        self.estimation.process_measurement(msg, runtime);
        self.build_outputs()
    }

    /// Feed new sensor messages to all mappers. Delegates to `MappingCore`.
    pub fn process_mapper_messages(
        &mut self,
        inputs: &[MeasurementMessage],
        runtime: &dyn AgentRuntime,
    ) {
        self.mapping.process_messages(inputs, runtime);
    }

    /// Push an odom pose update into all mappers. Delegates to `MappingCore`.
    pub fn process_mapper_pose_update(&mut self, pose: Isometry3<f64>) {
        self.mapping.process_pose_update(pose);
    }

    /// Current best estimate of the ego state. Delegates to `EstimationCore`.
    pub fn get_state(&self) -> Option<&FrameAwareState> {
        self.estimation.get_state()
    }

    /// Current map at the given level. Global level checks SLAM first.
    pub fn get_map(&self, level: &PipelineLevel) -> Option<&MapData> {
        if *level == PipelineLevel::Global {
            if let Some(map) = self.estimation.get_slam_map() {
                return Some(map);
            }
        }
        self.mapping.get_map(level)
    }

    /// Run the highest-priority controller. Delegates to `ControlCore`.
    pub fn step_controllers(
        &mut self,
        dt: f64,
        runtime: &dyn AgentRuntime,
    ) -> Option<ControlOutput> {
        let state = self.estimation.get_state()?.clone();
        self.control.step_controllers(&state, dt, runtime)
    }

    fn build_outputs(&self) -> PipelineOutputs {
        PipelineOutputs {
            ego_state: self.get_state().cloned(),
            global_map: self.get_map(&PipelineLevel::Global).cloned(),
            local_map: self.get_map(&PipelineLevel::Local).cloned(),
            control_output: None,
        }
    }
}

// =========================================================================
// == PipelineBuilder ==
// =========================================================================

pub struct PipelineBuilder {
    trackers: Vec<Box<dyn Tracker>>,
    estimator: Option<Box<dyn StateEstimator>>,
    slam: Option<Box<dyn SlamSystem>>,
    mappers: Vec<LeveledMapper>,
    planners: Vec<LeveledPlanner>,
    controllers: Vec<LeveledController>,
    control_dim: usize,
    goal: Option<PlannerGoal>,
}

impl PipelineBuilder {
    pub fn new() -> Self {
        Self {
            trackers: Vec::new(),
            estimator: None,
            slam: None,
            mappers: Vec::new(),
            planners: Vec::new(),
            controllers: Vec::new(),
            control_dim: 0,
            goal: None,
        }
    }

    pub fn with_goal(mut self, goal: PlannerGoal) -> Self {
        self.goal = Some(goal);
        self
    }

    pub fn with_estimator(mut self, estimator: Box<dyn StateEstimator>) -> Self {
        self.control_dim = estimator.get_dynamics_model().get_control_dim();
        self.estimator = Some(estimator);
        self
    }

    pub fn with_slam(mut self, slam: Box<dyn SlamSystem>) -> Self {
        self.slam = Some(slam);
        self
    }

    pub fn with_mapper(mut self, level: PipelineLevel, mapper: Box<dyn Mapper>) -> Self {
        self.mappers.push(LeveledMapper { level, mapper });
        self
    }

    pub fn with_planner(mut self, level: PipelineLevel, planner: Box<dyn Planner>) -> Self {
        self.planners.push(LeveledPlanner { level, planner });
        self
    }

    pub fn with_controller(
        mut self,
        level: PipelineLevel,
        controller: Box<dyn helios_core::control::Controller>,
    ) -> Self {
        self.controllers.push(LeveledController { level, controller });
        self
    }

    pub fn build(mut self) -> AutonomyPipeline {
        self.mappers.sort_by(|a, b| a.level.cmp(&b.level));
        self.planners.sort_by(|a, b| a.level.cmp(&b.level));
        self.controllers.sort_by(|a, b| a.level.cmp(&b.level));

        // Inject static goal into all planners if one was provided.
        if let Some(ref goal) = self.goal {
            for lp in &mut self.planners {
                lp.planner.set_goal(goal.clone());
            }
        }

        let slam_active = self.slam.is_some();

        AutonomyPipeline {
            estimation: EstimationCore {
                trackers: self.trackers,
                estimator: self.estimator,
                slam: self.slam,
                last_u: DVector::zeros(self.control_dim),
                control_dim: self.control_dim,
                injected_state: None,
            },
            mapping: MappingCore {
                mappers: self.mappers,
                slam_active,
            },
            control: ControlCore {
                planners: self.planners,
                controllers: self.controllers,
                cached_paths: HashMap::new(),
                lookahead_indices: HashMap::new(),
            },
        }
    }
}

impl Default for PipelineBuilder {
    fn default() -> Self {
        Self::new()
    }
}
