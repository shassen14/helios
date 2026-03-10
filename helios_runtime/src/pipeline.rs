// helios_runtime/src/pipeline.rs
//
// AutonomyPipeline: typed stage vector with canonical execution order.
//
// Each stage has a different rate — the Bevy systems call methods here
// at the appropriate rate via their own timers. The pipeline does not
// schedule stages internally; it just exposes methods per stage group.
//
// Estimation   → process_measurement()         (per measurement event)
// Mapping      → process_mapper_messages()     (every frame, mapper buffers data)
//                process_mapper_pose_update()  (on mapper timer fire)
// Planning     → future: plan(runtime)         (at planner rate)
// Control      → future: compute_control(runtime) (every physics tick)

use helios_core::{
    control::{ControlContext, ControlOutput, TrajectoryPoint},
    estimation::{FilterContext, StateEstimator},
    frames::FrameAwareState,
    mapping::{MapData, Mapper},
    messages::{MeasurementMessage, ModuleInput},
    planning::Planner,
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
    pub global_trajectory: Option<TrajectoryPoint>,
    pub local_trajectory: Option<TrajectoryPoint>,
    pub control_output: Option<ControlOutput>,
}

/// The autonomy pipeline. Constructed via `PipelineBuilder`.
pub struct AutonomyPipeline {
    pub trackers: Vec<Box<dyn Tracker>>,
    pub estimator: Option<Box<dyn StateEstimator>>,
    pub slam: Option<Box<dyn SlamSystem>>,
    pub mappers: Vec<LeveledMapper>,
    pub planners: Vec<LeveledPlanner>,
    pub controllers: Vec<LeveledController>,

    /// Last known control input used by the predict step.
    pub(crate) last_u: Control,
    pub(crate) control_dim: usize,
}

impl AutonomyPipeline {
    pub fn builder() -> PipelineBuilder {
        PipelineBuilder::new()
    }

    // =========================================================================
    // == Estimation stage ==
    // =========================================================================

    /// Process one incoming sensor measurement through trackers + estimator/SLAM.
    /// Called once per measurement event by the Bevy estimation system.
    pub fn process_measurement(
        &mut self,
        msg: &MeasurementMessage,
        runtime: &dyn AgentRuntime,
    ) -> PipelineOutputs {
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

        self.build_outputs()
    }

    // =========================================================================
    // == Mapping stage ==
    // =========================================================================

    /// Feed new sensor messages to all mappers.
    /// Called every frame by the Bevy mapping system so the mapper
    /// can buffer data between its own timer-controlled pose updates.
    /// Global mappers are skipped when SLAM is active.
    pub fn process_mapper_messages(
        &mut self,
        inputs: &[MeasurementMessage],
        runtime: &dyn AgentRuntime,
    ) {
        let adapter = TfProviderAdapter(runtime);
        let context = FilterContext { tf: Some(&adapter) };

        for leveled_mapper in &mut self.mappers {
            if self.slam.is_some() && leveled_mapper.level == PipelineLevel::Global {
                continue;
            }
            for msg in inputs {
                leveled_mapper
                    .mapper
                    .process(&ModuleInput::Measurement { message: msg }, &context);
            }
        }
    }

    /// Push an odom pose update into all mappers.
    /// Called by the Bevy mapping system when its `ModuleTimer` fires.
    pub fn process_mapper_pose_update(&mut self, pose: Isometry3<f64>) {
        let context = FilterContext { tf: None };
        for leveled_mapper in &mut self.mappers {
            leveled_mapper
                .mapper
                .process(&ModuleInput::PoseUpdate { pose }, &context);
        }
    }

    // =========================================================================
    // == State accessors ==
    // =========================================================================

    /// Current best estimate of the ego state (from estimator or SLAM).
    pub fn get_state(&self) -> Option<&FrameAwareState> {
        if let Some(est) = &self.estimator {
            Some(est.get_state())
        } else {
            self.slam.as_ref().map(|s| s.get_state())
        }
    }

    /// Current map at the given level. Global level defers to SLAM if present.
    pub fn get_map(&self, level: &PipelineLevel) -> Option<&MapData> {
        if *level == PipelineLevel::Global {
            if let Some(slam) = &self.slam {
                return Some(slam.get_map());
            }
        }
        self.mappers
            .iter()
            .find(|lm| &lm.level == level)
            .map(|lm| lm.mapper.get_map())
    }

    // =========================================================================
    // == Control stage ==
    // =========================================================================

    /// Run the highest-priority controller in the pipeline.
    /// Returns `None` if no controllers are registered or no state estimate is available.
    pub fn step_controllers(
        &mut self,
        dt: f64,
        runtime: &dyn AgentRuntime,
    ) -> Option<ControlOutput> {
        let state = self.get_state()?.clone();
        let adapter = TfProviderAdapter(runtime);
        let ctx = ControlContext { tf: Some(&adapter), reference: None };
        self.controllers
            .first_mut()
            .map(|lc| lc.controller.compute(&state, dt, &ctx))
    }

    fn build_outputs(&self) -> PipelineOutputs {
        PipelineOutputs {
            ego_state: self.get_state().cloned(),
            global_map: self.get_map(&PipelineLevel::Global).cloned(),
            local_map: self.get_map(&PipelineLevel::Local).cloned(),
            global_trajectory: None,
            local_trajectory: None,
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
        }
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

        AutonomyPipeline {
            trackers: self.trackers,
            estimator: self.estimator,
            slam: self.slam,
            mappers: self.mappers,
            planners: self.planners,
            controllers: self.controllers,
            last_u: DVector::zeros(self.control_dim),
            control_dim: self.control_dim,
        }
    }
}

impl Default for PipelineBuilder {
    fn default() -> Self {
        Self::new()
    }
}
