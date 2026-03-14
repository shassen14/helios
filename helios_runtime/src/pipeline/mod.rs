// helios_runtime/src/pipeline/mod.rs
//
// AutonomyPipeline, PipelineBuilder, PipelineOutputs + re-exports of stage submodules.
//
// helios_sim decomposes AutonomyPipeline into ECS components via into_parts()
// so Bevy can schedule EstimationCore and MappingCore in parallel.
// helios_hw uses AutonomyPipeline as a whole unit.

//! Autonomy pipeline: three independent stages and their composite.
//!
//! The pipeline is organized into three stages that can run independently:
//!
//! | Stage | Struct | Responsibility |
//! |---|---|---|
//! | Estimation | [`EstimationCore`] | EKF/UKF predict+update, SLAM |
//! | Mapping | [`MappingCore`] | Occupancy grids, map maintenance |
//! | Control | [`ControlCore`] | Planning, look-ahead, controller dispatch |
//!
//! The composite [`AutonomyPipeline`] wraps all three for `helios_hw` convenience.
//! In `helios_sim`, [`AutonomyPipeline::into_parts`] decomposes it into three
//! Bevy `Component` structs so estimation and mapping can be scheduled in parallel.
//!
//! ## Construction
//!
//! Always use [`PipelineBuilder`]. Never construct field structs directly.
//!
//! ```rust,ignore
//! let pipeline = PipelineBuilder::new()
//!     .with_estimator(ekf)
//!     .with_mapper(PipelineLevel::Local, occ_grid)
//!     .with_controller(PipelineLevel::Local, pure_pursuit)
//!     .with_goal(PlannerGoal::WorldPose(goal))
//!     .build();
//! ```

pub mod control_core;
pub mod estimation_core;
pub mod mapping_core;

pub use control_core::ControlCore;
pub use estimation_core::EstimationCore;
pub use mapping_core::MappingCore;

use std::collections::HashMap;

use helios_core::{
    control::ControlOutput,
    frames::FrameAwareState,
    mapping::{MapData, Mapper},
    messages::MeasurementMessage,
    planning::{types::PlannerGoal, Planner},
    slam::SlamSystem,
    tracking::Tracker,
};
use nalgebra::{DVector, Isometry3};

use crate::{
    runtime::AgentRuntime,
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

/// The only construction path for [`AutonomyPipeline`].
///
/// Stages are sorted by [`PipelineLevel`] at `build()` time so that
/// `Global` always runs before `Local` and `Local` before `Custom` variants.
///
/// An empty builder (no stages registered) produces a valid, no-op pipeline —
/// useful for agents that only need a subset of capabilities.
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

use helios_core::estimation::StateEstimator;

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
        self.controllers
            .push(LeveledController { level, controller });
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
