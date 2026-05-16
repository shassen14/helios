// helios_sim/src/simulation/registry/contexts.rs
//
// Factory type aliases and build context structs for the AutonomyRegistry.
// Spawning systems construct these from Bevy query results and pass them to registry.build_*().

use std::collections::HashMap;
use std::sync::Arc;

use bevy::prelude::Entity;
use helios_core::{
    control::Controller,
    data::primitives::FrameHandle,
    estimation::StateEstimator,
    estimation::{dynamics::EstimationDynamics, measurement::Measurement},
    mapping::Mapper,
    path_following::PathFollower,
    planning::SearchPlanner,
};

use crate::simulation::config::structs::{
    AckermannAdapterConfig, AgentConfig, ControllerConfig, EstimatorConfig, SearchPlannerConfig,
    MapLayerConfig, PathFollowingConfig,
};
use crate::simulation::plugins::vehicles::ackermann::adapter::AckermannOutputAdapter;

// =========================================================================
// == Factory Type Aliases ==
// Arc (not Box) so the dynamics map can be cheaply cloned into EstimatorBuildContext.
// =========================================================================

pub type DynamicsFactory =
    Arc<dyn Fn(DynamicsBuildContext) -> Result<Box<dyn EstimationDynamics>, String> + Send + Sync>;

pub type EstimatorFactory =
    Arc<dyn Fn(EstimatorBuildContext) -> Result<Box<dyn StateEstimator>, String> + Send + Sync>;

pub type MapperFactory =
    Arc<dyn Fn(MapperBuildContext) -> Result<Box<dyn Mapper>, String> + Send + Sync>;

pub type ControllerFactory =
    Arc<dyn Fn(ControllerBuildContext) -> Result<Box<dyn Controller>, String> + Send + Sync>;

pub type PlannerFactory =
    Arc<dyn Fn(PlannerBuildContext) -> Result<Box<dyn SearchPlanner>, String> + Send + Sync>;

pub type AdapterFactory = Arc<
    dyn Fn(AdapterBuildContext) -> Result<Box<dyn AckermannOutputAdapter>, String> + Send + Sync,
>;

pub type PathFollowerFactory =
    Arc<dyn Fn(PathFollowerBuildContext) -> Result<Box<dyn PathFollower>, String> + Send + Sync>;

// =========================================================================
// == Build Contexts ==
// Owned structs passed by value to factory closures.
// =========================================================================

pub struct DynamicsBuildContext {
    pub agent_entity: Entity,
    pub agent_config: AgentConfig,
    pub gravity_magnitude: f64,
}

pub struct EstimatorBuildContext {
    pub agent_entity: Entity,
    pub estimator_cfg: EstimatorConfig,
    pub agent_config: AgentConfig,
    pub gravity_magnitude: f64,
    pub measurement_models: HashMap<FrameHandle, Box<dyn Measurement>>,
    pub dynamics_factories: HashMap<String, DynamicsFactory>,
}

pub struct MapperBuildContext {
    pub agent_entity: Entity,
    pub map_layer_cfg: MapLayerConfig,
}

pub struct ControllerBuildContext {
    pub agent_entity: Entity,
    pub controller_cfg: ControllerConfig,
    pub agent_config: AgentConfig,
    pub dynamics_factories: Arc<HashMap<String, DynamicsFactory>>,
}

pub struct PlannerBuildContext {
    pub agent_entity: Entity,
    pub planner_cfg: SearchPlannerConfig,
    pub level: helios_runtime::stage::PipelineLevel,
}

pub struct AdapterBuildContext {
    pub agent_entity: Entity,
    pub adapter_cfg: AckermannAdapterConfig,
}

pub struct PathFollowerBuildContext {
    pub agent_entity: Entity,
    pub path_following_cfg: PathFollowingConfig,
}
