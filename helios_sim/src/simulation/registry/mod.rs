// helios_sim/src/simulation/registry/mod.rs
//
// Submodules
pub mod dynamics;
pub mod estimators;
pub mod mappers;
pub mod plugin;
pub mod slam;
//
// The AutonomyRegistry maps config-string keys to factory closures that produce
// algorithm trait objects. Spawning systems never name concrete types — they call
// registry.build_*(key, ctx) and receive a Box<dyn Trait>.
//
// Adding a new algorithm:
//   1. Implement the trait in helios_core.
//   2. Add a registration call in the appropriate Default*Plugin (e.g., DefaultEstimatorsPlugin).
//   3. Done. No spawning systems change.

use std::collections::HashMap;
use std::sync::Arc;

use bevy::prelude::{Entity, Resource};
use helios_core::{
    estimation::StateEstimator,
    mapping::Mapper,
    models::estimation::{dynamics::EstimationDynamics, measurement::Measurement},
    slam::SlamSystem,
    types::FrameHandle,
};

use crate::simulation::config::structs::{AgentConfig, EstimatorConfig, MapperConfig, SlamConfig};

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

pub type SlamFactory =
    Arc<dyn Fn(SlamBuildContext) -> Result<Box<dyn SlamSystem>, String> + Send + Sync>;

// =========================================================================
// == Build Contexts ==
// Owned structs passed by value to factory closures. The spawning system
// constructs them from Bevy query results + config data.
// =========================================================================

/// Context for constructing a dynamics model (e.g., IntegratedImuModel).
pub struct DynamicsBuildContext {
    pub agent_entity: Entity,
    /// Full agent config — dynamics models that depend on vehicle geometry (e.g., Ackermann)
    /// read wheelbase and similar parameters from here.
    pub agent_config: AgentConfig,
    pub gravity_magnitude: f64,
}

/// Context for constructing a state estimator (e.g., ExtendedKalmanFilter).
/// Includes a snapshot of all registered dynamics factories taken at spawn time.
/// Because all Plugin::build() calls complete before OnEnter(SceneBuilding) runs,
/// the snapshot contains every registered dynamics factory — no ordering constraints needed.
pub struct EstimatorBuildContext {
    pub agent_entity: Entity,
    pub estimator_cfg: EstimatorConfig,
    /// Full agent config forwarded to dynamics factories that need vehicle params.
    pub agent_config: AgentConfig,
    pub gravity_magnitude: f64,
    /// Pre-built measurement models from the agent's sensor children (already ECS-queried).
    pub measurement_models: HashMap<FrameHandle, Box<dyn Measurement>>,
    /// Snapshot of registry.dynamics at spawn time.
    pub dynamics_factories: HashMap<String, DynamicsFactory>,
}

/// Context for constructing a mapper (e.g., OccupancyGridMapper).
pub struct MapperBuildContext {
    pub agent_entity: Entity,
    pub mapper_cfg: MapperConfig,
}

/// Context for constructing a unified SLAM system (e.g., EkfSlam).
pub struct SlamBuildContext {
    pub agent_entity: Entity,
    pub slam_cfg: SlamConfig,
    pub agent_config: AgentConfig,
    pub gravity_magnitude: f64,
    pub measurement_models: HashMap<FrameHandle, Box<dyn Measurement>>,
    pub dynamics_factories: HashMap<String, DynamicsFactory>,
}

// =========================================================================
// == AutonomyRegistry ==
// =========================================================================

#[derive(Resource, Default)]
pub struct AutonomyRegistry {
    pub estimators: HashMap<String, EstimatorFactory>,
    pub dynamics: HashMap<String, DynamicsFactory>,
    pub mappers: HashMap<String, MapperFactory>,
    pub slam: HashMap<String, SlamFactory>,
    // controllers: HashMap<String, ControllerFactory>
    //   — slot reserved, add when Controller trait is defined in helios_core
}

impl AutonomyRegistry {
    // --- Registration ---

    pub fn register_estimator<F>(&mut self, key: &str, factory: F) -> &mut Self
    where
        F: Fn(EstimatorBuildContext) -> Result<Box<dyn StateEstimator>, String>
            + Send
            + Sync
            + 'static,
    {
        self.estimators.insert(key.to_string(), Arc::new(factory));
        self
    }

    pub fn register_dynamics<F>(&mut self, key: &str, factory: F) -> &mut Self
    where
        F: Fn(DynamicsBuildContext) -> Result<Box<dyn EstimationDynamics>, String>
            + Send
            + Sync
            + 'static,
    {
        self.dynamics.insert(key.to_string(), Arc::new(factory));
        self
    }

    pub fn register_mapper<F>(&mut self, key: &str, factory: F) -> &mut Self
    where
        F: Fn(MapperBuildContext) -> Result<Box<dyn Mapper>, String> + Send + Sync + 'static,
    {
        self.mappers.insert(key.to_string(), Arc::new(factory));
        self
    }

    pub fn register_slam<F>(&mut self, key: &str, factory: F) -> &mut Self
    where
        F: Fn(SlamBuildContext) -> Result<Box<dyn SlamSystem>, String> + Send + Sync + 'static,
    {
        self.slam.insert(key.to_string(), Arc::new(factory));
        self
    }

    // --- Construction ---

    pub fn build_estimator(
        &self,
        key: &str,
        ctx: EstimatorBuildContext,
    ) -> Result<Box<dyn StateEstimator>, String> {
        let factory = self
            .estimators
            .get(key)
            .ok_or_else(|| format!("No estimator registered for '{key}'. Call register_estimator()."))?;
        factory(ctx)
    }

    pub fn build_dynamics(
        &self,
        key: &str,
        ctx: DynamicsBuildContext,
    ) -> Result<Box<dyn EstimationDynamics>, String> {
        let factory = self
            .dynamics
            .get(key)
            .ok_or_else(|| format!("No dynamics registered for '{key}'. Call register_dynamics()."))?;
        factory(ctx)
    }

    pub fn build_mapper(
        &self,
        key: &str,
        ctx: MapperBuildContext,
    ) -> Result<Box<dyn Mapper>, String> {
        let factory = self
            .mappers
            .get(key)
            .ok_or_else(|| format!("No mapper registered for '{key}'. Call register_mapper()."))?;
        factory(ctx)
    }

    pub fn build_slam(
        &self,
        key: &str,
        ctx: SlamBuildContext,
    ) -> Result<Box<dyn SlamSystem>, String> {
        let factory = self
            .slam
            .get(key)
            .ok_or_else(|| format!("No SLAM system registered for '{key}'. Call register_slam()."))?;
        factory(ctx)
    }

    /// Returns a cheap clone of the dynamics factory map for embedding in EstimatorBuildContext.
    /// Arc internals make this O(n) in key count, not in factory size.
    pub fn clone_dynamics(&self) -> HashMap<String, DynamicsFactory> {
        self.dynamics.clone()
    }
}
