//! `AutonomyRegistry` — portable factory registry for autonomy pipeline nodes.
//!
//! Lives in `helios_runtime` so the same factories produce the same pipeline
//! in simulation and on hardware. `helios_sim` wraps this in a thin Bevy
//! `Resource`; `helios_hw` will construct it at startup from a config file.
//!
//! ## How it works
//!
//! Each algorithm family has a pair of methods:
//! - `register_<family>(key, factory)` — adds a factory closure.
//! - `build_<family>(key, ctx)` — looks up the factory and calls it.
//!
//! `AutonomyRegistry::default()` calls every sub-module's `register()` so all
//! built-in algorithms are available without any manual registration.
//!
//! ## Extension
//!
//! Register custom algorithms by calling `register_*` on an existing registry:
//! ```ignore
//! registry.register_controller("MyPid", my_factory);
//! ```

pub mod contexts;
mod controller;
mod dynamics;
mod gaussian_estimator;
mod mapper;
mod measurement_model;
mod mock_estimator;
mod path_follower;
mod search_planner;

use contexts::{
    ControllerBuildContext, DynamicsBuildContext, GaussianEstimatorBuildContext,
    MapperBuildContext, MeasurementModelBuildContext, MockEstimatorBuildContext,
    PathFollowerBuildContext, SearchPlannerBuildContext,
};

use crate::config::EstimatorConfig;
use crate::pipeline::node::PipelineNode;
use crate::validation::CapabilitySet;

use helios_core::estimation::dynamics::EstimationDynamics;
use helios_core::estimation::measurement::MeasurementModel;

use std::collections::HashMap;

type DynamicsFactory =
    Box<dyn Fn(DynamicsBuildContext) -> Result<Box<dyn EstimationDynamics>, String> + Send + Sync>;

type MeasurementModelFactory = Box<
    dyn Fn(MeasurementModelBuildContext) -> Result<Box<dyn MeasurementModel>, String> + Send + Sync,
>;

type GaussianEstimatorFactory = Box<
    dyn Fn(
            EstimatorConfig,
            GaussianEstimatorBuildContext,
            &AutonomyRegistry,
        ) -> Result<Box<dyn PipelineNode>, String>
        + Send
        + Sync,
>;

type MapperFactory =
    Box<dyn Fn(MapperBuildContext) -> Result<Box<dyn PipelineNode>, String> + Send + Sync>;

type ControllerFactory =
    Box<dyn Fn(ControllerBuildContext) -> Result<Box<dyn PipelineNode>, String> + Send + Sync>;

type SearchPlannerFactory =
    Box<dyn Fn(SearchPlannerBuildContext) -> Result<Box<dyn PipelineNode>, String> + Send + Sync>;

type PathFollowerFactory =
    Box<dyn Fn(PathFollowerBuildContext) -> Result<Box<dyn PipelineNode>, String> + Send + Sync>;

type MockEstimatorFactory = Box<
    dyn Fn(EstimatorConfig, MockEstimatorBuildContext) -> Result<Box<dyn PipelineNode>, String>
        + Send
        + Sync,
>;

/// Portable factory registry for autonomy pipeline nodes.
///
/// All `register_*` methods accept a `key: impl Into<String>` and a factory
/// function. Keys match the `kind` strings used in TOML config (e.g. `"Ekf"`,
/// `"AStar"`, `"PurePursuit"`).
pub struct AutonomyRegistry {
    dynamics: HashMap<String, DynamicsFactory>,
    measurement_models: HashMap<String, MeasurementModelFactory>,
    gaussian_estimators: HashMap<String, GaussianEstimatorFactory>,
    mappers: HashMap<String, MapperFactory>,
    controllers: HashMap<String, ControllerFactory>,
    search_planners: HashMap<String, SearchPlannerFactory>,
    path_followers: HashMap<String, PathFollowerFactory>,
    // mocks
    mock_estimators: HashMap<String, MockEstimatorFactory>,
}

impl Default for AutonomyRegistry {
    fn default() -> Self {
        let mut registry = Self {
            dynamics: HashMap::new(),
            measurement_models: HashMap::new(),
            gaussian_estimators: HashMap::new(),
            mappers: HashMap::new(),
            controllers: HashMap::new(),
            search_planners: HashMap::new(),
            path_followers: HashMap::new(),
            mock_estimators: HashMap::new(),
        };
        // Registration order: leaf dependencies before composites.
        dynamics::register(&mut registry);
        measurement_model::register(&mut registry);
        gaussian_estimator::register(&mut registry);
        mapper::register(&mut registry);
        controller::register(&mut registry);
        search_planner::register(&mut registry);
        path_follower::register(&mut registry);
        mock_estimator::register(&mut registry);
        registry
    }
}

impl AutonomyRegistry {
    // --- Registration ---

    pub fn register_dynamics(
        &mut self,
        key: impl Into<String>,
        factory: impl Fn(DynamicsBuildContext) -> Result<Box<dyn EstimationDynamics>, String>
            + Send
            + Sync
            + 'static,
    ) {
        self.dynamics.insert(key.into(), Box::new(factory));
    }

    pub fn register_measurement_model(
        &mut self,
        key: impl Into<String>,
        factory: impl Fn(MeasurementModelBuildContext) -> Result<Box<dyn MeasurementModel>, String>
            + Send
            + Sync
            + 'static,
    ) {
        self.measurement_models
            .insert(key.into(), Box::new(factory));
    }

    pub fn register_gaussian_estimator(
        &mut self,
        key: impl Into<String>,
        factory: impl Fn(
                EstimatorConfig,
                GaussianEstimatorBuildContext,
                &AutonomyRegistry,
            ) -> Result<Box<dyn PipelineNode>, String>
            + Send
            + Sync
            + 'static,
    ) {
        self.gaussian_estimators
            .insert(key.into(), Box::new(factory));
    }

    pub fn register_mapper(
        &mut self,
        key: impl Into<String>,
        factory: impl Fn(MapperBuildContext) -> Result<Box<dyn PipelineNode>, String>
            + Send
            + Sync
            + 'static,
    ) {
        self.mappers.insert(key.into(), Box::new(factory));
    }

    pub fn register_controller(
        &mut self,
        key: impl Into<String>,
        factory: impl Fn(ControllerBuildContext) -> Result<Box<dyn PipelineNode>, String>
            + Send
            + Sync
            + 'static,
    ) {
        self.controllers.insert(key.into(), Box::new(factory));
    }

    pub fn register_search_planner(
        &mut self,
        key: impl Into<String>,
        factory: impl Fn(SearchPlannerBuildContext) -> Result<Box<dyn PipelineNode>, String>
            + Send
            + Sync
            + 'static,
    ) {
        self.search_planners.insert(key.into(), Box::new(factory));
    }

    pub fn register_path_follower(
        &mut self,
        key: impl Into<String>,
        factory: impl Fn(PathFollowerBuildContext) -> Result<Box<dyn PipelineNode>, String>
            + Send
            + Sync
            + 'static,
    ) {
        self.path_followers.insert(key.into(), Box::new(factory));
    }

    pub fn register_mock_estimator(
        &mut self,
        key: impl Into<String>,
        factory: impl Fn(EstimatorConfig, MockEstimatorBuildContext) -> Result<Box<dyn PipelineNode>, String>
            + Send
            + Sync
            + 'static,
    ) {
        self.mock_estimators.insert(key.into(), Box::new(factory));
    }

    // --- Build ---

    pub fn build_dynamics(
        &self,
        key: &str,
        ctx: DynamicsBuildContext,
    ) -> Result<Box<dyn EstimationDynamics>, String> {
        self.dynamics
            .get(key)
            .ok_or_else(|| format!("No dynamics factory registered for '{key}'"))?(ctx)
    }

    pub fn build_measurement_model(
        &self,
        key: &str,
        ctx: MeasurementModelBuildContext,
    ) -> Result<Box<dyn MeasurementModel>, String> {
        self.measurement_models
            .get(key)
            .ok_or_else(|| format!("No measurement model factory registered for '{key}'"))?(
            ctx
        )
    }

    pub fn build_gaussian_estimator(
        &self,
        key: &str,
        config: EstimatorConfig,
        ctx: GaussianEstimatorBuildContext,
    ) -> Result<Box<dyn PipelineNode>, String> {
        self.gaussian_estimators
            .get(key)
            .ok_or_else(|| format!("No Gaussian estimator factory registered for '{key}'"))?(
            config, ctx, self,
        )
    }

    pub fn build_mapper(
        &self,
        key: &str,
        ctx: MapperBuildContext,
    ) -> Result<Box<dyn PipelineNode>, String> {
        self.mappers
            .get(key)
            .ok_or_else(|| format!("No mapper factory registered for '{key}'"))?(ctx)
    }

    pub fn build_controller(
        &self,
        key: &str,
        ctx: ControllerBuildContext,
    ) -> Result<Box<dyn PipelineNode>, String> {
        self.controllers
            .get(key)
            .ok_or_else(|| format!("No controller factory registered for '{key}'"))?(ctx)
    }

    pub fn build_search_planner(
        &self,
        key: &str,
        ctx: SearchPlannerBuildContext,
    ) -> Result<Box<dyn PipelineNode>, String> {
        self.search_planners
            .get(key)
            .ok_or_else(|| format!("No search planner factory registered for '{key}'"))?(ctx)
    }

    pub fn build_path_follower(
        &self,
        key: &str,
        ctx: PathFollowerBuildContext,
    ) -> Result<Box<dyn PipelineNode>, String> {
        self.path_followers
            .get(key)
            .ok_or_else(|| format!("No path follower factory registered for '{key}'"))?(ctx)
    }

    pub fn build_mock_estimator(
        &self,
        key: &str,
        config: EstimatorConfig,
        ctx: MockEstimatorBuildContext,
    ) -> Result<Box<dyn PipelineNode>, String> {
        self.mock_estimators.get(key).ok_or_else(|| {
            format!(
                "No mock estimator factory
  registered for '{key}'"
            )
        })?(config, ctx)
    }

    /// Snapshot of all registered keys per family, for `validate_autonomy_config`.
    pub fn capabilities(&self) -> CapabilitySet {
        CapabilitySet {
            gaussian_estimators: self.gaussian_estimators.keys().cloned().collect(),
            dynamics: self.dynamics.keys().cloned().collect(),
            measurement_models: self.measurement_models.keys().cloned().collect(),
            mappers: self.mappers.keys().cloned().collect(),
            controllers: self.controllers.keys().cloned().collect(),
            planners: self.search_planners.keys().cloned().collect(),
        }
    }
}
