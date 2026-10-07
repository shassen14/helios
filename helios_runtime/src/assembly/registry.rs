//! `AutonomyRegistry` — portable factory registry for autonomy pipeline nodes.
//!
//! Lives in `helios_runtime` so the same factories produce the same pipeline
//! in simulation and on hardware. `helios_sim` wraps this in a thin Bevy
//! `Resource`; `helios_hw` will construct it at startup from a config file.
//!
//! ## How it works
//!
//! One map takes a node kind to its factory:
//! - `register_node(kind, build)` — adds a kind; `build` is a plain function
//!   from the kind's typed config to a node.
//! - `build_node(kind, section, ctx)` — looks up the kind and builds the node
//!   from its TOML section.
//!
//! The per-family maps, with a `register_<family>` / `build_<family>` pair
//! each, still hold every kind not yet moved onto the one map.
//!
//! `AutonomyRegistry::default()` calls every sub-module's `register()` so all
//! built-in algorithms are available without any manual registration.
//!
//! ## Extension
//!
//! Register a custom node kind on an existing registry:
//! ```ignore
//! registry.register_node("MyPid", build_my_pid)?;
//! ```

use super::contexts::{
    AllocatorBuildContext, ControllerBuildContext, GaussianEstimatorBuildContext,
    MeasurementModelBuildContext, MockEstimatorBuildContext, PathFollowerBuildContext,
};

use super::error::PipelineAssemblyError;
use super::factory::{erase, BuildContext, BuiltNode, ErasedFactory, FactoryOutput};

use crate::config::EstimatorConfig;
use crate::pipeline::node::PipelineNode;
use crate::validation::CapabilitySet;

use helios_core::estimation::measurement::MeasurementModel;
use serde::de::DeserializeOwned;
use serde::Serialize;

use std::collections::{BTreeMap, HashMap};
use std::error::Error;
use std::fmt::Display;

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

type ControllerFactory =
    Box<dyn Fn(ControllerBuildContext) -> Result<Box<dyn PipelineNode>, String> + Send + Sync>;

type AllocatorFactory =
    Box<dyn Fn(AllocatorBuildContext) -> Result<Box<dyn PipelineNode>, String> + Send + Sync>;

type PathFollowerFactory =
    Box<dyn Fn(PathFollowerBuildContext) -> Result<Box<dyn PipelineNode>, String> + Send + Sync>;

type MockEstimatorFactory = Box<
    dyn Fn(EstimatorConfig, MockEstimatorBuildContext) -> Result<Box<dyn PipelineNode>, String>
        + Send
        + Sync,
>;

/// Portable factory registry for autonomy pipeline nodes.
///
/// Keys match the `kind` strings used in TOML config (e.g. `"Ekf"`, `"AStar"`,
/// `"PurePursuit"`).
pub struct AutonomyRegistry {
    // Node kind → factory. Sorted, so an unknown-kind error lists the
    // registered kinds in a stable order.
    nodes: BTreeMap<String, ErasedFactory>,
    measurement_models: HashMap<String, MeasurementModelFactory>,
    gaussian_estimators: HashMap<String, GaussianEstimatorFactory>,
    controllers: HashMap<String, ControllerFactory>,
    allocators: HashMap<String, AllocatorFactory>,
    path_followers: HashMap<String, PathFollowerFactory>,
    // mocks
    mock_estimators: HashMap<String, MockEstimatorFactory>,
}

impl Default for AutonomyRegistry {
    fn default() -> Self {
        let mut registry = Self {
            nodes: BTreeMap::new(),
            measurement_models: HashMap::new(),
            gaussian_estimators: HashMap::new(),
            controllers: HashMap::new(),
            allocators: HashMap::new(),
            path_followers: HashMap::new(),
            mock_estimators: HashMap::new(),
        };
        // Registration order: leaf dependencies before composites.
        crate::nodes::gaussian_estimator::register(&mut registry);
        crate::nodes::occupancy_grid::register(&mut registry);
        crate::nodes::controller::register(&mut registry);
        crate::nodes::planner::register(&mut registry);
        crate::nodes::path_follower::register(&mut registry);
        crate::nodes::mocks::register(&mut registry);
        crate::nodes::allocator::register(&mut registry);
        crate::nodes::deproject::register(&mut registry);
        registry
    }
}

impl AutonomyRegistry {
    // --- Registration ---

    /// Registers a node kind under `kind`, the string a `[nodes.<name>]` table
    /// names in its `kind` key.
    ///
    /// `build` takes the kind's own typed config and the build context and
    /// returns the node. The registry handles everything around it: parsing the
    /// TOML section into `C`, rejecting unknown keys, recording the resolved
    /// section for the config dump, and naming the node and kind in every
    /// error. `C` must carry `#[serde(deny_unknown_fields)]` for a misspelled
    /// key to be an error rather than silently ignored.
    ///
    /// Fails if `kind` is already registered; the first registration is kept.
    pub fn register_node<C, F>(
        &mut self,
        kind: impl Into<String>,
        build: F,
    ) -> Result<(), DuplicateKind>
    where
        C: DeserializeOwned + Serialize + 'static,
        F: Fn(C, &BuildContext<'_>) -> Result<FactoryOutput, String> + Send + Sync + 'static,
    {
        let kind = kind.into();
        if self.nodes.contains_key(&kind) {
            return Err(DuplicateKind { kind });
        }

        self.nodes.insert(kind.clone(), erase(kind, build));

        Ok(())
    }

    pub(crate) fn register_measurement_model(
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

    pub(crate) fn register_gaussian_estimator(
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

    pub(crate) fn register_controller(
        &mut self,
        key: impl Into<String>,
        factory: impl Fn(ControllerBuildContext) -> Result<Box<dyn PipelineNode>, String>
            + Send
            + Sync
            + 'static,
    ) {
        self.controllers.insert(key.into(), Box::new(factory));
    }

    pub(crate) fn register_allocator(
        &mut self,
        key: impl Into<String>,
        factory: impl Fn(AllocatorBuildContext) -> Result<Box<dyn PipelineNode>, String>
            + Send
            + Sync
            + 'static,
    ) {
        self.allocators.insert(key.into(), Box::new(factory));
    }

    pub(crate) fn register_path_follower(
        &mut self,
        key: impl Into<String>,
        factory: impl Fn(PathFollowerBuildContext) -> Result<Box<dyn PipelineNode>, String>
            + Send
            + Sync
            + 'static,
    ) {
        self.path_followers.insert(key.into(), Box::new(factory));
    }

    pub(crate) fn register_mock_estimator(
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

    /// Builds the node `ctx.node_name()` with the factory registered for
    /// `kind`, from its config `section` with the common keys removed.
    ///
    /// Fails with [`PipelineAssemblyError::UnknownNodeKind`], listing every
    /// registered kind, if no factory is registered for `kind`, or with
    /// [`PipelineAssemblyError::Factory`] if the factory rejects the section
    /// or fails to build.
    pub(crate) fn build_node(
        &self,
        kind: &str,
        section: toml::Table,
        ctx: &BuildContext<'_>,
    ) -> Result<BuiltNode, PipelineAssemblyError> {
        let Some(factory) = self.nodes.get(kind) else {
            return Err(PipelineAssemblyError::UnknownNodeKind {
                node_name: ctx.node_name().to_string(),
                kind: kind.to_string(),
                registered: self.nodes.keys().cloned().collect(),
            });
        };

        Ok(factory(section, ctx)?)
    }

    pub(crate) fn build_measurement_model(
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

    pub(crate) fn build_gaussian_estimator(
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

    pub(crate) fn build_controller(
        &self,
        key: &str,
        ctx: ControllerBuildContext,
    ) -> Result<Box<dyn PipelineNode>, String> {
        self.controllers
            .get(key)
            .ok_or_else(|| format!("No controller factory registered for '{key}'"))?(ctx)
    }

    pub(crate) fn build_allocator(
        &self,
        key: &str,
        ctx: AllocatorBuildContext,
    ) -> Result<Box<dyn PipelineNode>, String> {
        self.allocators
            .get(key)
            .ok_or_else(|| format!("No allocator factory registered for '{key}'"))?(ctx)
    }

    pub(crate) fn build_path_follower(
        &self,
        key: &str,
        ctx: PathFollowerBuildContext,
    ) -> Result<Box<dyn PipelineNode>, String> {
        self.path_followers
            .get(key)
            .ok_or_else(|| format!("No path follower factory registered for '{key}'"))?(ctx)
    }

    pub(crate) fn build_mock_estimator(
        &self,
        key: &str,
        config: EstimatorConfig,
        ctx: MockEstimatorBuildContext,
    ) -> Result<Box<dyn PipelineNode>, String> {
        self.mock_estimators
            .get(key)
            .ok_or_else(|| format!("No mock estimator factory registered for '{key}'"))?(
            config, ctx,
        )
    }

    /// Snapshot of all registered keys per family, for `validate_autonomy_config`.
    pub fn capabilities(&self) -> CapabilitySet {
        CapabilitySet {
            gaussian_estimators: self.gaussian_estimators.keys().cloned().collect(),
            mock_estimators: self.mock_estimators.keys().cloned().collect(),
            measurement_models: self.measurement_models.keys().cloned().collect(),
            controllers: self.controllers.keys().cloned().collect(),
            allocators: self.allocators.keys().cloned().collect(),
        }
    }
}

/// A node kind was registered twice.
#[derive(Debug)]
pub struct DuplicateKind {
    /// The kind string that was already taken.
    pub kind: String,
}

impl Display for DuplicateKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "node kind '{}' is already registered", self.kind)
    }
}

impl Error for DuplicateKind {}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assembly::factory::FactoryError;
    use crate::pipeline::node::TickContext;
    use crate::port::{AlgorithmNodePortDescriptor, PortBus, PortDescriptor};

    use helios_core::prelude::{AgentId, TfProvider};

    use serde::Deserialize;
    use std::collections::HashSet;

    const NODE: &str = "front_deproject";
    const KIND: &str = "Deproject";
    const OTHER_KIND: &str = "Accumulate";
    const UNKNOWN_KIND: &str = "Deprojcet";
    const DEFAULT_RATE: f64 = 10.0;

    /// A node that does nothing; the registry only passes it through.
    struct StubNode {
        descriptor: PortDescriptor,
    }

    impl PipelineNode for StubNode {
        fn name(&self) -> &str {
            NODE
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {}
    }

    #[derive(Deserialize, Serialize)]
    #[serde(deny_unknown_fields)]
    struct StubConfig {
        output: String,
        #[serde(default = "default_rate")]
        rate: f64,
    }

    fn default_rate() -> f64 {
        DEFAULT_RATE
    }

    fn build_stub(_config: StubConfig, _ctx: &BuildContext<'_>) -> Result<FactoryOutput, String> {
        Ok(FactoryOutput::new(Box::new(StubNode {
            descriptor: AlgorithmNodePortDescriptor::new().build(),
        })))
    }

    /// A registry with only the given kinds on its node map, all built by
    /// [`build_stub`]. The built-in kinds are cleared so the tests don't depend
    /// on which ones exist.
    fn registry_with(kinds: &[&str]) -> AutonomyRegistry {
        let mut registry = AutonomyRegistry::default();
        registry.nodes.clear();
        for kind in kinds {
            registry
                .register_node(*kind, build_stub)
                .expect("test kinds are distinct");
        }
        registry
    }

    /// Builds node [`NODE`] of `kind` from `section`, written as TOML.
    fn build(
        registry: &AutonomyRegistry,
        kind: &str,
        section: &str,
    ) -> Result<BuiltNode, PipelineAssemblyError> {
        let section: toml::Table = toml::from_str(section).expect("test TOML parses");
        let channels = HashSet::new();
        let ctx = BuildContext::new(AgentId::new("car"), NODE, &channels);
        registry.build_node(kind, section, &ctx)
    }

    /// Unwraps the failure of [`build`]; `BuiltNode` has no `Debug`, so
    /// `unwrap_err` isn't available.
    fn build_err(registry: &AutonomyRegistry, kind: &str, section: &str) -> PipelineAssemblyError {
        match build(registry, kind, section) {
            Ok(_) => panic!("expected the build to fail"),
            Err(err) => err,
        }
    }

    /// Registering a kind twice fails and names the kind.
    #[test]
    fn registering_a_kind_twice_is_rejected() {
        let mut registry = registry_with(&[KIND]);
        let err = registry
            .register_node(KIND, build_stub)
            .expect_err("a second registration fails");
        assert_eq!(err.kind, KIND);
        assert_eq!(
            err.to_string(),
            format!("node kind '{KIND}' is already registered")
        );
    }

    /// A registered kind builds, and the resolved section shows its defaults.
    #[test]
    fn registered_kind_builds_with_its_resolved_section() {
        let registry = registry_with(&[KIND]);
        let Ok(built) = build(&registry, KIND, "output = \"cloud\"") else {
            panic!("expected the build to succeed");
        };
        assert_eq!(built.resolved["output"].as_str(), Some("cloud"));
        assert_eq!(built.resolved["rate"].as_float(), Some(DEFAULT_RATE));
    }

    /// An unknown kind names the node and lists the registered kinds, sorted
    /// regardless of registration order.
    #[test]
    fn unknown_kind_lists_registered_kinds_sorted() {
        let registry = registry_with(&[KIND, OTHER_KIND]);
        let err = build_err(&registry, UNKNOWN_KIND, "output = \"cloud\"");
        let PipelineAssemblyError::UnknownNodeKind {
            node_name,
            kind,
            registered,
        } = &err
        else {
            panic!("expected UnknownNodeKind, got {err}");
        };
        assert_eq!((node_name.as_str(), kind.as_str()), (NODE, UNKNOWN_KIND));
        assert_eq!(registered, &[OTHER_KIND, KIND]);
        assert_eq!(
            err.to_string(),
            format!(
                "nodes.{NODE}: unknown kind '{UNKNOWN_KIND}'; \
                 registered kinds: {OTHER_KIND}, {KIND}"
            )
        );
    }

    /// With nothing registered, the unknown-kind message says so rather than
    /// ending in an empty list.
    #[test]
    fn unknown_kind_with_none_registered_says_none() {
        let registry = registry_with(&[]);
        let err = build_err(&registry, UNKNOWN_KIND, "");
        assert!(
            err.to_string().ends_with("registered kinds: (none)"),
            "{err}"
        );
    }

    /// A factory's error comes out wrapped, its message unchanged.
    #[test]
    fn factory_error_passes_through_unchanged() {
        let registry = registry_with(&[KIND]);
        let err = build_err(&registry, KIND, "output = \"cloud\"\noutptu = \"x\"");
        let PipelineAssemblyError::Factory(inner) = &err else {
            panic!("expected Factory, got {err}");
        };
        assert!(
            matches!(inner, FactoryError::InvalidConfig { .. }),
            "{inner}"
        );
        assert_eq!(err.to_string(), inner.to_string());
    }
}
