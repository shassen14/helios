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
//! - `register_node_with::<T>(kind, build)` — adds a kind whose `build` also
//!   receives the extension `T` it declares; declaring `T` creates it.
//! - `build_node(kind, section, ctx)` — looks up the kind and builds the node
//!   from its TOML section.
//!
//! Every other table lives in an **extension**, one value per type, owned by
//! the concept that gives the table meaning. The registry stores extensions
//! without knowing what they hold:
//! - `extension_mut::<T>()` — the extension of type `T`, added empty if
//!   absent; its own methods register into it.
//! - `extension::<T>()` — the extension of type `T`, for a pass or factory to
//!   look kinds up in.
//!
//! The built-in extensions are the command seam's `CommandTypes` and the
//! estimator's `EstimatorComponents`. An outside crate adds its own the same
//! way, without a change here.
//!
//! `AutonomyRegistry::default()` calls every sub-module's `register()` so all
//! built-in algorithms are available without any manual registration.
//!
//! ## Extension
//!
//! Register a custom node kind, command type or estimator filter on an
//! existing registry:
//! ```ignore
//! registry.register_node("MyPid", build_my_pid)?;
//! registry.extension_mut::<CommandTypes>().register::<Thrust>("Thrust")?;
//! registry
//!     .extension_mut::<EstimatorComponents>()
//!     .register_filter("IteratedEkf", build_iterated_ekf)?;
//! ```

use super::error::PipelineAssemblyError;
use super::factory::{
    erase, erase_with, BuildContext, BuildFailure, BuiltNode, ErasedFactory, FactoryOutput,
};

use serde::de::DeserializeOwned;
use serde::Serialize;

use std::any::{Any, TypeId};
use std::collections::{BTreeMap, HashMap};
use std::error::Error;
use std::fmt::Display;

/// Portable factory registry for autonomy pipeline nodes.
///
/// Keys match the `kind` strings used in TOML config (e.g. `"RecursiveEstimator"`, `"AStar"`,
/// `"PurePursuit"`).
pub struct AutonomyRegistry {
    // Node kind → factory. Sorted, so an unknown-kind error lists the
    // registered kinds in a stable order.
    nodes: BTreeMap<String, ErasedFactory>,
    // Extension type → the one value of that type. Each is a table owned by
    // the concept that defines it; the registry never looks inside.
    extensions: HashMap<TypeId, Box<dyn Any + Send + Sync>>,
}

impl Default for AutonomyRegistry {
    fn default() -> Self {
        let mut registry = Self::empty();
        // Registration order: leaf dependencies before composites.
        crate::nodes::estimation::register(&mut registry);
        crate::nodes::occupancy_grid::register(&mut registry);
        crate::nodes::controller::register(&mut registry);
        crate::nodes::planner::register(&mut registry);
        crate::nodes::path_follower::register(&mut registry);
        crate::nodes::teleop::register(&mut registry);
        crate::nodes::mocks::register(&mut registry);
        crate::nodes::allocator::register(&mut registry);
        crate::nodes::deproject::register(&mut registry);
        crate::nodes::recursive_estimator::register(&mut registry);
        super::seams::command::register(&mut registry);
        registry
    }
}

impl AutonomyRegistry {
    /// A registry with no kinds and no extensions. Built-ins are added by
    /// `default()`; tests use this to see what an absent table looks like.
    pub(crate) fn empty() -> Self {
        Self {
            nodes: BTreeMap::new(),
            extensions: HashMap::new(),
        }
    }

    // --- Registration ---

    /// Registers a node kind under `kind`, the string a `[nodes.<name>]` table
    /// names in its `kind` key.
    ///
    /// `build` takes the kind's own typed config and the build context and
    /// returns the node. The registry handles everything around it: parsing the
    /// TOML section into `C`, rejecting unknown keys, recording the resolved
    /// section for the config dump, and naming the node and kind in every
    /// error. `C` must carry `#[serde(deny_unknown_fields)]` for a misspelled
    /// key to be an error rather than silently ignored. `build` fails with a
    /// `String` reason, or with a [`BuildFailure`] when it builds components.
    ///
    /// Fails if `kind` is already registered; the first registration is kept.
    pub fn register_node<C, E, F>(
        &mut self,
        kind: impl Into<String>,
        build: F,
    ) -> Result<(), DuplicateKind>
    where
        C: DeserializeOwned + Serialize + 'static,
        E: Into<BuildFailure>,
        F: Fn(C, &BuildContext<'_>) -> Result<FactoryOutput, E> + Send + Sync + 'static,
    {
        let kind = kind.into();
        if self.nodes.contains_key(&kind) {
            return Err(DuplicateKind { kind });
        }

        self.nodes.insert(kind.clone(), erase(kind, build));

        Ok(())
    }

    /// Registers a node kind whose build function draws on the extension `T`,
    /// such as the component tables an estimator is assembled from.
    ///
    /// As [`register_node`](Self::register_node), but `build` also receives
    /// `&T`, looked up at each build so kinds added to `T` later are seen.
    /// Declaring `T` here adds it as `T::default()` if absent, so the table a
    /// kind declares always exists: an empty one reads as "none registered".
    ///
    /// Fails if `kind` is already registered; the first registration is kept,
    /// and `T` is not added.
    pub fn register_node_with<T, C, E, F>(
        &mut self,
        kind: impl Into<String>,
        build: F,
    ) -> Result<(), DuplicateKind>
    where
        T: Any + Send + Sync + Default,
        C: DeserializeOwned + Serialize + 'static,
        E: Into<BuildFailure>,
        F: Fn(C, &BuildContext<'_>, &T) -> Result<FactoryOutput, E> + Send + Sync + 'static,
    {
        let kind = kind.into();
        if self.nodes.contains_key(&kind) {
            return Err(DuplicateKind { kind });
        }

        self.extension_mut::<T>();
        self.nodes
            .insert(kind.clone(), erase_with::<T, _, _, _>(kind, build));

        Ok(())
    }

    // --- Extensions ---

    /// The extension of type `T`, or `None` if nothing has added one.
    ///
    /// The default registry adds every built-in extension, so a built-in pass
    /// finds its table here. `None` means a registry built without it, which
    /// reads the same as an empty table. A factory doesn't call this: it
    /// declares its extension through
    /// [`register_node_with`](Self::register_node_with) and receives it.
    pub fn extension<T>(&self) -> Option<&T>
    where
        T: Any + Send + Sync,
    {
        self.extensions
            .get(&TypeId::of::<T>())
            .and_then(|extension| extension.downcast_ref::<T>())
    }

    /// The extension of type `T`, added as `T::default()` first if absent.
    ///
    /// Registration goes through the extension's own methods, so the registry
    /// stays the same whatever tables a crate adds.
    pub fn extension_mut<T>(&mut self) -> &mut T
    where
        T: Any + Send + Sync + Default,
    {
        self.extensions
            .entry(TypeId::of::<T>())
            .or_insert_with(|| Box::new(T::default()))
            .downcast_mut::<T>()
            .expect("an extension is stored under its own TypeId")
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

        Ok(factory(section, ctx, self)?)
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
    use crate::assembly::test_stub::stub;

    use helios_core::prelude::AgentId;

    use serde::Deserialize;
    use std::collections::HashSet;

    const NODE: &str = "front_deproject";
    const KIND: &str = "Deproject";
    const OTHER_KIND: &str = "Accumulate";
    const UNKNOWN_KIND: &str = "Deprojcet";
    const DEFAULT_RATE: f64 = 10.0;

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
        Ok(FactoryOutput::new(stub(NODE, vec![])))
    }

    /// A registry with only the given kinds on its node map, all built by
    /// [`build_stub`]. Starts empty so the tests don't depend on which
    /// built-in kinds exist.
    fn registry_with(kinds: &[&str]) -> AutonomyRegistry {
        let mut registry = AutonomyRegistry::empty();
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

    /// Stands in for a table an outside crate keeps on the registry.
    #[derive(Default)]
    struct Tally(Vec<&'static str>);

    /// A second extension type, to show the two don't share a slot.
    #[derive(Default)]
    struct OtherTally(Vec<&'static str>);

    #[test]
    fn an_extension_is_absent_until_first_touched() {
        let mut registry = AutonomyRegistry::empty();
        assert!(registry.extension::<Tally>().is_none());

        registry.extension_mut::<Tally>().0.push("first");

        assert_eq!(
            registry
                .extension::<Tally>()
                .map(|tally| tally.0.as_slice()),
            Some(["first"].as_slice())
        );
    }

    #[test]
    fn extension_mut_keeps_what_was_added_before() {
        let mut registry = AutonomyRegistry::empty();
        registry.extension_mut::<Tally>().0.push("first");
        registry.extension_mut::<Tally>().0.push("second");

        assert_eq!(
            registry.extension::<Tally>().map(|tally| tally.0.len()),
            Some(2)
        );
    }

    #[test]
    fn extensions_are_keyed_by_type() {
        let mut registry = AutonomyRegistry::empty();
        registry.extension_mut::<Tally>().0.push("tally");

        assert!(registry.extension::<OtherTally>().is_none());
        registry.extension_mut::<OtherTally>().0.push("other");
        assert_eq!(
            registry
                .extension::<Tally>()
                .map(|tally| tally.0.as_slice()),
            Some(["tally"].as_slice())
        );
    }

    /// Fails with the extension's contents as the reason, so a test can see
    /// which [`Tally`] the build received.
    fn build_reporting_tally(
        _config: StubConfig,
        _ctx: &BuildContext<'_>,
        tally: &Tally,
    ) -> Result<FactoryOutput, String> {
        Err(format!("tally holds {:?}", tally.0))
    }

    /// Declaring an extension creates it, so the kind's table exists even
    /// when nothing has registered into it.
    #[test]
    fn declaring_an_extension_creates_it() {
        let mut registry = AutonomyRegistry::empty();
        registry
            .register_node_with::<Tally, _, _, _>(KIND, build_reporting_tally)
            .expect("one registration");

        assert_eq!(
            registry.extension::<Tally>().map(|tally| tally.0.len()),
            Some(0)
        );
    }

    /// A rejected registration leaves the registry as it was: the extension
    /// it declared is not added.
    #[test]
    fn a_duplicate_declaring_kind_adds_no_extension() {
        let mut registry = registry_with(&[KIND]);
        let err = registry
            .register_node_with::<Tally, _, _, _>(KIND, build_reporting_tally)
            .expect_err("a second registration fails");

        assert_eq!(err.kind, KIND);
        assert!(registry.extension::<Tally>().is_none());
    }

    /// The extension is looked up at each build, so what is added to it
    /// after the kind was registered reaches the build function.
    #[test]
    fn the_build_sees_what_was_added_after_registration() {
        let mut registry = AutonomyRegistry::empty();
        registry
            .register_node_with::<Tally, _, _, _>(KIND, build_reporting_tally)
            .expect("one registration");
        registry.extension_mut::<Tally>().0.push("late");

        let err = build_err(&registry, KIND, "output = \"cloud\"");
        assert!(err.to_string().contains("tally holds [\"late\"]"), "{err}");
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
