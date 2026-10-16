//! [`EstimatorComponents`]: the registry extension holding the estimator's
//! component tables, one per part a node draws by `kind`.

use super::dynamics::DynamicsComponent;
use super::filter::FilterParts;
use super::measurement::{
    reading_channel, source_factory, MeasurementDeclaration, MeasurementSource, MeasurementWiring,
};

use crate::assembly::{
    BuildContext, BuiltComponent, ComponentError, ComponentTable, DuplicateComponentKind,
    ParsedComponent,
};

use helios_core::estimation::measurement::MeasurementModel;
use helios_core::estimation::GaussianStateEstimator;
use helios_core::interchange::measurement::sensor::SensorPayload;
use helios_core::spatial::transforms::Convention;
use helios_core::spatial::FrameId;
use serde::de::DeserializeOwned;
use serde::Serialize;

/// The filter, dynamics and measurement kinds an estimator node may name in
/// its sub-tables.
///
/// A registry extension. The default registry adds the built-in kinds; an
/// outside crate adds its own through
/// `registry.extension_mut::<EstimatorComponents>().register_filter(…)` and its
/// siblings.
pub struct EstimatorComponents {
    filters: ComponentTable<FilterParts, Box<dyn GaussianStateEstimator>, ()>,
    dynamics: ComponentTable<(), DynamicsComponent, ()>,
    measurements:
        ComponentTable<MeasurementWiring, Box<dyn MeasurementSource>, MeasurementDeclaration>,
}

impl Default for EstimatorComponents {
    fn default() -> Self {
        Self {
            filters: ComponentTable::new("filter"),
            dynamics: ComponentTable::new("dynamics"),
            measurements: ComponentTable::new("measurement"),
        }
    }
}

impl EstimatorComponents {
    // --- Build ---

    /// Parses the filter described by `section`, the sub-table at `path` in
    /// node `ctx.node_name()`'s section, and runs its kind's check; it is
    /// built later, around the [`FilterParts`] the node builds.
    pub(crate) fn parse_filter(
        &self,
        path: &str,
        section: toml::Table,
        ctx: &BuildContext<'_>,
    ) -> Result<ParsedComponent<FilterParts, Box<dyn GaussianStateEstimator>, ()>, ComponentError>
    {
        self.filters.parse(path, section, ctx)
    }

    /// Builds the dynamics described by `section`, the sub-table at `path` in
    /// node `ctx.node_name()`'s section.
    pub(crate) fn build_dynamics(
        &self,
        path: &str,
        section: toml::Table,
        ctx: &BuildContext<'_>,
    ) -> Result<BuiltComponent<DynamicsComponent>, ComponentError> {
        self.dynamics.build(path, section, ctx, ())
    }

    /// Parses the measurement model described by `section`, the sub-table at
    /// `path` in node `ctx.node_name()`'s section, and runs its kind's check.
    /// Its kind declares the channel it reads and the axes it reads that
    /// channel's mount in; the source is built later, from
    /// the [`MeasurementWiring`] naming that channel and R.
    pub(crate) fn parse_measurement(
        &self,
        path: &str,
        section: toml::Table,
        ctx: &BuildContext<'_>,
    ) -> Result<
        ParsedComponent<MeasurementWiring, Box<dyn MeasurementSource>, MeasurementDeclaration>,
        ComponentError,
    > {
        self.measurements.parse(path, section, ctx)
    }

    // --- Registration ---

    /// Registers a filter kind under `kind`, the string a node's `filter`
    /// sub-table names in its `kind` key.
    ///
    /// `check` refuses a config that is unusable on its own (a non-finite
    /// tuning value), when the node is declared; pass [`no_check`] when
    /// parsing is check enough. `build` takes the kind's own typed config,
    /// which `check` accepted, the node's build context and the
    /// [`FilterParts`] the node built (the seeded state, Q and the dynamics),
    /// and returns the filter, refusing parts it can't run on. Parsing,
    /// rejecting unknown keys and naming the node, sub-table and kind in
    /// errors are the table's job. `C` must carry
    /// `#[serde(deny_unknown_fields)]`.
    ///
    /// Fails if `kind` is already a filter kind; the first registration is
    /// kept.
    ///
    /// [`no_check`]: crate::assembly::no_check
    pub fn register_filter<C, K, F>(
        &mut self,
        kind: impl Into<String>,
        check: K,
        build: F,
    ) -> Result<(), DuplicateComponentKind>
    where
        C: DeserializeOwned + Serialize + Send + 'static,
        K: Fn(&C) -> Result<(), String> + Send + Sync + 'static,
        F: Fn(C, &BuildContext<'_>, FilterParts) -> Result<Box<dyn GaussianStateEstimator>, String>
            + Send
            + Sync
            + 'static,
    {
        self.filters.register(kind, check, build)
    }

    /// Registers a dynamics kind under `kind`, the string a node's `dynamics`
    /// sub-table names in its `kind` key.
    ///
    /// `check` refuses a config that is unusable on its own, as for a filter;
    /// pass [`no_check`] when parsing is check enough. `build` runs when the
    /// node is declared, holds the state in the context's
    /// [`frames`](BuildContext::frames), and returns a [`DynamicsComponent`]: the process model,
    /// how a pose prior maps into its state, and the input builder feeding it
    /// as a [`DeclaredInput`], which states the input it assembles and the
    /// channels it reads now (checked against the model when the component is
    /// made) and is built with the node, from those channels' mounts. `C`
    /// must carry `#[serde(deny_unknown_fields)]`.
    ///
    /// [`DeclaredInput`]: super::DeclaredInput
    ///
    /// Fails if `kind` is already a dynamics kind; the first registration is
    /// kept.
    ///
    /// [`no_check`]: crate::assembly::no_check
    pub fn register_dynamics<C, K, F>(
        &mut self,
        kind: impl Into<String>,
        check: K,
        build: F,
    ) -> Result<(), DuplicateComponentKind>
    where
        C: DeserializeOwned + Serialize + Send + 'static,
        K: Fn(&C) -> Result<(), String> + Send + Sync + 'static,
        F: Fn(C, &BuildContext<'_>) -> Result<DynamicsComponent, String> + Send + Sync + 'static,
    {
        self.dynamics.register(
            kind,
            check,
            move |config: C, ctx: &BuildContext<'_>, (): ()| build(config, ctx),
        )
    }

    /// Registers a measurement kind under `kind`, reading payload type `P`.
    ///
    /// `mount_convention` is the axes the model reads its sensor's mount in
    /// through the tf tree, `None` for a model that never resolves the mount;
    /// the host refuses a mount the tree holds in other axes before the first
    /// tick. `check` refuses a config that is unusable on its own, when the
    /// node is declared; pass [`no_check`] when parsing is check enough. The
    /// kind declares the channel it reads from `P`, so `check` states nothing
    /// else. `build` takes the kind's own typed config, which `check`
    /// accepted, the node's build context, whose
    /// [`frames`](BuildContext::frames) are the body and root the model reads
    /// the state in, and the frame of the sensor whose readings it predicts,
    /// and returns the model. Registering the payload with the model is what lets one entry
    /// yield the model, the typed reader and the typed channel: config names
    /// only the kind, so a model can't be paired with a channel carrying a
    /// different payload. The same model may be registered again under another
    /// name for another payload. `C` must carry `#[serde(deny_unknown_fields)]`.
    ///
    /// Fails if `kind` is already a measurement kind; the first registration
    /// is kept.
    ///
    /// [`no_check`]: crate::assembly::no_check
    pub fn register_measurement<P, C, K, F>(
        &mut self,
        kind: impl Into<String>,
        mount_convention: Option<Convention>,
        check: K,
        build: F,
    ) -> Result<(), DuplicateComponentKind>
    where
        P: SensorPayload,
        C: DeserializeOwned + Serialize + Send + 'static,
        K: Fn(&C) -> Result<(), String> + Send + Sync + 'static,
        F: Fn(C, &BuildContext<'_>, &FrameId) -> Result<Box<dyn MeasurementModel>, String>
            + Send
            + Sync
            + 'static,
    {
        self.measurements.register(
            kind,
            move |config: &C| {
                check(config)?;
                Ok(MeasurementDeclaration {
                    channel: reading_channel::<P>(),
                    mount_convention,
                })
            },
            source_factory::<P, C, F>(build),
        )
    }
}
