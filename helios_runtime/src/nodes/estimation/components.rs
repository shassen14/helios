//! [`EstimatorComponents`]: the registry extension holding the estimator's
//! component tables, one per part a node draws by `kind`.

use super::dynamics::DynamicsComponent;
use super::filter::FilterParts;
use super::measurement::{source_factory, MeasurementSource, MeasurementWiring};

use crate::assembly::{
    BuildContext, BuiltComponent, ComponentError, ComponentTable, DuplicateComponentKind,
};

use helios_core::estimation::measurement::MeasurementModel;
use helios_core::estimation::GaussianStateEstimator;
use helios_core::interchange::measurement::sensor::SensorPayload;
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
    filters: ComponentTable<FilterParts, Box<dyn GaussianStateEstimator>>,
    dynamics: ComponentTable<(), DynamicsComponent>,
    measurements: ComponentTable<MeasurementWiring, Box<dyn MeasurementSource>>,
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

    /// Builds the filter described by `section`, the sub-table at `path` in
    /// node `ctx.node_name()`'s section, around `parts`.
    pub(crate) fn build_filter(
        &self,
        path: &str,
        section: toml::Table,
        ctx: &BuildContext<'_>,
        parts: FilterParts,
    ) -> Result<BuiltComponent<Box<dyn GaussianStateEstimator>>, ComponentError> {
        self.filters.build(path, section, ctx, parts)
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

    /// Builds the measurement source for the model described by `section`, the
    /// sub-table at `path` in node `ctx.node_name()`'s section, reading the
    /// channel and using the R that `wiring` names.
    pub(crate) fn build_measurement(
        &self,
        path: &str,
        section: toml::Table,
        ctx: &BuildContext<'_>,
        wiring: MeasurementWiring,
    ) -> Result<BuiltComponent<Box<dyn MeasurementSource>>, ComponentError> {
        self.measurements.build(path, section, ctx, wiring)
    }

    // --- Registration ---

    /// Registers a filter kind under `kind`, the string a node's `filter`
    /// sub-table names in its `kind` key.
    ///
    /// `build` takes the kind's own typed config, the node's build context and
    /// the [`FilterParts`] the node built (the seeded state, Q and the
    /// dynamics), and returns the filter. Parsing, rejecting unknown keys and
    /// naming the node, sub-table and kind in errors are the table's job. `C`
    /// must carry `#[serde(deny_unknown_fields)]`.
    ///
    /// Fails if `kind` is already a filter kind; the first registration is
    /// kept.
    pub fn register_filter<C, F>(
        &mut self,
        kind: impl Into<String>,
        build: F,
    ) -> Result<(), DuplicateComponentKind>
    where
        C: DeserializeOwned + Serialize + 'static,
        F: Fn(C, &BuildContext<'_>, FilterParts) -> Result<Box<dyn GaussianStateEstimator>, String>
            + Send
            + Sync
            + 'static,
    {
        self.filters.register(kind, build)
    }

    /// Registers a dynamics kind under `kind`, the string a node's `dynamics`
    /// sub-table names in its `kind` key.
    ///
    /// `build` returns a [`DynamicsComponent`]: the process model, the input
    /// builder feeding it (checked against each other when the component is
    /// made) and how a pose prior maps into its state. `C` must carry
    /// `#[serde(deny_unknown_fields)]`.
    ///
    /// Fails if `kind` is already a dynamics kind; the first registration is
    /// kept.
    pub fn register_dynamics<C, F>(
        &mut self,
        kind: impl Into<String>,
        build: F,
    ) -> Result<(), DuplicateComponentKind>
    where
        C: DeserializeOwned + Serialize + 'static,
        F: Fn(C, &BuildContext<'_>) -> Result<DynamicsComponent, String> + Send + Sync + 'static,
    {
        self.dynamics
            .register(kind, move |config: C, ctx: &BuildContext<'_>, (): ()| {
                build(config, ctx)
            })
    }

    /// Registers a measurement kind under `kind`, reading payload type `P`.
    ///
    /// `build` takes the kind's own typed config, the node's build context and
    /// the frame of the sensor whose readings it predicts, and returns the
    /// model. Registering the payload with the model is what lets one entry
    /// yield the model, the typed reader and the typed channel: config names
    /// only the kind, so a model can't be paired with a channel carrying a
    /// different payload. The same model may be registered again under another
    /// name for another payload. `C` must carry `#[serde(deny_unknown_fields)]`.
    ///
    /// Fails if `kind` is already a measurement kind; the first registration
    /// is kept.
    pub fn register_measurement<P, C, F>(
        &mut self,
        kind: impl Into<String>,
        build: F,
    ) -> Result<(), DuplicateComponentKind>
    where
        P: SensorPayload,
        C: DeserializeOwned + Serialize + 'static,
        F: Fn(C, &BuildContext<'_>, &FrameId) -> Result<Box<dyn MeasurementModel>, String>
            + Send
            + Sync
            + 'static,
    {
        self.measurements
            .register(kind, source_factory::<P, C, F>(build))
    }
}
