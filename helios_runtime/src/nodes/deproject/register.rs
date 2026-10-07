//! Registers the `Deproject` kind, which builds a [`DeprojectNode`] from a
//! `[nodes.<name>]` entry.

use crate::{
    nodes::deproject::{config::DeprojectConfig, DeprojectNode},
    AutonomyRegistry, BuildContext, FactoryOutput,
};

use helios_core::spatial::conventions::Flu;

/// The `kind` a profile writes to get a deproject node.
pub(crate) const DEPROJECT_KIND: &str = "Deproject";

/// Adds the `Deproject` kind to `registry`.
pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry
        .register_node(DEPROJECT_KIND, build)
        .expect("Deproject is registered once, by the default registry")
}

/// Builds the node for one entry. A kind string cannot pick the frame type,
/// so the kind is fixed to `Flu` fields.
fn build(config: DeprojectConfig, ctx: &BuildContext) -> Result<FactoryOutput, String> {
    Ok(FactoryOutput::new(Box::new(DeprojectNode::<Flu>::new(
        ctx.node_name(),
        &config.input,
        &config.output,
    ))))
}
