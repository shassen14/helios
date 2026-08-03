//! Registers built-in controller factories.

use super::input::DefaultControlInputBuilder;
use super::node::ControllerNode;

use crate::config::ControllerConfig;
use crate::pipeline::node::PipelineNode;
use crate::registry::{contexts::ControllerBuildContext, AutonomyRegistry};

use helios_core::control::direct_velocity::DirectVelocityController;

pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry.register_controller("DirectVelocity", build_direct_velocity);
}

fn build_direct_velocity(ctx: ControllerBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    // `ControllerConfig` has one variant today, so this destructure is irrefutable.
    // Kept as an explicit binding so adding a second variant turns this into a
    // compile error, forcing the new factory to be written rather than silently
    // mis-dispatching.
    let ControllerConfig::DirectVelocity { .. } = ctx.config;

    let controller = DirectVelocityController::new();
    let input_builder = Box::new(DefaultControlInputBuilder::new());

    Ok(Box::new(ControllerNode::new(
        "direct_velocity",
        controller,
        input_builder,
        ctx.output_channel,
    )))
}
