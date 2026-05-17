//! Registers built-in controller factories.

use helios_core::control::{
    direct_velocity::DirectVelocityController, lqr::LqrController, pid::VelocityPidController,
    Controller,
};

use crate::config::ControllerConfig;
use crate::pipeline::builders::controller::DefaultControlInputBuilder;
use crate::pipeline::node::PipelineNode;
use crate::pipeline::nodes::controller::ControllerNode;

use super::{contexts::ControllerBuildContext, AutonomyRegistry};

pub fn register(registry: &mut AutonomyRegistry) {
    registry.register_controller("DirectVelocity", build_direct_velocity);
    registry.register_controller("Pid", build_pid);
    registry.register_controller("Lqr", build_lqr);
    registry.register_controller("FeedforwardPid", |ctx| {
        Err(format!(
            "FeedforwardPid controller not yet implemented (config: {:?})",
            ctx.config.get_kind_str()
        ))
    });
}

fn build_direct_velocity(ctx: ControllerBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    let ControllerConfig::DirectVelocity { .. } = ctx.config else {
        return Err("build_direct_velocity received wrong config variant".to_string());
    };
    let controller: Box<dyn Controller> = Box::new(DirectVelocityController::new());
    let input_builder = Box::new(DefaultControlInputBuilder::new());
    Ok(Box::new(ControllerNode::new(
        "direct_velocity",
        controller,
        input_builder,
    )))
}

fn build_pid(ctx: ControllerBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    let ControllerConfig::Pid { kp, ki, kd, .. } = ctx.config else {
        return Err("build_pid received wrong config variant".to_string());
    };
    let controller: Box<dyn Controller> =
        Box::new(VelocityPidController::new(kp as f64, ki as f64, kd as f64));
    let input_builder = Box::new(DefaultControlInputBuilder::new());
    Ok(Box::new(ControllerNode::new("pid", controller, input_builder)))
}

fn build_lqr(ctx: ControllerBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    let ControllerConfig::Lqr {
        gain_matrix,
        state_dim,
        control_dim,
        u_min,
        u_max,
        ..
    } = ctx.config
    else {
        return Err("build_lqr received wrong config variant".to_string());
    };
    let controller: Box<dyn Controller> =
        Box::new(LqrController::new(gain_matrix, state_dim, control_dim, u_min, u_max)?);
    let input_builder = Box::new(DefaultControlInputBuilder::new());
    Ok(Box::new(ControllerNode::new("lqr", controller, input_builder)))
}
