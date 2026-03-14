// helios_sim/src/simulation/registry/dynamics.rs
//
// Registers all known dynamics models with the AutonomyRegistry.
// To add a new dynamics model:
//   1. Implement EstimationDynamics in helios_core.
//   2. Add a register_dynamics call below.
//   Zero other files change.

use bevy::prelude::{App, Plugin};
use helios_core::{
    models::estimation::dynamics::{integrated_imu::IntegratedImuModel, EstimationDynamics},
    types::FrameHandle,
};

use super::{AutonomyRegistry, DynamicsBuildContext};

pub struct DefaultDynamicsPlugin;

impl Plugin for DefaultDynamicsPlugin {
    fn build(&self, app: &mut App) {
        app.world_mut()
            .resource_mut::<AutonomyRegistry>()
            .register_dynamics("IntegratedImu", build_integrated_imu)
            .register_dynamics("AckermannOdometry", build_ackermann_odometry)
            .register_dynamics("Quadcopter", build_quadcopter);
    }
}

fn build_integrated_imu(ctx: DynamicsBuildContext) -> Result<Box<dyn EstimationDynamics>, String> {
    Ok(Box::new(IntegratedImuModel {
        agent_handle: FrameHandle::from_entity(ctx.agent_entity),
        gravity_magnitude: ctx.gravity_magnitude,
    }))
}

fn build_ackermann_odometry(
    _ctx: DynamicsBuildContext,
) -> Result<Box<dyn EstimationDynamics>, String> {
    Err("AckermannOdometry dynamics not yet implemented".to_string())
}

fn build_quadcopter(_ctx: DynamicsBuildContext) -> Result<Box<dyn EstimationDynamics>, String> {
    Err("Quadcopter dynamics not yet implemented".to_string())
}
