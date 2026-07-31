//! Registers all built-in dynamics factories.

use helios_core::estimation::dynamics::{integrated_imu::IntegratedImuModel, EstimationDynamics};
use nalgebra::Vector3;

use super::{contexts::DynamicsBuildContext, AutonomyRegistry};

pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry.register_dynamics("IntegratedImu", build_integrated_imu);
    registry.register_dynamics("AckermannOdometry", |_| {
        Err("AckermannOdometry dynamics not yet implemented".to_string())
    });
    registry.register_dynamics("Quadcopter", |_| {
        Err("Quadcopter dynamics not yet implemented".to_string())
    });
}

fn build_integrated_imu(ctx: DynamicsBuildContext) -> Result<Box<dyn EstimationDynamics>, String> {
    let g = ctx.gravity_enu;
    Ok(Box::new(IntegratedImuModel {
        agent_handle: ctx.agent_handle,
        gravity_world: Vector3::new(g[0], g[1], g[2]),
    }))
}
