//! Registers all built-in dynamics factories.

use helios_core::estimation::dynamics::{integrated_imu::IntegratedImuModel, EstimationDynamics};

use super::{contexts::DynamicsBuildContext, AutonomyRegistry};

pub fn register(registry: &mut AutonomyRegistry) {
    registry.register_dynamics("IntegratedImu", build_integrated_imu);
    registry.register_dynamics("AckermannOdometry", |_| {
        Err("AckermannOdometry dynamics not yet implemented".to_string())
    });
    registry.register_dynamics("Quadcopter", |_| {
        Err("Quadcopter dynamics not yet implemented".to_string())
    });
}

fn build_integrated_imu(ctx: DynamicsBuildContext) -> Result<Box<dyn EstimationDynamics>, String> {
    Ok(Box::new(IntegratedImuModel {
        agent_handle: ctx.agent_handle,
        gravity_magnitude: ctx.gravity,
    }))
}
