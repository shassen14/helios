// helios_sim/src/simulation/registry/dynamics.rs
//
// Registers all known dynamics models with the AutonomyRegistry.
// To add a new dynamics model:
//   1. Implement EstimationDynamics in helios_core.
//   2. Add a register_dynamics call below.
//   Zero other files change.

use bevy::prelude::*;
use helios_core::{
    models::estimation::dynamics::{EstimationDynamics, integrated_imu::IntegratedImuModel},
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

fn build_integrated_imu(ctx: DynamicsBuildContext) -> Option<Box<dyn EstimationDynamics>> {
    Some(Box::new(IntegratedImuModel {
        agent_handle: FrameHandle::from_entity(ctx.agent_entity),
        gravity_magnitude: ctx.gravity_magnitude,
    }))
}

fn build_ackermann_odometry(_ctx: DynamicsBuildContext) -> Option<Box<dyn EstimationDynamics>> {
    warn!("AutonomyRegistry: AckermannOdometry dynamics not yet implemented.");
    None
}

fn build_quadcopter(_ctx: DynamicsBuildContext) -> Option<Box<dyn EstimationDynamics>> {
    warn!("AutonomyRegistry: Quadcopter dynamics not yet implemented.");
    None
}
