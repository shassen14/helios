// helios_sim/src/simulation/registry/controllers.rs
//
// Registers all built-in controller factories into AutonomyRegistry.
// Add new algorithms here — zero spawning systems change.

use bevy::prelude::*;
use helios_core::control::{
    direct_velocity::DirectVelocityController, lqr::LqrController,
    pid::VelocityPidController,
};

use super::{AutonomyRegistry, ControllerBuildContext};
use crate::simulation::config::structs::ControllerConfig;

pub struct DefaultControllersPlugin;

impl Plugin for DefaultControllersPlugin {
    fn build(&self, app: &mut App) {
        let Some(mut registry) = app.world_mut().get_resource_mut::<AutonomyRegistry>() else {
            error!("DefaultControllersPlugin: AutonomyRegistry resource not found. Add AutonomyRegistryPlugin first.");
            return;
        };

        // --- Pid ---
        registry.register_controller("Pid", |ctx: ControllerBuildContext| {
            if let ControllerConfig::Pid { kp, ki, kd, .. } = ctx.controller_cfg {
                Ok(Box::new(VelocityPidController::new(
                    kp as f64, ki as f64, kd as f64,
                )))
            } else {
                Err("Pid factory received wrong ControllerConfig variant".into())
            }
        });

        // --- Lqr ---
        registry.register_controller("Lqr", |ctx: ControllerBuildContext| {
            if let ControllerConfig::Lqr {
                gain_matrix,
                state_dim,
                control_dim,
                u_min,
                u_max,
                ..
            } = ctx.controller_cfg
            {
                // Supply symmetric default bounds if none given.
                let u_min = if u_min.is_empty() {
                    vec![f64::NEG_INFINITY; control_dim]
                } else {
                    u_min
                };
                let u_max = if u_max.is_empty() {
                    vec![f64::INFINITY; control_dim]
                } else {
                    u_max
                };
                let ctrl = LqrController::new(gain_matrix, state_dim, control_dim, u_min, u_max)?;
                Ok(Box::new(ctrl))
            } else {
                Err("Lqr factory received wrong ControllerConfig variant".into())
            }
        });

        // --- DirectVelocity ---
        registry.register_controller("DirectVelocity", |_ctx: ControllerBuildContext| {
            Ok(Box::new(DirectVelocityController::new()))
        });

        // FeedforwardPid requires a ControlDynamics model at runtime.
        // That model comes from a DynamicsFactory, which is itself an EstimationDynamics-rooted
        // factory. Since ControlDynamics is a separate trait, the FeedforwardPid factory is
        // left as a stub until concrete ControlDynamics impls exist.
        // registry.register_controller("FeedforwardPid", |ctx| { ... });
    }
}
