// helios_sim/src/simulation/registry/adapters.rs
//
// Registers all built-in AckermannOutputAdapter factories into AutonomyRegistry.
// Add new adapter implementations here — zero spawning systems change.

use bevy::prelude::*;
use helios_core::control::siso_pid::SisoPid;

use super::{AdapterBuildContext, AutonomyRegistry};
use crate::simulation::config::structs::AckermannAdapterConfig;
use crate::simulation::plugins::vehicles::ackermann::adapter::{
    DefaultAckermannAdapter, DualSisoPidAdapter,
};

pub struct DefaultAdaptersPlugin;

impl Plugin for DefaultAdaptersPlugin {
    fn build(&self, app: &mut App) {
        let Some(mut registry) = app.world_mut().get_resource_mut::<AutonomyRegistry>() else {
            error!("DefaultAdaptersPlugin: AutonomyRegistry resource not found. Add AutonomyRegistryPlugin first.");
            return;
        };

        registry.register_adapter("Default", |_ctx: AdapterBuildContext| {
            Ok(Box::new(DefaultAckermannAdapter))
        });

        registry.register_adapter("DualSisoPid", |ctx: AdapterBuildContext| {
            let AckermannAdapterConfig::DualSisoPid {
                longitudinal,
                lateral,
            } = ctx.adapter_cfg
            else {
                return Err(
                    "DualSisoPid factory received wrong AckermannAdapterConfig variant".into(),
                );
            };
            let long_pid = SisoPid::new(longitudinal.kp, longitudinal.ki, longitudinal.kd)
                .with_integral_clamp(longitudinal.integral_clamp);
            let lat_pid = SisoPid::new(lateral.kp, lateral.ki, lateral.kd)
                .with_integral_clamp(lateral.integral_clamp);
            Ok(Box::new(DualSisoPidAdapter::new(long_pid, lat_pid)))
        });
    }
}
