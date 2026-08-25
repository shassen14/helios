use helios_core::control::actuation_model::ActuationModel;

use bevy::prelude::*;

/// Wrench gains for the L0 arcade shim, read from the `[plant]` TOML section.
/// Open-loop feedforward that scales a resolved setpoint into a chassis wrench;
/// retired when a dynamic plant replaces the shim.
#[derive(Component, Clone)]
pub struct AckermannActuator {
    pub l0_force_gain: f32,
    pub l0_yaw_gain: f32,
}

#[derive(Component, Clone)]
pub struct ActuationModelComponent(pub ActuationModel);
