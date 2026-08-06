use helios_core::control::actuation_model::ActuationModel;

use bevy::prelude::*;

/// Static parameters for an Ackermann-steering vehicle.
/// Populated from config during `process_ackermann_logic`.
#[derive(Component, Clone)]
pub struct AckermannParameters {
    pub wheelbase: f64,
    pub wheel_radius: f32,
}

/// Wrench gains for the L0 arcade shim, read from the `[actuator]` TOML section.
/// Open-loop feedforward that scales a resolved setpoint into a chassis wrench;
/// retired when L1 dynamic actuation replaces the shim.
#[derive(Component, Clone)]
pub struct AckermannActuator {
    pub l0_force_gain: f32,
    pub l0_yaw_gain: f32,
}

#[derive(Component, Clone)]
pub struct ActuationModelComponent(pub ActuationModel);
