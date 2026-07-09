// helios_sim/src/simulation/plugins/vehicles/ackermann/components.rs

use bevy::prelude::*;

/// Static parameters for an Ackermann-steering vehicle.
/// Populated from config during `process_ackermann_logic`.
#[derive(Component, Clone)]
pub struct AckermannParameters {
    pub wheelbase: f64,
    pub max_steering_angle: f32,
}

/// Actuator physics limits for an Ackermann vehicle.
/// Read from the `[vehicle.actuator]` TOML section with sensible defaults.
#[derive(Component, Clone)]
pub struct AckermannActuator {
    pub max_force: f32,  // N
    pub max_torque: f32, // N·m
    pub max_speed: f32,  // m/s
}

/// Normalised command produced by an `AckermannOutputAdapter`.
/// Both fields are in [-1.0, 1.0]; all vehicle-specific kinematics are resolved inside the adapter.
/// Persisted as a Bevy component so the debug HUD can read the last applied command.
#[derive(Component, Clone, Copy, Default)]
pub struct AckermannCommand {
    /// Normalised forward force demand [-1.0, 1.0], positive = forward.
    pub throttle_norm: f32,
    /// Normalised yaw torque demand [-1.0, 1.0], positive = left (FLU convention).
    pub steering_torque_norm: f32,
}
