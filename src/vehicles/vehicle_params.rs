// src/vehicles/car_setup.rs (or a dedicated vehicle_params.rs)
use bevy::prelude::*;

#[derive(Component, Clone, Debug)]
pub struct VehiclePhysicalParams {
    pub wheelbase: f64,
    pub track_width_front: f64, // Distance between center of front wheels
    pub track_width_rear: f64,  // Distance between center of rear wheels
    pub front_axle_offset_from_center: f64, // Positive if front axle is ahead of CoM/origin
    pub rear_axle_offset_from_center: f64, // Negative if rear axle is behind CoM/origin

    pub wheel_radius: f32,
    pub mass: f32, // Total vehicle mass

    // Tire properties (simplified)
    pub front_tire_cornering_stiffness: f32, // N/radian
    pub rear_tire_cornering_stiffness: f32,  // N/radian
    pub max_grip_force_longitudinal: f32,    // Max combined accel/brake force per axle (simplified)
    pub rolling_resistance_coefficient: f32,

    // Control related (can also be in controller struct)
    pub max_steer_angle_rad: f64,
    // Gains for low-level controllers if you add them later
    // pub throttle_pid_gains: (f32, f32, f32),
    // pub steering_pid_gains: (f32, f32, f32),
}

// CarConfig remains for spawning, populating VehiclePhysicalParams
#[derive(Clone, Debug)]
pub struct CarConfig {
    // Entity
    pub name: String,

    // --- Visuals ---
    pub body_color: Color,
    pub wheel_color: Color,
    pub body_size: Vec3, // length, height, width
    pub wheel_width: f32,

    pub initial_transform: Transform,
    pub initial_velocity_enu: f64, // ENU forward velocity

    // Parameters to populate VehiclePhysicalParams
    pub wheelbase: f64,
    pub track_width: f64,      // Assume front/rear same for simplicity here
    pub cg_to_front_axle: f64, // Distance from CG to front axle
    pub cg_to_rear_axle: f64,  // Distance from CG to rear axle
    pub wheel_radius: f32,
    pub mass: f32,
    pub front_tire_stiffness: f32,
    pub rear_tire_stiffness: f32,
    pub max_long_force: f32,
    pub rolling_resistance: f32,
    pub max_steer_angle_deg: f64,
}

impl Default for CarConfig {
    fn default() -> Self {
        let wheelbase = 2.7;
        let track_width = 1.5;
        Self {
            initial_transform: Transform::from_xyz(0.0, 0.35, 0.0), // Assuming 0.35 is wheel_radius
            initial_velocity_enu: 0.0,
            wheelbase,
            track_width,
            cg_to_front_axle: wheelbase * 0.4, // Example: CG slightly rear-biased
            cg_to_rear_axle: wheelbase * 0.6,
            wheel_radius: 0.35,
            mass: 1500.0,
            front_tire_stiffness: 150000.0, // N/rad, High value
            rear_tire_stiffness: 180000.0,  // N/rad
            max_long_force: 8000.0,         // N
            rolling_resistance: 0.015,
            max_steer_angle_deg: 35.0,
            name: "DefaultCarPhysics".to_string(),
            // ... other CarConfig fields like body_color, etc.
            body_color: Color::srgb(0.8, 0.1, 0.1),
            wheel_color: Color::srgb(0.1, 0.1, 0.1),
            body_size: Vec3::new(4.5, 1.4, 1.8),
            wheel_width: 0.25,
        }
    }
}
