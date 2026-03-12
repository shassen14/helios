use serde::Deserialize;

// =========================================================================
// == Physics Config ==
// =========================================================================

/// Rigid-body physics parameters for an Ackermann vehicle.
/// Applied to the Avian3D `RigidBody` in `attach_ackermann_physics`.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct AckermannPhysicsConfig {
    /// Vehicle mass in kg. Must exceed friction threshold to move: mass * 9.81 * friction.
    #[serde(default = "AckermannPhysicsConfig::default_mass")]
    pub mass: f32,
    /// Avian3D surface friction coefficient.
    #[serde(default = "AckermannPhysicsConfig::default_friction")]
    pub friction: f32,
    /// Avian3D linear velocity damping (passive drag). 0 = no damping.
    #[serde(default = "AckermannPhysicsConfig::default_linear_damping")]
    pub linear_damping: f32,
    /// Avian3D angular velocity damping (passive yaw decay). 0 = no damping.
    #[serde(default = "AckermannPhysicsConfig::default_angular_damping")]
    pub angular_damping: f32,
}

impl AckermannPhysicsConfig {
    fn default_mass() -> f32 { 1500.0 }
    fn default_friction() -> f32 { 0.7 }
    fn default_linear_damping() -> f32 { 0.0 }
    fn default_angular_damping() -> f32 { 0.0 }
}

impl Default for AckermannPhysicsConfig {
    fn default() -> Self {
        Self {
            mass: Self::default_mass(),
            friction: Self::default_friction(),
            linear_damping: Self::default_linear_damping(),
            angular_damping: Self::default_angular_damping(),
        }
    }
}

// =========================================================================
// == Actuator Config ==
// =========================================================================

/// Actuator saturation limits for an Ackermann vehicle.
/// With the open-loop pipeline, `max_force` is an absolute physical force limit (N),
/// not a P-controller gain. It must exceed the static friction threshold to move the car.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct AckermannActuatorConfig {
    #[serde(default = "AckermannActuatorConfig::default_max_force")]
    pub max_force: f32,  // N
    #[serde(default = "AckermannActuatorConfig::default_max_torque")]
    pub max_torque: f32, // N·m
    #[serde(default = "AckermannActuatorConfig::default_max_speed")]
    pub max_speed: f32,  // m/s
}

impl AckermannActuatorConfig {
    fn default_max_force() -> f32 { 5000.0 }
    fn default_max_torque() -> f32 { 2500.0 }
    fn default_max_speed() -> f32 { 20.0 }
}

impl Default for AckermannActuatorConfig {
    fn default() -> Self {
        Self {
            max_force: Self::default_max_force(),
            max_torque: Self::default_max_torque(),
            max_speed: Self::default_max_speed(),
        }
    }
}

// =========================================================================
// == Adapter Config ==
// =========================================================================

/// Gains for a single SISO PID channel in an adapter.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct SisoPidConfig {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    /// Maximum absolute integral accumulator value. 0.0 = unclamped.
    #[serde(default)]
    pub integral_clamp: f64,
}

/// Selects which `AckermannOutputAdapter` implementation to use per vehicle.
/// The `kind` field in TOML drives registry lookup.
///
/// ```toml
/// [adapter]
/// kind = "DualSisoPid"
///
/// [adapter.longitudinal]
/// kp = 2000.0
/// ki = 100.0
/// kd = 200.0
/// integral_clamp = 0.5
///
/// [adapter.lateral]
/// kp = 5000.0
/// ki = 0.0
/// kd = 500.0
/// integral_clamp = 0.5
/// ```
#[derive(Debug, Deserialize, Clone, Default)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum AckermannAdapterConfig {
    /// Open-loop, no feedback. Good for teleop and quick prototyping.
    #[default]
    Default,
    /// Two SISO PIDs: longitudinal (speed error → throttle) + lateral (yaw rate error → torque).
    DualSisoPid {
        longitudinal: SisoPidConfig,
        lateral: SisoPidConfig,
    },
}

impl AckermannAdapterConfig {
    pub fn kind_str(&self) -> &str {
        match self {
            Self::Default => "Default",
            Self::DualSisoPid { .. } => "DualSisoPid",
        }
    }
}

// =========================================================================
// == Vehicle Enum ==
// =========================================================================

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum Vehicle {
    Ackermann {
        wheelbase: f32,
        max_steering_angle: f32, // degrees
        max_steering_rate: f32,  // degrees/sec
        #[serde(default)]
        physics: AckermannPhysicsConfig,
        #[serde(default)]
        actuator: AckermannActuatorConfig,
        #[serde(default)]
        adapter: AckermannAdapterConfig,
    },
    Quadcopter {
        rotor_thrust_coefficient: f32,
        rotor_drag_coefficient: f32,
    },
}

impl Vehicle {
    pub fn get_kind_str(&self) -> &str {
        match self {
            Vehicle::Ackermann { .. } => "Ackermann",
            Vehicle::Quadcopter { .. } => "Quadcopter",
        }
    }
}
