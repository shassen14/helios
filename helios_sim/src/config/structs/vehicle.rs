use helios_core::control::actuation_model::ActuationModel;
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
    fn default_mass() -> f32 {
        1500.0
    }
    fn default_friction() -> f32 {
        0.7
    }
    fn default_linear_damping() -> f32 {
        0.0
    }
    fn default_angular_damping() -> f32 {
        0.0
    }
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
    #[serde(default = "AckermannActuatorConfig::default_l0_force_gain")]
    pub l0_force_gain: f32,
    #[serde(default = "AckermannActuatorConfig::default_l0_yaw_gain")]
    pub l0_yaw_gain: f32,
}

impl AckermannActuatorConfig {
    fn default_l0_force_gain() -> f32 {
        1.0
    }

    fn default_l0_yaw_gain() -> f32 {
        1.0
    }
}

impl Default for AckermannActuatorConfig {
    fn default() -> Self {
        Self {
            l0_force_gain: Self::default_l0_force_gain(),
            l0_yaw_gain: Self::default_l0_yaw_gain(),
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
        wheel_radius: f32,
        actuation: ActuationModel,
        #[serde(default)]
        physics: AckermannPhysicsConfig,
        #[serde(default)]
        actuator: AckermannActuatorConfig,
    },
}

impl Vehicle {
    pub fn get_kind_str(&self) -> &str {
        match self {
            Vehicle::Ackermann { .. } => "Ackermann",
        }
    }
}
