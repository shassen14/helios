use serde::Deserialize;

/// Actuator physics parameters for an Ackermann vehicle.
/// All fields have defaults matching the previously hardcoded values, so existing
/// scenario files without an `[actuator]` section continue to work unchanged.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct AckermannActuatorConfig {
    #[serde(default = "AckermannActuatorConfig::default_max_force")]
    pub max_force: f32, // N
    #[serde(default = "AckermannActuatorConfig::default_max_torque")]
    pub max_torque: f32, // N·m
    #[serde(default = "AckermannActuatorConfig::default_max_speed")]
    pub max_speed: f32, // m/s
}

impl AckermannActuatorConfig {
    fn default_max_force() -> f32 {
        5000.0
    }
    fn default_max_torque() -> f32 {
        2500.0
    }
    fn default_max_speed() -> f32 {
        20.0
    }
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

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum Vehicle {
    Ackermann {
        wheelbase: f32,
        max_steering_angle: f32, // in degrees
        max_steering_rate: f32,  // in deg/sec
        #[serde(default)]
        actuator: AckermannActuatorConfig,
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
