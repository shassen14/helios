use crate::config::CommandSpace;

use helios_core::control::actuators::SetpointKind;

use serde::Deserialize;

/// The `kind` tag of [`AllocatorConfig::WheelTorque`], which is also the key its
/// factory is registered under in the allocator family.
pub(crate) const WHEEL_TORQUE_KIND: &str = "WheelTorque";

/// The `kind` tag of [`AllocatorConfig::SteerPosition`], which is also the key its
/// factory is registered under in the allocator family.
pub(crate) const STEER_POSITION_KIND: &str = "SteerPosition";

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum AllocatorConfig {
    /// `input` names the `[command]` fold the allocator reads its `DriveForce`
    /// from.
    WheelTorque {
        input: String,
        wheel_radius: f64,
        drive: String,
    },
    /// `input` names the `[command]` fold the allocator reads its `SteerAngle`
    /// from.
    SteerPosition { input: String, steer: String },
}

impl AllocatorConfig {
    pub(crate) fn get_kind_str(&self) -> &str {
        match self {
            AllocatorConfig::WheelTorque { .. } => WHEEL_TORQUE_KIND,
            AllocatorConfig::SteerPosition { .. } => STEER_POSITION_KIND,
        }
    }

    /// The `[command]` fold this allocator reads its command from.
    pub(crate) fn input(&self) -> &str {
        match self {
            AllocatorConfig::WheelTorque { input, .. } => input,
            AllocatorConfig::SteerPosition { input, .. } => input,
        }
    }

    /// The command type this allocator consumes: the type of its input
    /// channel.
    pub(crate) fn command_space(&self) -> CommandSpace {
        match self {
            AllocatorConfig::WheelTorque { .. } => CommandSpace::DriveForce,
            AllocatorConfig::SteerPosition { .. } => CommandSpace::SteerAngle,
        }
    }

    pub(crate) fn actuator_ids(&self) -> Vec<&str> {
        match self {
            AllocatorConfig::WheelTorque { drive, .. } => vec![drive],
            AllocatorConfig::SteerPosition { steer, .. } => vec![steer],
        }
    }

    /// The setpoint kind this allocator emits onto each actuator it drives — the
    /// dual of [`command_space`](Self::command_space): that names the body-level
    /// command consumed, this the per-actuator quantity produced. The host checks
    /// it against the body's declared actuator kind at spawn.
    pub(crate) fn output_kind(&self) -> SetpointKind {
        match self {
            AllocatorConfig::WheelTorque { .. } => SetpointKind::Torque,
            AllocatorConfig::SteerPosition { .. } => SetpointKind::Position,
        }
    }
}
