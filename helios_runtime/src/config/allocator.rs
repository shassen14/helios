use crate::config::CommandSpace;

use helios_core::control::actuators::SetpointKind;

use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum AllocatorConfig {
    WheelTorque { wheel_radius: f64, drive: String },
    SteerPosition { steer: String },
}

impl AllocatorConfig {
    pub(crate) fn get_kind_str(&self) -> &str {
        match self {
            AllocatorConfig::WheelTorque { .. } => "WheelTorque",
            AllocatorConfig::SteerPosition { .. } => "SteerPosition",
        }
    }

    /// The command space this allocator consumes. The allocator *defines* the
    /// command seam, so this is the authoritative `T` the assembler wires the
    /// `command` channel, the fold, and this allocator's input around.
    pub(crate) fn command_space(&self) -> CommandSpace {
        match self {
            AllocatorConfig::WheelTorque { .. } => CommandSpace::DriveForce,
            AllocatorConfig::SteerPosition { .. } => CommandSpace::SteerAngle,
        }
    }

    pub(crate) fn actuator_ids(&self) -> Vec<&str> {
        match self {
            AllocatorConfig::WheelTorque { drive, .. } => vec![drive],
            AllocatorConfig::SteerPosition { steer } => vec![steer],
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
