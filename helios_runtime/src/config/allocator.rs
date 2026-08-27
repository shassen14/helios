use crate::config::CommandSpace;

use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum AllocatorConfig {
    KinematicAckermann {
        wheelbase: f64,
        wheel_radius: f64,
        drive: String,
        steer: String,
    },
    WheelTorque {
        wheel_radius: f64,
        drive: String,
    },
    SteerPosition {
        steer: String,
    },
}

impl AllocatorConfig {
    pub(crate) fn get_kind_str(&self) -> &str {
        match self {
            AllocatorConfig::KinematicAckermann { .. } => "KinematicAckermann",
            AllocatorConfig::WheelTorque { .. } => "WheelTorque",
            AllocatorConfig::SteerPosition { .. } => "SteerPosition",
        }
    }

    /// The command space this allocator consumes. The allocator *defines* the
    /// command seam, so this is the authoritative `T` the assembler wires the
    /// `command` channel, the fold, and this allocator's input around.
    pub(crate) fn command_space(&self) -> CommandSpace {
        match self {
            AllocatorConfig::KinematicAckermann { .. } => CommandSpace::BodyTwist,
            AllocatorConfig::WheelTorque { .. } => CommandSpace::DriveForce,
            AllocatorConfig::SteerPosition { .. } => CommandSpace::SteerAngle,
        }
    }
}
