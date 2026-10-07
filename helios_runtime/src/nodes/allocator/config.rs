//! The `[nodes.<name>]` sections of the allocator kinds: [`WheelTorqueConfig`]
//! and [`SteerPositionConfig`].
//!
//! An allocator publishes its partial actuator command on a channel named
//! after the node, so its name is the table key and has no field here. Whether
//! that partial reaches the body is the stack's `[actuators]` section.

use serde::{Deserialize, Serialize};

/// Turns a `DriveForce` into one wheel-torque setpoint, `τ = F·r`.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct WheelTorqueConfig {
    /// The `[command]` fold the `DriveForce` is read from.
    pub(crate) input: String,
    /// Wheel radius, in meters.
    pub(crate) wheel_radius: f64,
    /// The body actuator the torque is written to.
    pub(crate) drive: String,
}

/// Turns a `SteerAngle` into one steer-position setpoint.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct SteerPositionConfig {
    /// The `[command]` fold the `SteerAngle` is read from.
    pub(crate) input: String,
    /// The body actuator the steer angle is written to.
    pub(crate) steer: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wheel_torque_section_parses() {
        let config: WheelTorqueConfig = toml::from_str(
            r#"
            input = "drive_cmd"
            wheel_radius = 0.3
            drive = "rear_wheels"
            "#,
        )
        .expect("a full section parses");

        assert_eq!(config.input, "drive_cmd");
        assert_eq!(config.drive, "rear_wheels");
    }

    #[test]
    fn input_is_required() {
        let result: Result<SteerPositionConfig, _> = toml::from_str(r#"steer = "front_axle""#);

        assert!(
            result.is_err(),
            "an allocator without an input must not parse"
        );
    }

    #[test]
    fn an_unknown_key_is_rejected() {
        let result: Result<SteerPositionConfig, _> =
            toml::from_str("input = \"steer_cmd\"\nsteer = \"front_axle\"\nkind = \"x\"");

        assert!(
            result.is_err(),
            "`kind` is removed before the factory sees the section"
        );
    }
}
