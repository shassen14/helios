//! [`TwistTeleopConfig`] — the `[nodes.<name>]` section of a `TwistTeleop` entry.

use serde::{Deserialize, Serialize};

/// Per-DOF scale from the operator's normalized intent to a body twist:
/// translation (`surge`/`sway`/`heave`) in m/s, rotation (`roll`/`pitch`/`yaw`)
/// in rad/s, applied to the matching deflection. An axis left out scales at
/// `0.0`, a true statement that the body cannot drive it (a car names only
/// `surge` and `yaw`), not a dead field.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct TwistTeleopConfig {
    #[serde(default)]
    pub(crate) surge: f64,
    #[serde(default)]
    pub(crate) sway: f64,
    #[serde(default)]
    pub(crate) heave: f64,
    #[serde(default)]
    pub(crate) roll: f64,
    #[serde(default)]
    pub(crate) pitch: f64,
    #[serde(default)]
    pub(crate) yaw: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_omitted_axis_scales_at_zero() {
        let config: TwistTeleopConfig =
            toml::from_str("surge = 5.0\nyaw = 1.0").expect("a car's section parses");

        assert_eq!(
            [config.sway, config.heave, config.roll, config.pitch],
            [0.0; 4]
        );
    }

    #[test]
    fn a_misspelled_axis_is_rejected() {
        let result: Result<TwistTeleopConfig, _> = toml::from_str("surg = 5.0");

        assert!(result.is_err(), "`surg` is not an axis");
    }
}
