//! The `[nodes.<name>]` sections of the controller kinds: [`DirectTwistConfig`],
//! [`LongitudinalVelocityConfig`], [`RoadLoadConfig`] and [`BicycleSteerConfig`].
//!
//! A controller publishes its command on a channel named after the node, so its
//! name is the table key and has no field here. Which command fold it joins,
//! and whether the fold requires it, is the stack's `[command]` section.

use serde::{Deserialize, Serialize};

/// Passes the reference's body-frame velocity through as a `BodyTwist`. Use it
/// when a path follower already computes the velocity and no feedback is
/// needed. It has no settings.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct DirectTwistConfig {}

/// A PID on body-forward speed, emitting a `DriveForce`.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct LongitudinalVelocityConfig {
    /// Newtons per m/s of speed error.
    pub(crate) proportional_gain: f64,
    pub(crate) integral_gain: f64,
    pub(crate) derivative_gain: f64,
    /// Symmetric bound on the integral accumulator (anti-windup). Omitted or
    /// `0.0` leaves the integrator unclamped; a positive value caps the
    /// integral's force authority at `integral_gain · integral_clamp`.
    #[serde(default)]
    pub(crate) integral_clamp: f64,
}

/// The open-loop road-load force that holds the reference speed, emitting a
/// `DriveForce`: rolling resistance plus drag.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct RoadLoadConfig {
    /// Rolling-resistance force, in newtons.
    pub(crate) c_roll: f64,
    /// Drag coefficient, in N·s²/m², applied to the square of the reference
    /// speed.
    pub(crate) c_drag: f64,
}

/// Inverts the bicycle model, turning the reference curvature into a
/// `SteerAngle`.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct BicycleSteerConfig {
    /// Distance between the axles, in meters.
    pub(crate) wheelbase: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn direct_twist_takes_an_empty_section() {
        let config: Result<DirectTwistConfig, _> = toml::from_str("");

        assert!(config.is_ok(), "DirectTwist has no settings");
    }

    #[test]
    fn integral_clamp_defaults_to_unclamped() {
        let config: LongitudinalVelocityConfig = toml::from_str(
            "proportional_gain = 3000.0\nintegral_gain = 30.0\nderivative_gain = 0.0",
        )
        .expect("a section without a clamp parses");

        assert_eq!(config.integral_clamp, 0.0);
    }

    #[test]
    fn state_source_is_rejected() {
        // `state_source` was accepted and never read; a controller reads the
        // estimate the graph wires to it.
        let result: Result<LongitudinalVelocityConfig, _> = toml::from_str(
            "proportional_gain = 3000.0\nintegral_gain = 30.0\nderivative_gain = 0.0\n\
             state_source = \"ground_truth\"",
        );

        assert!(result.is_err(), "`state_source` is not a controller key");
    }
}
