//! Scalar steering-joint angle command.

use serde::{Deserialize, Serialize};
use std::ops::{Add, Mul, Neg, Sub};

/// A scalar angle command for a single steering actuator, in radians (rad). SI
/// units; units are not encoded in the type.
///
/// # Axis-less by design
///
/// Like [`DriveForce`](crate::control::commands::DriveForce) and unlike
/// [`Wrench`](crate::control::commands::Wrench), this command carries no
/// coordinate frame — a single scalar has no basis to rotate. It names the desired
/// steering angle, not which joint realizes it; that identity is stamped as an
/// `ActuatorId` downstream, where the command is lifted into an actuator setpoint.
///
/// # A bounded joint angle, not a circular quantity
///
/// A steering joint travels a limited arc, well within ±π, so this is an ordinary
/// bounded scalar — **not** a heading on the circle. Arithmetic is plain: adding a
/// feedforward angle and a feedback correction ([`Add`]) is real superposition,
/// and there is deliberately no angle-wrapping. Wrapping would be correct for a
/// heading and wrong here: a commanded 3.2 rad is an out-of-range steering angle to
/// be clamped by the actuator, not a synonym for −3.08 rad.
///
/// # Summation is superposition
///
/// Adding two `SteerAngle`s combines contributions to the same steering actuator.
/// Today the lateral law is feedforward-only, so a single term flows through; when
/// a feedback correction is added, it sums here with no change to this type.
/// [`zero`](SteerAngle::zero) is the additive identity.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SteerAngle(f64);

impl SteerAngle {
    /// Constructs a steering angle from a value in radians.
    pub fn new(radians: f64) -> Self {
        Self(radians)
    }

    /// The zero steering angle — the additive identity for control summation.
    pub fn zero() -> Self {
        Self(0.0)
    }

    /// The commanded angle, in radians (rad).
    pub fn radians(&self) -> f64 {
        self.0
    }
}

impl Add for SteerAngle {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.0 + rhs.0)
    }
}

impl Sub for SteerAngle {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.0 - rhs.0)
    }
}

impl Neg for SteerAngle {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.0)
    }
}

impl Mul<f64> for SteerAngle {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self::new(self.0 * scalar)
    }
}

impl Default for SteerAngle {
    fn default() -> Self {
        Self::zero()
    }
}

#[cfg(test)]
mod tests {
    use super::SteerAngle;

    // Values are binary-exact fractions (quarters/eighths) so equality asserts on
    // f64 hold without rounding slack.

    #[test]
    fn add_sums_angle() {
        assert_eq!(
            SteerAngle::new(0.25) + SteerAngle::new(0.125),
            SteerAngle::new(0.375)
        );
    }

    #[test]
    fn sub_differences_angle() {
        assert_eq!(
            SteerAngle::new(0.5) - SteerAngle::new(0.125),
            SteerAngle::new(0.375)
        );
    }

    #[test]
    fn neg_flips_angle() {
        assert_eq!(-SteerAngle::new(0.25), SteerAngle::new(-0.25));
    }

    #[test]
    fn mul_scales_angle() {
        // Guards against multiplying-by-adding: 0.25 * 3 is 0.75, not 3.25.
        assert_eq!(SteerAngle::new(0.25) * 3.0, SteerAngle::new(0.75));
    }

    #[test]
    fn zero_is_the_additive_identity() {
        let a = SteerAngle::new(0.25);
        assert_eq!(a + SteerAngle::zero(), a);
        assert_eq!(SteerAngle::default(), SteerAngle::zero());
    }

    #[test]
    fn addition_does_not_wrap() {
        // A steering angle is a bounded joint angle, not a heading: summing past
        // the geometric range yields an out-of-range value to be clamped by the
        // actuator, never a wrapped-around one.
        let sum = SteerAngle::new(3.0) + SteerAngle::new(0.25);
        assert_eq!(sum, SteerAngle::new(3.25));
    }

    #[test]
    fn accessor_returns_radians() {
        assert_eq!(SteerAngle::new(0.25).radians(), 0.25);
    }

    #[test]
    fn serde_round_trips_through_json() {
        let a = SteerAngle::new(-0.35);
        let json = serde_json::to_string(&a).expect("serialize");
        let back: SteerAngle = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(a, back);
    }
}
