//! Scalar drive-axis force command.

use serde::{Deserialize, Serialize};
use std::ops::{Add, Mul, Neg, Sub};

/// A scalar force command for a single drive actuator, in newtons (N). SI units;
/// units are not encoded in the type.
///
/// # Axis-less by design
///
/// Unlike [`Wrench`](crate::control::commands::Wrench) and
/// [`Twist`](crate::control::commands::Twist), this command carries no coordinate
/// frame: a single scalar has no basis to rotate. It names *how much* drive effort
/// is wanted, not *which* actuator delivers it — that identity is stamped as an
/// `ActuatorId` downstream, where the command is lifted into an actuator setpoint.
/// This is the decoupled-vehicle shape: per-degree-of-freedom scalars that stay
/// anonymous until allocation.
///
/// # Summation is superposition
///
/// Adding two `DriveForce`s ([`Add`]) combines contributions to the *same* drive
/// actuator — a feedforward term plus a feedback correction collapse to one
/// commanded force. [`zero`](DriveForce::zero) is the additive identity, so a set
/// of contributions folds onto a single output.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DriveForce(f64);

impl DriveForce {
    /// Constructs a drive force from a value in newtons.
    pub fn new(newtons: f64) -> Self {
        Self(newtons)
    }

    /// The zero drive force — the additive identity for control summation.
    pub fn zero() -> Self {
        Self(0.0)
    }

    /// The commanded force, in newtons (N).
    pub fn newtons(&self) -> f64 {
        self.0
    }
}

impl Add for DriveForce {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.0 + rhs.0)
    }
}

impl Sub for DriveForce {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.0 - rhs.0)
    }
}

impl Neg for DriveForce {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.0)
    }
}

impl Mul<f64> for DriveForce {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self::new(self.0 * scalar)
    }
}

impl Default for DriveForce {
    fn default() -> Self {
        Self::zero()
    }
}

#[cfg(test)]
mod tests {
    use super::DriveForce;

    #[test]
    fn add_sums_force() {
        assert_eq!(
            DriveForce::new(120.0) + DriveForce::new(30.0),
            DriveForce::new(150.0)
        );
    }

    #[test]
    fn sub_differences_force() {
        assert_eq!(
            DriveForce::new(120.0) - DriveForce::new(30.0),
            DriveForce::new(90.0)
        );
    }

    #[test]
    fn neg_flips_force() {
        assert_eq!(-DriveForce::new(120.0), DriveForce::new(-120.0));
    }

    #[test]
    fn mul_scales_force() {
        // Guards against multiplying-by-adding: 120 * 2 is 240, not 122.
        assert_eq!(DriveForce::new(120.0) * 2.0, DriveForce::new(240.0));
    }

    #[test]
    fn zero_is_the_additive_identity() {
        let f = DriveForce::new(120.0);
        assert_eq!(f + DriveForce::zero(), f);
        assert_eq!(DriveForce::default(), DriveForce::zero());
    }

    #[test]
    fn folds_feedforward_and_feedback_onto_one_output() {
        // A feedforward force plus a feedback correction collapse to one commanded
        // drive force by summing from the zero identity — the longitudinal loop's
        // FF + FB superposition.
        let contributions = [DriveForce::new(100.0), DriveForce::new(20.0)];
        let total = contributions
            .into_iter()
            .fold(DriveForce::zero(), |acc, c| acc + c);
        assert_eq!(total, DriveForce::new(120.0));
    }

    #[test]
    fn accessor_returns_newtons() {
        assert_eq!(DriveForce::new(120.0).newtons(), 120.0);
    }

    #[test]
    fn serde_round_trips_through_json() {
        let f = DriveForce::new(-42.5);
        let json = serde_json::to_string(&f).expect("serialize");
        let back: DriveForce = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(f, back);
    }
}
