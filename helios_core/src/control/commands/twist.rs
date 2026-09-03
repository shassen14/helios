//! Spatial velocity command (linear + angular).

use crate::frames::conventions::{Flu, Frame};
use crate::frames::quantities::{FluVector, FreeVector};

use serde::{Deserialize, Serialize};
use std::ops::{Add, Mul, Neg, Sub};

/// A spatial velocity command: a `linear` velocity (m/s) and an `angular`
/// velocity (rad/s), both expressed in frame `F`. SI units; units are not
/// encoded in the type.
///
/// # Reference-point convention
///
/// `F` names the **axis basis only**. Every `Twist` is understood to describe
/// the motion of the body reference point (the center of mass); `F` does not
/// carry a point of application. The type leans on this in two places:
///
/// - **Summation is meaningful.** Adding a feedforward and a feedback twist
///   ([`Add`]) is only valid when both describe the same point's motion; the
///   convention guarantees that, so the combined command is their sum. A spatial
///   velocity is point-dependent — two points on a rotating body move at
///   different velocities (`v_p = v_o + ω × r`) — so summing twists taken about
///   different points would be meaningless.
/// - **A frame change is a pure rotation.** Converting, say, an
///   [`Enu`](crate::frames::conventions::Enu) twist to an
///   [`Flu`](crate::frames::conventions::Flu) one rotates `linear` and `angular`
///   and nothing else — the reference point is unchanged, so no `ω × r` coupling
///   term arises. Were the point allowed to differ, a plain rotation would be
///   silently wrong.
///
/// # Angular velocity is a pseudovector
///
/// `angular` transforms like an ordinary vector only under rotations between
/// *right-handed* frames. Every convention shipped here
/// ([`Enu`](crate::frames::conventions::Enu),
/// [`Flu`](crate::frames::conventions::Flu)) is right-handed, so rotating a
/// `Twist` is correct. A left-handed convention would require a sign correction
/// on the angular part that a plain rotation would not apply.
///
/// Frame safety is enforced at compile time: linear and angular share one `F`,
/// and arithmetic is defined only between twists of the same frame.
///
/// ```compile_fail
/// use helios_core::control::commands::Twist;
/// use helios_core::frames::conventions::{Enu, Flu};
/// use helios_core::frames::quantities::FreeVector;
///
/// let world: Twist<Enu> = Twist::new(FreeVector::zeros(), FreeVector::zeros());
/// let body: Twist<Flu> = Twist::new(FreeVector::zeros(), FreeVector::zeros());
/// let _ = world + body; // frames differ — does not compile
/// ```
//
// `#[serde(bound = "")]` erases the bounds serde would otherwise infer. `F`
// appears in the serialized fields (`FreeVector<F>`), so serde's derive would
// add `F: Serialize`/`F: Deserialize` — but those are wrong: `FreeVector<F>` is
// (de)serializable for *every* `F: Frame`. The empty bound drops serde's guess,
// leaving only the struct's own `F: Frame`.
#[derive(Debug, PartialEq, Serialize, Deserialize)]
#[serde(bound = "")]
pub struct Twist<F: Frame> {
    linear: FreeVector<F>,
    angular: FreeVector<F>,
}

// Hand-written for the same reason as `FreeVector`: `#[derive(Clone)]`/`Copy`
// would constrain `F: Clone`/`F: Copy`, which generic callers over `F: Frame`
// cannot satisfy. The honest bound is `F: Frame`; the body is `*self` because
// every field is already `Copy`.
impl<F: Frame> Clone for Twist<F> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<F: Frame> Copy for Twist<F> {}

impl<F: Frame> Twist<F> {
    /// Constructs a twist from a linear and an angular velocity, both in frame `F`.
    pub fn new(linear: FreeVector<F>, angular: FreeVector<F>) -> Self {
        Self { linear, angular }
    }

    /// The zero twist in frame `F` — the additive identity for control
    /// summation.
    pub fn zero() -> Self {
        Self {
            linear: FreeVector::zeros(),
            angular: FreeVector::zeros(),
        }
    }

    /// The linear velocity (m/s), in frame `F`.
    pub fn linear(&self) -> FreeVector<F> {
        self.linear
    }

    /// The angular velocity (rad/s), in frame `F`.
    pub fn angular(&self) -> FreeVector<F> {
        self.angular
    }
}

impl Twist<Flu> {
    /// A nonholonomic ground vehicle's command: a forward speed (`surge`, m/s) on
    /// +X and a yaw rate (`yaw`, rad/s) on +Z, with the other four
    /// spatial-velocity DOF — lateral, vertical, roll, pitch — structurally zero.
    ///
    /// This is the planar drive shared by Ackermann and differential-drive
    /// bodies: they steer by turning, never by translating sideways, so lateral
    /// velocity is not merely unset but unactuated. Defined only for the body
    /// [`Flu`] frame, where forward is +X and a positive yaw rate turns left.
    pub fn unicycle(surge: f64, yaw: f64) -> Self {
        Self::new(
            FluVector::new(surge, 0.0, 0.0),
            FluVector::new(0.0, 0.0, yaw),
        )
    }
}

impl<F: Frame> Add for Twist<F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.linear + rhs.linear, self.angular + rhs.angular)
    }
}

impl<F: Frame> Sub for Twist<F> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.linear - rhs.linear, self.angular - rhs.angular)
    }
}

impl<F: Frame> Neg for Twist<F> {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.linear, -self.angular)
    }
}

impl<F: Frame> Mul<f64> for Twist<F> {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self::new(self.linear * scalar, self.angular * scalar)
    }
}

impl<F: Frame> Default for Twist<F> {
    fn default() -> Self {
        Self::zero()
    }
}

#[cfg(test)]
mod tests {
    use crate::control::commands::BodyTwist;
    use crate::frames::quantities::FluVector;

    /// Builds a body-frame twist from linear `(vx, vy, vz)` and angular
    /// `(wx, wy, wz)`.
    fn twist(vx: f64, vy: f64, vz: f64, wx: f64, wy: f64, wz: f64) -> BodyTwist {
        BodyTwist::new(FluVector::new(vx, vy, vz), FluVector::new(wx, wy, wz))
    }

    #[test]
    fn add_sums_linear_and_angular() {
        let sum = twist(1.0, 0.0, 0.0, 0.0, 0.0, 1.0) + twist(0.0, 2.0, 0.0, 0.0, 0.0, 3.0);
        assert_eq!(sum, twist(1.0, 2.0, 0.0, 0.0, 0.0, 4.0));
    }

    #[test]
    fn sub_differences_linear_and_angular() {
        let diff = twist(1.0, 2.0, 3.0, 4.0, 5.0, 6.0) - twist(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
        assert_eq!(diff, twist(0.0, 1.0, 2.0, 3.0, 4.0, 5.0));
    }

    #[test]
    fn neg_flips_linear_and_angular() {
        assert_eq!(
            -twist(1.0, -2.0, 3.0, -4.0, 5.0, -6.0),
            twist(-1.0, 2.0, -3.0, 4.0, -5.0, 6.0)
        );
    }

    #[test]
    fn mul_scales_linear_and_angular() {
        assert_eq!(
            twist(1.0, 2.0, 3.0, 4.0, 5.0, 6.0) * 2.0,
            twist(2.0, 4.0, 6.0, 8.0, 10.0, 12.0)
        );
    }

    #[test]
    fn zero_is_the_additive_identity() {
        let t = twist(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(t + BodyTwist::zero(), t);
        assert_eq!(BodyTwist::default(), BodyTwist::zero());
    }

    #[test]
    fn folds_control_contributions_onto_one_output() {
        // Feedforward + feedback + … collapse to a single output twist by summing
        // from the zero identity.
        let contributions = [
            twist(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            twist(0.0, 2.0, 0.0, 0.0, 0.0, 0.0),
            twist(0.0, 0.0, 3.0, 0.0, 0.0, 0.0),
        ];
        let total = contributions
            .into_iter()
            .fold(BodyTwist::zero(), |acc, c| acc + c);
        assert_eq!(total, twist(1.0, 2.0, 3.0, 0.0, 0.0, 0.0));
    }

    #[test]
    fn accessors_return_linear_and_angular() {
        let t = twist(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(t.linear(), FluVector::new(1.0, 2.0, 3.0));
        assert_eq!(t.angular(), FluVector::new(4.0, 5.0, 6.0));
    }

    #[test]
    fn unicycle_places_surge_on_x_and_yaw_on_z() {
        assert_eq!(
            BodyTwist::unicycle(2.0, 0.5),
            twist(2.0, 0.0, 0.0, 0.0, 0.0, 0.5)
        );
    }

    #[test]
    fn unicycle_leaves_the_other_four_dof_zero() {
        // No sway or heave in the linear part, no roll or pitch in the angular.
        let t = BodyTwist::unicycle(3.0, -1.0);
        assert_eq!(t.linear(), FluVector::new(3.0, 0.0, 0.0));
        assert_eq!(t.angular(), FluVector::new(0.0, 0.0, -1.0));
    }

    #[test]
    fn unicycle_zero_inputs_is_the_zero_twist() {
        assert_eq!(BodyTwist::unicycle(0.0, 0.0), BodyTwist::zero());
    }

    #[test]
    fn serde_round_trips_through_json() {
        let t = twist(1.5, -2.5, 3.5, -4.5, 5.5, -6.5);
        let json = serde_json::to_string(&t).expect("serialize");
        let back: BodyTwist = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(t, back);
    }
}
