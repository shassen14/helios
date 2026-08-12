//! Spatial force command (force + torque).

use crate::frames::conventions::Frame;
use crate::frames::quantities::FreeVector;

use serde::{Deserialize, Serialize};
use std::ops::{Add, Mul, Neg, Sub};

/// A spatial force command: a `force` (N) and a `torque` (N·m), both expressed
/// in frame `F`. SI units; units are not encoded in the type.
///
/// # Reference-point convention
///
/// `F` names the **axis basis only**. Every `Wrench` is understood to act about
/// the body reference point (the center of mass); `F` does not carry a point of
/// application. The type leans on this in two places:
///
/// - **Summation is meaningful.** Adding a feedforward and a feedback wrench
///   ([`Add`]) is only valid when both act about the same point; the convention
///   guarantees that, so the final command is their sum.
/// - **A frame change is a pure rotation.** Converting, say, an
///   [`Enu`](crate::frames::conventions::Enu) wrench to an
///   [`Flu`](crate::frames::conventions::Flu) one rotates `force` and `torque`
///   and nothing else — the reference point is unchanged, so no `r × f` coupling
///   term arises. Were the point allowed to differ, a plain rotation would be
///   silently wrong.
///
/// # Torque is a pseudovector
///
/// `torque` transforms like an ordinary vector only under rotations between
/// *right-handed* frames. Every convention shipped here
/// ([`Enu`](crate::frames::conventions::Enu),
/// [`Flu`](crate::frames::conventions::Flu)) is right-handed, so rotating a
/// `Wrench` is correct. A left-handed convention would require a sign correction
/// on the torque that a plain rotation would not apply.
///
/// Frame safety is enforced at compile time: force and torque share one `F`, and
/// arithmetic is defined only between wrenches of the same frame.
///
/// ```compile_fail
/// use helios_core::control::commands::Wrench;
/// use helios_core::frames::conventions::{Enu, Flu};
/// use helios_core::frames::quantities::FreeVector;
///
/// let world: Wrench<Enu> = Wrench::new(FreeVector::zeros(), FreeVector::zeros());
/// let body: Wrench<Flu> = Wrench::new(FreeVector::zeros(), FreeVector::zeros());
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
pub struct Wrench<F: Frame> {
    force: FreeVector<F>,
    torque: FreeVector<F>,
}

// Hand-written for the same reason as `FreeVector`: `#[derive(Clone)]`/`Copy`
// would constrain `F: Clone`/`F: Copy`, which generic callers over `F: Frame`
// cannot satisfy. The honest bound is `F: Frame`; the body is `*self` because
// every field is already `Copy`.
impl<F: Frame> Clone for Wrench<F> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<F: Frame> Copy for Wrench<F> {}

impl<F: Frame> Wrench<F> {
    /// Constructs a wrench from a force and a torque, both in frame `F`.
    pub fn new(force: FreeVector<F>, torque: FreeVector<F>) -> Self {
        Self { force, torque }
    }

    /// The zero wrench in frame `F` — the additive identity for control
    /// summation.
    pub fn zero() -> Self {
        Self {
            force: FreeVector::zeros(),
            torque: FreeVector::zeros(),
        }
    }

    /// The force component (N), in frame `F`.
    pub fn force(&self) -> FreeVector<F> {
        self.force
    }

    /// The torque component (N·m), in frame `F`.
    pub fn torque(&self) -> FreeVector<F> {
        self.torque
    }
}

impl<F: Frame> Add for Wrench<F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.force + rhs.force, self.torque + rhs.torque)
    }
}

impl<F: Frame> Sub for Wrench<F> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.force - rhs.force, self.torque - rhs.torque)
    }
}

impl<F: Frame> Neg for Wrench<F> {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.force, -self.torque)
    }
}

impl<F: Frame> Mul<f64> for Wrench<F> {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self::new(self.force * scalar, self.torque * scalar)
    }
}

impl<F: Frame> Default for Wrench<F> {
    fn default() -> Self {
        Self::zero()
    }
}

#[cfg(test)]
mod tests {
    use crate::control::commands::BodyWrench;
    use crate::frames::quantities::FluVector;

    /// Builds a body-frame wrench from force `(fx, fy, fz)` and torque
    /// `(tx, ty, tz)`.
    fn wrench(fx: f64, fy: f64, fz: f64, tx: f64, ty: f64, tz: f64) -> BodyWrench {
        BodyWrench::new(FluVector::new(fx, fy, fz), FluVector::new(tx, ty, tz))
    }

    #[test]
    fn add_sums_force_and_torque() {
        let sum = wrench(1.0, 0.0, 0.0, 0.0, 0.0, 1.0) + wrench(0.0, 2.0, 0.0, 0.0, 0.0, 3.0);
        assert_eq!(sum, wrench(1.0, 2.0, 0.0, 0.0, 0.0, 4.0));
    }

    #[test]
    fn sub_differences_force_and_torque() {
        let diff = wrench(1.0, 2.0, 3.0, 4.0, 5.0, 6.0) - wrench(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
        assert_eq!(diff, wrench(0.0, 1.0, 2.0, 3.0, 4.0, 5.0));
    }

    #[test]
    fn neg_flips_force_and_torque() {
        assert_eq!(
            -wrench(1.0, -2.0, 3.0, -4.0, 5.0, -6.0),
            wrench(-1.0, 2.0, -3.0, 4.0, -5.0, 6.0)
        );
    }

    #[test]
    fn mul_scales_force_and_torque() {
        assert_eq!(
            wrench(1.0, 2.0, 3.0, 4.0, 5.0, 6.0) * 2.0,
            wrench(2.0, 4.0, 6.0, 8.0, 10.0, 12.0)
        );
    }

    #[test]
    fn zero_is_the_additive_identity() {
        let w = wrench(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(w + BodyWrench::zero(), w);
        assert_eq!(BodyWrench::default(), BodyWrench::zero());
    }

    #[test]
    fn folds_control_contributions_onto_one_output() {
        // Feedforward + feedback + … collapse to a single output wrench by summing
        // from the zero identity.
        let contributions = [
            wrench(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            wrench(0.0, 2.0, 0.0, 0.0, 0.0, 0.0),
            wrench(0.0, 0.0, 3.0, 0.0, 0.0, 0.0),
        ];
        let total = contributions
            .into_iter()
            .fold(BodyWrench::zero(), |acc, c| acc + c);
        assert_eq!(total, wrench(1.0, 2.0, 3.0, 0.0, 0.0, 0.0));
    }

    #[test]
    fn accessors_return_force_and_torque() {
        let w = wrench(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(w.force(), FluVector::new(1.0, 2.0, 3.0));
        assert_eq!(w.torque(), FluVector::new(4.0, 5.0, 6.0));
    }

    #[test]
    fn serde_round_trips_through_json() {
        let w = wrench(1.5, -2.5, 3.5, -4.5, 5.5, -6.5);
        let json = serde_json::to_string(&w).expect("serialize");
        let back: BodyWrench = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(w, back);
    }
}
