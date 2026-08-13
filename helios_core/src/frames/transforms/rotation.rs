//! Frame-tagged rotation (SO(3)).

use crate::frames::{conventions::Frame, quantities::FreeVector};

use nalgebra::UnitQuaternion;
use std::any::type_name;
use std::fmt;
use std::marker::PhantomData;

/// A rotation known at compile time to carry frame `From` into frame `To`.
///
/// Wraps a [`nalgebra::UnitQuaternion`]; the [`PhantomData`] carries the frame
/// pair and costs nothing at runtime. The type parameters read in the same
/// direction as the rotation itself — `Rotation<From, To>` takes a quantity
/// *out of* `From` and *into* `To`, matching the `Qx(from, to)` convention used
/// by [`StateVariable`](crate::frames::StateVariable).
///
/// A rotation acts on a [`FreeVector`] — the *linear* half of the frame algebra
/// (`R·v`, no translation). Its affine sibling [`Transform`](super::Transform)
/// acts on a [`Point`](crate::frames::quantities::Point) and adds the `t`. The
/// carrier a value flows through is the record of which law applies, so
/// "translated a velocity" cannot be written.
///
/// Frame safety is enforced at compile time: [`then`](Self::then) only composes
/// rotations that share the middle frame, and [`act`](Self::act) only accepts a
/// vector already in `From`.
//
// `Clone`/`Copy` are hand-written, not derived: `#[derive(Clone)]` over a
// `PhantomData<(From, To)>` would emit `impl<From: Clone, To: Clone>`,
// constraining the frame markers even though they live only inside the phantom,
// and that over-constrained impl is then invisible to generic code that knows
// only `From: Frame`. The hand-written impls state the honest bound; the body is
// `*self` because the inner quaternion is already `Copy`.
pub struct Rotation<From: Frame, To: Frame>(UnitQuaternion<f64>, PhantomData<(From, To)>);

impl<From: Frame, To: Frame> Clone for Rotation<From, To> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<From: Frame, To: Frame> Copy for Rotation<From, To> {}

// Hand-written for the same reason as `Clone`/`Copy`: a derive would demand
// `From: Debug`/`To: Debug`, which the marker bound does not give. The frame
// tags are printed via `type_name` so the rotation's direction is visible in a
// log line; the quaternion follows.
impl<From: Frame, To: Frame> fmt::Debug for Rotation<From, To> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("Rotation")
            .field(&format_args!(
                "{} -> {}",
                type_name::<From>(),
                type_name::<To>()
            ))
            .field(&self.0)
            .finish()
    }
}

impl<From: Frame, To: Frame> Rotation<From, To> {
    /// Tags a raw quaternion as the rotation `From → To`.
    ///
    /// The caller asserts the quaternion already rotates out of `From` into
    /// `To`; this is the entry point from untyped `nalgebra` rotation math.
    pub fn from_unit_quaternion(q: UnitQuaternion<f64>) -> Self {
        Self(q, PhantomData)
    }

    /// Composition: `self` (`From → To`) followed by `next` (`To → C`), giving
    /// the direct rotation `From → C`. The shared middle frame `To` must match,
    /// so composing mismatched rotations does not compile.
    pub fn then<C: Frame>(self, next: Rotation<To, C>) -> Rotation<From, C> {
        // Quaternion multiplication applies its right operand first, so "`self`
        // then `next`" is `next.0 * self.0`, not the reverse.
        Rotation(next.0 * self.0, PhantomData)
    }

    /// The reverse rotation, `To → From`. The type parameters swap with it.
    pub fn inverse(self) -> Rotation<To, From> {
        Rotation(self.0.inverse(), PhantomData)
    }

    /// Rotates a free vector out of `From` and into `To` (`R·v`; a rotation has
    /// no translation to apply).
    pub fn act(self, v: FreeVector<From>) -> FreeVector<To> {
        FreeVector::from_raw(self.0 * v.into_inner())
    }

    /// Consumes the wrapper, returning the underlying quaternion and dropping
    /// the frame tags.
    pub fn into_inner(self) -> UnitQuaternion<f64> {
        self.0
    }
}

impl<F: Frame> Rotation<F, F> {
    /// The identity rotation of a frame onto itself.
    ///
    /// Defined only for `Rotation<F, F>`: an identity that changed frames would
    /// be a contradiction, so the shared `F` makes it unspellable.
    pub fn identity() -> Self {
        Self(UnitQuaternion::identity(), PhantomData)
    }
}

#[cfg(test)]
mod tests {
    use super::Rotation;
    use crate::frames::conventions::{Enu, Flu};
    use crate::frames::quantities::FreeVector;

    use nalgebra::{UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;

    /// A quarter turn about +Z, read as the body→world rotation `Flu → Enu`.
    fn quarter_turn_z() -> Rotation<Flu, Enu> {
        Rotation::from_unit_quaternion(UnitQuaternion::from_axis_angle(
            &Vector3::z_axis(),
            FRAC_PI_2,
        ))
    }

    /// Floats through a quaternion are not bit-exact; compare within tolerance.
    fn close(a: Vector3<f64>, b: Vector3<f64>) -> bool {
        (a - b).norm() < 1e-9
    }

    #[test]
    fn act_rotates_a_free_vector_into_the_target_frame() {
        // +X turned a quarter turn about +Z lands on +Y.
        let forward = FreeVector::<Flu>::new(1.0, 0.0, 0.0);
        let rotated = quarter_turn_z().act(forward);
        assert!(close(rotated.into_inner(), Vector3::new(0.0, 1.0, 0.0)));
    }

    #[test]
    fn then_composes_through_the_shared_frame() {
        // Two quarter turns about +Z make a half turn: +X → -X. The middle frame
        // (`Enu`) cancels, leaving a `Flu → Flu` rotation.
        let first: Rotation<Flu, Enu> = quarter_turn_z();
        let second: Rotation<Enu, Flu> = Rotation::from_unit_quaternion(
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2),
        );
        let half: Rotation<Flu, Flu> = first.then(second);
        let rotated = half.act(FreeVector::<Flu>::new(1.0, 0.0, 0.0));
        assert!(close(rotated.into_inner(), Vector3::new(-1.0, 0.0, 0.0)));
    }

    #[test]
    fn inverse_undoes_the_rotation() {
        let r = quarter_turn_z();
        let v = FreeVector::<Flu>::new(1.0, 2.0, 3.0);
        // Flu → Enu → Flu round-trips back to the original vector.
        let round_tripped = r.inverse().act(r.act(v));
        assert!(close(round_tripped.into_inner(), v.into_inner()));
    }

    #[test]
    fn identity_leaves_a_vector_unchanged() {
        let v = FreeVector::<Flu>::new(1.0, 2.0, 3.0);
        let acted = Rotation::<Flu, Flu>::identity().act(v);
        assert!(close(acted.into_inner(), v.into_inner()));
    }
}
