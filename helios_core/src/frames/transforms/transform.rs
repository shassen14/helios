//! Frame-tagged rigid transform (SE(3)).

use crate::frames::{conventions::Frame, quantities::Point, transforms::rotation::Rotation};

use nalgebra::{Isometry3, Point3, Translation3};
use std::any::type_name;
use std::fmt;
use std::marker::PhantomData;

/// A rigid transform known at compile time to carry frame `From` into `To`.
///
/// Wraps a [`nalgebra::Isometry3`] (a rotation plus a translation); the
/// [`PhantomData`] carries the frame pair at no runtime cost. Like
/// [`Rotation`], the parameters read `From → To`.
///
/// A transform acts on a [`Point`] — the *affine* half of the frame algebra
/// (`R·p + t`). Contrast [`Rotation::act`], which acts on a
/// [`FreeVector`](crate::frames::quantities::FreeVector) and applies `R` alone:
/// a location translates, a free vector does not, and the argument types are
/// what enforce the distinction. Pull the rotational part out with
/// [`rotation`](Self::rotation) to act on free vectors.
///
/// Frame safety is enforced at compile time: [`then`](Self::then) only composes
/// transforms that share the middle frame, and [`act`](Self::act) only accepts
/// a point already in `From`.
//
// `Clone`/`Copy` are hand-written for the same reason as [`Rotation`]: deriving
// them over the `PhantomData<(From, To)>` would constrain the frame markers,
// hiding the impls from generic code that knows only `From: Frame`. The body is
// `*self` because the inner isometry is already `Copy`.
pub struct Transform<From: Frame, To: Frame>(Isometry3<f64>, PhantomData<(From, To)>);

impl<From: Frame, To: Frame> Clone for Transform<From, To> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<From: Frame, To: Frame> Copy for Transform<From, To> {}

// Hand-written for the same reason as `Clone`/`Copy`: a derive would demand
// `From: Debug`/`To: Debug`. The frame tags are printed via `type_name` so the
// transform's direction is visible in a log line; the isometry follows.
impl<From: Frame, To: Frame> fmt::Debug for Transform<From, To> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("Transform")
            .field(&format_args!(
                "{} -> {}",
                type_name::<From>(),
                type_name::<To>()
            ))
            .field(&self.0)
            .finish()
    }
}

impl<From: Frame, To: Frame> Transform<From, To> {
    /// Tags a raw isometry as the transform `From → To`.
    ///
    /// The caller asserts it already carries `From` into `To`; the entry point
    /// from untyped `nalgebra` transform math.
    pub fn from_isometry(iso: Isometry3<f64>) -> Self {
        Self(iso, PhantomData)
    }

    /// Builds a transform from a typed [`Rotation`] and a translation, keeping
    /// the rotational part as its own carrier rather than re-deriving a
    /// quaternion here.
    pub fn from_parts(rotation: Rotation<From, To>, translation: Translation3<f64>) -> Self {
        Self(
            Isometry3::from_parts(translation, rotation.into_inner()),
            PhantomData,
        )
    }

    /// Composition: `self` (`From → To`) followed by `next` (`To → C`), giving
    /// the direct transform `From → C`. The shared middle frame `To` must match,
    /// so composing mismatched transforms does not compile.
    pub fn then<C: Frame>(self, next: Transform<To, C>) -> Transform<From, C> {
        // Isometry multiplication applies its right operand first, so "`self`
        // then `next`" is `next.0 * self.0`, not the reverse.
        Transform(next.0 * self.0, PhantomData)
    }

    /// The inverse transform, `To → From`. The type parameters swap with it.
    pub fn inverse(self) -> Transform<To, From> {
        Transform(self.0.inverse(), PhantomData)
    }

    /// Maps a point out of `From` and into `To` (`R·p + t`).
    pub fn act(self, p: Point<From>) -> Point<To> {
        // `Isometry3 * Vector3` would rotate only; feeding it a `Point3` is what
        // pulls in the translation, giving the affine `R·p + t`. `.coords` drops
        // back to the `Vector3` that `Point::from_raw` tags.
        let mapped = self.0 * Point3::from(p.into_inner());
        Point::from_raw(mapped.coords)
    }

    /// The rotational part as its typed sibling, so a caller holding a transform
    /// can act on free vectors via `t.rotation().act(v)` without translating
    /// them.
    pub fn rotation(self) -> Rotation<From, To> {
        Rotation::from_unit_quaternion(self.0.rotation)
    }

    /// Consumes the wrapper, returning the underlying isometry and dropping the
    /// frame tags.
    pub fn into_inner(self) -> Isometry3<f64> {
        self.0
    }
}

impl<F: Frame> Transform<F, F> {
    /// The identity transform of a frame onto itself.
    ///
    /// Defined only for `Transform<F, F>`: an identity that changed frames would
    /// be a contradiction, so the shared `F` makes it unspellable.
    pub fn identity() -> Self {
        Self(Isometry3::identity(), PhantomData)
    }
}

#[cfg(test)]
mod tests {
    use super::{Rotation, Transform};
    use crate::frames::conventions::{Enu, Flu};
    use crate::frames::quantities::{FreeVector, Point};

    use nalgebra::{Translation3, UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;

    /// A quarter turn about +Z paired with a +10 shift along X, read `Flu → Enu`.
    fn turn_and_shift() -> Transform<Flu, Enu> {
        let rotation = Rotation::from_unit_quaternion(UnitQuaternion::from_axis_angle(
            &Vector3::z_axis(),
            FRAC_PI_2,
        ));
        Transform::from_parts(rotation, Translation3::new(10.0, 0.0, 0.0))
    }

    /// A pure +10 shift along X, no rotation, read `Flu → Enu`.
    fn shift_x() -> Transform<Flu, Enu> {
        Transform::from_parts(
            Rotation::from_unit_quaternion(UnitQuaternion::identity()),
            Translation3::new(10.0, 0.0, 0.0),
        )
    }

    /// Floats through an isometry are not bit-exact; compare within tolerance.
    fn close(a: Vector3<f64>, b: Vector3<f64>) -> bool {
        (a - b).norm() < 1e-9
    }

    #[test]
    fn act_applies_rotation_then_translation() {
        // (1,0,0) rotates a quarter turn to (0,1,0), then shifts +10 in X.
        let mapped = turn_and_shift().act(Point::<Flu>::new(1.0, 0.0, 0.0));
        assert!(close(mapped.into_inner(), Vector3::new(10.0, 1.0, 0.0)));
    }

    #[test]
    fn translation_moves_a_point_but_not_a_free_vector() {
        // The load-bearing test for the affine/linear split. Under the same pure
        // translation, a point moves by +10 in X while a free vector — acted on
        // through the rotational part, which is identity here — is unchanged.
        let t = shift_x();
        let moved_point = t.act(Point::<Flu>::new(1.0, 2.0, 3.0));
        let moved_vector = t.rotation().act(FreeVector::<Flu>::new(1.0, 2.0, 3.0));
        assert!(close(
            moved_point.into_inner(),
            Vector3::new(11.0, 2.0, 3.0)
        ));
        assert!(close(
            moved_vector.into_inner(),
            Vector3::new(1.0, 2.0, 3.0)
        ));
    }

    #[test]
    fn then_composes_through_the_shared_frame() {
        // Shift +10 (Flu → Enu) then shift −10 (Enu → Flu) cancels to identity.
        let out: Transform<Flu, Enu> = shift_x();
        let back: Transform<Enu, Flu> = Transform::from_parts(
            Rotation::from_unit_quaternion(UnitQuaternion::identity()),
            Translation3::new(-10.0, 0.0, 0.0),
        );
        let round: Transform<Flu, Flu> = out.then(back);
        let p = Point::<Flu>::new(1.0, 2.0, 3.0);
        assert!(close(round.act(p).into_inner(), p.into_inner()));
    }

    #[test]
    fn inverse_round_trips_a_point() {
        let t = turn_and_shift();
        let p = Point::<Flu>::new(1.0, 2.0, 3.0);
        // Flu → Enu → Flu returns the original point.
        let round_tripped = t.inverse().act(t.act(p));
        assert!(close(round_tripped.into_inner(), p.into_inner()));
    }

    #[test]
    fn rotation_matches_the_transform_rotational_part() {
        // `t.rotation().act(v)` agrees with rotating the same vector directly.
        let t = turn_and_shift();
        let v = FreeVector::<Flu>::new(1.0, 0.0, 0.0);
        let via_transform = t.rotation().act(v);
        assert!(close(
            via_transform.into_inner(),
            Vector3::new(0.0, 1.0, 0.0)
        ));
    }
}
