//! Frame-tagged affine point.

use crate::frames::conventions::Frame;
use crate::frames::quantities::FreeVector;
use crate::frames::transforms::Transform;

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use std::marker::PhantomData;
use std::ops::{Add, Sub};

/// An affine position known at compile time to be in frame `F`.
///
/// A point is a *location*, not a magnitude: under a transform `T = (R, t)` it
/// maps as `R·p + t` — it both rotates and translates. That translation is what
/// separates it from [`FreeVector`], which only rotates; encoding the difference
/// in the type is what makes "translated a velocity" (a silent, dimensionally
/// valid bug) unspellable.
///
/// The algebra is deliberately affine, not linear:
///
/// - [`Point`] − [`Point`] → [`FreeVector`] — the displacement between two
///   locations.
/// - [`Point`] ± [`FreeVector`] → [`Point`] — translating a location.
/// - [`Point`] + [`Point`] is **not defined** — the sum of two locations is
///   geometrically meaningless, and there is no scalar multiplication or
///   negation for the same reason. Attempting it fails to compile:
///
/// ```compile_fail
/// use helios_core::frames::conventions::Enu;
/// use helios_core::frames::quantities::Point;
///
/// let a = Point::<Enu>::new(1.0, 0.0, 0.0);
/// let b = Point::<Enu>::new(0.0, 1.0, 0.0);
/// let _ = a + b; // no `Add<Point>` for `Point` — does not compile
/// ```
///
/// Use [`raw`](Self::raw) / [`into_inner`](Self::into_inner) to drop the frame
/// tag when handing coordinates to code that operates on untyped vectors (frame
/// conversions, TF math).
//
// `#[serde(skip)]` on the phantom keeps `F` out of the serialized fields. serde's
// derive adds a `Serialize`/`Deserialize` bound for every type parameter that
// appears in a serialized field; skipping the phantom means `F` appears in none,
// so no bound on `F` is inferred. The wire form is just the inner vector, and the
// phantom is rebuilt from `Default` on read.
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct Point<F: Frame>(Vector3<f64>, #[serde(skip)] PhantomData<F>);

// `Clone`/`Copy` are hand-written rather than derived, for the same reason as
// `FreeVector`: `#[derive(Clone)]` on a type holding `PhantomData<F>` generates
// `impl<F: Clone> Clone`, over-constraining `F` even though it lives only inside
// a `PhantomData`. That impl is then invisible to generic code that knows only
// `F: Frame`. Writing the impls by hand states the true bound (`F: Frame`), which
// every marker satisfies. The body is `*self`: both real fields are `Copy`.
impl<F: Frame> Clone for Point<F> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<F: Frame> Copy for Point<F> {}

impl<F: Frame> Point<F> {
    /// Constructs a point from its coordinates, tagged with frame `F`.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vector3::new(x, y, z), PhantomData)
    }

    /// Tags an existing raw vector as a point in frame `F`.
    ///
    /// The caller asserts the coordinates are already expressed in `F`; this is
    /// the entry point from untyped vector math.
    pub fn from_raw(vec: Vector3<f64>) -> Self {
        Self(vec, PhantomData)
    }

    /// The origin (all-zero coordinates) of frame `F`.
    pub fn origin() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// The first coordinate (E for [`Enu`](crate::frames::conventions::Enu), F for [`Flu`](crate::frames::conventions::Flu)).
    pub fn x(&self) -> f64 {
        self.0.x
    }

    /// The second coordinate (N for [`Enu`](crate::frames::conventions::Enu), L for [`Flu`](crate::frames::conventions::Flu)).
    pub fn y(&self) -> f64 {
        self.0.y
    }

    /// The third coordinate (U for both [`Enu`](crate::frames::conventions::Enu) and [`Flu`](crate::frames::conventions::Flu)).
    pub fn z(&self) -> f64 {
        self.0.z
    }

    /// Borrows the underlying raw vector, dropping the frame tag.
    pub fn raw(&self) -> &Vector3<f64> {
        &self.0
    }

    /// Consumes the wrapper, returning the underlying raw vector.
    pub fn into_inner(self) -> Vector3<f64> {
        self.0
    }

    /// Re-expresses this point in frame `To` under the transform's full affine
    /// law, `R·p + t` — a location both rotates and translates.
    ///
    /// Forwards to [`Transform::act`], which owns the math; this method is the
    /// point-side spelling of the same crossing. The transform's `F → To` pair
    /// must match this point's frame, so re-expressing through the wrong edge
    /// does not compile. The affine `t` is applied here precisely because the
    /// receiver is a [`Point`]; [`FreeVector::reexpress`](super::FreeVector::reexpress)
    /// drops it.
    pub fn reexpress<To: Frame>(self, transform: &Transform<F, To>) -> Point<To> {
        transform.act(self)
    }
}

// Point − Point → FreeVector: the displacement from `rhs` to `self`. The result
// is a free vector, not a point — a difference of locations has direction and
// magnitude but no anchor, and translates by `R·v` alone.
impl<F: Frame> Sub for Point<F> {
    type Output = FreeVector<F>;
    fn sub(self, rhs: Self) -> FreeVector<F> {
        FreeVector::from_raw(self.0 - rhs.0)
    }
}

// Point + FreeVector → Point: translate a location by a displacement, staying a
// location in the same frame.
impl<F: Frame> Add<FreeVector<F>> for Point<F> {
    type Output = Self;
    fn add(self, rhs: FreeVector<F>) -> Self {
        let translated = self.0 + rhs.raw();
        Self::from_raw(translated)
    }
}

// Point − FreeVector → Point: the inverse translation.
impl<F: Frame> Sub<FreeVector<F>> for Point<F> {
    type Output = Self;
    fn sub(self, rhs: FreeVector<F>) -> Self {
        let translated = self.0 - rhs.raw();
        Self::from_raw(translated)
    }
}

#[cfg(test)]
mod tests {
    use crate::frames::conventions::{Enu, Flu};
    use crate::frames::quantities::{FreeVector, Point};
    use crate::frames::transforms::{Rotation, Transform};

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
    fn point_minus_point_is_a_free_vector() {
        // The `let: FreeVector<Enu>` binding also pins the return type: this fails
        // to compile if `Sub` yields anything other than a free vector.
        let displacement: FreeVector<Enu> = Point::new(4.0, 5.0, 6.0) - Point::new(1.0, 2.0, 3.0);
        assert_eq!(displacement, FreeVector::new(3.0, 3.0, 3.0));
    }

    #[test]
    fn translating_a_point_stays_a_point() {
        let moved = Point::<Enu>::new(1.0, 2.0, 3.0) + FreeVector::new(10.0, 20.0, 30.0);
        assert_eq!(moved, Point::new(11.0, 22.0, 33.0));
    }

    #[test]
    fn translate_then_untranslate_round_trips() {
        let start = Point::<Enu>::new(1.0, 2.0, 3.0);
        let step = FreeVector::new(4.0, -5.0, 6.0);
        assert_eq!((start + step) - step, start);
    }

    #[test]
    fn displacement_reconstructs_the_endpoint() {
        // (b − a) is the vector that carries a to b.
        let a = Point::<Enu>::new(1.0, 2.0, 3.0);
        let b = Point::<Enu>::new(4.0, 6.0, 8.0);
        assert_eq!(a + (b - a), b);
    }

    #[test]
    fn origin_is_all_zero() {
        assert_eq!(Point::<Enu>::origin(), Point::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn from_raw_then_into_inner_round_trips() {
        let raw = Vector3::new(1.5, -2.5, 3.5);
        assert_eq!(Point::<Enu>::from_raw(raw).into_inner(), raw);
    }

    #[test]
    fn accessors_report_each_coordinate() {
        let p = Point::<Enu>::new(1.0, 2.0, 3.0);
        assert_eq!((p.x(), p.y(), p.z()), (1.0, 2.0, 3.0));
    }

    #[test]
    fn reexpress_applies_rotation_and_translation() {
        // Forwards to `Transform::act`: Flu (1,0,0) rotates a quarter turn to
        // (0,1,0), then shifts +10 in X, landing at Enu (10,1,0) — the full
        // affine law `R·p + t`.
        let mapped = Point::<Flu>::new(1.0, 0.0, 0.0).reexpress(&turn_and_shift());
        assert!(close(mapped.into_inner(), Vector3::new(10.0, 1.0, 0.0)));
    }

    #[test]
    fn reexpress_translates_a_point_but_not_a_free_vector() {
        // The done-bar for the affine/linear split, witnessed at the `reexpress`
        // layer. Under one translation-only transform the point moves by exactly
        // `t = (10,0,0)` while an identically-valued free vector is unchanged, so
        // the two re-expressions differ by precisely `t`.
        let t = shift_x();
        let point = Point::<Flu>::new(1.0, 2.0, 3.0).reexpress(&t);
        let vector = FreeVector::<Flu>::new(1.0, 2.0, 3.0).reexpress(&t);
        assert!(close(point.into_inner(), Vector3::new(11.0, 2.0, 3.0)));
        assert!(close(vector.into_inner(), Vector3::new(1.0, 2.0, 3.0)));
        assert!(close(
            point.into_inner() - vector.into_inner(),
            Vector3::new(10.0, 0.0, 0.0)
        ));
    }
}
