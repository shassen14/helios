//! Frame-tagged affine point.

use crate::frames::conventions::Frame;
use crate::frames::quantities::FreeVector;

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
    use crate::frames::conventions::Enu;
    use crate::frames::quantities::{FreeVector, Point};

    use nalgebra::Vector3;

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
}
