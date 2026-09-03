//! Frame-tagged free vector.

use crate::frames::conventions::Frame;
use crate::frames::transforms::Transform;

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use std::marker::PhantomData;
use std::ops::{Add, Div, Mul, Neg, Sub};

/// A free vector (velocity, force, displacement) known at compile time to be in
/// frame `F`.
///
/// "Free" means it has magnitude and direction but no anchor point, so under a
/// transform it rotates but does **not** translate (`R·v`, no `t`). That is what
/// separates it from [`Point`](super::point::Point), whose position *does*
/// translate — the type is the record of which transform law applies.
///
/// Wraps a plain [`nalgebra::Vector3<f64>`]; the [`PhantomData`] carries the
/// frame and costs nothing at runtime. Arithmetic is frame-preserving and only
/// defined between vectors of the *same* `F`, so a frame mismatch fails to
/// compile:
///
/// ```compile_fail
/// use helios_core::frames::quantities::{EnuVector, FluVector};
///
/// let world = EnuVector::new(1.0, 0.0, 0.0);
/// let body = FluVector::new(0.0, 1.0, 0.0);
/// let _ = world + body; // frames differ — does not compile
/// ```
///
/// Use [`raw`](Self::raw) / [`into_inner`](Self::into_inner) to drop the frame
/// tag when handing components to code that operates on untyped vectors (frame
/// conversions, TF math).
//
// `#[serde(skip)]` on the phantom keeps `F` out of the serialized fields. serde's
// derive adds a `Serialize`/`Deserialize` bound for every type parameter that
// appears in a serialized field; skipping the phantom means `F` appears in none,
// so no bound on `F` is inferred. The wire form is just the inner vector, and the
// phantom is rebuilt from `Default` on read.
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct FreeVector<F: Frame>(Vector3<f64>, #[serde(skip)] PhantomData<F>);

// `Clone`/`Copy` are hand-written rather than derived. `#[derive(Clone)]` on a
// type holding `PhantomData<F>` generates `impl<F: Clone> Clone` — it constrains
// *every* type parameter, including `F`, even though `F` lives only inside a
// `PhantomData` and needs no bound. That over-constrained impl is then invisible
// to generic code that knows only `F: Frame` (such as `Wrench<F>`), because
// `F: Frame` does not imply `F: Clone`. Writing the impls by hand states the true
// bound (`F: Frame`), which every marker satisfies. The body is `*self`: both
// real fields (`Vector3<f64>` and `PhantomData<F>`) are already `Copy`.
impl<F: Frame> Clone for FreeVector<F> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<F: Frame> Copy for FreeVector<F> {}

impl<F: Frame> FreeVector<F> {
    /// Constructs a vector from its components, tagged with frame `F`.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vector3::new(x, y, z), PhantomData)
    }

    /// Tags an existing raw vector as being in frame `F`.
    ///
    /// The caller asserts the components are already expressed in `F`; this is
    /// the entry point from untyped vector math.
    pub fn from_raw(vec: Vector3<f64>) -> Self {
        Self(vec, PhantomData)
    }

    /// The zero vector in frame `F`.
    pub fn zeros() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// The first component (E for [`Enu`](crate::frames::conventions::Enu), F for [`Flu`](crate::frames::conventions::Flu)).
    pub fn x(&self) -> f64 {
        self.0.x
    }

    /// The second component (N for [`Enu`](crate::frames::conventions::Enu), L for [`Flu`](crate::frames::conventions::Flu)).
    pub fn y(&self) -> f64 {
        self.0.y
    }

    /// The third component (U for both [`Enu`](crate::frames::conventions::Enu) and [`Flu`](crate::frames::conventions::Flu)).
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

    /// Re-expresses this vector in frame `To` under only the transform's
    /// rotational part, `R·v` — a free vector rotates but never translates.
    ///
    /// Forwards through the transform's [`rotation`](Transform::rotation), so the
    /// translation `t` is structurally out of reach: there is no path by which
    /// `reexpress` adds a translation to a free vector, which is the whole point
    /// of the [`Point`](super::Point)/`FreeVector` split. The transform's
    /// `F → To` pair must match this vector's frame or it does not compile.
    ///
    /// The affine, translation-applying operation is [`Transform::act`], and it
    /// accepts only a [`Point`](super::Point). Handing it a free vector — the one
    /// way you could "translate a velocity" — does not compile:
    ///
    /// ```compile_fail
    /// use helios_core::frames::conventions::{Enu, Flu};
    /// use helios_core::frames::quantities::FluVector;
    /// use helios_core::frames::transforms::Transform;
    /// use nalgebra::Isometry3;
    ///
    /// let t = Transform::<Flu, Enu>::from_isometry(Isometry3::identity());
    /// let v = FluVector::new(1.0, 0.0, 0.0);
    /// let _ = t.act(v); // `act` wants a `Point<Flu>`, not a `FreeVector` — no compile
    /// ```
    pub fn reexpress<To: Frame>(self, transform: &Transform<F, To>) -> FreeVector<To> {
        transform.rotation().act(self)
    }
}

impl<F: Frame> Add for FreeVector<F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0, PhantomData)
    }
}

impl<F: Frame> Sub for FreeVector<F> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self(self.0 - rhs.0, PhantomData)
    }
}

impl<F: Frame> Neg for FreeVector<F> {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0, PhantomData)
    }
}

impl<F: Frame> Mul<f64> for FreeVector<F> {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self(self.0 * scalar, PhantomData)
    }
}

impl<F: Frame> Div<f64> for FreeVector<F> {
    type Output = Self;
    fn div(self, scalar: f64) -> Self {
        Self(self.0 / scalar, PhantomData)
    }
}

#[cfg(test)]
mod tests {
    use crate::frames::conventions::{Enu, Flu};
    use crate::frames::quantities::{EnuVector, FluVector};
    use crate::frames::transforms::{Rotation, Transform};

    use nalgebra::{Translation3, UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;

    /// A quarter turn about +Z paired with a +10 shift along X, read `Flu → Enu`.
    /// The translation is present precisely so a correct free-vector re-expression
    /// must ignore it.
    fn turn_and_shift() -> Transform<Flu, Enu> {
        let rotation = Rotation::from_unit_quaternion(UnitQuaternion::from_axis_angle(
            &Vector3::z_axis(),
            FRAC_PI_2,
        ));
        Transform::from_parts(rotation, Translation3::new(10.0, 0.0, 0.0))
    }

    /// Floats through a quaternion are not bit-exact; compare within tolerance.
    fn close(a: Vector3<f64>, b: Vector3<f64>) -> bool {
        (a - b).norm() < 1e-9
    }

    #[test]
    fn add_is_componentwise() {
        let sum = EnuVector::new(1.0, 2.0, 3.0) + EnuVector::new(4.0, 5.0, 6.0);
        assert_eq!(sum, EnuVector::new(5.0, 7.0, 9.0));
    }

    #[test]
    fn sub_is_componentwise() {
        let diff = EnuVector::new(4.0, 5.0, 6.0) - EnuVector::new(1.0, 2.0, 3.0);
        assert_eq!(diff, EnuVector::new(3.0, 3.0, 3.0));
    }

    #[test]
    fn neg_flips_every_component() {
        assert_eq!(
            -EnuVector::new(1.0, -2.0, 3.0),
            EnuVector::new(-1.0, 2.0, -3.0)
        );
    }

    #[test]
    fn mul_scales_every_component() {
        assert_eq!(
            EnuVector::new(1.0, 2.0, 3.0) * 2.0,
            EnuVector::new(2.0, 4.0, 6.0)
        );
    }

    #[test]
    fn div_scales_every_component() {
        assert_eq!(
            EnuVector::new(2.0, 4.0, 6.0) / 2.0,
            EnuVector::new(1.0, 2.0, 3.0)
        );
    }

    #[test]
    fn from_raw_then_into_inner_round_trips() {
        let raw = Vector3::new(1.5, -2.5, 3.5);
        assert_eq!(EnuVector::from_raw(raw).into_inner(), raw);
    }

    #[test]
    fn accessors_report_each_component() {
        let v = EnuVector::new(1.0, 2.0, 3.0);
        assert_eq!((v.x(), v.y(), v.z()), (1.0, 2.0, 3.0));
    }

    #[test]
    fn zeros_is_all_zero() {
        assert_eq!(EnuVector::zeros(), EnuVector::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn reexpress_rotates_without_translating() {
        // Forwards through the transform's rotational part only: Flu +X under a
        // quarter turn about +Z lands on Enu +Y, and the transform's +10 X shift
        // is ignored because a free vector has no anchor to move.
        let rotated = FluVector::new(1.0, 0.0, 0.0).reexpress(&turn_and_shift());
        assert!(close(rotated.into_inner(), Vector3::new(0.0, 1.0, 0.0)));
    }
}
