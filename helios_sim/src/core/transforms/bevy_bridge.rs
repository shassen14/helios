// Typed conversions across the sim↔Bevy coordinate boundary, and the single
// source of truth for every axis swap between the two. A manual reorder
// anywhere outside this file is a latent bug.
//
// Bevy renders in a Y-up, −Z-forward frame; robotics data uses ENU world and
// FLU body frames. The vector/point crossing runs through core's typed frame
// algebra: `Bevy` is a frame marker, so each swap is a real `Rotation`. A value
// crosses via the `ToBevy` / `FromBevy` traits, which dispatch on the core frame
// type — any frame that states its basis (`EnuBasis::to_enu`) converts,
// with no per-frame code here. The crossed value lands fully typed as
// `Point<Bevy>` / `FreeVector<Bevy>`; the only untyped step left is the component
// copy to `bevy::Vec3`, which by construction carries no reorder.
//
// Every crossing routes through ENU: `enu_to_bevy` is the one anchored fact (the
// render relabel), and each frame's `to_enu` supplies the rest, so the whole
// conversion is `F::to_enu().then(enu_to_bevy())`.
//
// Poses cross the same way in both directions: `ToBevy for Transform<From, To>`
// conjugates a pose into `Transform<Bevy, Bevy>` and `FromBevy` inverts it, with
// `transform_bevy_to_bevy_transform` / `bevy_transform_to_transform_bevy` copying
// between that and `bevy::Transform`. No axis swap is hand-written anymore — every
// crossing, points and poses alike, is the typed frame algebra above.

use helios_core::frames::conventions::{Enu, Frame};
use helios_core::frames::quantities::{FreeVector, Point};
use helios_core::frames::transforms::{EnuBasis, Rotation, Transform};

use bevy::prelude::{Quat as BevyQuat, Transform as BevyTransform, Vec3 as BevyVec3};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
use std::f64::consts::FRAC_PI_2;
use std::sync::LazyLock;

/// The host's rendering frame: Bevy's Y-up, −Z-forward coordinate system.
///
/// Implements `Frame` so it can tag core quantities (`Point<Bevy>`,
/// `FreeVector<Bevy>`) and drive typed rotations, but deliberately does **not**
/// implement `ConventionOf`. It is a rendering relabel, not a robotics frame, so
/// it has no `Convention` and structurally cannot enter the `TfTree` /
/// `ErasedTransform` graph, both of which require one. A `Point<Bevy>` is a
/// legal terminal value; it can never be routed through the transform tree.
pub struct Bevy;

impl Frame for Bevy {}

/// Crosses a core-framed quantity into Bevy's render frame.
///
/// Blanket-implemented for every [`EnuBasis`] frame, so a new core frame
/// renders with no code here — it dispatches on the source frame and routes
/// through ENU (`F::to_enu().then(enu_to_bevy())`). Living in sim (not core) is
/// what makes the blanket impl over the foreign `Point<F>` / `FreeVector<F>`
/// orphan-legal: the trait itself is local. `Bevy` is a sim concept core cannot
/// name, which is exactly why the split lands here.
pub trait ToBevy {
    type Output;
    fn to_bevy(self) -> Self::Output;
}

impl<F: EnuBasis> ToBevy for Point<F> {
    type Output = Point<Bevy>;
    /// A location translates, so it crosses through a `Transform` (the rotation
    /// with an identity translation), not a bare rotation.
    fn to_bevy(self) -> Self::Output {
        Transform::from_rotation(to_bevy_rotation::<F>()).act(self)
    }
}

impl<F: EnuBasis> ToBevy for FreeVector<F> {
    type Output = FreeVector<Bevy>;
    /// A direction only rotates, so it crosses on the bare rotation.
    fn to_bevy(self) -> Self::Output {
        to_bevy_rotation::<F>().act(self)
    }
}

impl<From: EnuBasis, To: EnuBasis> ToBevy for Transform<From, To> {
    type Output = Transform<Bevy, Bevy>;
    /// A pose re-expresses in Bevy on both sides via a two-sided conjugation:
    /// pull the source out of Bevy into `From`, apply the pose into `To`, then
    /// push the result back into Bevy. Both robotics conventions collapse onto
    /// the single Bevy frame, so the output is `Transform<Bevy, Bevy>`. Composing
    /// typed `Transform`s (not bare rotations) carries the translation for free.
    fn to_bevy(self) -> Self::Output {
        Transform::from_rotation(to_bevy_rotation::<From>())
            .inverse()
            .then(self)
            .then(Transform::from_rotation(to_bevy_rotation::<To>()))
    }
}

/// Crosses a Bevy-framed quantity back into a core frame — the inverse of
/// [`ToBevy`].
///
/// The source is always `Bevy`, so the target cannot be inferred and is carried
/// by the `Out` type parameter — the whole output type, not merely a frame, so a
/// target with more than one frame (a pose) stays nameable. The call site pins
/// it by the binding's type or by UFCS (`FromBevy::<Point<Enu>>::from_bevy(v)`),
/// the same explicit wiring the boundary favors elsewhere. The rotation is the
/// inverse of the forward crossing.
// `from_bevy` takes `self`: it consumes a Bevy-framed value to produce a
// core-framed one, the mirror of `to_bevy`. That is not the `from_x`-constructor
// shape the convention lint expects, so silence it for this pair.
#[allow(clippy::wrong_self_convention)]
pub trait FromBevy<Out> {
    fn from_bevy(self) -> Out;
}

impl<F: EnuBasis> FromBevy<Point<F>> for Point<Bevy> {
    fn from_bevy(self) -> Point<F> {
        Transform::from_rotation(to_bevy_rotation::<F>().inverse()).act(self)
    }
}

impl<F: EnuBasis> FromBevy<FreeVector<F>> for FreeVector<Bevy> {
    fn from_bevy(self) -> FreeVector<F> {
        to_bevy_rotation::<F>().inverse().act(self)
    }
}

impl<From: EnuBasis, To: EnuBasis> FromBevy<Transform<From, To>> for Transform<Bevy, Bevy> {
    fn from_bevy(self) -> Transform<From, To> {
        Transform::from_rotation(to_bevy_rotation::<From>())
            .then(self)
            .then(Transform::from_rotation(to_bevy_rotation::<To>()).inverse())
    }
}

static ENU_TO_BEVY_QUAT: LazyLock<UnitQuaternion<f64>> =
    LazyLock::new(|| UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -FRAC_PI_2));

/// Rotation from the ENU world frame into Bevy's world frame: a −90° turn about
/// the shared X axis (ENU East→+X, North→−Z, Up→+Y).
pub fn enu_to_bevy() -> Rotation<Enu, Bevy> {
    Rotation::from_unit_quaternion(*ENU_TO_BEVY_QUAT)
}

pub fn to_bevy_rotation<F: EnuBasis>() -> Rotation<F, Bevy> {
    F::to_enu().then(enu_to_bevy())
}

/// Copies a `Point<Bevy>` into a `bevy::Vec3` (f64→f32). No axis reorder — every
/// swap already happened in the rotation that produced the `Bevy`-framed value.
pub fn point_bevy_to_vec3(point: Point<Bevy>) -> BevyVec3 {
    let p = point.into_inner();
    BevyVec3::new(p.x as f32, p.y as f32, p.z as f32)
}

/// Tags a `bevy::Vec3` as a `Point<Bevy>` (f32→f64). No axis reorder.
pub fn vec3_to_point_bevy(vec: BevyVec3) -> Point<Bevy> {
    let v = Vector3::new(vec.x as f64, vec.y as f64, vec.z as f64);
    Point::<Bevy>::from_raw(v)
}

/// Copies a `FreeVector<Bevy>` into a `bevy::Vec3` (f64→f32). No axis reorder.
pub fn freevector_bevy_to_vec3(vec: FreeVector<Bevy>) -> BevyVec3 {
    let v = vec.into_inner();
    BevyVec3::new(v.x as f32, v.y as f32, v.z as f32)
}

/// Tags a `bevy::Vec3` as a `FreeVector<Bevy>` (f32→f64). No axis reorder.
pub fn vec3_to_freevector_bevy(vec: BevyVec3) -> FreeVector<Bevy> {
    let v = Vector3::new(vec.x as f64, vec.y as f64, vec.z as f64);
    FreeVector::<Bevy>::from_raw(v)
}

/// Copies a `Transform<Bevy, Bevy>` into a `bevy::Transform` (f64→f32). No axis
/// reorder — every swap already happened in the conjugation that produced the
/// `Bevy`-framed transform. Scale is identity: a coordinate crossing carries no
/// scale of its own.
pub fn transform_bevy_to_bevy_transform(transform: Transform<Bevy, Bevy>) -> BevyTransform {
    let iso = transform.into_inner();
    BevyTransform {
        translation: BevyVec3::new(
            iso.translation.x as f32,
            iso.translation.y as f32,
            iso.translation.z as f32,
        ),
        rotation: BevyQuat::from_xyzw(
            iso.rotation.coords.x as f32,
            iso.rotation.coords.y as f32,
            iso.rotation.coords.z as f32,
            iso.rotation.coords.w as f32,
        ),
        scale: BevyVec3::ONE,
    }
}

/// Tags a `bevy::Transform` as a `Transform<Bevy, Bevy>` (f32→f64). No axis
/// reorder — the swap happens afterward in `from_bevy`, on the `Bevy`-framed
/// value this produces. Scale is dropped: a coordinate crossing carries none.
pub fn bevy_transform_to_transform_bevy(transform: BevyTransform) -> Transform<Bevy, Bevy> {
    let translation = Translation3::new(
        transform.translation.x as f64,
        transform.translation.y as f64,
        transform.translation.z as f64,
    );
    let rotation = UnitQuaternion::from_quaternion(Quaternion::new(
        transform.rotation.w as f64,
        transform.rotation.x as f64,
        transform.rotation.y as f64,
        transform.rotation.z as f64,
    ));
    Transform::<Bevy, Bevy>::from_isometry(Isometry3::from_parts(translation, rotation))
}

#[cfg(test)]
#[path = "bevy_bridge_tests.rs"]
mod tests;
