// Typed conversions across the sim↔Bevy coordinate boundary, and the single
// source of truth for every axis swap between the two. A manual reorder
// anywhere outside this file is a latent bug.
//
// Bevy renders in a Y-up, −Z-forward frame; robotics data uses ENU world and
// FLU body frames. The vector/point crossing runs through core's typed frame
// algebra: `Bevy` is a frame marker, so each swap is a real `Rotation`. A value
// crosses via the `ToBevy` / `FromBevy` traits, which dispatch on the core frame
// type — any frame that states its basis (`CanonicalBasis::to_enu`) converts,
// with no per-frame code here. The crossed value lands fully typed as
// `Point<Bevy>` / `FreeVector<Bevy>`; the only untyped step left is the component
// copy to `bevy::Vec3`, which by construction carries no reorder.
//
// Every crossing routes through ENU: `enu_to_bevy` is the one anchored fact (the
// render relabel), and each frame's `to_enu` supplies the rest, so the whole
// conversion is `F::to_enu().then(enu_to_bevy())`.
//
// The pose `From` impls further down convert ENU/FLU poses to `bevy::Transform`
// with hand-written quaternion swaps.

use super::constants::{Q_ENU_FRAME_TO_BEVY_FRAME, Q_FLU_BODY_TO_BEVY_LOCAL};
use super::frame_types::{EnuBodyPose, EnuWorldPose, FluLocalPose};

use helios_core::frames::conventions::{Enu, Frame};
use helios_core::frames::quantities::{FreeVector, Point};
use helios_core::frames::transforms::{CanonicalBasis, Rotation, Transform};

use bevy::prelude::{
    GlobalTransform, Quat as BevyQuat, Transform as BevyTransform, Vec3 as BevyVec3,
};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
use std::f64::consts::FRAC_PI_2;

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
/// Blanket-implemented for every [`CanonicalBasis`] frame, so a new core frame
/// renders with no code here — it dispatches on the source frame and routes
/// through ENU (`F::to_enu().then(enu_to_bevy())`). Living in sim (not core) is
/// what makes the blanket impl over the foreign `Point<F>` / `FreeVector<F>`
/// orphan-legal: the trait itself is local. `Bevy` is a sim concept core cannot
/// name, which is exactly why the split lands here.
pub trait ToBevy {
    type Output;
    fn to_bevy(self) -> Self::Output;
}

impl<F: CanonicalBasis> ToBevy for Point<F> {
    type Output = Point<Bevy>;
    /// A location translates, so it crosses through a `Transform` (the rotation
    /// with an identity translation), not a bare rotation.
    fn to_bevy(self) -> Self::Output {
        Transform::from_rotation(F::to_enu().then(enu_to_bevy())).act(self)
    }
}

impl<F: CanonicalBasis> ToBevy for FreeVector<F> {
    type Output = FreeVector<Bevy>;
    /// A direction only rotates, so it crosses on the bare rotation.
    fn to_bevy(self) -> Self::Output {
        F::to_enu().then(enu_to_bevy()).act(self)
    }
}

/// Crosses a Bevy-framed quantity back into a core frame — the inverse of
/// [`ToBevy`].
///
/// The source is always `Bevy`, so the target frame cannot be inferred and is
/// named by turbofish at the call site (`v.from_bevy::<Enu>()`) — the same
/// explicit wiring the boundary favors elsewhere. The [`Output`](Self::Output)
/// GAT tracks that chosen frame. The rotation is the inverse of the forward
/// crossing.
// `from_bevy` takes `self`: it consumes a Bevy-framed value to produce a
// core-framed one, the mirror of `to_bevy`. That is not the `from_x`-constructor
// shape the convention lint expects, so silence it for this pair.
#[allow(clippy::wrong_self_convention)]
pub trait FromBevy {
    type Output<F: CanonicalBasis>;
    fn from_bevy<F: CanonicalBasis>(self) -> Self::Output<F>;
}

impl FromBevy for Point<Bevy> {
    type Output<F: CanonicalBasis> = Point<F>;
    fn from_bevy<F: CanonicalBasis>(self) -> Self::Output<F> {
        Transform::from_rotation(F::to_enu().then(enu_to_bevy()).inverse()).act(self)
    }
}

impl FromBevy for FreeVector<Bevy> {
    type Output<F: CanonicalBasis> = FreeVector<F>;
    fn from_bevy<F: CanonicalBasis>(self) -> Self::Output<F> {
        F::to_enu().then(enu_to_bevy()).inverse().act(self)
    }
}

/// Rotation from the ENU world frame into Bevy's world frame: a −90° turn about
/// the shared X axis (ENU East→+X, North→−Z, Up→+Y).
pub fn enu_to_bevy() -> Rotation<Enu, Bevy> {
    let quat = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -FRAC_PI_2);
    Rotation::from_unit_quaternion(quat)
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

// Pose conversions still cross on hand-written quaternion and translation
// swaps. They await their own typed migration; until then the axis math stays
// confined to the private helpers below.

// ---------------------------------------------------------------------------
// Private helpers — all axis-swap math lives here
// ---------------------------------------------------------------------------

fn enu_vector_to_bevy_vec3(v: &Vector3<f64>) -> BevyVec3 {
    BevyVec3::new(v.x as f32, v.z as f32, -v.y as f32)
}

fn bevy_vec3_to_enu_vector(v: &BevyVec3) -> Vector3<f64> {
    Vector3::new(v.x as f64, -v.z as f64, v.y as f64)
}

fn flu_vec3_to_bevy_local_vec3(v: &Vector3<f64>) -> BevyVec3 {
    BevyVec3::new(-v.y as f32, v.z as f32, -v.x as f32)
}

fn enu_quat_to_bevy_quat(q: &UnitQuaternion<f64>) -> BevyQuat {
    let r = Q_ENU_FRAME_TO_BEVY_FRAME.with(|qb| *qb * q * qb.inverse());
    BevyQuat::from_xyzw(
        r.coords.x as f32,
        r.coords.y as f32,
        r.coords.z as f32,
        r.coords.w as f32,
    )
}

fn enu_body_quat_to_bevy_quat(q: &UnitQuaternion<f64>) -> BevyQuat {
    let r = Q_ENU_FRAME_TO_BEVY_FRAME
        .with(|qb| Q_FLU_BODY_TO_BEVY_LOCAL.with(|qf| *qb * q * qf.inverse()));
    BevyQuat::from_xyzw(
        r.coords.x as f32,
        r.coords.y as f32,
        r.coords.z as f32,
        r.coords.w as f32,
    )
}

fn bevy_quat_to_enu_quat(q: &BevyQuat) -> UnitQuaternion<f64> {
    let q64 = UnitQuaternion::from_quaternion(Quaternion::new(
        q.w as f64, q.x as f64, q.y as f64, q.z as f64,
    ));
    Q_ENU_FRAME_TO_BEVY_FRAME
        .with(|qb| Q_FLU_BODY_TO_BEVY_LOCAL.with(|qf| qb.inverse() * q64 * *qf))
}

fn flu_quat_to_bevy_local_quat(q: &UnitQuaternion<f64>) -> BevyQuat {
    let r = Q_FLU_BODY_TO_BEVY_LOCAL.with(|qf| *qf * q * qf.inverse());
    BevyQuat::from_xyzw(
        r.coords.x as f32,
        r.coords.y as f32,
        r.coords.z as f32,
        r.coords.w as f32,
    )
}

// ---------------------------------------------------------------------------
// From impls: Pose newtypes → BevyTransform
// ---------------------------------------------------------------------------

/// Static world objects: ENU → Bevy (pure similarity, identity ENU → identity Bevy).
impl From<EnuWorldPose> for BevyTransform {
    fn from(p: EnuWorldPose) -> Self {
        BevyTransform {
            translation: enu_vector_to_bevy_vec3(&p.0.translation.vector),
            rotation: enu_quat_to_bevy_quat(&p.0.rotation),
            scale: BevyVec3::ONE,
        }
    }
}

/// Agent/vehicle body pose: ENU (FLU body convention) → Bevy world.
/// ENU identity (heading East) → Bevy R_y(−π/2).
impl From<EnuBodyPose> for BevyTransform {
    fn from(p: EnuBodyPose) -> Self {
        BevyTransform {
            translation: enu_vector_to_bevy_vec3(&p.0.translation.vector),
            rotation: enu_body_quat_to_bevy_quat(&p.0.rotation),
            scale: BevyVec3::ONE,
        }
    }
}

/// Sensor child transform: FLU body-relative → Bevy local.
impl From<FluLocalPose> for BevyTransform {
    fn from(p: FluLocalPose) -> Self {
        BevyTransform {
            translation: flu_vec3_to_bevy_local_vec3(&p.0.translation.vector),
            rotation: flu_quat_to_bevy_local_quat(&p.0.rotation),
            scale: BevyVec3::ONE,
        }
    }
}

// ---------------------------------------------------------------------------
// From impls: BevyTransform / GlobalTransform → EnuBodyPose
// ---------------------------------------------------------------------------

impl From<&BevyTransform> for EnuBodyPose {
    fn from(t: &BevyTransform) -> Self {
        let enu_t = bevy_vec3_to_enu_vector(&t.translation);
        let enu_r = bevy_quat_to_enu_quat(&t.rotation);
        EnuBodyPose(Isometry3::from_parts(Translation3::from(enu_t), enu_r))
    }
}

impl From<&GlobalTransform> for EnuBodyPose {
    fn from(t: &GlobalTransform) -> Self {
        EnuBodyPose::from(&t.compute_transform())
    }
}

#[cfg(test)]
#[path = "bevy_bridge_tests.rs"]
mod tests;
