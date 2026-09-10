// Typed conversions across the sim↔Bevy coordinate boundary, and the single
// source of truth for every axis swap between the two. A manual reorder
// anywhere outside this file is a latent bug.
//
// Bevy renders in a Y-up, −Z-forward frame; robotics data uses ENU world and
// FLU body frames. The vector/point crossing runs through core's typed frame
// algebra: `Bevy` is a frame marker, so each swap is a real `Rotation`
// (`enu_to_bevy`, `flu_to_bevy_local`) and a crossed value lands fully typed as
// `Point<Bevy>` / `FreeVector<Bevy>`. The only untyped step left is the
// component copy to `bevy::Vec3`, which by construction carries no reorder.
//
// The pose `From` impls further down convert ENU/FLU poses to `bevy::Transform`
// with hand-written quaternion swaps.

use super::constants::{Q_ENU_FRAME_TO_BEVY_FRAME, Q_FLU_BODY_TO_BEVY_LOCAL};
use super::frame_types::{EnuBodyPose, EnuWorldPose, FluLocalPose};

use helios_core::frames::conventions::{Enu, Flu, Frame};
use helios_core::frames::quantities::{FreeVector, Point};
use helios_core::frames::transforms::{Rotation, Transform};

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

/// Rotation from the ENU world frame into Bevy's world frame: a −90° turn about
/// the shared X axis (ENU East→+X, North→−Z, Up→+Y).
pub fn enu_to_bevy() -> Rotation<Enu, Bevy> {
    let quat = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -FRAC_PI_2);
    Rotation::from_unit_quaternion(quat)
}

/// Rotation from the FLU body frame into ENU: a +90° turn about Z. Its only use
/// is composing `flu_to_bevy_local`; the crossing to Bevy always routes via ENU.
fn flu_to_enu() -> Rotation<Flu, Enu> {
    let quat = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
    Rotation::from_unit_quaternion(quat)
}

/// Rotation from the FLU body frame into Bevy's local frame, composed as FLU→ENU
/// then ENU→Bevy. `then` only typechecks because the middle frame is ENU, so the
/// composition is verified by the types rather than asserted.
pub fn flu_to_bevy_local() -> Rotation<Flu, Bevy> {
    flu_to_enu().then(enu_to_bevy())
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

/// Maps an ENU-world location into Bevy world space. A location translates, so
/// it crosses as a `Transform` (the rotation with an identity translation)
/// acting on the point, not as a bare rotation.
pub fn enu_point_to_bevy(point: Point<Enu>) -> Point<Bevy> {
    Transform::from_rotation(enu_to_bevy()).act(point)
}

/// Maps a Bevy-world location back into the ENU world frame — the inverse of
/// [`enu_point_to_bevy`], for reading a picked or physics-side position as ENU.
pub fn bevy_to_enu_point(point: Point<Bevy>) -> Point<Enu> {
    Transform::from_rotation(enu_to_bevy().inverse()).act(point)
}

/// Maps an FLU body-local location into Bevy's local frame, e.g. a lidar return
/// expressed in the sensor's own frame before it is placed under the body entity.
pub fn flu_point_to_bevy_local(point: Point<Flu>) -> Point<Bevy> {
    Transform::from_rotation(flu_to_bevy_local()).act(point)
}

/// Rotates an ENU-world direction into Bevy world space. A direction only
/// rotates, so it crosses on the bare rotation with no translation.
pub fn enu_freevector_to_bevy(vec: FreeVector<Enu>) -> FreeVector<Bevy> {
    enu_to_bevy().act(vec)
}

/// Rotates a Bevy-world direction back into ENU — the inverse of
/// [`enu_freevector_to_bevy`] (e.g. reading Avian's Y-up gravity as ENU).
pub fn bevy_to_enu_freevector(vec: FreeVector<Bevy>) -> FreeVector<Enu> {
    enu_to_bevy().inverse().act(vec)
}

/// Rotates an FLU body-local direction into Bevy's local frame, e.g. a ray
/// direction the raycasting sensor expresses in the sensor's own FLU frame.
pub fn flu_freevector_to_bevy_local(vec: FreeVector<Flu>) -> FreeVector<Bevy> {
    flu_to_bevy_local().act(vec)
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
