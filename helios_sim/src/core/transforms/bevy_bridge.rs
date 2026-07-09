// helios_sim/src/simulation/core/transforms/bevy_bridge.rs
//
// All `From`/`Into` impls between ENU/FLU typed newtypes and Bevy types.
// This is the single source of truth for axis-swap logic. Never perform
// manual axis swaps outside this file.
//
// Two distinct conversion contexts:
//
//   WORLD FRAME (ENU ↔ Bevy world):
//     From<EnuWorldPose> for BevyTransform   — static objects (pure similarity)
//     From<EnuBodyPose>  for BevyTransform   — agents (FLU body convention)
//     From<&BevyTransform>  for EnuBodyPose  — read physics transform
//     From<&GlobalTransform> for EnuBodyPose — read physics GlobalTransform
//     From<EnuVector> for Vec3               — ENU → Bevy world vector
//     From<Vec3> for EnuVector               — Bevy world → ENU vector
//
//   BODY/SENSOR FRAME (FLU ↔ Bevy local):
//     From<FluLocalPose> for BevyTransform   — sensor child transforms
//     From<FluVector> for Vec3               — FLU → Bevy local vector
//     From<Vec3> for FluVector               — Bevy local → FLU vector

use bevy::prelude::{
    GlobalTransform, Quat as BevyQuat, Transform as BevyTransform, Vec3 as BevyVec3,
};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};

use super::constants::{Q_ENU_FRAME_TO_BEVY_FRAME, Q_FLU_BODY_TO_BEVY_LOCAL};
use super::frame_types::{EnuBodyPose, EnuVector, EnuWorldPose, FluLocalPose, FluVector};

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

fn bevy_local_vec3_to_flu_vec3(v: &BevyVec3) -> Vector3<f64> {
    Vector3::new(-v.z as f64, -v.x as f64, v.y as f64)
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

// ---------------------------------------------------------------------------
// From impls: Vector newtypes ↔ BevyVec3
// ---------------------------------------------------------------------------

impl From<EnuVector> for BevyVec3 {
    fn from(v: EnuVector) -> Self {
        enu_vector_to_bevy_vec3(&v.0)
    }
}

impl From<BevyVec3> for EnuVector {
    fn from(v: BevyVec3) -> Self {
        EnuVector(bevy_vec3_to_enu_vector(&v))
    }
}

impl From<FluVector> for BevyVec3 {
    fn from(v: FluVector) -> Self {
        flu_vec3_to_bevy_local_vec3(&v.0)
    }
}

impl From<BevyVec3> for FluVector {
    fn from(v: BevyVec3) -> Self {
        FluVector(bevy_local_vec3_to_flu_vec3(&v))
    }
}

#[cfg(test)]
#[path = "bevy_bridge_tests.rs"]
mod tests;
