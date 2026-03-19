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

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use std::f32::consts::PI as PI_F32;
    use std::f64::consts::PI as PI_F64;

    const F64_EPSILON: f64 = 1e-7;
    const F32_EPSILON: f32 = 1e-5;

    fn assert_bevy_quat_approx_eq(q1: &BevyQuat, q2: &BevyQuat, epsilon: f32) {
        let dot = q1.dot(*q2);
        assert!(
            dot.abs() > 1.0 - epsilon,
            "BevyQuats not approx equal: {:?} vs {:?}, dot: {}",
            q1,
            q2,
            dot
        );
    }

    fn assert_nalgebra_quat_approx_eq(
        q1: &UnitQuaternion<f64>,
        q2: &UnitQuaternion<f64>,
        epsilon: f64,
    ) {
        let angle_diff = q1.angle_to(q2);
        assert!(
            angle_diff.abs() < epsilon || (angle_diff.abs() - PI_F64).abs() < epsilon,
            "UnitQuaternions not approx equal. q1: {:?}, q2: {:?}, angle_diff: {}",
            q1.coords,
            q2.coords,
            angle_diff
        );
    }

    fn assert_bevy_vec3_approx_eq(v1: &BevyVec3, v2: &BevyVec3, epsilon: f32) {
        assert_abs_diff_eq!(v1.x, v2.x, epsilon = epsilon);
        assert_abs_diff_eq!(v1.y, v2.y, epsilon = epsilon);
        assert_abs_diff_eq!(v1.z, v2.z, epsilon = epsilon);
    }

    fn assert_nalgebra_vector3_approx_eq(v1: &Vector3<f64>, v2: &Vector3<f64>, epsilon: f64) {
        assert_abs_diff_eq!(v1.x, v2.x, epsilon = epsilon);
        assert_abs_diff_eq!(v1.y, v2.y, epsilon = epsilon);
        assert_abs_diff_eq!(v1.z, v2.z, epsilon = epsilon);
    }

    // --- ENU <-> Bevy Vector ---

    #[test]
    fn test_enu_vector_to_bevy_vector_and_back() {
        let enu_vec = Vector3::new(1.0, 2.0, 3.0); // E=1, N=2, U=3
        let bevy_vec = BevyVec3::from(EnuVector(enu_vec));
        let expected = BevyVec3::new(1.0, 3.0, -2.0); // X=E, Y=U, Z=-N
        assert_bevy_vec3_approx_eq(&bevy_vec, &expected, F32_EPSILON);

        let enu_back = EnuVector::from(bevy_vec).0;
        assert_nalgebra_vector3_approx_eq(&enu_back, &enu_vec, F64_EPSILON);
    }

    // --- FLU <-> Bevy Local Vector ---

    #[test]
    fn test_flu_vector_to_bevy_local_vector_axes() {
        // FLU +X (Forward) → Bevy -Z
        assert_bevy_vec3_approx_eq(
            &BevyVec3::from(FluVector(Vector3::new(1.0, 0.0, 0.0))),
            &BevyVec3::new(0.0, 0.0, -1.0),
            F32_EPSILON,
        );
        // FLU +Y (Left) → Bevy -X
        assert_bevy_vec3_approx_eq(
            &BevyVec3::from(FluVector(Vector3::new(0.0, 1.0, 0.0))),
            &BevyVec3::new(-1.0, 0.0, 0.0),
            F32_EPSILON,
        );
        // FLU +Z (Up) → Bevy +Y
        assert_bevy_vec3_approx_eq(
            &BevyVec3::from(FluVector(Vector3::new(0.0, 0.0, 1.0))),
            &BevyVec3::new(0.0, 1.0, 0.0),
            F32_EPSILON,
        );
    }

    #[test]
    fn test_flu_vector_round_trip() {
        let flu_vec = Vector3::new(1.5, -0.5, 2.0);
        let bevy_vec = BevyVec3::from(FluVector(flu_vec));
        let flu_back = FluVector::from(bevy_vec).0;
        assert_nalgebra_vector3_approx_eq(&flu_back, &flu_vec, F64_EPSILON);
    }

    // --- ENU World Pose (pure similarity) ---

    #[test]
    fn test_enu_world_pose_identity() {
        // World objects: ENU identity → Bevy identity (mesh placed as authored). ✓
        let bevy = BevyTransform::from(EnuWorldPose(Isometry3::identity()));
        assert_bevy_vec3_approx_eq(&bevy.translation, &BevyVec3::ZERO, F32_EPSILON);
        assert_bevy_quat_approx_eq(&bevy.rotation, &BevyQuat::IDENTITY, F32_EPSILON);
    }

    #[test]
    fn test_enu_world_pose_yaw_90() {
        // ENU yaw 90° (North) → Bevy R_y(+π/2). ✓
        let enu_north = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI_F64 / 2.0);
        let iso = Isometry3::from_parts(nalgebra::Translation3::identity(), enu_north);
        let bevy = BevyTransform::from(EnuWorldPose(iso));
        let expected = BevyQuat::from_rotation_y(PI_F32 / 2.0);
        assert_bevy_quat_approx_eq(&bevy.rotation, &expected, F32_EPSILON);
    }

    // --- ENU Body Pose (agent, FLU convention) ---

    #[test]
    fn test_enu_body_pose_identity() {
        // Agents: ENU identity (heading East) → Bevy R_y(-π/2). ✓
        let bevy = BevyTransform::from(EnuBodyPose(Isometry3::identity()));
        assert_bevy_vec3_approx_eq(&bevy.translation, &BevyVec3::ZERO, F32_EPSILON);
        assert_bevy_quat_approx_eq(
            &bevy.rotation,
            &BevyQuat::from_rotation_y(-PI_F32 / 2.0),
            F32_EPSILON,
        );
    }

    #[test]
    fn test_enu_body_pose_north() {
        // ENU yaw 90° (North) → Bevy identity (-Z forward = ENU North). ✓
        let enu_north = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI_F64 / 2.0);
        let iso = Isometry3::from_parts(nalgebra::Translation3::identity(), enu_north);
        let bevy = BevyTransform::from(EnuBodyPose(iso));
        assert_bevy_quat_approx_eq(&bevy.rotation, &BevyQuat::IDENTITY, F32_EPSILON);
    }

    #[test]
    fn test_bevy_transform_to_enu_body_pose_identity() {
        // Bevy identity (-Z → North) → ENU R_z(π/2) = yaw 90°. ✓
        let enu = EnuBodyPose::from(&BevyTransform::IDENTITY);
        assert_nalgebra_vector3_approx_eq(
            &enu.0.translation.vector,
            &Vector3::zeros(),
            F64_EPSILON,
        );
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI_F64 / 2.0);
        assert_nalgebra_quat_approx_eq(&enu.0.rotation, &expected, F64_EPSILON);
    }

    #[test]
    fn test_enu_body_pose_round_trip() {
        // Round trip: ENU body yaw → Bevy → ENU must recover the original.
        for deg in [0.0f64, 45.0, 90.0, 135.0, 180.0, -45.0, -90.0] {
            let enu_q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), deg.to_radians());
            let iso = Isometry3::from_parts(nalgebra::Translation3::identity(), enu_q);
            let bevy = BevyTransform::from(EnuBodyPose(iso));
            let enu_back = EnuBodyPose::from(&bevy);
            assert_nalgebra_quat_approx_eq(&enu_back.0.rotation, &enu_q, F64_EPSILON);
        }
    }

    #[test]
    fn test_enu_body_pose_general_and_back() {
        // ENU yaw π/4 (45°, heading NE), translation (1,2,0.5).
        // → Bevy R_y(π/4 − π/2) = R_y(−π/4), translation (1, 0.5, −2). ✓
        let enu_t = Vector3::new(1.0, 2.0, 0.5);
        let enu_r = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI_F64 / 4.0);
        let iso = Isometry3::from_parts(nalgebra::Translation3::from(enu_t), enu_r);

        let bevy = BevyTransform::from(EnuBodyPose(iso));

        assert_bevy_vec3_approx_eq(
            &bevy.translation,
            &BevyVec3::new(1.0, 0.5, -2.0),
            F32_EPSILON,
        );
        assert_bevy_quat_approx_eq(
            &bevy.rotation,
            &BevyQuat::from_rotation_y(-PI_F32 / 4.0),
            F32_EPSILON,
        );

        let enu_back = EnuBodyPose::from(&bevy);
        assert_nalgebra_vector3_approx_eq(&enu_back.0.translation.vector, &enu_t, F64_EPSILON);
        assert_nalgebra_quat_approx_eq(&enu_back.0.rotation, &enu_r, F64_EPSILON);
    }

    // --- FLU Local Pose ---

    #[test]
    fn test_flu_local_pose_identity() {
        let bevy = BevyTransform::from(FluLocalPose(Isometry3::identity()));
        assert_bevy_vec3_approx_eq(&bevy.translation, &BevyVec3::ZERO, F32_EPSILON);
        assert_bevy_quat_approx_eq(&bevy.rotation, &BevyQuat::IDENTITY, F32_EPSILON);
    }

    #[test]
    fn test_flu_local_pose_yaw_90() {
        // FLU yaw +90° (around +Z, turning left) → Bevy rotation_y(+90°). ✓
        let flu_r = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI_F64 / 2.0);
        let iso = Isometry3::from_parts(nalgebra::Translation3::identity(), flu_r);
        let bevy = BevyTransform::from(FluLocalPose(iso));
        assert_bevy_quat_approx_eq(
            &bevy.rotation,
            &BevyQuat::from_rotation_y(PI_F32 / 2.0),
            F32_EPSILON,
        );
    }
}
