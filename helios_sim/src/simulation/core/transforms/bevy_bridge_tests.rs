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

#[test]
fn test_enu_vector_to_bevy_vector_and_back() {
    let enu_vec = Vector3::new(1.0, 2.0, 3.0);
    let bevy_vec = BevyVec3::from(EnuVector(enu_vec));
    let expected = BevyVec3::new(1.0, 3.0, -2.0);
    assert_bevy_vec3_approx_eq(&bevy_vec, &expected, F32_EPSILON);

    let enu_back = EnuVector::from(bevy_vec).0;
    assert_nalgebra_vector3_approx_eq(&enu_back, &enu_vec, F64_EPSILON);
}

#[test]
fn test_flu_vector_to_bevy_local_vector_axes() {
    assert_bevy_vec3_approx_eq(
        &BevyVec3::from(FluVector(Vector3::new(1.0, 0.0, 0.0))),
        &BevyVec3::new(0.0, 0.0, -1.0),
        F32_EPSILON,
    );
    assert_bevy_vec3_approx_eq(
        &BevyVec3::from(FluVector(Vector3::new(0.0, 1.0, 0.0))),
        &BevyVec3::new(-1.0, 0.0, 0.0),
        F32_EPSILON,
    );
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

#[test]
fn test_enu_world_pose_identity() {
    let bevy = BevyTransform::from(EnuWorldPose(Isometry3::identity()));
    assert_bevy_vec3_approx_eq(&bevy.translation, &BevyVec3::ZERO, F32_EPSILON);
    assert_bevy_quat_approx_eq(&bevy.rotation, &BevyQuat::IDENTITY, F32_EPSILON);
}

#[test]
fn test_enu_world_pose_yaw_90() {
    let enu_north = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI_F64 / 2.0);
    let iso = Isometry3::from_parts(nalgebra::Translation3::identity(), enu_north);
    let bevy = BevyTransform::from(EnuWorldPose(iso));
    let expected = BevyQuat::from_rotation_y(PI_F32 / 2.0);
    assert_bevy_quat_approx_eq(&bevy.rotation, &expected, F32_EPSILON);
}

#[test]
fn test_enu_body_pose_identity() {
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
    let enu_north = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI_F64 / 2.0);
    let iso = Isometry3::from_parts(nalgebra::Translation3::identity(), enu_north);
    let bevy = BevyTransform::from(EnuBodyPose(iso));
    assert_bevy_quat_approx_eq(&bevy.rotation, &BevyQuat::IDENTITY, F32_EPSILON);
}

#[test]
fn test_bevy_transform_to_enu_body_pose_identity() {
    let enu = EnuBodyPose::from(&BevyTransform::IDENTITY);
    assert_nalgebra_vector3_approx_eq(&enu.0.translation.vector, &Vector3::zeros(), F64_EPSILON);
    let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI_F64 / 2.0);
    assert_nalgebra_quat_approx_eq(&enu.0.rotation, &expected, F64_EPSILON);
}

#[test]
fn test_enu_body_pose_round_trip() {
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

#[test]
fn test_flu_local_pose_identity() {
    let bevy = BevyTransform::from(FluLocalPose(Isometry3::identity()));
    assert_bevy_vec3_approx_eq(&bevy.translation, &BevyVec3::ZERO, F32_EPSILON);
    assert_bevy_quat_approx_eq(&bevy.rotation, &BevyQuat::IDENTITY, F32_EPSILON);
}

#[test]
fn test_flu_local_pose_yaw_90() {
    let flu_r = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI_F64 / 2.0);
    let iso = Isometry3::from_parts(nalgebra::Translation3::identity(), flu_r);
    let bevy = BevyTransform::from(FluLocalPose(iso));
    assert_bevy_quat_approx_eq(
        &bevy.rotation,
        &BevyQuat::from_rotation_y(PI_F32 / 2.0),
        F32_EPSILON,
    );
}
