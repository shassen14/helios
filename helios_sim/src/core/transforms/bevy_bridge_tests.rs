use super::*;
use helios_core::frames::conventions::Flu;

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

// ---------------------------------------------------------------------------
// Typed Bevy-frame boundary: rotations carry the reorder, casts are pure copies.
// ---------------------------------------------------------------------------

#[test]
fn enu_to_bevy_rotation_reorders_axes() {
    // ENU East→Bevy +X, North→−Z, Up→+Y.
    let east = enu_to_bevy().act(FreeVector::<Enu>::new(1.0, 0.0, 0.0));
    let north = enu_to_bevy().act(FreeVector::<Enu>::new(0.0, 1.0, 0.0));
    let up = enu_to_bevy().act(FreeVector::<Enu>::new(0.0, 0.0, 1.0));
    assert_nalgebra_vector3_approx_eq(&east.into_inner(), &Vector3::new(1.0, 0.0, 0.0), F64_EPSILON);
    assert_nalgebra_vector3_approx_eq(
        &north.into_inner(),
        &Vector3::new(0.0, 0.0, -1.0),
        F64_EPSILON,
    );
    assert_nalgebra_vector3_approx_eq(&up.into_inner(), &Vector3::new(0.0, 1.0, 0.0), F64_EPSILON);
}

#[test]
fn enu_to_bevy_inverse_round_trips() {
    let v = FreeVector::<Enu>::new(1.5, -0.5, 2.0);
    let round = enu_to_bevy().inverse().act(enu_to_bevy().act(v));
    assert_nalgebra_vector3_approx_eq(&round.into_inner(), &v.into_inner(), F64_EPSILON);
}

#[test]
fn point_vec3_casts_are_pure_copies() {
    // A cast never reorders: components pass straight through, only precision changes.
    let p = Point::<Bevy>::from_raw(Vector3::new(1.0, 2.0, 3.0));
    assert_bevy_vec3_approx_eq(
        &point_bevy_to_vec3(p),
        &BevyVec3::new(1.0, 2.0, 3.0),
        F32_EPSILON,
    );
    let back = vec3_to_point_bevy(BevyVec3::new(1.0, 2.0, 3.0));
    assert_nalgebra_vector3_approx_eq(&back.into_inner(), &Vector3::new(1.0, 2.0, 3.0), F64_EPSILON);
}

#[test]
fn freevector_vec3_casts_are_pure_copies() {
    let v = FreeVector::<Bevy>::from_raw(Vector3::new(1.0, 2.0, 3.0));
    assert_bevy_vec3_approx_eq(
        &freevector_bevy_to_vec3(v),
        &BevyVec3::new(1.0, 2.0, 3.0),
        F32_EPSILON,
    );
    let back = vec3_to_freevector_bevy(BevyVec3::new(1.0, 2.0, 3.0));
    assert_nalgebra_vector3_approx_eq(&back.into_inner(), &Vector3::new(1.0, 2.0, 3.0), F64_EPSILON);
}

// ---------------------------------------------------------------------------
// ToBevy / FromBevy: the trait dispatches on the source frame, and the reorder
// matches the frame's basis composed with the ENU→Bevy anchor.
// ---------------------------------------------------------------------------

#[test]
fn enu_point_to_bevy_matches_the_rotation() {
    // The crossing is a pure rotation (identity translation), so a location at
    // ENU North reorders to Bevy −Z just as a free vector would.
    let bevy_point = Point::<Enu>::new(0.0, 1.0, 0.0).to_bevy();
    assert_bevy_vec3_approx_eq(
        &point_bevy_to_vec3(bevy_point),
        &BevyVec3::new(0.0, 0.0, -1.0),
        F32_EPSILON,
    );
}

#[test]
fn flu_point_to_bevy_reorders_axes() {
    // FLU Forward→Bevy −Z, Left→−X, Up→+Y (same reorder as the free vector).
    let fwd = Point::<Flu>::new(1.0, 0.0, 0.0).to_bevy();
    let left = Point::<Flu>::new(0.0, 1.0, 0.0).to_bevy();
    let up = Point::<Flu>::new(0.0, 0.0, 1.0).to_bevy();
    assert_bevy_vec3_approx_eq(&point_bevy_to_vec3(fwd), &BevyVec3::new(0.0, 0.0, -1.0), F32_EPSILON);
    assert_bevy_vec3_approx_eq(&point_bevy_to_vec3(left), &BevyVec3::new(-1.0, 0.0, 0.0), F32_EPSILON);
    assert_bevy_vec3_approx_eq(&point_bevy_to_vec3(up), &BevyVec3::new(0.0, 1.0, 0.0), F32_EPSILON);
}

#[test]
fn enu_freevector_to_bevy_reorders_axes() {
    // ENU North direction → Bevy −Z.
    let north = FreeVector::<Enu>::new(0.0, 1.0, 0.0).to_bevy();
    assert_nalgebra_vector3_approx_eq(&north.into_inner(), &Vector3::new(0.0, 0.0, -1.0), F64_EPSILON);
}

#[test]
fn flu_freevector_to_bevy_reorders_axes() {
    // FLU Forward→Bevy −Z, Left→−X, Up→+Y.
    let fwd = FreeVector::<Flu>::new(1.0, 0.0, 0.0).to_bevy();
    let left = FreeVector::<Flu>::new(0.0, 1.0, 0.0).to_bevy();
    let up = FreeVector::<Flu>::new(0.0, 0.0, 1.0).to_bevy();
    assert_nalgebra_vector3_approx_eq(&fwd.into_inner(), &Vector3::new(0.0, 0.0, -1.0), F64_EPSILON);
    assert_nalgebra_vector3_approx_eq(&left.into_inner(), &Vector3::new(-1.0, 0.0, 0.0), F64_EPSILON);
    assert_nalgebra_vector3_approx_eq(&up.into_inner(), &Vector3::new(0.0, 1.0, 0.0), F64_EPSILON);
}

#[test]
fn from_bevy_inverts_to_bevy_for_a_point() {
    let enu = Point::<Enu>::new(1.0, -2.0, 3.0);
    let round = enu.to_bevy().from_bevy::<Enu>();
    assert_nalgebra_vector3_approx_eq(&round.into_inner(), &enu.into_inner(), F64_EPSILON);
}

#[test]
fn from_bevy_inverts_to_bevy_for_a_free_vector() {
    let enu = FreeVector::<Enu>::new(1.0, -2.0, 3.0);
    let round = enu.to_bevy().from_bevy::<Enu>();
    assert_nalgebra_vector3_approx_eq(&round.into_inner(), &enu.into_inner(), F64_EPSILON);
}
