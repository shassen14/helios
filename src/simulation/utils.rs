// src/simulation/utils.rs (or near the top of systems.rs) use bevy::math::{EulerRot, Isometry3d};
use bevy::math::Isometry3d;
use bevy::prelude::{
    GlobalTransform, Quat as BevyQuat, Transform as BevyTransform, Vec3 as BevyVec3,
};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
use std::f64::consts::FRAC_PI_2;

/// Converts a Bevy Transform or GlobalTransform into an nalgebra Isometry3<f64>.
pub fn bevy_transform_to_nalgebra_isometry(transform: &BevyTransform) -> Isometry3<f64> {
    // Extract Bevy components (likely f32)
    let translation = transform.translation;
    let rotation = transform.rotation;

    // Convert to nalgebra types and f64
    let iso_translation = Translation3::new(
        translation.x as f64,
        translation.y as f64,
        translation.z as f64,
    );
    let iso_rotation = UnitQuaternion::from_quaternion(Quaternion::new(
        rotation.w as f64, // Note: nalgebra Quaternion is w, x, y, z
        rotation.x as f64,
        rotation.y as f64,
        rotation.z as f64,
    ));

    Isometry3::from_parts(iso_translation, iso_rotation)
}

// Helper for GlobalTransform (it derefs to Transform)
pub fn bevy_global_transform_to_nalgebra_isometry(transform: &GlobalTransform) -> Isometry3<f64> {
    bevy_transform_to_nalgebra_isometry(&transform.compute_transform())
}

/// Converts a Bevy Transform into the bevy::math::Isometry3d type used by Gizmos.
pub fn transform_to_isometry3d(transform: &BevyTransform) -> Isometry3d {
    let translation: BevyVec3 = transform.translation; // Bevy Vec3 (f32)
    let rotation: BevyQuat = transform.rotation; // Bevy Quat (f32)

    Isometry3d::new(translation, rotation)
}

// Helper for GlobalTransform remains the same conceptually
pub fn global_transform_to_isometry3d(transform: &GlobalTransform) -> Isometry3d {
    transform_to_isometry3d(&transform.compute_transform())
}

// --- Constant defining the core rotation between frames ---
// Rotation from ENU (+X East, +Y North, +Z Up) to Bevy (+X East, +Y Up, +Z South)
// This is -90 degrees around the X axis.
thread_local! {
    // Define only one fundamental frame rotation quaternion
    pub static Q_ENU_FRAME_TO_BEVY_FRAME: UnitQuaternion<f64> =
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -FRAC_PI_2);
}
// --- Vector Transformations ---

/// Converts a 3D vector from ENU coordinates to Bevy world coordinates.
pub fn enu_vector_to_bevy_vector(enu_vec: &Vector3<f64>) -> BevyVec3 {
    BevyVec3::new(
        enu_vec.x as f32,  // East -> Bevy X
        enu_vec.z as f32,  // Up -> Bevy Y
        -enu_vec.y as f32, // North -> Bevy -Z (South)
    )
}

/// Converts a 3D vector from Bevy world coordinates to ENU coordinates.
pub fn bevy_vector_to_enu_vector(bevy_vec: &BevyVec3) -> Vector3<f64> {
    Vector3::new(
        bevy_vec.x as f64,  // Bevy X -> East
        -bevy_vec.z as f64, // Bevy -Z (South) -> North
        bevy_vec.y as f64,  // Bevy Y -> Up
    )
}

/// Converts a 3D point from ENU coordinates to Bevy world coordinates.
pub fn enu_vector_as_bevy_point(enu_vec: &Vector3<f64>) -> BevyVec3 {
    BevyVec3::new(
        enu_vec.x as f32,  // East -> Bevy X
        enu_vec.z as f32,  // Up -> Bevy Y
        -enu_vec.y as f32, // North -> Bevy -Z (South)
    )
}

/// Converts a 3D point from Bevy world coordinates to ENU coordinates.
/// (This function is used for the translation part of the pose)
pub fn bevy_point_as_enu_vector(bevy_point: &BevyVec3) -> Vector3<f64> {
    Vector3::new(
        bevy_point.x as f64,  // Bevy X -> East
        -bevy_point.z as f64, // Bevy -Z (South) -> North
        bevy_point.y as f64,  // Bevy Y -> Up
    )
}

/// Converts an orientation (quaternion) from ENU frame to Bevy world frame.
pub fn enu_quat_to_bevy_quat(enu_quat: &UnitQuaternion<f64>) -> BevyQuat {
    let final_rot_f64 = Q_ENU_FRAME_TO_BEVY_FRAME.with(|q_ef_bf| {
        q_ef_bf * enu_quat * q_ef_bf.inverse() // T_E->B * q_E * T_B->E
    });

    BevyQuat::from_xyzw(
        final_rot_f64.coords.x as f32,
        final_rot_f64.coords.y as f32,
        final_rot_f64.coords.z as f32,
        final_rot_f64.coords.w as f32,
    )
}

pub fn bevy_quat_to_enu_quat(bevy_quat: &BevyQuat) -> UnitQuaternion<f64> {
    // Ensure BevyQuat fields are correctly mapped to nalgebra Quaternion constructor (w, x, y, z)
    let bevy_q_f64 = UnitQuaternion::from_quaternion(Quaternion::new(
        bevy_quat.w as f64, // w first for nalgebra::Quaternion::new
        bevy_quat.x as f64,
        bevy_quat.y as f64,
        bevy_quat.z as f64,
    ));

    let final_rot_f64 = Q_ENU_FRAME_TO_BEVY_FRAME.with(|q_ef_bf| {
        q_ef_bf.inverse() * bevy_q_f64 * *q_ef_bf // T_B->E * q_B * T_E->B
    });
    final_rot_f64
}

// --- Full Pose Transformations ---

/// Converts a full pose (Isometry3) from ENU frame to Bevy Transform.
/// Use this for converting simulation state pose to rendering pose.
pub fn enu_iso_to_bevy_transform(enu_pose: &Isometry3<f64>) -> BevyTransform {
    let bevy_translation = enu_vector_as_bevy_point(&enu_pose.translation.vector);
    let bevy_rotation = enu_quat_to_bevy_quat(&enu_pose.rotation);

    BevyTransform {
        translation: bevy_translation,
        rotation: bevy_rotation,
        scale: BevyVec3::ONE, // Assume scale is 1 unless otherwise specified
    }
}

/// Converts a Bevy Transform to a full pose (Isometry3) in the ENU frame.
/// Use this for converting user input (e.g., mouse click -> Bevy pose) to simulation pose.
pub fn bevy_transform_to_enu_iso(bevy_transform: &BevyTransform) -> Isometry3<f64> {
    let enu_translation_vec = bevy_vector_to_enu_vector(&bevy_transform.translation);
    let enu_rotation = bevy_quat_to_enu_quat(&bevy_transform.rotation);

    Isometry3::from_parts(Translation3::from(enu_translation_vec), enu_rotation)
}

// --- Utility for Bevy GlobalTransform ---

/// Converts a Bevy GlobalTransform to a full pose (Isometry3) in the ENU frame.
pub fn bevy_global_transform_to_enu_iso(
    bevy_global_transform: &bevy::prelude::GlobalTransform,
) -> Isometry3<f64> {
    bevy_transform_to_enu_iso(&bevy_global_transform.compute_transform())
}
