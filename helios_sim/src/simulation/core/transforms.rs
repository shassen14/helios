// helios_sim/src/simulation/core/transforms.rs

use bevy::prelude::{
    GlobalTransform, Quat as BevyQuat, Transform as BevyTransform, Vec3 as BevyVec3, *,
};
use helios_core::prelude::{FrameHandle, TfProvider}; // Use the pure types from core
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
use std::collections::HashMap;
use std::f64::consts::FRAC_PI_2;

// =========================================================================
// == TF Tree Infrastructure (The "Service") ==
// =========================================================================

/// A Bevy component to mark any entity that should be tracked by the TF system.
#[derive(Component)]
pub struct TrackedFrame;

/// The Bevy resource that holds the complete transform graph for a single frame.
#[derive(Resource, Default, Debug)]
pub struct TfTree {
    /// Stores the WORLD pose (in nalgebra's Isometry3 format) of every tracked entity.
    transforms_to_world: HashMap<Entity, Isometry3<f64>>,
}

// We implement the pure `TfProvider` trait for our Bevy-specific `TfTree`.
impl TfProvider for TfTree {
    fn get_transform(
        &self,
        from_frame: FrameHandle,
        to_frame: FrameHandle,
    ) -> Option<Isometry3<f64>> {
        // Translate from the pure FrameHandle to the Bevy Entity.
        let from_entity = Entity::from_bits(from_frame.0);
        let to_entity = Entity::from_bits(to_frame.0);

        let pose_from_world = self.transforms_to_world.get(&from_entity)?;
        let pose_to_world = self.transforms_to_world.get(&to_entity)?;

        // The math remains the same: T_B_A = (T_W_A)^-1 * T_W_B
        Some(pose_from_world.inverse() * pose_to_world)
    }
}

/// The Bevy system that runs once per frame to build the TfTree resource.
pub fn tf_tree_builder_system(
    mut tf_tree: ResMut<TfTree>,
    query: Query<(Entity, &GlobalTransform), With<TrackedFrame>>,
) {
    tf_tree.transforms_to_world.clear();
    for (entity, transform) in &query {
        // Use the helper function below to convert from Bevy's GlobalTransform
        // to our robotics-standard nalgebra Isometry3.
        let iso = bevy_global_transform_to_nalgebra_isometry(transform);
        tf_tree.transforms_to_world.insert(entity, iso);
    }
}

// =========================================================================
// == Coordinate System Conversion Helpers (Your original code, slightly polished) ==
// =========================================================================

/// Converts a Bevy `Transform` into a `nalgebra::Isometry3<f64>`.
/// This function does NOT handle ENU/Bevy coordinate system swaps.
pub fn bevy_transform_to_nalgebra_isometry(transform: &BevyTransform) -> Isometry3<f64> {
    let t = transform.translation;
    let r = transform.rotation;
    Isometry3::from_parts(
        Translation3::new(t.x as f64, t.y as f64, t.z as f64),
        UnitQuaternion::from_quaternion(Quaternion::new(
            r.w as f64, r.x as f64, r.y as f64, r.z as f64,
        )),
    )
}

/// Converts a Bevy `GlobalTransform` into a `nalgebra::Isometry3<f64>`.
pub fn bevy_global_transform_to_nalgebra_isometry(transform: &GlobalTransform) -> Isometry3<f64> {
    bevy_transform_to_nalgebra_isometry(&transform.compute_transform())
}

// --- ENU <-> BEVY Transformations ---
thread_local! {
    /// Quaternion representing the rotation from the ENU coordinate frame to the Bevy coordinate frame.
    /// If you have basis vectors of ENU, multiplying by this Q gives you those basis vectors
    /// expressed in Bevy's coordinate system.
    /// E.g., ENU's X-axis (1,0,0) becomes (1,0,0) in Bevy coords.
    /// ENU's Y-axis (0,1,0) (North) becomes (0,0,-1) in Bevy coords (Bevy -Z).
    /// ENU's Z-axis (0,0,1) (Up) becomes (0,1,0) in Bevy coords (Bevy Y).
    /// This corresponds to a -90 degree rotation around the X-axis.
    pub static Q_ENU_FRAME_TO_BEVY_FRAME: UnitQuaternion<f64> =
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -FRAC_PI_2);
}

/// Converts a 3D coordinate vector from ENU to Bevy world.
pub fn enu_vector_to_bevy_vector(enu_vec: &Vector3<f64>) -> BevyVec3 {
    // Equivalent to Q_ENU_FRAME_TO_BEVY_FRAME.with(|q| *q * enu_vec) then f32 cast
    BevyVec3::new(
        enu_vec.x as f32,  // East -> Bevy X
        enu_vec.z as f32,  // ENU Up -> Bevy Y
        -enu_vec.y as f32, // ENU North -> Bevy -Z (South direction vector)
    )
}

/// Converts a 3D coordinate vector from Bevy world to ENU.
pub fn bevy_vector_to_enu_vector(bevy_vec: &BevyVec3) -> Vector3<f64> {
    // Equivalent to Q_ENU_FRAME_TO_BEVY_FRAME.with(|q| q.inverse() * bevy_vec_f64)
    Vector3::new(
        bevy_vec.x as f64,  // Bevy X -> ENU East
        -bevy_vec.z as f64, // Bevy -Z (South direction) -> ENU North
        bevy_vec.y as f64,  // Bevy Y -> ENU Up
    )
}

// These two are for clarity if dealing with points vs vectors for translation parts
// but essentially do the same as enu_vector_to_bevy_vector / bevy_vector_to_enu_vector
pub fn enu_vector_as_bevy_point(enu_vec: &Vector3<f64>) -> BevyVec3 {
    enu_vector_to_bevy_vector(enu_vec)
}
pub fn bevy_point_as_enu_vector(bevy_point: &BevyVec3) -> Vector3<f64> {
    bevy_vector_to_enu_vector(bevy_point)
}

/// Converts an object's orientation from ENU frame to Bevy world frame.
pub fn enu_quat_to_bevy_quat(enu_obj_quat: &UnitQuaternion<f64>) -> BevyQuat {
    // q_obj_in_Bevy = Q_Frame(Bevy_from_ENU) * q_obj_in_ENU * Q_Frame(Bevy_from_ENU)_inverse
    let final_rot_f64 = Q_ENU_FRAME_TO_BEVY_FRAME.with(|q_enu_to_bevy_frame| {
        *q_enu_to_bevy_frame * enu_obj_quat * q_enu_to_bevy_frame.inverse()
    });

    BevyQuat::from_xyzw(
        final_rot_f64.coords.x as f32,
        final_rot_f64.coords.y as f32,
        final_rot_f64.coords.z as f32,
        final_rot_f64.coords.w as f32,
    )
}

/// Converts an object's orientation from Bevy world frame to ENU frame.
pub fn bevy_quat_to_enu_quat(bevy_obj_quat: &BevyQuat) -> UnitQuaternion<f64> {
    let bevy_q_f64 = UnitQuaternion::from_quaternion(Quaternion::new(
        bevy_obj_quat.w as f64, // nalgebra Quaternion::new is w,x,y,z
        bevy_obj_quat.x as f64,
        bevy_obj_quat.y as f64,
        bevy_obj_quat.z as f64,
    ));

    // q_obj_in_ENU = Q_Frame(Bevy_from_ENU)_inverse * q_obj_in_Bevy * Q_Frame(Bevy_from_ENU)
    let final_rot_f64 = Q_ENU_FRAME_TO_BEVY_FRAME.with(|q_enu_to_bevy_frame| {
        q_enu_to_bevy_frame.inverse() * bevy_q_f64 * *q_enu_to_bevy_frame
    });
    final_rot_f64
}

/// Converts a full pose (Isometry3) from ENU frame to Bevy Transform.
pub fn enu_iso_to_bevy_transform(enu_pose: &Isometry3<f64>) -> BevyTransform {
    let bevy_translation = enu_vector_to_bevy_vector(&enu_pose.translation.vector);
    let bevy_rotation = enu_quat_to_bevy_quat(&enu_pose.rotation);

    BevyTransform {
        translation: bevy_translation,
        rotation: bevy_rotation,
        scale: BevyVec3::ONE,
    }
}

/// Converts a Bevy Transform to a full pose (Isometry3) in the ENU frame.
pub fn bevy_transform_to_enu_iso(bevy_transform: &BevyTransform) -> Isometry3<f64> {
    let enu_translation_vector = bevy_vector_to_enu_vector(&bevy_transform.translation);
    let enu_rotation = bevy_quat_to_enu_quat(&bevy_transform.rotation);

    Isometry3::from_parts(Translation3::from(enu_translation_vector), enu_rotation)
}

pub fn bevy_global_transform_to_enu_iso(bevy_global_transform: &GlobalTransform) -> Isometry3<f64> {
    bevy_transform_to_enu_iso(&bevy_global_transform.compute_transform())
}

// --- Unit Test Module with Renamed Tests ---
#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use std::f32::consts::PI as PI_F32;
    use std::f64::consts::PI as PI_F64; // Using approx for float comparisons

    const F64_EPSILON: f64 = 1e-7;
    const F32_EPSILON: f32 = 1e-5;

    fn assert_bevy_quat_approx_eq(q1: &BevyQuat, q2: &BevyQuat, epsilon: f32) {
        let dot_product = q1.dot(*q2);
        // If q1 and q2 are normalized and represent same rotation, dot is cos(angle_between/2 * 2) = cos(angle_between)
        // Or, more directly, for q and -q representing same rotation, abs(dot) should be 1.0
        assert!(
            dot_product.abs() > 1.0 - epsilon,
            "BevyQuats not approx equal: {:?} vs {:?}, dot: {}",
            q1,
            q2,
            dot_product
        );
    }

    fn assert_nalgebra_quat_approx_eq(
        q1: &UnitQuaternion<f64>,
        q2: &UnitQuaternion<f64>,
        epsilon: f64,
    ) {
        let angle_diff = q1.angle_to(q2);
        assert!(
            angle_diff.abs() < epsilon || (angle_diff.abs() - PI_F64).abs() < epsilon, // Checks for q or -q
            "Nalgebra UnitQuaternions not approx equal. q1: {:?}, q2: {:?}, angle_diff: {}",
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

    // --- Tests for Bevy <-> Nalgebra Isometry (Original Utils) ---
    #[test]
    fn test_bevy_transform_to_nalgebra_isometry_identity() {
        let bevy_tf = BevyTransform::IDENTITY;
        let nalgebra_iso = bevy_transform_to_nalgebra_isometry(&bevy_tf);
        assert_nalgebra_vector3_approx_eq(
            &nalgebra_iso.translation.vector,
            &Vector3::zeros(),
            F64_EPSILON,
        );
        assert_nalgebra_quat_approx_eq(
            &nalgebra_iso.rotation,
            &UnitQuaternion::identity(),
            F64_EPSILON,
        );
    }

    #[test]
    fn test_bevy_transform_to_nalgebra_isometry_general() {
        let bevy_tf = BevyTransform::from_xyz(1.0, 2.0, 3.0)
            .with_rotation(BevyQuat::from_rotation_y(PI_F32 / 2.0));
        let nalgebra_iso = bevy_transform_to_nalgebra_isometry(&bevy_tf);

        let expected_translation = Vector3::new(1.0, 2.0, 3.0);
        let expected_rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI_F64 / 2.0);

        assert_nalgebra_vector3_approx_eq(
            &nalgebra_iso.translation.vector,
            &expected_translation,
            F64_EPSILON,
        );
        assert_nalgebra_quat_approx_eq(&nalgebra_iso.rotation, &expected_rotation, F64_EPSILON);
    }

    // --- Tests for ENU <-> Bevy Coordinate Vector Transformations ---
    #[test]
    fn test_enu_vector_to_bevy_vector_and_back() {
        let enu_vec = Vector3::new(1.0, 2.0, 3.0); // E=1, N=2, U=3
        let bevy_vec = enu_vector_to_bevy_vector(&enu_vec);
        let expected_bevy_vec = BevyVec3::new(1.0, 3.0, -2.0); // X=E, Y=U, Z=-N
        assert_bevy_vec3_approx_eq(&bevy_vec, &expected_bevy_vec, F32_EPSILON);

        let enu_vec_back = bevy_vector_to_enu_vector(&bevy_vec);
        assert_nalgebra_vector3_approx_eq(&enu_vec_back, &enu_vec, F64_EPSILON);
    }

    #[test]
    fn test_enu_vector_as_bevy_point_and_back() {
        // Assuming these are for translation parts
        let enu_trans_vec = Vector3::new(4.0, 5.0, 6.0); // E, N, U
        let bevy_point_vec = enu_vector_as_bevy_point(&enu_trans_vec);
        let expected_bevy_point_vec = BevyVec3::new(4.0, 6.0, -5.0); // X=E, Y=U, Z=-N
        assert_bevy_vec3_approx_eq(&bevy_point_vec, &expected_bevy_point_vec, F32_EPSILON);

        let enu_trans_vec_back = bevy_point_as_enu_vector(&bevy_point_vec);
        assert_nalgebra_vector3_approx_eq(&enu_trans_vec_back, &enu_trans_vec, F64_EPSILON);
    }

    // --- Tests for ENU <-> Bevy Object Orientation Transformations ---
    #[test]
    fn test_enu_quat_to_bevy_quat_identity() {
        let enu_id = UnitQuaternion::identity();
        let bevy_id_from_enu = enu_quat_to_bevy_quat(&enu_id);
        assert_bevy_quat_approx_eq(&bevy_id_from_enu, &BevyQuat::IDENTITY, F32_EPSILON);
    }

    #[test]
    fn test_bevy_quat_to_enu_quat_identity() {
        let bevy_id = BevyQuat::IDENTITY;
        let enu_id_from_bevy = bevy_quat_to_enu_quat(&bevy_id);
        assert_nalgebra_quat_approx_eq(&enu_id_from_bevy, &UnitQuaternion::identity(), F64_EPSILON);
    }

    #[test]
    fn test_enu_quat_to_bevy_quat_enu_yaw() {
        let enu_yaw_90 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI_F64 / 2.0); // ENU Yaw (around Z_up)
        let bevy_equiv_yaw_90 = enu_quat_to_bevy_quat(&enu_yaw_90);
        let expected_bevy_yaw_90 = BevyQuat::from_rotation_y(PI_F32 / 2.0); // Bevy Yaw (around Y_up)
        assert_bevy_quat_approx_eq(&bevy_equiv_yaw_90, &expected_bevy_yaw_90, F32_EPSILON);
    }

    #[test]
    fn test_bevy_quat_to_enu_quat_bevy_yaw() {
        let bevy_yaw_90 = BevyQuat::from_rotation_y(PI_F32 / 2.0); // Bevy Yaw
        let enu_equiv_yaw_90 = bevy_quat_to_enu_quat(&bevy_yaw_90);
        let expected_enu_yaw_90 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI_F64 / 2.0); // ENU Yaw
        assert_nalgebra_quat_approx_eq(&enu_equiv_yaw_90, &expected_enu_yaw_90, F64_EPSILON);
    }

    #[test]
    fn test_enu_quat_to_bevy_quat_enu_roll() {
        // Roll around common X-axis
        let enu_roll_90 = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI_F64 / 2.0);
        let bevy_equiv_roll_90 = enu_quat_to_bevy_quat(&enu_roll_90);
        let expected_bevy_roll_90 = BevyQuat::from_rotation_x(PI_F32 / 2.0);
        assert_bevy_quat_approx_eq(&bevy_equiv_roll_90, &expected_bevy_roll_90, F32_EPSILON);
    }

    // --- Tests for ENU <-> Bevy Full Pose Transformations ---
    #[test]
    fn test_enu_iso_to_bevy_transform_identity() {
        let enu_pose_id = Isometry3::identity();
        let bevy_transform_id = enu_iso_to_bevy_transform(&enu_pose_id);
        assert_bevy_vec3_approx_eq(&bevy_transform_id.translation, &BevyVec3::ZERO, F32_EPSILON);
        assert_bevy_quat_approx_eq(
            &bevy_transform_id.rotation,
            &BevyQuat::IDENTITY,
            F32_EPSILON,
        );
    }

    #[test]
    fn test_bevy_transform_to_enu_iso_identity() {
        let bevy_transform_id = BevyTransform::IDENTITY;
        let enu_pose_id_back = bevy_transform_to_enu_iso(&bevy_transform_id);
        assert_nalgebra_vector3_approx_eq(
            &enu_pose_id_back.translation.vector,
            &Vector3::zeros(),
            F64_EPSILON,
        );
        assert_nalgebra_quat_approx_eq(
            &enu_pose_id_back.rotation,
            &UnitQuaternion::identity(),
            F64_EPSILON,
        );
    }

    #[test]
    fn test_enu_iso_to_bevy_transform_general_and_back() {
        let enu_translation = Vector3::new(1.0, 2.0, 0.5); // E, N, U
        let enu_yaw_rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI_F64 / 4.0);
        let enu_pose = Isometry3::from_parts(Translation3::from(enu_translation), enu_yaw_rot);

        let bevy_transform = enu_iso_to_bevy_transform(&enu_pose);

        let expected_bevy_translation = BevyVec3::new(1.0, 0.5, -2.0); // X=E, Y=U, Z=-N
        let expected_bevy_rotation = BevyQuat::from_rotation_y(PI_F32 / 4.0);

        assert_bevy_vec3_approx_eq(
            &bevy_transform.translation,
            &expected_bevy_translation,
            F32_EPSILON,
        );
        assert_bevy_quat_approx_eq(
            &bevy_transform.rotation,
            &expected_bevy_rotation,
            F32_EPSILON,
        );

        let enu_pose_back = bevy_transform_to_enu_iso(&bevy_transform);
        assert_nalgebra_vector3_approx_eq(
            &enu_pose_back.translation.vector,
            &enu_translation,
            F64_EPSILON,
        );
        assert_nalgebra_quat_approx_eq(&enu_pose_back.rotation, &enu_yaw_rot, F64_EPSILON);
    }
}
