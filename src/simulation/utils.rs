// src/simulation/utils.rs (or near the top of systems.rs)
use bevy::math::{EulerRot, Isometry3d};
use bevy::prelude::{GlobalTransform, Quat, Transform, Vec3};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};

/// Converts a Bevy Transform or GlobalTransform into an nalgebra Isometry3<f64>.
pub fn bevy_transform_to_nalgebra_isometry(transform: &Transform) -> Isometry3<f64> {
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
pub fn transform_to_isometry3d(transform: &Transform) -> Isometry3d {
    let translation: Vec3 = transform.translation; // Bevy Vec3 (f32)
    let rotation: Quat = transform.rotation; // Bevy Quat (f32)

    Isometry3d::new(translation, rotation)
}

// Helper for GlobalTransform remains the same conceptually
pub fn global_transform_to_isometry3d(transform: &GlobalTransform) -> Isometry3d {
    transform_to_isometry3d(&transform.compute_transform())
}

// --- NEW: Simulation Frame <-> Bevy Frame Conversions ---

/// Converts a Simulation Frame pose (X=Right, Y=Up, Z=Forward) to a Bevy Transform.
/// Sets Bevy Y based on the provided ground_y_offset.
pub fn sim_pose_to_bevy_transform(
    sim_x: f64,
    sim_z: f64,
    sim_yaw: f64, // Radians around Y_sim (Up)
    ground_y_offset: f32,
) -> Transform {
    Transform {
        translation: Vec3::new(
            sim_x as f32,
            ground_y_offset, // Set Bevy Y based on offset
            -sim_z as f32,   // Bevy Z = -Sim Z
        ),
        rotation: Quat::from_rotation_y(sim_yaw as f32), // Bevy Y rotation = Sim Yaw around Y
        ..Default::default()
    }
}

/// Converts a Bevy Transform to a Simulation Frame pose (Isometry3<f64>).
/// Assumes the Bevy Transform represents an object on the XZ plane.
pub fn bevy_transform_to_sim_pose(transform: &Transform) -> Isometry3<f64> {
    let bevy_pos = transform.translation;
    let bevy_rot = transform.rotation;

    // Convert position
    let sim_x = bevy_pos.x as f64;
    let sim_y = bevy_pos.y as f64; // Keep track of altitude if needed, often 0 for ground poses
    let sim_z = -bevy_pos.z as f64; // Sim Z = -Bevy Z

    // Convert rotation (assuming primary rotation is Yaw around Bevy Y)
    // Bevy Y rotation maps directly to Sim Yaw around Sim Y
    // For pure yaw:
    let sim_yaw = bevy_rot.to_euler(EulerRot::YXZ).0 as f64; // Extract Y rotation as Sim Yaw
    let sim_rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), sim_yaw);

    // More general conversion (handles pitch/roll if necessary, though maybe not needed for ground obstacles)
    // let nalgebra_rot = UnitQuaternion::from_quaternion(Quaternion::new(
    //     bevy_rot.w as f64, bevy_rot.x as f64, bevy_rot.y as f64, bevy_rot.z as f64
    // ));
    // // Need to potentially adjust axes if Bevy pitch/roll were used relative to Bevy Z/X axes
    // // For now, assume primarily Yaw for ground objects/vehicles
    // let sim_rotation = nalgebra_rot; // Placeholder if more complex rotation needed

    Isometry3::from_parts(Translation3::new(sim_x, sim_y, sim_z), sim_rotation)
}

/// Converts a Simulation Frame position (X=Right, Z=Forward) to a Bevy Vec3.
/// Uses ground_y_offset for the Bevy Y coordinate.
pub fn sim_pos_to_bevy_pos(sim_x: f64, sim_z: f64, ground_y_offset: f32) -> Vec3 {
    Vec3::new(sim_x as f32, ground_y_offset, -sim_z as f32)
}
