// src/simulation/utils.rs (or near the top of systems.rs)
use bevy::math::Isometry3d;
use bevy::prelude::{GlobalTransform, Quat, Transform, Vec3};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};

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
