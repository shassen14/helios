use crate::{
    prelude::*,
    simulation::{core::topics::GroundTruthState, plugins::world_model::WorldModelComponent},
};
use bevy::prelude::*;
use nalgebra::{Isometry3, SymmetricEigen, Translation3, UnitQuaternion, Vector3};

/// A resource to hold the shared meshes and materials for the gimbals.
#[derive(Resource)]
struct GimbalAssets {
    x_axis_mesh: Handle<Mesh>,
    y_axis_mesh: Handle<Mesh>,
    z_axis_mesh: Handle<Mesh>,
    x_material: Handle<StandardMaterial>,
    y_material: Handle<StandardMaterial>,
    z_material: Handle<StandardMaterial>,
    ground_truth_axis_length: f32,
    estimated_axis_length: f32,
}

pub struct DebugGimbalPlugin;

impl Plugin for DebugGimbalPlugin {
    fn build(&self, app: &mut App) {
        app
            // We only need to add the two drawing systems to the Update schedule.
            // They will run every frame and draw the gizmos from scratch.
            .add_systems(
                Update,
                (
                    draw_ground_truth_gimbals,
                    draw_estimated_pose_gimbals,
                    // draw_covariance_ellipsoids,
                )
                    .run_if(in_state(AppState::Running)),
            );
    }
}
// =========================================================================
// == Systems ==
// =========================================================================

/// Draws a gimbal (a set of 3 colored axes) at the agent's ground truth pose.
fn draw_ground_truth_gimbals(
    mut gizmos: Gizmos,
    // We query for the agent's GlobalTransform directly.
    agent_query: Query<(Entity, &GlobalTransform), With<GroundTruthState>>,
) {
    for (entity, agent_transform) in &agent_query {
        let length = 1.5;

        // The `gizmos.axes` function is the simplest way to draw a coordinate frame.
        // It draws three lines: Red for X, Green for Y, Blue for Z.
        gizmos.axes(*agent_transform, length);

        // Optional: Add a text label to identify the ground truth gimbal.
        // We draw it slightly above the agent's origin.
        // let label_position = agent_transform.translation() + agent_transform.up() * (length + 0.3);
        // gizmos.text(label_position, format!("GT: {:?}", entity), Color::WHITE);
    }
}

/// Draws a gimbal at the pose estimated by the WorldModelComponent.
fn draw_estimated_pose_gimbals(
    mut gizmos: Gizmos,
    module_query: Query<(Entity, &WorldModelComponent)>,
) {
    for (entity, module) in &module_query {
        if let WorldModelComponent::Separate { estimator, .. } = module {
            if let Some(estimated_pose_enu) = estimator.get_state().get_pose_isometry() {
                let bevy_transform = crate::simulation::core::transforms::enu_iso_to_bevy_transform(
                    &estimated_pose_enu,
                );
                let global_transform = GlobalTransform::from(bevy_transform);

                let length = 1.2; // Smaller for the estimate

                // We can use `gizmos.axes` again for the estimated pose.
                // To distinguish it, we can either make it smaller or use different colors
                // by drawing the lines manually. Let's draw manually for variety.

                let start = global_transform.translation();

                // X-Axis (Cyan to distinguish from GT's red)
                gizmos.line(
                    start,
                    start + global_transform.right() * length,
                    Color::srgb(1.0, 0.2, 0.2),
                );
                // Y-Axis (Yellow to distinguish from GT's green)
                gizmos.line(
                    start,
                    start + global_transform.up() * length,
                    Color::srgb(0.2, 1.0, 0.2),
                );
                // Z-Axis (Magenta to distinguish from GT's blue)
                gizmos.line(
                    start,
                    start + global_transform.back() * length,
                    Color::srgb(0.2, 0.2, 1.0),
                );

                // Add a text label
                // let label_position = start + global_transform.up() * (length + 0.3);
                // gizmos.text(label_position, format!("Est: {:?}", entity), Color::WHITE);
            }
        }
    }
}

/// Draws the 3D position covariance from the estimator as a semi-transparent ellipsoid.
fn draw_covariance_ellipsoids(mut gizmos: Gizmos, module_query: Query<&WorldModelComponent>) {
    // Use get_single() to avoid panicking if there are zero or multiple agents.
    if let Ok(module) = module_query.single() {
        if let WorldModelComponent::Separate { estimator, .. } = module {
            let state = estimator.get_state();

            let position_covariance_3x3 =
                match state.get_sub_covariance_3x3(&StateVariable::Px(FrameId::World)) {
                    Some(cov) => cov,
                    None => return,
                };

            // --- 1. Perform Eigen-decomposition Safely ---
            let eigen = SymmetricEigen::new(position_covariance_3x3);
            let eigenvalues = eigen.eigenvalues;
            let eigenvectors = eigen.eigenvectors;

            // --- 2. VALIDATE the Eigenvalues ---
            // This is the critical safeguard. Check if any eigenvalue is negative or non-finite.
            if !eigenvalues.iter().all(|&val| val.is_finite() && val >= 0.0) {
                // If we have an invalid eigenvalue (negative or NaN), we cannot draw the
                // ellipsoid. We simply return early for this frame.
                // You could add a log::warn! here if you wanted to be notified of this.
                return;
            }

            // --- 3. Calculate the Ellipsoid's Transform ---
            let center_enu = state
                .get_vector3(&StateVariable::Px(FrameId::World))
                .unwrap_or_default();

            // --- THE FIX: Ensure the eigenvectors form a valid rotation matrix ---
            let mut rotation_matrix = eigenvectors;

            // Check the determinant. If it's negative, it's a reflection, not a rotation.
            if rotation_matrix.determinant() < 0.0 {
                // We can fix this by flipping one of the axes (columns).
                // Let's flip the last one. This turns the reflection into a proper rotation.
                rotation_matrix.column_mut(2).scale_mut(-1.0);
            }

            // Using from_matrix is fine, but it can panic if the matrix is not a valid rotation.
            // A safer alternative is to ensure the matrix is special orthogonal (has determinant +1).
            // For now, we trust the output of SymmetricEigen.
            let orientation_enu = UnitQuaternion::from_matrix(&rotation_matrix);

            let sigma_scale_factor = 3.0;
            // Now that we've checked for negative values, the .sqrt() calls are safe.
            let scale_enu = Vector3::new(
                (eigenvalues[0] * sigma_scale_factor).sqrt(),
                (eigenvalues[1] * sigma_scale_factor).sqrt(),
                (eigenvalues[2] * sigma_scale_factor).sqrt(),
            );

            // Another safeguard: if scale is NaN or infinite, don't draw.
            if !scale_enu.iter().all(|&val| val.is_finite()) {
                return;
            }

            let ellipsoid_pose_enu =
                Isometry3::from_parts(Translation3::from(center_enu), orientation_enu);

            // --- 4. Convert to Bevy Coordinates and Draw ---
            let bevy_transform =
                crate::simulation::core::transforms::enu_iso_to_bevy_transform(&ellipsoid_pose_enu);

            // The gizmos.sphere method takes a position, rotation, radius, and color.
            // We can't directly use a scaled transform. We must draw a unit sphere
            // and apply the transform to it. Bevy's gizmos don't have a direct "draw ellipsoid"
            // method. We can use a workaround by creating a custom transform.
            // A simpler approach is to draw a wireframe sphere.

            // Let's use a simpler gizmo method for now that is less likely to have issues.
            // `gizmos.sphere` is for drawing, not a bundle. The previous code was incorrect.
            // The gizmos API does not have a scaled sphere. We can draw the axes instead.

            // Draw the principal axes of the ellipsoid. This is very informative.
            let center_bevy = bevy_transform.translation;
            let rot_bevy = bevy_transform.rotation;

            // Axis 1
            gizmos.line(
                center_bevy,
                center_bevy + rot_bevy * (Vec3::X * scale_enu.x as f32),
                Color::WHITE,
            );
            // Axis 2
            gizmos.line(
                center_bevy,
                center_bevy + rot_bevy * (Vec3::Y * scale_enu.y as f32),
                Color::WHITE,
            );
            // Axis 3
            gizmos.line(
                center_bevy,
                center_bevy + rot_bevy * (Vec3::Z * scale_enu.z as f32),
                Color::WHITE,
            );
        }
    }
}
