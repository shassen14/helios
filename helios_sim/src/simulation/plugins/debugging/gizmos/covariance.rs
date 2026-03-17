use std::f32::consts::TAU;

use bevy::prelude::*;
use nalgebra::{Isometry3, SymmetricEigen, Translation3, UnitQuaternion, Vector3};

use crate::simulation::core::transforms::enu_body_iso_to_bevy_transform;
use crate::simulation::plugins::autonomy::EstimatorComponent;
use crate::simulation::plugins::debugging::components::DebugVisualizationConfig;
use helios_core::prelude::{FrameId, StateVariable};

/// Draws a wireframe ellipsoid representing the 3-sigma position covariance.
pub fn draw_covariance_ellipsoid(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    module_query: Query<&EstimatorComponent>,
) {
    if !config.show_covariance {
        return;
    }
    for module in &module_query {
        let Some(state) = module.0.get_state() else {
            continue;
        };

        let cov_3x3 = match state.get_sub_covariance_3x3(&StateVariable::Px(FrameId::World)) {
            Some(c) => c,
            None => {
                warn!("[Debug] draw_covariance_ellipsoid: get_sub_covariance_3x3 returned None");
                continue;
            }
        };

        let eigen = SymmetricEigen::new(cov_3x3);
        let eigenvalues = eigen.eigenvalues;
        let eigenvectors = eigen.eigenvectors;

        if !eigenvalues.iter().all(|&v| v.is_finite() && v >= 0.0) {
            warn!(
                "[Debug] draw_covariance_ellipsoid: invalid eigenvalues {:?}",
                eigenvalues
            );
            continue;
        }

        let center_enu = state
            .get_vector3(&StateVariable::Px(FrameId::World))
            .unwrap_or_default();

        let mut rot_mat = eigenvectors;
        if rot_mat.determinant() < 0.0 {
            rot_mat.column_mut(2).scale_mut(-1.0);
        }
        let orientation_enu = UnitQuaternion::from_matrix(&rot_mat);

        let sigma = 3.0_f64;
        let scale = Vector3::new(
            (eigenvalues[0] * sigma).sqrt().max(0.05),
            (eigenvalues[1] * sigma).sqrt().max(0.05),
            (eigenvalues[2] * sigma).sqrt().max(0.05),
        );
        if !scale.iter().all(|&v| v.is_finite()) {
            warn!(
                "[Debug] draw_covariance_ellipsoid: non-finite scale {:?}",
                scale
            );
            continue;
        }

        let pose_enu = Isometry3::from_parts(Translation3::from(center_enu), orientation_enu);
        let bevy_tf = enu_body_iso_to_bevy_transform(&pose_enu);
        let center = bevy_tf.translation;
        let rot = bevy_tf.rotation;

        let e1 = rot * Vec3::X * scale.x as f32;
        let e2 = rot * Vec3::Y * scale.y as f32;
        let e3 = rot * Vec3::Z * scale.z as f32;

        let color = Color::srgba(1.0, 1.0, 1.0, 0.8);
        let n = 64usize;
        for i in 0..n {
            let t0 = (i as f32 / n as f32) * TAU;
            let t1 = ((i + 1) as f32 / n as f32) * TAU;
            gizmos.line(
                center + t0.cos() * e1 + t0.sin() * e2,
                center + t1.cos() * e1 + t1.sin() * e2,
                color,
            );
            gizmos.line(
                center + t0.cos() * e1 + t0.sin() * e3,
                center + t1.cos() * e1 + t1.sin() * e3,
                color,
            );
            gizmos.line(
                center + t0.cos() * e2 + t0.sin() * e3,
                center + t1.cos() * e2 + t1.sin() * e3,
                color,
            );
        }
    }
}
