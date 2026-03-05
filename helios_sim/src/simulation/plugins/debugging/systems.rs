use std::collections::VecDeque;
use std::f32::consts::TAU;

use bevy::prelude::*;
use nalgebra::{Isometry3, SymmetricEigen, Translation3, UnitQuaternion, Vector3};

use super::components::{DebugLegendNode, DebugSensorCache, DebugVisualizationConfig, PathTrail};
use crate::prelude::*;
use crate::simulation::core::components::GroundTruthState;
use crate::simulation::core::events::BevyMeasurementMessage;
use crate::simulation::core::transforms::{
    enu_iso_to_bevy_transform, enu_vector_to_bevy_vector, flu_vector_to_bevy_local_vector,
};
use crate::simulation::plugins::world_model::WorldModelComponent;
use helios_core::messages::MeasurementData;

// =========================================================================
// == Toggle System ==
// =========================================================================

/// Handles all debug visualization keybindings (F1–F6 and H).
pub fn handle_debug_keybindings(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut config: ResMut<DebugVisualizationConfig>,
) {
    if keyboard.just_pressed(KeyCode::F1) {
        config.show_pose_gimbals = !config.show_pose_gimbals;
        info!("[Debug] Pose Gimbals {}", on_off(config.show_pose_gimbals));
    }
    if keyboard.just_pressed(KeyCode::F2) {
        config.show_covariance = !config.show_covariance;
        info!("[Debug] Covariance Ellipsoid {}", on_off(config.show_covariance));
    }
    if keyboard.just_pressed(KeyCode::F3) {
        config.show_point_cloud = !config.show_point_cloud;
        info!("[Debug] Point Cloud {}", on_off(config.show_point_cloud));
    }
    if keyboard.just_pressed(KeyCode::F4) {
        config.show_velocity = !config.show_velocity;
        info!("[Debug] Velocity Vector {}", on_off(config.show_velocity));
    }
    if keyboard.just_pressed(KeyCode::F5) {
        config.show_error_line = !config.show_error_line;
        info!("[Debug] Estimation Error Line {}", on_off(config.show_error_line));
    }
    if keyboard.just_pressed(KeyCode::F6) {
        config.show_path_trail = !config.show_path_trail;
        info!("[Debug] Path Trail {}", on_off(config.show_path_trail));
    }
    if keyboard.just_pressed(KeyCode::KeyH) {
        config.show_legend = !config.show_legend;
    }
}

fn on_off(b: bool) -> &'static str {
    if b { "ON" } else { "OFF" }
}

// =========================================================================
// == Data Cache ==
// =========================================================================

/// Reads PointCloud events and stores the latest world-space points per sensor.
pub fn cache_sensor_data(
    mut events: EventReader<BevyMeasurementMessage>,
    transform_query: Query<&GlobalTransform>,
    mut cache: ResMut<DebugSensorCache>,
) {
    for msg in events.read() {
        if let MeasurementData::PointCloud(ref pc) = msg.0.data {
            let sensor_entity = msg.0.sensor_handle.to_entity();
            if let Ok(transform) = transform_query.get(sensor_entity) {
                let world_pts: Vec<Vec3> = pc
                    .points
                    .iter()
                    .map(|p| {
                        let flu = Vector3::new(p.position.x, p.position.y, p.position.z);
                        let local = flu_vector_to_bevy_local_vector(&flu);
                        transform.transform_point(local)
                    })
                    .collect();
                cache.point_clouds.insert(sensor_entity, world_pts);
            }
        }
    }
}

// =========================================================================
// == Legend UI ==
// =========================================================================

/// Spawns the debug legend UI panel once on entering Running state.
pub fn spawn_debug_legend(mut commands: Commands) {
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            right: Val::Px(10.0),
            padding: UiRect::all(Val::Px(10.0)),
            ..default()
        },
        BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.7)),
        Visibility::Hidden,
        Text::new(""),
        TextFont {
            font_size: 14.0,
            ..default()
        },
        TextColor(Color::WHITE),
        DebugLegendNode,
    ));
}

/// Updates the legend text content and visibility each frame.
pub fn update_legend_text(
    config: Res<DebugVisualizationConfig>,
    mut query: Query<(&mut Text, &mut Visibility), With<DebugLegendNode>>,
) {
    if let Ok((mut text, mut vis)) = query.single_mut() {
        *vis = if config.show_legend {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
        if config.show_legend {
            text.0 = format!(
                "=== Debug (H to hide) ===\nF1  Pose Gimbals    [{}]\nF2  Covariance      [{}]\nF3  Point Cloud     [{}]\nF4  Velocity        [{}]\nF5  Est. Error      [{}]\nF6  Path Trail      [{}]",
                on_off_bracket(config.show_pose_gimbals),
                on_off_bracket(config.show_covariance),
                on_off_bracket(config.show_point_cloud),
                on_off_bracket(config.show_velocity),
                on_off_bracket(config.show_error_line),
                on_off_bracket(config.show_path_trail),
            );
        }
    }
}

fn on_off_bracket(b: bool) -> &'static str {
    if b { "ON " } else { "OFF" }
}

// =========================================================================
// == Drawing Systems ==
// =========================================================================

/// Draws coordinate frame axes at the ground truth pose of each agent.
pub fn draw_ground_truth_gimbals(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    agent_query: Query<&GlobalTransform, With<GroundTruthState>>,
) {
    if !config.show_pose_gimbals {
        return;
    }
    for transform in &agent_query {
        gizmos.axes(*transform, 1.5);
    }
}

/// Draws coordinate frame axes at the estimated pose of each agent.
pub fn draw_estimated_pose_gimbals(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    module_query: Query<&WorldModelComponent>,
) {
    if !config.show_pose_gimbals {
        return;
    }
    for module in &module_query {
        if let WorldModelComponent::Separate { estimator, .. } = module {
            if let Some(estimated_pose_enu) = estimator.get_state().get_pose_isometry() {
                let bevy_transform = enu_iso_to_bevy_transform(&estimated_pose_enu);
                let global_transform = GlobalTransform::from(bevy_transform);
                let start = global_transform.translation();
                let length = 1.2;
                // Distinct colors from the GT axes (R/G/B)
                gizmos.line(start, start + global_transform.right() * length, Color::srgb(1.0, 0.2, 0.2));
                gizmos.line(start, start + global_transform.up() * length, Color::srgb(0.2, 1.0, 0.2));
                gizmos.line(start, start + global_transform.back() * length, Color::srgb(0.2, 0.2, 1.0));
            }
        }
    }
}

/// Draws a wireframe ellipsoid representing the 3-sigma position covariance.
pub fn draw_covariance_ellipsoid(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    module_query: Query<&WorldModelComponent>,
) {
    if !config.show_covariance {
        return;
    }
    for module in &module_query {
        let WorldModelComponent::Separate { estimator, .. } = module else {
            continue;
        };
        let state = estimator.get_state();

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
            warn!("[Debug] draw_covariance_ellipsoid: invalid eigenvalues {:?}", eigenvalues);
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
        // Clamp to a minimum of 0.05 m so the ellipsoid is always visible.
        let scale = Vector3::new(
            (eigenvalues[0] * sigma).sqrt().max(0.05),
            (eigenvalues[1] * sigma).sqrt().max(0.05),
            (eigenvalues[2] * sigma).sqrt().max(0.05),
        );
        if !scale.iter().all(|&v| v.is_finite()) {
            warn!("[Debug] draw_covariance_ellipsoid: non-finite scale {:?}", scale);
            continue;
        }

        let pose_enu = Isometry3::from_parts(Translation3::from(center_enu), orientation_enu);
        let bevy_tf = enu_iso_to_bevy_transform(&pose_enu);
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
            // Ring 1: e1–e2 plane
            gizmos.line(
                center + t0.cos() * e1 + t0.sin() * e2,
                center + t1.cos() * e1 + t1.sin() * e2,
                color,
            );
            // Ring 2: e1–e3 plane
            gizmos.line(
                center + t0.cos() * e1 + t0.sin() * e3,
                center + t1.cos() * e1 + t1.sin() * e3,
                color,
            );
            // Ring 3: e2–e3 plane
            gizmos.line(
                center + t0.cos() * e2 + t0.sin() * e3,
                center + t1.cos() * e2 + t1.sin() * e3,
                color,
            );
        }
    }
}

/// Draws cyan cross markers at each cached world-space point cloud position.
pub fn draw_point_cloud(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    cache: Res<DebugSensorCache>,
) {
    if !config.show_point_cloud {
        return;
    }
    let color = Color::srgba(0.0, 1.0, 0.8, 0.9);
    for pts in cache.point_clouds.values() {
        for &pt in pts {
            gizmos.sphere(Isometry3d::from_translation(pt), 0.05, color);
        }
    }
}

/// Draws a yellow arrow showing the ground truth linear velocity.
pub fn draw_velocity_vector(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    query: Query<(&GroundTruthState, &GlobalTransform)>,
) {
    if !config.show_velocity {
        return;
    }
    for (gt, transform) in &query {
        let origin = transform.translation();
        let vel_bevy = enu_vector_to_bevy_vector(&gt.linear_velocity);
        if vel_bevy.length_squared() > 1e-6 {
            gizmos.arrow(origin, origin + vel_bevy, Color::srgb(1.0, 1.0, 0.0));
        }
    }
}

/// Draws a red line between the ground truth position and the estimated position.
pub fn draw_estimation_error_line(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    query: Query<(&GroundTruthState, &WorldModelComponent)>,
) {
    if !config.show_error_line {
        return;
    }
    for (gt, module) in &query {
        if let WorldModelComponent::Separate { estimator, .. } = module {
            if let Some(estimated_pose) = estimator.get_state().get_pose_isometry() {
                let gt_bevy = enu_iso_to_bevy_transform(&gt.pose).translation;
                let est_bevy = enu_iso_to_bevy_transform(&estimated_pose).translation;
                gizmos.line(gt_bevy, est_bevy, Color::srgb(1.0, 0.0, 0.0));
            }
        }
    }
}

/// Appends the current world position to each agent's path trail ring buffer.
/// Runs in FixedUpdate / Validation so the sampling rate matches physics.
pub fn update_path_trail(
    mut commands: Commands,
    mut query: Query<(Entity, &GlobalTransform, Option<&mut PathTrail>), With<GroundTruthState>>,
) {
    for (entity, transform, trail_opt) in &mut query {
        let current_pos = transform.translation();
        if let Some(mut trail) = trail_opt {
            trail.positions.push_back(current_pos);
            while trail.positions.len() > trail.max_len {
                trail.positions.pop_front();
            }
        } else {
            let mut positions = VecDeque::new();
            positions.push_back(current_pos);
            commands.entity(entity).insert(PathTrail {
                positions,
                max_len: 300,
            });
        }
    }
}

/// Draws a green polyline tracing the recent path of each agent.
pub fn draw_path_trail(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    query: Query<&PathTrail>,
) {
    if !config.show_path_trail {
        return;
    }
    for trail in &query {
        let pts: Vec<Vec3> = trail.positions.iter().copied().collect();
        if pts.len() >= 2 {
            gizmos.linestrip(pts, Color::srgba(0.5, 1.0, 0.5, 0.7));
        }
    }
}
