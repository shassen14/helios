use std::collections::VecDeque;
use std::f32::consts::TAU;

use bevy::prelude::*;
use nalgebra::{Isometry3, SymmetricEigen, Translation3, UnitQuaternion, Vector3};
use helios_core::mapping::MapData;

use super::components::{
    DebugLegendNode, DebugSensorCache, DebugVisualizationConfig, PathTrail, TfLabelEntities,
};
use crate::prelude::*;
use crate::simulation::core::components::GroundTruthState;
use crate::simulation::core::events::BevyMeasurementMessage;
use crate::simulation::core::transforms::{
    enu_iso_to_bevy_transform, enu_vector_to_bevy_vector, flu_vector_to_bevy_local_vector, TfTree,
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
    if keyboard.just_pressed(KeyCode::F7) {
        config.show_occupancy_grid = !config.show_occupancy_grid;
        info!("[Debug] Occupancy Grid {}", on_off(config.show_occupancy_grid));
    }
    if keyboard.just_pressed(KeyCode::F8) {
        config.show_tf_frames = !config.show_tf_frames;
        info!("[Debug] TF Frames {}", on_off(config.show_tf_frames));
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
                "=== Debug (H to hide) ===\nF1  Pose Gimbals    [{}]\nF2  Covariance      [{}]\nF3  Point Cloud     [{}]\nF4  Velocity        [{}]\nF5  Est. Error      [{}]\nF6  Path Trail      [{}]\nF7  Occupancy Grid  [{}]\nF8  TF Frames       [{}]",
                on_off_bracket(config.show_pose_gimbals),
                on_off_bracket(config.show_covariance),
                on_off_bracket(config.show_point_cloud),
                on_off_bracket(config.show_velocity),
                on_off_bracket(config.show_error_line),
                on_off_bracket(config.show_path_trail),
                on_off_bracket(config.show_occupancy_grid),
                on_off_bracket(config.show_tf_frames),
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

// =========================================================================
// == TF Frame Axes (F8) ==
// =========================================================================

/// Draws coordinate-frame axes for every tracked TF frame plus the world ENU origin.
///
/// Color convention (matches ROS/RViz standard):
///   Red   = X axis  (ENU East  / FLU Forward)
///   Green = Y axis  (ENU North / FLU Left)
///   Blue  = Z axis  (ENU Up    / FLU Up)
///
/// The world origin axes are drawn at 2× length so they are visually distinct.
/// A grey line connects each child frame to its tracked parent.
///
/// Frame legend:
///   World ENU origin  — thick 2 m axes at (0, 0, 0)
///   Agent body        — 0.8 m axes at the vehicle body origin
///   Sensor child      — 0.5 m axes at the sensor mount point
pub fn draw_tf_frames(
    config: Res<DebugVisualizationConfig>,
    tf_tree: Res<TfTree>,
    mut gizmos: Gizmos,
) {
    if !config.show_tf_frames {
        return;
    }

    // World ENU origin — long axes so it is unmistakeable.
    // ENU East  (+X) lives on Bevy +X.
    // ENU North (+Y) lives on Bevy -Z.
    // ENU Up    (+Z) lives on Bevy +Y.
    let origin_len = 2.0_f32;
    gizmos.arrow(Vec3::ZERO, Vec3::X * origin_len,     Color::srgb(1.0, 0.0, 0.0));
    gizmos.arrow(Vec3::ZERO, Vec3::NEG_Z * origin_len, Color::srgb(0.0, 1.0, 0.0));
    gizmos.arrow(Vec3::ZERO, Vec3::Y * origin_len,     Color::srgb(0.0, 0.0, 1.0));

    // All tracked frames.
    for (_entity, world_iso, _local_iso, parent_entity) in tf_tree.iter_frames() {
        let bevy_tf = enu_iso_to_bevy_transform(&world_iso);
        let frame_origin = bevy_tf.translation;
        let rot = bevy_tf.rotation;

        // Axis length varies with depth: root frames get 0.8 m, children 0.5 m.
        let axis_len = if parent_entity.is_none() { 0.8_f32 } else { 0.5_f32 };

        gizmos.arrow(frame_origin, frame_origin + rot * Vec3::X * axis_len, Color::srgb(1.0, 0.2, 0.2));
        gizmos.arrow(frame_origin, frame_origin + rot * Vec3::Y * axis_len, Color::srgb(0.2, 1.0, 0.2));
        gizmos.arrow(frame_origin, frame_origin + rot * Vec3::Z * axis_len, Color::srgb(0.2, 0.2, 1.0));

        // Draw a grey line to the parent to make the tree structure visible.
        if let Some(parent_entity) = parent_entity {
            if let Some(parent_world_iso) = tf_tree.lookup_by_entity(parent_entity) {
                let parent_bevy = enu_iso_to_bevy_transform(&parent_world_iso);
                gizmos.line(frame_origin, parent_bevy.translation, Color::srgba(0.6, 0.6, 0.6, 0.5));
            }
        }
    }
}

/// Projects each tracked TF frame's world position to screen space and
/// positions a UI `Text` label there.
///
/// Why UI nodes instead of `Text2d`: `Text2d` renders in the 2D pipeline
/// (Transparent2d pass) and is invisible to a `Camera3d`. UI nodes are
/// rendered last, always on top, and can be placed at any pixel via
/// `Camera::world_to_viewport`.
///
/// Lifecycle:
/// - F8 on  + labels empty  → spawn one UI label per named TF frame
/// - F8 on  + labels exist  → update screen position each frame
/// - F8 off + labels exist  → despawn all, clear map
pub fn update_tf_labels(
    config: Res<DebugVisualizationConfig>,
    tf_tree: Res<TfTree>,
    mut labels: ResMut<TfLabelEntities>,
    mut commands: Commands,
    camera_query: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    mut node_query: Query<(&mut Node, &mut Visibility)>,
) {
    if !config.show_tf_frames {
        if !labels.0.is_empty() {
            for (_, entity) in labels.0.drain() {
                commands.entity(entity).despawn();
            }
        }
        return;
    }

    // Need a 3D camera to project world → screen.
    let Ok((camera, camera_tf)) = camera_query.single() else { return };

    // --- Spawn missing labels ---
    for (frame_entity, _, _, _) in tf_tree.iter_frames() {
        let Some(frame_name) = tf_tree.entity_to_name.get(&frame_entity) else { continue };
        let name_str = frame_name.to_string();
        if labels.0.contains_key(&name_str) {
            continue;
        }
        let label_entity = commands
            .spawn((
                Node {
                    position_type: PositionType::Absolute,
                    ..default()
                },
                Text::new(name_str.clone()),
                TextFont { font_size: 11.0, ..default() },
                TextColor(Color::srgb(1.0, 1.0, 0.2)),
                Visibility::Hidden,
            ))
            .id();
        labels.0.insert(name_str, label_entity);
    }

    // --- Update screen positions ---
    for (frame_entity, world_iso, _, _) in tf_tree.iter_frames() {
        let Some(frame_name) = tf_tree.entity_to_name.get(&frame_entity) else { continue };
        let Some(&label_entity) = labels.0.get(frame_name.as_ref()) else { continue };
        let Ok((mut node, mut vis)) = node_query.get_mut(label_entity) else { continue };

        let bevy_tf = enu_iso_to_bevy_transform(&world_iso);
        // Offset slightly above the frame origin so the label clears the axes.
        let world_pos = bevy_tf.translation + Vec3::Y * 0.4;

        match camera.world_to_viewport(camera_tf, world_pos) {
            Ok(screen_pos) => {
                node.left = Val::Px(screen_pos.x + 6.0);
                node.top = Val::Px(screen_pos.y - 8.0);
                *vis = Visibility::Visible;
            }
            Err(_) => {
                // Frame is behind the camera — hide the label.
                *vis = Visibility::Hidden;
            }
        }
    }
}

// =========================================================================
// == Occupancy Grid (F7) ==
// =========================================================================

/// Render radius around each agent. Only cells within this many metres of
/// the agent's ground-truth position are drawn — keeps the gizmo count
/// manageable for a 200 m × 200 m grid.
const GRID_DRAW_RADIUS_M: f32 = 150.0;

/// Draws a bird's-eye view of the occupancy grid for every agent that has
/// an active `OccupancyGrid2D` mapper.
///
/// Coordinate convention:
///   ENU  (x_east, y_north, 0)  →  Bevy (x_east, ground_y, -y_north)
///
/// The rect gizmo lives in the XY plane by default; rotating -90° around
/// the X axis flattens it onto the XZ (ground) plane in Bevy space.
pub fn draw_occupancy_grid(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    query: Query<(&WorldModelComponent, &GlobalTransform), With<GroundTruthState>>,
) {
    if !config.show_occupancy_grid {
        return;
    }

    // Rotation that turns a vertical XY rect into a horizontal ground-plane rect.
    let flat = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);

    for (module, gt_transform) in &query {
        let mapper = match module {
            WorldModelComponent::Separate { mapper, .. } => mapper,
            _ => continue,
        };

        let MapData::OccupancyGrid2D { origin, resolution, data } = mapper.get_map() else {
            continue;
        };

        let res = *resolution as f32;
        let rows = data.nrows();
        let cols = data.ncols();
        let ox = origin.translation.x as f32;
        let oy = origin.translation.y as f32;

        // Agent ground position in ENU (take GT bevy pos and invert ENU→Bevy).
        // Bevy: (bx, by, bz)  →  ENU: (bx, -bz)
        let agent_bevy = gt_transform.translation();
        let agent_ex = agent_bevy.x;
        let agent_ey = -agent_bevy.z;

        // Ground height from the agent's Bevy Y, kept constant for all cells.
        let ground_y = agent_bevy.y;

        // Draw the grid boundary as a thin white rectangle.
        let grid_w = cols as f32 * res;
        let grid_h = rows as f32 * res;
        let border_cx_enu = ox + grid_w * 0.5;
        let border_cy_enu = oy + grid_h * 0.5;
        let border_center = Vec3::new(border_cx_enu, ground_y, -border_cy_enu);
        gizmos.rect(
            Isometry3d::new(border_center, flat),
            Vec2::new(grid_w, grid_h),
            Color::srgba(1.0, 1.0, 1.0, 0.3),
        );

        // Draw individual cells within the draw radius.
        let r_cells_x = (GRID_DRAW_RADIUS_M / res).ceil() as i64;
        let r_cells_y = (GRID_DRAW_RADIUS_M / res).ceil() as i64;

        // Find the cell containing the agent.
        let agent_col = ((agent_ex - ox) / res).floor() as i64;
        let agent_row = ((agent_ey - oy) / res).floor() as i64;

        let col_min = (agent_col - r_cells_x).max(0) as usize;
        let col_max = (agent_col + r_cells_x).min(cols as i64 - 1) as usize;
        let row_min = (agent_row - r_cells_y).max(0) as usize;
        let row_max = (agent_row + r_cells_y).min(rows as i64 - 1) as usize;

        let r2 = GRID_DRAW_RADIUS_M * GRID_DRAW_RADIUS_M;

        for row in row_min..=row_max {
            for col in col_min..=col_max {
                let p = data[(row, col)] as f32 / 255.0;

                // Skip unknown cells (p ≈ 0.5, i.e., log-odds ≈ 0).
                let color = if p > 0.65 {
                    // Occupied — red, alpha scales with confidence.
                    let a = ((p - 0.65) / 0.35).clamp(0.1, 0.85);
                    Color::srgba(1.0, 0.15, 0.0, a)
                } else if p < 0.45 {
                    // Free — green, alpha scales with confidence.
                    let a = ((0.45 - p) / 0.45).clamp(0.05, 0.4);
                    Color::srgba(0.1, 0.9, 0.2, a)
                } else {
                    continue; // Unknown — skip.
                };

                let cx_enu = ox + (col as f32 + 0.5) * res;
                let cy_enu = oy + (row as f32 + 0.5) * res;

                // Distance check — keep cells within draw radius.
                let dx = cx_enu - agent_ex;
                let dy = cy_enu - agent_ey;
                if dx * dx + dy * dy > r2 {
                    continue;
                }

                let center = Vec3::new(cx_enu, ground_y + 0.02, -cy_enu);
                // Shrink slightly so adjacent cells have a visible gap.
                gizmos.rect(Isometry3d::new(center, flat), Vec2::splat(res * 0.92), color);
            }
        }
    }
}
