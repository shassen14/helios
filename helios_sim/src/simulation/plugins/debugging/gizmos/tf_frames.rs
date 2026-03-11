use bevy::prelude::*;

use crate::simulation::core::transforms::{enu_iso_to_bevy_transform, TfTree};
use crate::simulation::plugins::debugging::components::{DebugVisualizationConfig, TfLabelEntities};

/// Draws coordinate-frame axes for every tracked TF frame plus the world ENU origin.
///
/// Color convention (matches ROS/RViz standard):
///   Red   = X axis  (ENU East  / FLU Forward)
///   Green = Y axis  (ENU North / FLU Left)
///   Blue  = Z axis  (ENU Up    / FLU Up)
pub fn draw_tf_frames(
    config: Res<DebugVisualizationConfig>,
    tf_tree: Res<TfTree>,
    mut gizmos: Gizmos,
) {
    if !config.show_tf_frames {
        return;
    }

    let origin_len = 2.0_f32;
    gizmos.arrow(Vec3::ZERO, Vec3::X * origin_len,     Color::srgb(1.0, 0.0, 0.0));
    gizmos.arrow(Vec3::ZERO, Vec3::NEG_Z * origin_len, Color::srgb(0.0, 1.0, 0.0));
    gizmos.arrow(Vec3::ZERO, Vec3::Y * origin_len,     Color::srgb(0.0, 0.0, 1.0));

    for (_entity, world_iso, _local_iso, parent_entity) in tf_tree.iter_frames() {
        let bevy_tf = enu_iso_to_bevy_transform(&world_iso);
        let frame_origin = bevy_tf.translation;
        let rot = bevy_tf.rotation;

        let axis_len = if parent_entity.is_none() { 0.8_f32 } else { 0.5_f32 };

        gizmos.arrow(frame_origin, frame_origin + rot * Vec3::X * axis_len, Color::srgb(1.0, 0.2, 0.2));
        gizmos.arrow(frame_origin, frame_origin + rot * Vec3::Y * axis_len, Color::srgb(0.2, 1.0, 0.2));
        gizmos.arrow(frame_origin, frame_origin + rot * Vec3::Z * axis_len, Color::srgb(0.2, 0.2, 1.0));

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

    let Ok((camera, camera_tf)) = camera_query.single() else { return };

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

    for (frame_entity, world_iso, _, _) in tf_tree.iter_frames() {
        let Some(frame_name) = tf_tree.entity_to_name.get(&frame_entity) else { continue };
        let Some(&label_entity) = labels.0.get(frame_name.as_ref()) else { continue };
        let Ok((mut node, mut vis)) = node_query.get_mut(label_entity) else { continue };

        let bevy_tf = enu_iso_to_bevy_transform(&world_iso);
        let world_pos = bevy_tf.translation + Vec3::Y * 0.4;

        match camera.world_to_viewport(camera_tf, world_pos) {
            Ok(screen_pos) => {
                node.left = Val::Px(screen_pos.x + 6.0);
                node.top = Val::Px(screen_pos.y - 8.0);
                *vis = Visibility::Visible;
            }
            Err(_) => {
                *vis = Visibility::Hidden;
            }
        }
    }
}
