use bevy::prelude::*;

use crate::simulation::plugins::debugging::components::{DebugLegendNode, DebugVisualizationConfig};

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
                on_off(config.show_pose_gimbals),
                on_off(config.show_covariance),
                on_off(config.show_point_cloud),
                on_off(config.show_velocity),
                on_off(config.show_error_line),
                on_off(config.show_path_trail),
                on_off(config.show_occupancy_grid),
                on_off(config.show_tf_frames),
            );
        }
    }
}

fn on_off(b: bool) -> &'static str {
    if b { "ON " } else { "OFF" }
}
