use bevy::prelude::*;

use crate::simulation::plugins::debugging::components::{DebugLegendNode, DebugVisualizationConfig};
use crate::simulation::plugins::debugging::key_action_registry::{DebugToggle, KeyActionRegistry};

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
/// Renders only the actions registered in `KeyActionRegistry`, so the legend
/// automatically matches the active profile.
pub fn update_legend_text(
    config: Res<DebugVisualizationConfig>,
    registry: Res<KeyActionRegistry>,
    mut query: Query<(&mut Text, &mut Visibility), With<DebugLegendNode>>,
) {
    if let Ok((mut text, mut vis)) = query.single_mut() {
        *vis = if config.show_legend {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };

        if config.show_legend {
            let mut lines = vec!["=== Debug (H to hide) ===".to_string()];
            for action in &registry.0 {
                if action.toggle == DebugToggle::Legend {
                    continue; // Skip the "toggle legend" entry itself.
                }
                let state = toggle_state(&config, action.toggle);
                lines.push(format!("{}  [{}]", action.label, state));
            }
            text.0 = lines.join("\n");
        }
    }
}

fn toggle_state(config: &DebugVisualizationConfig, toggle: DebugToggle) -> &'static str {
    let active = match toggle {
        DebugToggle::Pose         => config.show_pose_gimbals,
        DebugToggle::Covariance   => config.show_covariance,
        DebugToggle::PointCloud   => config.show_point_cloud,
        DebugToggle::Velocity     => config.show_velocity,
        DebugToggle::ErrorLine    => config.show_error_line,
        DebugToggle::PathTrail    => config.show_path_trail,
        DebugToggle::OccupancyGrid => config.show_occupancy_grid,
        DebugToggle::TfFrames     => config.show_tf_frames,
        DebugToggle::PlannedPath  => config.show_planned_path,
        DebugToggle::Legend       => config.show_legend,
    };
    if active { "ON " } else { "OFF" }
}
