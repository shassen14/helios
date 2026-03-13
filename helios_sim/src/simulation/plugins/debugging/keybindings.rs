use bevy::prelude::*;

use super::components::DebugVisualizationConfig;
use super::key_action_registry::{DebugToggle, KeyActionRegistry};

/// Iterates the `KeyActionRegistry` each frame and toggles the corresponding
/// `DebugVisualizationConfig` field when a registered key is just-pressed.
/// Only loaded plugins ever register actions, so this handles any profile automatically.
pub fn handle_debug_keybindings(
    keyboard: Res<ButtonInput<KeyCode>>,
    registry: Res<KeyActionRegistry>,
    mut config: ResMut<DebugVisualizationConfig>,
) {
    for action in &registry.0 {
        if keyboard.just_pressed(action.bound_key) {
            match action.toggle {
                DebugToggle::Pose => {
                    config.show_pose_gimbals = !config.show_pose_gimbals;
                    info!("[Debug] Pose Gimbals {}", on_off(config.show_pose_gimbals));
                }
                DebugToggle::Covariance => {
                    config.show_covariance = !config.show_covariance;
                    info!("[Debug] Covariance Ellipsoid {}", on_off(config.show_covariance));
                }
                DebugToggle::PointCloud => {
                    config.show_point_cloud = !config.show_point_cloud;
                    info!("[Debug] Point Cloud {}", on_off(config.show_point_cloud));
                }
                DebugToggle::Velocity => {
                    config.show_velocity = !config.show_velocity;
                    info!("[Debug] Velocity Vector {}", on_off(config.show_velocity));
                }
                DebugToggle::ErrorLine => {
                    config.show_error_line = !config.show_error_line;
                    info!("[Debug] Estimation Error Line {}", on_off(config.show_error_line));
                }
                DebugToggle::PathTrail => {
                    config.show_path_trail = !config.show_path_trail;
                    info!("[Debug] Path Trail {}", on_off(config.show_path_trail));
                }
                DebugToggle::OccupancyGrid => {
                    config.show_occupancy_grid = !config.show_occupancy_grid;
                    info!("[Debug] Occupancy Grid {}", on_off(config.show_occupancy_grid));
                }
                DebugToggle::TfFrames => {
                    config.show_tf_frames = !config.show_tf_frames;
                    info!("[Debug] TF Frames {}", on_off(config.show_tf_frames));
                }
                DebugToggle::PlannedPath => {
                    config.show_planned_path = !config.show_planned_path;
                    info!("[Debug] Planned Path {}", on_off(config.show_planned_path));
                }
                DebugToggle::Legend => {
                    config.show_legend = !config.show_legend;
                }
            }
        }
    }
}

pub fn on_off(b: bool) -> &'static str {
    if b { "ON" } else { "OFF" }
}
