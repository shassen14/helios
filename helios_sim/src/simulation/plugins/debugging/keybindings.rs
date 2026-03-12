use bevy::prelude::*;

use super::components::DebugVisualizationConfig;

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

pub fn on_off(b: bool) -> &'static str {
    if b { "ON" } else { "OFF" }
}
