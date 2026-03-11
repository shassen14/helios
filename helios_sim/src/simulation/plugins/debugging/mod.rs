use bevy::prelude::*;

mod cache;
mod components;
mod keybindings;
pub mod gizmos;
pub mod ui;

pub use components::{
    DebugLegendNode, DebugSensorCache, DebugVisualizationConfig, PathTrail, TfLabelEntities,
};

use crate::prelude::AppState;
use crate::simulation::config::ScenarioConfig;
use crate::simulation::core::app_state::SimulationSet;

fn apply_debug_config(
    scenario: Res<ScenarioConfig>,
    mut viz: ResMut<DebugVisualizationConfig>,
) {
    let d = &scenario.debug;
    viz.show_pose_gimbals   = d.show_pose_gimbals;
    viz.show_covariance     = d.show_covariance;
    viz.show_point_cloud    = d.show_point_cloud;
    viz.show_velocity       = d.show_velocity;
    viz.show_error_line     = d.show_error_line;
    viz.show_path_trail     = d.show_path_trail;
    viz.show_occupancy_grid = d.show_occupancy_grid;
    viz.show_tf_frames      = d.show_tf_frames;
    viz.show_legend         = d.show_legend;
}

/// Top-level plugin that provides all debug visualization tooling.
pub struct DebuggingPlugin;

impl Plugin for DebuggingPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DebugVisualizationConfig>()
            .init_resource::<DebugSensorCache>()
            .init_resource::<TfLabelEntities>()
            .add_systems(
                OnEnter(AppState::Running),
                (apply_debug_config, ui::legend::spawn_debug_legend).chain(),
            )
            .add_systems(
                Update,
                (
                    keybindings::handle_debug_keybindings,
                    cache::cache_sensor_data,
                    ui::legend::update_legend_text,
                    gizmos::pose::draw_ground_truth_gimbals,
                    gizmos::pose::draw_estimated_pose_gimbals,
                    gizmos::covariance::draw_covariance_ellipsoid,
                    gizmos::point_cloud::draw_point_cloud,
                    gizmos::velocity::draw_velocity_vector,
                    gizmos::error::draw_estimation_error_line,
                    gizmos::trail::draw_path_trail,
                    gizmos::occupancy::draw_occupancy_grid,
                    gizmos::tf_frames::draw_tf_frames,
                    gizmos::tf_frames::update_tf_labels,
                )
                    .run_if(in_state(AppState::Running)),
            )
            .add_systems(
                FixedUpdate,
                gizmos::trail::update_path_trail
                    .in_set(SimulationSet::Validation)
                    .run_if(in_state(AppState::Running)),
            );
    }
}
