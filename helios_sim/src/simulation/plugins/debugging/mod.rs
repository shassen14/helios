use bevy::prelude::*;

mod components;
mod systems;

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
                (apply_debug_config, systems::spawn_debug_legend).chain(),
            )
            .add_systems(
                Update,
                (
                    systems::handle_debug_keybindings,
                    systems::cache_sensor_data,
                    systems::update_legend_text,
                    systems::draw_ground_truth_gimbals,
                    systems::draw_estimated_pose_gimbals,
                    systems::draw_covariance_ellipsoid,
                    systems::draw_point_cloud,
                    systems::draw_velocity_vector,
                    systems::draw_estimation_error_line,
                    systems::draw_path_trail,
                    systems::draw_occupancy_grid,
                    systems::draw_tf_frames,
                    systems::update_tf_labels,
                )
                    .run_if(in_state(AppState::Running)),
            )
            .add_systems(
                FixedUpdate,
                systems::update_path_trail
                    .in_set(SimulationSet::Validation)
                    .run_if(in_state(AppState::Running)),
            );
    }
}
