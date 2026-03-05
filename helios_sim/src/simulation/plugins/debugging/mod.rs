use bevy::prelude::*;

mod components;
mod systems;

pub use components::{DebugLegendNode, DebugSensorCache, DebugVisualizationConfig, PathTrail};

use crate::prelude::AppState;
use crate::simulation::core::app_state::SimulationSet;

/// Top-level plugin that provides all debug visualization tooling.
pub struct DebuggingPlugin;

impl Plugin for DebuggingPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DebugVisualizationConfig>()
            .init_resource::<DebugSensorCache>()
            .add_systems(
                OnEnter(AppState::Running),
                systems::spawn_debug_legend,
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
