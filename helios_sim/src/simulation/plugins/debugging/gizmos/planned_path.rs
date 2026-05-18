// helios_sim/src/simulation/plugins/debugging/gizmos/planned_path.rs
//
// Draws each agent's currently planned path as a yellow polyline. Reads
// `Path` directly from the pipeline bus via `read_any_path()`.

use bevy::prelude::*;

use helios_core::frames::{FrameId, StateVariable};

use super::super::DebugVisualizationConfig;
use crate::simulation::core::transforms::EnuVector;
use crate::simulation::plugins::autonomy::AutonomyPipelineComponent;

const COLOR_PATH: Color = Color::srgb(1.0, 0.9, 0.0);
const HOVER_HEIGHT: f32 = 0.15;

pub fn draw_planned_path(
    config: Res<DebugVisualizationConfig>,
    pipelines: Query<&AutonomyPipelineComponent>,
    mut gizmos: Gizmos,
) {
    if !config.show_planned_path {
        return;
    }

    for pipeline in &pipelines {
        let Some(stamped) = pipeline.0.read_any_path() else {
            continue;
        };
        let path = &stamped.value;
        if path.waypoints.len() < 2 {
            continue;
        }

        let mut prev: Option<Vec3> = None;
        for wp in &path.waypoints {
            let Some(p_enu) = wp.state.get_vector3(&StateVariable::Px(FrameId::World)) else {
                continue;
            };
            let mut bevy_p = Vec3::from(EnuVector(p_enu));
            bevy_p.y = HOVER_HEIGHT;
            if let Some(prev_p) = prev {
                gizmos.line(prev_p, bevy_p, COLOR_PATH);
            }
            prev = Some(bevy_p);
        }
    }
}
