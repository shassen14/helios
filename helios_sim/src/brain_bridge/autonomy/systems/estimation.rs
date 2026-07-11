// helios_sim/src/simulation/plugins/autonomy/systems/estimation.rs
//
// Odom frame update system. Sensor routing and estimation now happen inside
// `run_pipeline_tick` via `AutonomyPipelineComponent`.

use bevy::prelude::*;

use crate::brain_bridge::autonomy::components::{AutonomyPipelineComponent, OdomFrameOf};
use crate::core::transforms::EnuBodyPose;

/// Updates each odom frame's `Transform` from the pipeline's current pose estimate.
pub fn update_odom_frames(
    agent_query: Query<&AutonomyPipelineComponent>,
    mut odom_query: Query<(&OdomFrameOf, &mut Transform)>,
) {
    for (odom_of, mut transform) in &mut odom_query {
        let Ok(pipeline) = agent_query.get(odom_of.0) else {
            continue;
        };

        if let Some(iso) = pipeline
            .0
            .read_state()
            .and_then(|st| st.value.get_pose_isometry())
        {
            *transform = Transform::from(EnuBodyPose(iso));
        }
    }
}
