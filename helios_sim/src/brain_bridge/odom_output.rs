//! Egress: writes each agent's pose estimate onto its odom frame entity.
//!
//! Estimation itself happens inside `run_pipeline_tick`; this only reads the
//! pipeline's current state and applies it to the odom `Transform`.

use bevy::prelude::*;

use crate::brain_bridge::components::{AutonomyPipelineComponent, OdomFrameOf};
use crate::core::transforms::EnuBodyPose;

/// Updates each odom frame's `Transform` from the pipeline's current pose estimate.
///
/// Runs in `BrainTick` (chained right after `run_pipeline_tick`), not in
/// `BrainOutput` where an egress system might be expected. The odom frame is a
/// `TrackedFrame`, so any later system doing a TF lookup this tick must see the
/// pose the pipeline just produced — leaving it a tick stale would desync the
/// TF tree from the estimate. Chaining it into the same set as the tick that
/// produces the estimate is what keeps them coherent.
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
