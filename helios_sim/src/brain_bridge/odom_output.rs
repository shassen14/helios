//! Egress: writes each agent's pose estimate onto its odom frame entity.
//!
//! Estimation itself happens inside `run_pipeline_tick`; this only reads the
//! pipeline's current state and applies it to the odom `Transform`.

use bevy::prelude::*;

use helios_core::frames::conventions::{Enu, Flu};
use helios_core::frames::FrameId;

use crate::brain_bridge::components::{AgentIdComponent, AutonomyPipelineComponent, OdomFrameOf};
use crate::core::transforms::{transform_bevy_to_bevy_transform, ToBevy};

/// Updates each odom frame's `Transform` from the pipeline's current pose estimate.
///
/// Runs in `BrainTick` (chained right after `run_pipeline_tick`), not in
/// `BrainOutput` where an egress system might be expected. The odom frame is a
/// `TrackedFrame`, so any later system doing a TF lookup this tick must see the
/// pose the pipeline just produced — leaving it a tick stale would desync the
/// TF tree from the estimate. Chaining it into the same set as the tick that
/// produces the estimate is what keeps them coherent.
pub fn update_odom_frames(
    agent_query: Query<(&AutonomyPipelineComponent, &AgentIdComponent)>,
    mut odom_query: Query<(&OdomFrameOf, &mut Transform)>,
) {
    for (odom_of, mut transform) in &mut odom_query {
        let Ok((pipeline, agent_id)) = agent_query.get(odom_of.0) else {
            continue;
        };

        let agent = agent_id.0.clone();
        let body = FrameId::base_link(agent.clone());
        if let Some(pose) = pipeline
            .0
            .read_state()
            .and_then(|st| st.value.pose::<Flu, Enu>(body.clone(), FrameId::odom(agent.clone())))
        {
            *transform = transform_bevy_to_bevy_transform(pose.to_bevy());
        }
    }
}
