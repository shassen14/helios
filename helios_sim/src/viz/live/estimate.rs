//! Pose-estimate gizmo — an axis triad drawn at each agent's estimated pose.
//!
//! The bus-consumer counterpart to `pose.rs`. Where the ground-truth gizmo
//! reads the `GroundTruthState` component (truth has no brain between it and
//! reality), this reads the pipeline's canonical state output via
//! `read_state()` — the estimate is the brain's *belief*, which lives on the
//! bus, not a physics component. `None` before the estimator's first output,
//! or when the stack has no estimator; either way the triad is simply not drawn.
//!
//! Overlaid on the ground-truth triad, the gap between the two *is* the
//! estimator's error — the whole point of drawing both.

use bevy::prelude::*;

use helios_core::spatial::conventions::{Enu, Flu};
use helios_core::spatial::transforms::Convention;
use helios_core::spatial::FrameId;

use crate::{
    core::transforms::{frame_triad_to_bevy, freevector_bevy_to_vec3, point_bevy_to_vec3},
    prelude::{AgentIdComponent, AutonomyPipelineComponent},
    viz::live::{pose::PoseOverlayTuning, triad::draw_triad},
};

pub fn estimate_update_system(
    tuning: Res<PoseOverlayTuning>,
    query: Query<(&AutonomyPipelineComponent, &AgentIdComponent)>,
    mut gizmos: Gizmos,
) {
    for (pipeline, agent_id) in &query {
        let agent = agent_id.0.clone();
        let body = FrameId::base_link(agent.clone());
        let Some(pose) = pipeline.0.read_state().and_then(|st| {
            st.value
                .pose::<Flu, Enu>(body.clone(), FrameId::odom(agent.clone()))
        }) else {
            continue;
        };

        // The estimate is base_link (FLU) relative to odom (ENU); its FLU
        // convention rides in the pose's rotation, so the triad crosses one-sided
        // and points where the brain believes the body points.
        let Some((origin, axes)) = frame_triad_to_bevy(pose.into_inner(), Convention::Enu) else {
            continue;
        };

        draw_triad(
            &mut gizmos,
            point_bevy_to_vec3(origin),
            axes.map(freevector_bevy_to_vec3),
            tuning.estimate_len,
        );
    }
}
