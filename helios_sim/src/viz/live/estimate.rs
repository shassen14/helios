//! Pose-estimate gizmo — an axis triad drawn at each agent's estimated pose.
//!
//! The bus-consumer counterpart to `pose.rs`. Where the ground-truth gizmo
//! reads the `GroundTruthState` component (truth has no brain between it and
//! reality), this reads the brain's *belief*: `base_link` in the agent's
//! estimated tf tree, looked up from the world's datum. The tree reaches the
//! datum only through the anchor the build placed the estimate's root by, so
//! a root nothing anchors has no place in the world and isn't drawn. Nor is
//! anything before the estimate's first output, or when the stack has no
//! estimator.
//!
//! Overlaid on the ground-truth triad, the gap between the two *is* the
//! estimator's error — the whole point of drawing both.

use bevy::prelude::*;

use helios_core::spatial::FrameId;

use crate::{
    core::transforms::TfTree,
    prelude::{AgentIdComponent, TfServiceComponent},
    viz::live::{pose::PoseOverlayTuning, tf::frame_triad_in_root, triad::draw_triad},
};

pub fn estimate_update_system(
    tuning: Res<PoseOverlayTuning>,
    tf_tree: Res<TfTree>,
    query: Query<(&TfServiceComponent, &AgentIdComponent)>,
    mut gizmos: Gizmos,
) {
    // A world without a datum builds no pipeline, so there is nothing to draw.
    let Some(datum) = tf_tree.datum() else {
        return;
    };
    for (tf_service, agent_id) in &query {
        // The tree holds base_link in its own axes, so the triad points where
        // the brain believes the body points.
        let body = FrameId::base_link(agent_id.0.clone());
        let Some((origin, axes)) = frame_triad_in_root(tf_service.0.buffer(), &body, datum) else {
            continue;
        };

        draw_triad(&mut gizmos, origin, axes, tuning.estimate_len);
    }
}
