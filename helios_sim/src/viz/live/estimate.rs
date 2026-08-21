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

use helios_core::data::primitives::FrameHandle;
use helios_core::frames::conventions::{Enu, Flu};
use helios_core::frames::FrameId;

use crate::{core::transforms::EnuBodyPose, prelude::AutonomyPipelineComponent};

/// Axis-triad length for the estimate gizmo. Deliberately shorter than the
/// ground-truth triad ([`crate::viz::live::pose::GROUND_TRUTH_TRIAD_LEN`]) so
/// the two are distinguishable where they overlap.
// TODO: pull the triad length from viz config once that surface exists.
const ESTIMATE_TRIAD_LEN: f32 = 3.0;

pub fn estimate_update_system(
    query: Query<(Entity, &AutonomyPipelineComponent)>,
    mut gizmos: Gizmos,
) {
    for (agent, pipeline) in &query {
        let handle = FrameHandle::from_entity(agent);
        let body = FrameId::Body(handle);
        let Some(iso) = pipeline.0.read_state().and_then(|st| {
            st.value
                .pose::<Flu, Enu>(body.clone(), FrameId::Odom(handle))
                .map(|t| t.into_inner())
        }) else {
            continue;
        };

        let transform = Transform::from(EnuBodyPose(iso));

        gizmos.axes(transform, ESTIMATE_TRIAD_LEN);
    }
}
