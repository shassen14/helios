//! Ground-truth pose gizmo — an axis triad drawn at each agent's true pose.
//!
//! Reads the `GroundTruthState` component off the agent entity (the physics
//! truth Avian writes each `StateSync`), not the bus — there is no estimator or
//! filter between this and reality. The estimate gizmo (a bus consumer) is the
//! counterpart; overlaid, the gap between the two triads *is* the estimator's
//! error.
//!
//! Doubles as a live check on `core/transforms/`: the pose is converted through
//! the sole sanctioned ENU→Bevy helper, so the triad lands exactly on the mesh
//! iff that conversion is correct. Any offset is a `transforms/` bug, not a
//! rendering one.

use bevy::prelude::*;

use crate::{core::transforms::EnuBodyPose, prelude::GroundTruthState};

/// Axis-triad length for the ground-truth gizmo. Longer than the estimate
/// triad ([`crate::viz::live::estimate::ESTIMATE_TRIAD_LEN`]) so the two are
/// distinguishable where they overlap.
// TODO: pull the triad length from viz config once that surface exists.
const GROUND_TRUTH_TRIAD_LEN: f32 = 5.0;

pub fn pose_update_system(query: Query<&GroundTruthState>, mut gizmos: Gizmos) {
    for ground_truth in query {
        // ENU ground-truth pose → Bevy transform via the one sanctioned helper.
        let transform = Transform::from(EnuBodyPose(ground_truth.pose));

        gizmos.axes(transform, GROUND_TRUTH_TRIAD_LEN);
    }
}
