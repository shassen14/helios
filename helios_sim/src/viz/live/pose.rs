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
use serde::Deserialize;

use helios_core::spatial::transforms::Convention;

use crate::{
    core::transforms::{frame_triad_to_bevy, freevector_bevy_to_vec3, point_bevy_to_vec3},
    prelude::GroundTruthState,
    viz::{
        interaction::tuning::{require_positive, InteractionTuningError},
        live::triad::draw_triad,
    },
};

/// Sparse TOML overrides for the live pose overlay's triad lengths. Both fields
/// are optional; anything omitted falls back to [`PoseOverlayTuning::default`].
#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct PoseOverlayTuningFile {
    pub ground_truth_len: Option<f32>,
    pub estimate_len: Option<f32>,
}

/// Resolved axis-triad lengths for the live pose overlay: the ground-truth gizmo
/// ([`pose_update_system`]) and its estimate counterpart
/// ([`crate::viz::live::estimate::estimate_update_system`]). One resource for the
/// pair because the two lengths are chosen together — the truth triad is drawn
/// longer so it reads as the reference where the two overlap. Read by both
/// systems; defaults reproduce the values compiled in before the tuning surface
/// existed.
#[derive(Resource, Debug, Clone)]
pub struct PoseOverlayTuning {
    /// Axis-triad length for the ground-truth gizmo.
    pub ground_truth_len: f32,
    /// Axis-triad length for the estimate gizmo — shorter than `ground_truth_len`
    /// so the two are distinguishable where they overlap.
    pub estimate_len: f32,
}

impl Default for PoseOverlayTuning {
    fn default() -> Self {
        Self {
            ground_truth_len: 1.0,
            estimate_len: 0.7,
        }
    }
}

impl PoseOverlayTuning {
    /// Overlay sparse overrides onto [`Default`], rejecting a non-positive length.
    pub(crate) fn resolve(
        overrides: &PoseOverlayTuningFile,
    ) -> Result<Self, InteractionTuningError> {
        let mut t = Self::default();
        if let Some(v) = overrides.ground_truth_len {
            t.ground_truth_len = v;
        }
        if let Some(v) = overrides.estimate_len {
            t.estimate_len = v;
        }

        require_positive("pose_overlay.ground_truth_len", t.ground_truth_len)?;
        require_positive("pose_overlay.estimate_len", t.estimate_len)?;
        Ok(t)
    }
}

pub fn pose_update_system(
    tuning: Res<PoseOverlayTuning>,
    query: Query<&GroundTruthState>,
    mut gizmos: Gizmos,
) {
    for ground_truth in query {
        // ENU ground-truth pose → its native-axis triad via the one sanctioned
        // helper. The body's FLU convention is carried by the pose's rotation, so
        // the arrows point where the vehicle actually points.
        let Some((origin, axes)) = frame_triad_to_bevy(ground_truth.pose, Convention::Enu) else {
            continue;
        };

        draw_triad(
            &mut gizmos,
            point_bevy_to_vec3(origin),
            axes.map(freevector_bevy_to_vec3),
            tuning.ground_truth_len,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_overlay_tuning_applies_overrides_and_keeps_other_defaults() {
        let file = PoseOverlayTuningFile {
            estimate_len: Some(2.0),
            ..Default::default()
        };

        let t = PoseOverlayTuning::resolve(&file).expect("valid override resolves");

        assert_eq!(t.estimate_len, 2.0);
        assert_eq!(
            t.ground_truth_len,
            PoseOverlayTuning::default().ground_truth_len
        );
    }

    #[test]
    fn pose_overlay_tuning_rejects_a_nonpositive_length() {
        let file = PoseOverlayTuningFile {
            ground_truth_len: Some(0.0),
            ..Default::default()
        };

        assert!(PoseOverlayTuning::resolve(&file).is_err());
    }
}
