//! TF-tree overlay — an axis triad at every frame of an agent's *estimated*
//! tf tree, not just its body.
//!
//! Where `estimate.rs` draws the single `base_link` triad the pipeline believes,
//! this walks the whole per-agent `TfServiceComponent` buffer and draws each
//! frame — `base_link` plus every sensor mount — so the entire estimated body
//! shows at once. The two coincide at `base_link`; that double-draw is
//! acceptable (same pose).
//!
//! On top of the estimated frames it draws a truth `base_link` triad and an
//! error connector from the estimated `base_link` to the truth one. `base_link`
//! is the only dynamic frame — everything below it is a static mount — so that
//! single offset *is* the whole estimator error, which the mounts merely repeat
//! rigidly; one triad and one line carry it without doubling every frame. Truth
//! for `base_link` comes from `GroundTruthState` (the same pose `pose.rs`
//! draws). A full per-frame truth tree is deferred until frames can diverge
//! independently (a map→odom correction, or online-estimated extrinsics).
//!
//! Topology comes from the buffer's `edges()`; each frame is placed by looking
//! it up relative to `odom`, the same tree root and ENU→Bevy path `estimate.rs`
//! uses, so a triad lands exactly where the estimator believes the frame is.
//! Each edge also draws a parent→child segment, so the frames read as a tree
//! rather than a loose scatter of triads.
//!
//! Anchoring at `odom`, each frame draws in its own native robotics axes: the
//! child's convention is baked into the looked-up isometry, so the triad's arrows
//! point where the frame actually points (RViz-style, no `<Flu, Enu>` hardcode),
//! crossing one-sided to Bevy through `frame_triad_to_bevy`. The only assumption
//! left is that the anchor resolves to an ENU root — which `odom` (and `map`,
//! once SLAM lands) does. A frame whose lookup fails, or a non-ENU anchor, is
//! skipped this pass.

use crate::{
    core::transforms::{frame_triad_to_bevy, freevector_bevy_to_vec3, point_bevy_to_vec3},
    prelude::{AgentIdComponent, GroundTruthState, TfServiceComponent},
    viz::interaction::{
        actions::{
            handle::{ActionHandle, ActionId},
            registry::ActionRegistry,
        },
        sampling::ActionState,
        selection::Selected,
        tuning::{require_positive, InteractionTuningError},
    },
    viz::live::triad::draw_triad,
};

use bevy::prelude::*;
use helios_core::spatial::{
    transforms::{
        tf::buffer::{TfBuffer, TfQuery},
        Convention,
    },
    FrameId,
};
use serde::Deserialize;

/// Sparse TOML overrides for the tf overlay's styling. Every field is optional;
/// anything omitted falls back to the compiled-in [`TfOverlayTuning::default`].
#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct TfOverlayTuningFile {
    pub triad_len: Option<f32>,
    pub truth_triad_len: Option<f32>,
    pub connector_color: Option<[f32; 3]>,
    pub error_color: Option<[f32; 3]>,
    pub divergence_threshold: Option<f32>,
    pub divergence_color: Option<[f32; 3]>,
}

/// Resolved styling for the overlay's triads and connectors. A `Resource` — one
/// operator preference for the session — read by [`tf_overlay_system`]: the
/// master toggle and selection scope decide *whether* a frame draws, this decides
/// how. Defaults reproduce the values compiled in before the tuning surface
/// existed.
#[derive(Resource, Debug, Clone)]
pub struct TfOverlayTuning {
    /// Axis-triad length for the estimated frames.
    pub triad_len: f32,
    /// Axis-triad length for the truth `base_link` — longer than `triad_len` so
    /// the truth reference reads as the longer triad where the two overlap.
    pub truth_triad_len: f32,
    /// Neutral color of the parent→child edge connectors.
    pub connector_color: Color,
    /// Resting color of the estimated→truth error connector, whose length is the
    /// error magnitude.
    pub error_color: Color,
    /// Error-connector length (meters) at or beyond which it escalates to
    /// `divergence_color` — drift worth flagging, not merely showing.
    pub divergence_threshold: f32,
    /// Escalated error-connector color once the gap passes `divergence_threshold`,
    /// so a large divergence reads at a glance.
    pub divergence_color: Color,
}

impl Default for TfOverlayTuning {
    fn default() -> Self {
        Self {
            triad_len: 0.6,
            truth_triad_len: 1.0,
            connector_color: Color::srgb(0.6, 0.6, 0.6),
            error_color: Color::srgb(0.9, 0.3, 0.2),
            divergence_threshold: 1.0,
            divergence_color: Color::srgb(1.0, 0.1, 0.1),
        }
    }
}

impl TfOverlayTuning {
    /// Overlay sparse overrides onto [`Default`], packing each `[r, g, b]` triple
    /// into an sRGB [`Color`], and reject a non-positive triad length or
    /// divergence threshold.
    pub(crate) fn resolve(overrides: &TfOverlayTuningFile) -> Result<Self, InteractionTuningError> {
        let mut t = Self::default();
        if let Some(v) = overrides.triad_len {
            t.triad_len = v;
        }
        if let Some(v) = overrides.truth_triad_len {
            t.truth_triad_len = v;
        }
        if let Some([r, g, b]) = overrides.connector_color {
            t.connector_color = Color::srgb(r, g, b);
        }
        if let Some([r, g, b]) = overrides.error_color {
            t.error_color = Color::srgb(r, g, b);
        }
        if let Some(v) = overrides.divergence_threshold {
            t.divergence_threshold = v;
        }
        if let Some([r, g, b]) = overrides.divergence_color {
            t.divergence_color = Color::srgb(r, g, b);
        }

        require_positive("tf_overlay.triad_len", t.triad_len)?;
        require_positive("tf_overlay.truth_triad_len", t.truth_triad_len)?;
        require_positive("tf_overlay.divergence_threshold", t.divergence_threshold)?;
        Ok(t)
    }
}

pub fn tf_overlay_system(
    tuning: Res<TfOverlayTuning>,
    query: Query<(&TfServiceComponent, &AgentIdComponent, &GroundTruthState), With<Selected>>,
    mut gizmos: Gizmos,
) {
    for (tf_service, agent_id, ground_truth) in query {
        let agent = agent_id.0.clone();
        let root = FrameId::odom(agent.clone());
        // Each non-root frame is the child of exactly one edge (single-parent
        // tree), so iterating children visits every drawable frame once — no
        // dedup. The root (`odom`) is identity, nothing to draw. `_kind` is the
        // static/dynamic tag, unused until a full per-frame truth walk branches
        // on it (today only `base_link` carries a truth pose, below).
        for (edge, _kind) in tf_service.0.buffer().edges() {
            let Some((child_origin, child_axes)) =
                frame_triad_in_root(tf_service.0.buffer(), &edge.child, &root)
            else {
                continue;
            };
            draw_triad(&mut gizmos, child_origin, child_axes, tuning.triad_len);

            // Connector, drawn after the triad so a parent that fails to
            // resolve drops only the line — never the child's triad.
            if let Some(parent_origin) =
                frame_origin_in_root(tf_service.0.buffer(), &edge.parent, &root)
            {
                gizmos.line(parent_origin, child_origin, tuning.connector_color);
            }
        }

        // Truth reference at `base_link` + the error connector. `base_link` is
        // the only dynamic frame, so this single truth triad and segment carry
        // the whole estimator error; the static mounts below repeat that rigid
        // offset and are not re-drawn from truth. Truth pose is `GroundTruthState`
        // (ENU world), the same source `pose.rs` uses, through the one sanctioned
        // ENU→Bevy helper. While the tree is `map`-less, `odom` coincides with the
        // world origin, so the estimated (odom-anchored) and truth (world) triads
        // share a frame and the segment length reads as drift.
        // Ground truth is a `base_link` (FLU) pose in the ENU world, so its triad
        // crosses through the one sanctioned ENU root — the same `frame_triad_to_bevy`
        // the estimated frames take, just from `GroundTruthState` instead of a tf
        // lookup. The ENU literal can never fail the anchor guard, but a mismatch
        // would drop the truth reference rather than draw it wrong.
        if let Some((truth_origin, truth_axes)) =
            frame_triad_to_bevy(ground_truth.pose, Convention::Enu)
        {
            let truth_origin = point_bevy_to_vec3(truth_origin);
            draw_triad(
                &mut gizmos,
                truth_origin,
                truth_axes.map(freevector_bevy_to_vec3),
                tuning.truth_triad_len,
            );

            // Triad drawn first, so an unresolvable estimate drops only the segment.
            // The segment escalates to the divergence color once the gap it spans
            // passes the threshold, so a large drift reads without measuring the line.
            let base_link = FrameId::base_link(agent.clone());
            if let Some(estimate_origin) =
                frame_origin_in_root(tf_service.0.buffer(), &base_link, &root)
            {
                let gap = estimate_origin.distance(truth_origin);
                let color = if diverged(gap, tuning.divergence_threshold) {
                    tuning.divergence_color
                } else {
                    tuning.error_color
                };
                gizmos.line(estimate_origin, truth_origin, color);
            }
        }
    }
}

/// Whether a truth↔estimate gap (meters) is large enough to escalate the error
/// connector to the divergence highlight. Extracted from the draw loop so the
/// threshold decision is unit-testable without an `App` or gizmos — the gizmo
/// output itself can't be asserted.
fn diverged(gap: f32, threshold: f32) -> bool {
    gap >= threshold
}

/// A frame's drawable axis triad expressed in `root`: its Bevy-space origin and
/// three unit axis directions (+X, +Y, +Z), each pointing where the frame's real
/// axis points. The shared `lookup` → `frame_triad_to_bevy` path, returning
/// `None` when the tree can't resolve the frame this tick or the anchor is not an
/// ENU root. The child's convention is carried by the looked-up isometry, so its
/// axes are correct with no per-frame branch.
pub(crate) fn frame_triad_in_root(
    buffer: &TfBuffer,
    frame: &FrameId,
    root: &FrameId,
) -> Option<(Vec3, [Vec3; 3])> {
    let erased = buffer.lookup(frame, root, TfQuery::Latest).ok()?;
    let (origin, axes) = frame_triad_to_bevy(erased.isometry(), erased.to_convention())?;
    Some((point_bevy_to_vec3(origin), axes.map(freevector_bevy_to_vec3)))
}

/// A frame's origin in `root`, as a Bevy world position — the triad's origin
/// alone. Shared with the label overlay (which places its text here) and the
/// parent→child connectors, neither of which needs the axes. `None` on the same
/// conditions as [`frame_triad_in_root`]; the origin is identical either way,
/// since only the axes read the rotation.
pub(crate) fn frame_origin_in_root(
    buffer: &TfBuffer,
    frame: &FrameId,
    root: &FrameId,
) -> Option<Vec3> {
    frame_triad_in_root(buffer, frame, root).map(|(origin, _)| origin)
}

/// Master on/off for the whole tf overlay — triads, connectors, and labels.
///
/// A single global switch, unlike the per-agent `MapVisible`: the overlay has
/// one surface master, and scope-to-selection (a `With<Selected>` filter on the
/// draw systems), not this flag, narrows *which* agent draws. Off by default
/// (`bool::default()` is `false`) — the overlay is an opt-in diagnostic, not
/// always-on furniture.
#[derive(Resource, Default)]
pub(crate) struct TfOverlayVisible(pub bool);

/// Flip the overlay master when the `viz.toggle_tf` action fires. Mirrors
/// `toggle_map_visibility`, but the master is one global resource rather than a
/// per-agent component, so it toggles a single bool. The handle can't change
/// after startup, so it is resolved once and cached in a `Local`.
pub(crate) fn toggle_tf_overlay(
    registry: Res<ActionRegistry>,
    state: Res<ActionState>,
    mut overlay: ResMut<TfOverlayVisible>,
    mut handle: Local<Option<ActionHandle>>,
) {
    let h = *handle.get_or_insert_with(|| {
        registry
            .handle(ActionId("viz.toggle_tf"))
            .expect("registered")
    });

    if state.is_active(h) {
        overlay.0 = !overlay.0;
    }
}

/// Run condition gating the immediate-mode overlay system on the master toggle.
/// The label system deliberately can't share it: being retained, it must keep
/// running while the overlay is off so it can reconcile its entities away —
/// only the stateless gizmo system is safe to skip entirely.
pub(crate) fn tf_overlay_visible(overlay: Res<TfOverlayVisible>) -> bool {
    overlay.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_small_gap_does_not_diverge() {
        assert!(!diverged(0.25, 1.0));
    }

    #[test]
    fn a_gap_at_or_past_the_threshold_diverges() {
        assert!(diverged(1.0, 1.0));
        assert!(diverged(3.0, 1.0));
    }

    #[test]
    fn overlay_tuning_applies_overrides_and_keeps_other_defaults() {
        let file = TfOverlayTuningFile {
            triad_len: Some(2.0),
            divergence_color: Some([0.0, 1.0, 0.0]),
            ..Default::default()
        };

        let t = TfOverlayTuning::resolve(&file).expect("valid overrides resolve");

        assert_eq!(t.triad_len, 2.0);
        assert_eq!(t.divergence_color, Color::srgb(0.0, 1.0, 0.0));
        assert_eq!(
            t.truth_triad_len,
            TfOverlayTuning::default().truth_triad_len
        );
    }

    #[test]
    fn overlay_tuning_rejects_a_nonpositive_triad_len() {
        let file = TfOverlayTuningFile {
            triad_len: Some(0.0),
            ..Default::default()
        };

        assert!(TfOverlayTuning::resolve(&file).is_err());
    }
}
