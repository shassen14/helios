//! Shared axis-triad drawing for the live 3D overlays.
//!
//! One home for the two things every triad-drawing overlay (`pose`, `estimate`,
//! `tf`) needs identically: the REP-103 axis coloring and the three-line draw.
//! The triad *geometry* — origin plus the three axis directions in Bevy space —
//! comes from [`crate::core::transforms::frame_triad_to_bevy`], which points each
//! arrow where the frame's real axis points; this only turns that into gizmo
//! lines, so forward and up read off the arrows the way RViz shows them.

use bevy::prelude::*;

/// Draws a frame's axis triad as three REP-103-colored segments of length `len`
/// from `origin`: +X red (forward), +Y green (left), +Z up blue — the coloring
/// RViz uses, so forward and up read straight off the geometry. A fixed
/// convention, not an operator tunable, so the colors live here.
pub(crate) fn draw_triad(gizmos: &mut Gizmos, origin: Vec3, axes: [Vec3; 3], len: f32) {
    const AXIS_COLORS: [Color; 3] = [
        Color::srgb(1.0, 0.0, 0.0),
        Color::srgb(0.0, 1.0, 0.0),
        Color::srgb(0.0, 0.0, 1.0),
    ];

    for (axis, color) in axes.into_iter().zip(AXIS_COLORS) {
        gizmos.line(origin, origin + axis * len, color);
    }
}
