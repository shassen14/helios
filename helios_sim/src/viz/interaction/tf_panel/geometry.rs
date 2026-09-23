//! The tf panel's pure cell→pixel geometry and verdict→colour mappings.
//!
//! Everything here computes and nothing spawns: it turns a laid-out
//! [`LayoutCell`](super::layout::LayoutCell) into placed pixel rectangles and an
//! [`EdgeHealth`] verdict into a colour, with no `bevy_ui` and no `App`. That is what
//! keeps [`node_origin`], [`elbow_segments`], [`graph_extent`], and [`edge_color`]
//! Tier-1 testable; the spawn/despawn shell that consumes them lives next door in
//! [`render`](super::render).
//!
//! The cell→pixel mapping has two orientations, chosen by [`PanelOrientation`]:
//! `TopDown` (root at top, `depth` down, `x` across) and `Sideways` (root at left,
//! `depth` across, `x` down). Only the three geometry functions and the band-stacking
//! axis branch on it; the layout cells themselves are orientation-free, so neither this
//! pass nor the model knows which way the tree will be drawn.

use crate::viz::interaction::tf_panel::layout::LayoutCell;
use crate::viz::interaction::tf_panel::model::{AgentGraph, EdgeHealth, HealthVerdict};

use bevy::prelude::*;

// Interim presentation constants, named rather than inlined — no bare magic numbers in
// the geometry. A later `[tf_panel]` config surface lifts these out of source, the same
// move the camera-rate consts are headed for.
/// The node box size. Owned here because the geometry reserves spacing around it; the
/// renderer imports the same two values to size the box it draws, so the box and the
/// space held for it can never diverge.
pub(crate) const NODE_WIDTH: f32 = 120.0;
pub(crate) const NODE_HEIGHT: f32 = 26.0;
/// Centre-to-centre spacing of adjacent slots and depths. Both exceed the node box,
/// so a gutter is left between rows (for the elbow) and between columns.
const COL_PITCH: f32 = 128.0;
const ROW_PITCH: f32 = 58.0;
/// Small top padding inside the canvas, above the root row, so it is not flush against
/// the viewport's top edge. (The agent name lives in the pinned header now, not in the
/// canvas, so no full header row is reserved here.)
const CANVAS_PAD_TOP: f32 = 6.0;
/// Left padding for the `Sideways` layout, so the root column is not flush against the
/// viewport's left edge — the counterpart of `CANVAS_PAD_TOP`.
const CANVAS_PAD_LEFT: f32 = 6.0;
const CONNECTOR_THICKNESS: f32 = 2.0;

/// Neutral connector: a static edge, or a dynamic one with no samples yet — neither
/// carries a rate or staleness signal, so neither earns a health colour.
const EDGE_COLOR_NONE: Color = Color::srgb(0.45, 0.47, 0.52);
const EDGE_COLOR_OK: Color = Color::srgb(0.36, 0.72, 0.45);
const EDGE_COLOR_STALE: Color = Color::srgb(0.90, 0.72, 0.25);
const EDGE_COLOR_DEAD: Color = Color::srgb(0.86, 0.32, 0.32);

/// Which way the tf tree grows, chosen per session and flipped live by the
/// `viz.toggle_tf_panel_orientation` action. Both layouts are kept on purpose: they
/// suit different tree shapes (see the variants), and the two are one `match` apart,
/// so keeping both costs little.
///
/// Orientation is *only* a cell→pixel concern — it changes [`node_origin`],
/// [`elbow_segments`], [`graph_extent`], and how unjoined per-agent bands stack. The
/// layout cells from [`tree_layout`](super::layout::tree_layout) are orientation-free,
/// so neither the geometry pass nor the model knows which way the tree will be drawn.
#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PanelOrientation {
    /// Root at top, `depth` grows downward, siblings spread across. The familiar
    /// rqt-style tree; best when trees are deep and narrow, so width stays bounded.
    TopDown,
    /// Root at left, `depth` grows rightward, siblings spread downward. The default:
    /// helios trees are wide sensor fans over a shallow spine, so growth falls on the
    /// vertical (natural-scroll) axis and the tall dock rather than forcing horizontal
    /// scroll.
    #[default]
    Sideways,
}

/// The pixel size of one laid-out graph as `(width, height)`. Which axis `depth` and
/// `x` drive flips with orientation: `TopDown` spreads slots across (width) and depth
/// down (height); `Sideways` spreads depth across (width) and slots down (height).
pub fn graph_extent(graph: &AgentGraph, orient: PanelOrientation) -> (f32, f32) {
    let max_x = graph.nodes.iter().map(|n| n.cell.x).fold(0.0, f32::max);
    let max_depth = graph.nodes.iter().map(|n| n.cell.depth).max().unwrap_or(0);
    match orient {
        PanelOrientation::TopDown => {
            let width = max_x * COL_PITCH + NODE_WIDTH;
            let height = CANVAS_PAD_TOP + max_depth as f32 * ROW_PITCH + NODE_HEIGHT;
            (width, height)
        }
        PanelOrientation::Sideways => {
            let width = CANVAS_PAD_LEFT + max_depth as f32 * COL_PITCH + NODE_WIDTH;
            let height = max_x * ROW_PITCH + NODE_HEIGHT;
            (width, height)
        }
    }
}

/// The top-left of a node's box, in canvas pixels. The `band_offset` always shifts the
/// cross axis (the one siblings spread on). Orientation decides which pixel axis
/// `depth` and `x` drive:
/// - `TopDown`: `depth`→row (`top`), `x`→column (`left` + band). Root at top.
/// - `Sideways`: `depth`→column (`left`), `x`→row (`top` + band). Root at left.
///
/// This is the whole cell→pixel mapping — pure, so it is Tier-1 tested.
pub fn node_origin(cell: LayoutCell, band_offset: f32, orient: PanelOrientation) -> (f32, f32) {
    match orient {
        PanelOrientation::TopDown => {
            let left = band_offset + cell.x * COL_PITCH;
            let top = CANVAS_PAD_TOP + cell.depth as f32 * ROW_PITCH;
            (left, top)
        }
        PanelOrientation::Sideways => {
            let left = CANVAS_PAD_LEFT + cell.depth as f32 * COL_PITCH;
            let top = band_offset + cell.x * ROW_PITCH;
            (left, top)
        }
    }
}

/// The three axis-aligned runs of a parent→child elbow. Orientation picks the shape:
///
/// - `TopDown`: a vertical stub down from the parent's bottom-centre, a horizontal run
///   across the row gutter at the midline, a vertical stub down into the child's
///   top-centre. Shared column ⇒ the horizontal run collapses and it is one straight drop.
/// - `Sideways`: the same elbow rotated a quarter turn — a horizontal stub right from the
///   parent's right-centre, a vertical run across the column gutter at the mid-x, a
///   horizontal stub into the child's left-centre. Shared row ⇒ one straight run right.
pub fn elbow_segments(
    parent: LayoutCell,
    child: LayoutCell,
    band_offset: f32,
    orient: PanelOrientation,
) -> [SegRect; 3] {
    let (parent_left, parent_top) = node_origin(parent, band_offset, orient);
    let (child_left, child_top) = node_origin(child, band_offset, orient);
    let half = CONNECTOR_THICKNESS / 2.0;

    match orient {
        PanelOrientation::TopDown => {
            let parent_cx = parent_left + NODE_WIDTH / 2.0;
            let child_cx = child_left + NODE_WIDTH / 2.0;
            let parent_bottom = parent_top + NODE_HEIGHT;
            let mid_y = (parent_bottom + child_top) / 2.0;

            [
                SegRect {
                    left: parent_cx - half,
                    top: parent_bottom,
                    width: CONNECTOR_THICKNESS,
                    height: mid_y - parent_bottom,
                },
                SegRect {
                    left: parent_cx.min(child_cx) - half,
                    top: mid_y - half,
                    width: (parent_cx - child_cx).abs() + CONNECTOR_THICKNESS,
                    height: CONNECTOR_THICKNESS,
                },
                SegRect {
                    left: child_cx - half,
                    top: mid_y,
                    width: CONNECTOR_THICKNESS,
                    height: child_top - mid_y,
                },
            ]
        }
        PanelOrientation::Sideways => {
            let parent_cy = parent_top + NODE_HEIGHT / 2.0;
            let child_cy = child_top + NODE_HEIGHT / 2.0;
            let parent_right = parent_left + NODE_WIDTH;
            let mid_x = (parent_right + child_left) / 2.0;

            [
                SegRect {
                    left: parent_right,
                    top: parent_cy - half,
                    width: mid_x - parent_right,
                    height: CONNECTOR_THICKNESS,
                },
                SegRect {
                    left: mid_x - half,
                    top: parent_cy.min(child_cy) - half,
                    width: CONNECTOR_THICKNESS,
                    height: (parent_cy - child_cy).abs() + CONNECTOR_THICKNESS,
                },
                SegRect {
                    left: mid_x,
                    top: child_cy - half,
                    width: child_left - mid_x,
                    height: CONNECTOR_THICKNESS,
                },
            ]
        }
    }
}

/// One connector run as a placed rectangle, ready to become an absolute `Node`. A plain
/// geometry value with no `bevy_ui` in it, so [`elbow_segments`] stays testable; the
/// fields are `pub(crate)` so the renderer next door can read them straight onto a
/// `Node`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SegRect {
    pub(crate) left: f32,
    pub(crate) top: f32,
    pub(crate) width: f32,
    pub(crate) height: f32,
}

/// The connector colour for an edge: its verdict colour when it carries health, the
/// neutral colour otherwise. Only matches the enum — the thresholds that produced the
/// verdict ran in the gather step.
pub fn edge_color(health: &Option<EdgeHealth>) -> Color {
    match health {
        Some(h) => verdict_color(h.verdict),
        None => EDGE_COLOR_NONE,
    }
}

/// Maps a staleness verdict to its one-glance colour: calm green, warning amber, alarm
/// red. The three are deliberately distinct so the states separate at a glance.
fn verdict_color(verdict: HealthVerdict) -> Color {
    match verdict {
        HealthVerdict::Ok => EDGE_COLOR_OK,
        HealthVerdict::Stale => EDGE_COLOR_STALE,
        HealthVerdict::Dead => EDGE_COLOR_DEAD,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn health(verdict: HealthVerdict) -> EdgeHealth {
        EdgeHealth {
            rate_hz: 0.0,
            staleness_s: 0.0,
            verdict,
        }
    }

    /// The cell→pixel map: root-at-top means `depth 0` sits at the header line, deeper
    /// rows step down by `ROW_PITCH`, slots step across by `COL_PITCH`, and the band
    /// offset shifts a whole graph sideways.
    #[test]
    fn node_origin_maps_depth_to_row_and_column_to_left() {
        let td = PanelOrientation::TopDown;
        let (left, top) = node_origin(LayoutCell { depth: 0, x: 0.0 }, 0.0, td);
        assert_eq!(left, 0.0);
        assert_eq!(top, CANVAS_PAD_TOP);

        // A fractional column (a centred parent) maps to a fractional pixel offset.
        let (left, top) = node_origin(LayoutCell { depth: 2, x: 1.5 }, 50.0, td);
        assert_eq!(left, 50.0 + 1.5 * COL_PITCH);
        assert_eq!(top, CANVAS_PAD_TOP + 2.0 * ROW_PITCH);
    }

    /// The sideways map is the top-down one with the axes swapped — `depth` drives the
    /// column (`left`) and `x` drives the row (`top` + band). The pixel-level claim the
    /// whole `Sideways` layout rests on.
    #[test]
    fn node_origin_sideways_maps_depth_to_column_and_slot_to_row() {
        let sw = PanelOrientation::Sideways;
        let (left, top) = node_origin(LayoutCell { depth: 0, x: 0.0 }, 0.0, sw);
        assert_eq!(left, CANVAS_PAD_LEFT);
        assert_eq!(top, 0.0);

        // depth → column, fractional slot → fractional row, band shifts the row.
        let (left, top) = node_origin(LayoutCell { depth: 2, x: 1.5 }, 50.0, sw);
        assert_eq!(left, CANVAS_PAD_LEFT + 2.0 * COL_PITCH);
        assert_eq!(top, 50.0 + 1.5 * ROW_PITCH);
    }

    /// An elbow between offset columns is three axis-aligned runs: two one-thickness
    /// verticals on the two node centre-columns and a one-thickness horizontal joining
    /// them, contiguous in y (parent stub bottom = child stub top = the midline).
    #[test]
    fn elbow_is_three_contiguous_axis_aligned_runs() {
        let parent = LayoutCell { depth: 0, x: 0.0 };
        let child = LayoutCell { depth: 1, x: 1.0 };
        let segs = elbow_segments(parent, child, 0.0, PanelOrientation::TopDown);

        assert_eq!(segs[0].width, CONNECTOR_THICKNESS, "parent stub is vertical");
        assert_eq!(segs[2].width, CONNECTOR_THICKNESS, "child stub is vertical");
        assert_eq!(segs[1].height, CONNECTOR_THICKNESS, "middle run is horizontal");

        let parent_cx = NODE_WIDTH / 2.0;
        let child_cx = COL_PITCH + NODE_WIDTH / 2.0;
        assert_eq!(segs[0].left, parent_cx - CONNECTOR_THICKNESS / 2.0);
        assert_eq!(segs[2].left, child_cx - CONNECTOR_THICKNESS / 2.0);
        assert_eq!(segs[1].width, (parent_cx - child_cx).abs() + CONNECTOR_THICKNESS);

        assert!(
            (segs[0].top + segs[0].height - segs[2].top).abs() < 1e-6,
            "the parent stub, midline, and child stub meet without a gap",
        );
    }

    /// When parent and child share a column the elbow is a straight drop: both stubs
    /// sit on one x and the horizontal run collapses to a single joint.
    #[test]
    fn elbow_collapses_to_a_straight_drop_when_columns_align() {
        let parent = LayoutCell { depth: 0, x: 2.0 };
        let child = LayoutCell { depth: 1, x: 2.0 };
        let segs = elbow_segments(parent, child, 10.0, PanelOrientation::TopDown);

        assert_eq!(segs[0].left, segs[2].left, "both stubs share a column");
        assert_eq!(
            segs[1].width, CONNECTOR_THICKNESS,
            "the horizontal run is just the joint",
        );
    }

    /// The sideways elbow is the top-down one rotated a quarter turn — two horizontal
    /// stubs on the node centre-rows and a vertical run joining them, contiguous in x
    /// (parent stub right = child stub left = the mid-x).
    #[test]
    fn sideways_elbow_is_three_contiguous_axis_aligned_runs() {
        let parent = LayoutCell { depth: 0, x: 0.0 };
        let child = LayoutCell { depth: 1, x: 1.0 };
        let segs = elbow_segments(parent, child, 0.0, PanelOrientation::Sideways);

        assert_eq!(segs[0].height, CONNECTOR_THICKNESS, "parent stub is horizontal");
        assert_eq!(segs[2].height, CONNECTOR_THICKNESS, "child stub is horizontal");
        assert_eq!(segs[1].width, CONNECTOR_THICKNESS, "middle run is vertical");

        assert!(
            (segs[0].left + segs[0].width - segs[2].left).abs() < 1e-6,
            "the parent stub, mid-x run, and child stub meet without a gap",
        );
    }

    /// When parent and child share a row the sideways elbow is a straight run right —
    /// both stubs sit on one y and the vertical run collapses to a joint.
    #[test]
    fn sideways_elbow_collapses_to_a_straight_run_when_rows_align() {
        let parent = LayoutCell { depth: 0, x: 2.0 };
        let child = LayoutCell { depth: 1, x: 2.0 };
        let segs = elbow_segments(parent, child, 10.0, PanelOrientation::Sideways);

        assert_eq!(segs[0].top, segs[2].top, "both stubs share a row");
        assert_eq!(
            segs[1].height, CONNECTOR_THICKNESS,
            "the vertical run is just the joint",
        );
    }

    /// The connector colour tracks the verdict and falls back to neutral without
    /// health, and the three verdict colours are distinct so the states read apart.
    #[test]
    fn edge_colour_tracks_verdict_and_defaults_to_neutral() {
        assert_eq!(edge_color(&None), EDGE_COLOR_NONE);
        assert_eq!(edge_color(&Some(health(HealthVerdict::Ok))), EDGE_COLOR_OK);
        assert_eq!(edge_color(&Some(health(HealthVerdict::Stale))), EDGE_COLOR_STALE);
        assert_eq!(edge_color(&Some(health(HealthVerdict::Dead))), EDGE_COLOR_DEAD);

        assert_ne!(EDGE_COLOR_OK, EDGE_COLOR_STALE);
        assert_ne!(EDGE_COLOR_STALE, EDGE_COLOR_DEAD);
        assert_ne!(EDGE_COLOR_OK, EDGE_COLOR_DEAD);
    }
}
