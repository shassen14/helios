//! The tf panel's pure cell→pixel geometry and verdict→colour mappings, plus the
//! resolved tuning they read.
//!
//! Everything computational here computes and nothing spawns: it turns a laid-out
//! [`LayoutCell`](super::layout::LayoutCell) into placed pixel rectangles and an
//! [`EdgeHealth`] verdict into a colour, with no `bevy_ui` and no `App`. The geometry
//! functions take their sizes and colours as borrowed tuning ([`TfPanelGraphTuning`],
//! [`TfPanelHealthColors`]) rather than reading a `Res`, which is what keeps
//! [`node_origin`], [`elbow_segments`], [`graph_extent`], and [`edge_color`] Tier-1
//! testable; the spawn/despawn shell that supplies those resources lives next door in
//! [`render`](super::render).
//!
//! The cell→pixel mapping has two orientations, chosen by [`PanelOrientation`]:
//! `TopDown` (root at top, `depth` down, `x` across) and `Sideways` (root at left,
//! `depth` across, `x` down). Only the three geometry functions and the band-stacking
//! axis branch on it; the layout cells themselves are orientation-free, so neither this
//! pass nor the model knows which way the tree will be drawn.

use crate::viz::interaction::tf_panel::layout::LayoutCell;
use crate::viz::interaction::tf_panel::model::{AgentGraph, EdgeHealth, HealthVerdict};
use crate::viz::interaction::tuning::{require_positive, InteractionTuningError};

use bevy::prelude::*;
use serde::Deserialize;

/// The config `default_orientation` string for [`PanelOrientation::Sideways`], matched
/// on load. A reserved string — producer (this parse) and the operator's TOML must
/// agree — so it is named once here rather than inlined.
const ORIENTATION_SIDEWAYS: &str = "sideways";
/// The config `default_orientation` string for [`PanelOrientation::TopDown`].
const ORIENTATION_TOP_DOWN: &str = "top_down";

/// Which way the tf tree grows, chosen per session and flipped live by the
/// `viz.toggle_tf_panel_orientation` action. Both layouts are kept on purpose: they
/// suit different tree shapes (see the variants), and the two are one `match` apart,
/// so keeping both costs little.
///
/// Orientation is *only* a cell→pixel concern — it changes [`node_origin`],
/// [`elbow_segments`], [`graph_extent`], and how unjoined per-agent bands stack. The
/// layout cells from [`tree_layout`](super::layout::tree_layout) are orientation-free,
/// so neither the geometry pass nor the model knows which way the tree will be drawn.
/// The session's starting value comes from `[tf_panel.graph].default_orientation`
/// (resolved in [`TfPanelGraphTuning::resolve`]); the toggle flips it from there.
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

impl PanelOrientation {
    /// Parses the config string, or `None` for an unknown value (the caller turns that
    /// into an [`InteractionTuningError::UnknownOrientation`]).
    fn from_config(s: &str) -> Option<Self> {
        match s {
            ORIENTATION_SIDEWAYS => Some(Self::Sideways),
            ORIENTATION_TOP_DOWN => Some(Self::TopDown),
            _ => None,
        }
    }
}

/// Sparse TOML overrides for the panel's graph geometry and node chrome. Every field is
/// optional; anything omitted falls back to the compiled-in [`TfPanelGraphTuning::default`].
#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct TfPanelGraphTuningFile {
    pub default_orientation: Option<String>,
    pub node_width: Option<f32>,
    pub node_height: Option<f32>,
    pub col_pitch: Option<f32>,
    pub row_pitch: Option<f32>,
    pub canvas_pad_top: Option<f32>,
    pub canvas_pad_left: Option<f32>,
    pub band_gap: Option<f32>,
    pub connector_thickness: Option<f32>,
    pub node_padding: Option<f32>,
    pub label_font_size: Option<f32>,
    pub header_font_size: Option<f32>,
    pub node_bg: Option<[f32; 4]>,
    pub label_color: Option<[f32; 3]>,
    pub header_color: Option<[f32; 3]>,
}

/// Resolved geometry and node chrome for the panel — one operator preference for the
/// session, read by the geometry functions here and the renderer next door. Defaults
/// reproduce the values compiled in before the tuning surface existed. `default_orientation`
/// resolves into the separate [`PanelOrientation`] resource, not a field here, since it
/// is live-toggled independently of the styling.
#[derive(Resource, Debug, Clone)]
pub struct TfPanelGraphTuning {
    /// Node box width; the geometry reserves spacing around it and the renderer sizes
    /// the box to it, so the box and the space held for it never diverge.
    pub node_width: f32,
    /// Node box height, used the same way as `node_width`.
    pub node_height: f32,
    /// Centre-to-centre spacing of adjacent depths (exceeds the box, leaving a gutter
    /// for the elbow).
    pub col_pitch: f32,
    /// Centre-to-centre spacing of adjacent slots (exceeds the box, leaving a gutter).
    pub row_pitch: f32,
    /// Top padding above the root row so it is not flush against the viewport edge.
    pub canvas_pad_top: f32,
    /// Left padding before the root column (`Sideways`), the counterpart of `canvas_pad_top`.
    pub canvas_pad_left: f32,
    /// Gap between one agent's band and the next (multi-agent layout).
    pub band_gap: f32,
    /// Connector-line thickness.
    pub connector_thickness: f32,
    /// Inner padding of a node box, between its border and the label.
    pub node_padding: f32,
    /// Node-label font size; small enough that a leaf name fits on one line in the box.
    pub label_font_size: f32,
    /// Pinned-header (agent name) font size.
    pub header_font_size: f32,
    /// Node box background, `[r, g, b, a]`.
    pub node_bg: Color,
    /// Node label text colour.
    pub label_color: Color,
    /// Pinned-header accent, shared with the inspector's section titles.
    pub header_color: Color,
}

impl Default for TfPanelGraphTuning {
    fn default() -> Self {
        Self {
            node_width: 120.0,
            node_height: 26.0,
            col_pitch: 128.0,
            row_pitch: 58.0,
            canvas_pad_top: 6.0,
            canvas_pad_left: 6.0,
            band_gap: 28.0,
            connector_thickness: 2.0,
            node_padding: 4.0,
            label_font_size: 10.0,
            header_font_size: 13.0,
            node_bg: Color::srgba(0.12, 0.13, 0.18, 0.95),
            label_color: Color::srgb(0.85, 0.88, 0.95),
            header_color: Color::srgb(0.62, 0.80, 1.0),
        }
    }
}

impl TfPanelGraphTuning {
    /// Overlays sparse overrides onto [`Default`], packing each colour array into an
    /// sRGB [`Color`], and returns the session's starting [`PanelOrientation`] alongside
    /// the styling — the one field that resolves to a separate resource because it is
    /// live-toggled. Rejects a non-positive size, pitch, thickness, or font (a zero box
    /// or pitch collapses the layout) and an unknown orientation string.
    pub(crate) fn resolve(
        overrides: &TfPanelGraphTuningFile,
    ) -> Result<(PanelOrientation, Self), InteractionTuningError> {
        let orientation = match &overrides.default_orientation {
            Some(s) => PanelOrientation::from_config(s)
                .ok_or_else(|| InteractionTuningError::UnknownOrientation { value: s.clone() })?,
            None => PanelOrientation::default(),
        };

        let mut t = Self::default();
        if let Some(v) = overrides.node_width {
            t.node_width = v;
        }
        if let Some(v) = overrides.node_height {
            t.node_height = v;
        }
        if let Some(v) = overrides.col_pitch {
            t.col_pitch = v;
        }
        if let Some(v) = overrides.row_pitch {
            t.row_pitch = v;
        }
        if let Some(v) = overrides.canvas_pad_top {
            t.canvas_pad_top = v;
        }
        if let Some(v) = overrides.canvas_pad_left {
            t.canvas_pad_left = v;
        }
        if let Some(v) = overrides.band_gap {
            t.band_gap = v;
        }
        if let Some(v) = overrides.connector_thickness {
            t.connector_thickness = v;
        }
        if let Some(v) = overrides.node_padding {
            t.node_padding = v;
        }
        if let Some(v) = overrides.label_font_size {
            t.label_font_size = v;
        }
        if let Some(v) = overrides.header_font_size {
            t.header_font_size = v;
        }
        if let Some([r, g, b, a]) = overrides.node_bg {
            t.node_bg = Color::srgba(r, g, b, a);
        }
        if let Some([r, g, b]) = overrides.label_color {
            t.label_color = Color::srgb(r, g, b);
        }
        if let Some([r, g, b]) = overrides.header_color {
            t.header_color = Color::srgb(r, g, b);
        }

        // Sizes that must not collapse the layout. Pads and gaps are allowed to be
        // zero (flush is a legitimate look), so they are not checked.
        require_positive("tf_panel.graph.node_width", t.node_width)?;
        require_positive("tf_panel.graph.node_height", t.node_height)?;
        require_positive("tf_panel.graph.col_pitch", t.col_pitch)?;
        require_positive("tf_panel.graph.row_pitch", t.row_pitch)?;
        require_positive("tf_panel.graph.connector_thickness", t.connector_thickness)?;
        require_positive("tf_panel.graph.label_font_size", t.label_font_size)?;
        require_positive("tf_panel.graph.header_font_size", t.header_font_size)?;
        Ok((orientation, t))
    }
}

/// Resolved verdict colours for the connectors — one operator preference for the
/// session, read by [`edge_color`]. Split out of the `[tf_panel.health]` section (whose
/// thresholds live with the verdict computation in `gather`) so each side reads only
/// what it consumes. Defaults reproduce the values compiled in before the tuning
/// surface existed.
#[derive(Resource, Debug, Clone)]
pub struct TfPanelHealthColors {
    /// A static edge, or a dynamic one with no samples yet — neither carries a rate or
    /// staleness signal, so neither earns a health colour.
    pub none: Color,
    /// Fresh: samples arriving within the stale threshold.
    pub ok: Color,
    /// Warning: no sample within the stale threshold, but not yet dead.
    pub stale: Color,
    /// Alarm: no sample within the dead threshold.
    pub dead: Color,
}

impl Default for TfPanelHealthColors {
    fn default() -> Self {
        Self {
            none: Color::srgb(0.45, 0.47, 0.52),
            ok: Color::srgb(0.36, 0.72, 0.45),
            stale: Color::srgb(0.90, 0.72, 0.25),
            dead: Color::srgb(0.86, 0.32, 0.32),
        }
    }
}

/// The pixel size of one laid-out graph as `(width, height)`. Which axis `depth` and
/// `x` drive flips with orientation: `TopDown` spreads slots across (width) and depth
/// down (height); `Sideways` spreads depth across (width) and slots down (height).
pub fn graph_extent(
    graph: &AgentGraph,
    orient: PanelOrientation,
    tuning: &TfPanelGraphTuning,
) -> (f32, f32) {
    let max_x = graph.nodes.iter().map(|n| n.cell.x).fold(0.0, f32::max);
    let max_depth = graph.nodes.iter().map(|n| n.cell.depth).max().unwrap_or(0);
    match orient {
        PanelOrientation::TopDown => {
            let width = max_x * tuning.col_pitch + tuning.node_width;
            let height =
                tuning.canvas_pad_top + max_depth as f32 * tuning.row_pitch + tuning.node_height;
            (width, height)
        }
        PanelOrientation::Sideways => {
            let width =
                tuning.canvas_pad_left + max_depth as f32 * tuning.col_pitch + tuning.node_width;
            let height = max_x * tuning.row_pitch + tuning.node_height;
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
pub fn node_origin(
    cell: LayoutCell,
    band_offset: f32,
    orient: PanelOrientation,
    tuning: &TfPanelGraphTuning,
) -> (f32, f32) {
    match orient {
        PanelOrientation::TopDown => {
            let left = band_offset + cell.x * tuning.col_pitch;
            let top = tuning.canvas_pad_top + cell.depth as f32 * tuning.row_pitch;
            (left, top)
        }
        PanelOrientation::Sideways => {
            let left = tuning.canvas_pad_left + cell.depth as f32 * tuning.col_pitch;
            let top = band_offset + cell.x * tuning.row_pitch;
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
    tuning: &TfPanelGraphTuning,
) -> [SegRect; 3] {
    let (parent_left, parent_top) = node_origin(parent, band_offset, orient, tuning);
    let (child_left, child_top) = node_origin(child, band_offset, orient, tuning);
    let thickness = tuning.connector_thickness;
    let half = thickness / 2.0;

    match orient {
        PanelOrientation::TopDown => {
            let parent_cx = parent_left + tuning.node_width / 2.0;
            let child_cx = child_left + tuning.node_width / 2.0;
            let parent_bottom = parent_top + tuning.node_height;
            let mid_y = (parent_bottom + child_top) / 2.0;

            [
                SegRect {
                    left: parent_cx - half,
                    top: parent_bottom,
                    width: thickness,
                    height: mid_y - parent_bottom,
                },
                SegRect {
                    left: parent_cx.min(child_cx) - half,
                    top: mid_y - half,
                    width: (parent_cx - child_cx).abs() + thickness,
                    height: thickness,
                },
                SegRect {
                    left: child_cx - half,
                    top: mid_y,
                    width: thickness,
                    height: child_top - mid_y,
                },
            ]
        }
        PanelOrientation::Sideways => {
            let parent_cy = parent_top + tuning.node_height / 2.0;
            let child_cy = child_top + tuning.node_height / 2.0;
            let parent_right = parent_left + tuning.node_width;
            let mid_x = (parent_right + child_left) / 2.0;

            [
                SegRect {
                    left: parent_right,
                    top: parent_cy - half,
                    width: mid_x - parent_right,
                    height: thickness,
                },
                SegRect {
                    left: mid_x - half,
                    top: parent_cy.min(child_cy) - half,
                    width: thickness,
                    height: (parent_cy - child_cy).abs() + thickness,
                },
                SegRect {
                    left: mid_x,
                    top: child_cy - half,
                    width: child_left - mid_x,
                    height: thickness,
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
pub fn edge_color(health: &Option<EdgeHealth>, colors: &TfPanelHealthColors) -> Color {
    match health {
        Some(h) => verdict_color(h.verdict, colors),
        None => colors.none,
    }
}

/// Maps a staleness verdict to its one-glance colour: calm green, warning amber, alarm
/// red. The three are deliberately distinct so the states separate at a glance.
fn verdict_color(verdict: HealthVerdict, colors: &TfPanelHealthColors) -> Color {
    match verdict {
        HealthVerdict::Ok => colors.ok,
        HealthVerdict::Stale => colors.stale,
        HealthVerdict::Dead => colors.dead,
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
    /// rows step down by `row_pitch`, slots step across by `col_pitch`, and the band
    /// offset shifts a whole graph sideways.
    #[test]
    fn node_origin_maps_depth_to_row_and_column_to_left() {
        let t = TfPanelGraphTuning::default();
        let td = PanelOrientation::TopDown;
        let (left, top) = node_origin(LayoutCell { depth: 0, x: 0.0 }, 0.0, td, &t);
        assert_eq!(left, 0.0);
        assert_eq!(top, t.canvas_pad_top);

        // A fractional column (a centred parent) maps to a fractional pixel offset.
        let (left, top) = node_origin(LayoutCell { depth: 2, x: 1.5 }, 50.0, td, &t);
        assert_eq!(left, 50.0 + 1.5 * t.col_pitch);
        assert_eq!(top, t.canvas_pad_top + 2.0 * t.row_pitch);
    }

    /// The sideways map is the top-down one with the axes swapped — `depth` drives the
    /// column (`left`) and `x` drives the row (`top` + band). The pixel-level claim the
    /// whole `Sideways` layout rests on.
    #[test]
    fn node_origin_sideways_maps_depth_to_column_and_slot_to_row() {
        let t = TfPanelGraphTuning::default();
        let sw = PanelOrientation::Sideways;
        let (left, top) = node_origin(LayoutCell { depth: 0, x: 0.0 }, 0.0, sw, &t);
        assert_eq!(left, t.canvas_pad_left);
        assert_eq!(top, 0.0);

        // depth → column, fractional slot → fractional row, band shifts the row.
        let (left, top) = node_origin(LayoutCell { depth: 2, x: 1.5 }, 50.0, sw, &t);
        assert_eq!(left, t.canvas_pad_left + 2.0 * t.col_pitch);
        assert_eq!(top, 50.0 + 1.5 * t.row_pitch);
    }

    /// An elbow between offset columns is three axis-aligned runs: two one-thickness
    /// verticals on the two node centre-columns and a one-thickness horizontal joining
    /// them, contiguous in y (parent stub bottom = child stub top = the midline).
    #[test]
    fn elbow_is_three_contiguous_axis_aligned_runs() {
        let t = TfPanelGraphTuning::default();
        let parent = LayoutCell { depth: 0, x: 0.0 };
        let child = LayoutCell { depth: 1, x: 1.0 };
        let segs = elbow_segments(parent, child, 0.0, PanelOrientation::TopDown, &t);

        assert_eq!(
            segs[0].width, t.connector_thickness,
            "parent stub is vertical"
        );
        assert_eq!(
            segs[2].width, t.connector_thickness,
            "child stub is vertical"
        );
        assert_eq!(
            segs[1].height, t.connector_thickness,
            "middle run is horizontal"
        );

        let parent_cx = t.node_width / 2.0;
        let child_cx = t.col_pitch + t.node_width / 2.0;
        assert_eq!(segs[0].left, parent_cx - t.connector_thickness / 2.0);
        assert_eq!(segs[2].left, child_cx - t.connector_thickness / 2.0);
        assert_eq!(
            segs[1].width,
            (parent_cx - child_cx).abs() + t.connector_thickness
        );

        assert!(
            (segs[0].top + segs[0].height - segs[2].top).abs() < 1e-6,
            "the parent stub, midline, and child stub meet without a gap",
        );
    }

    /// When parent and child share a column the elbow is a straight drop: both stubs
    /// sit on one x and the horizontal run collapses to a single joint.
    #[test]
    fn elbow_collapses_to_a_straight_drop_when_columns_align() {
        let t = TfPanelGraphTuning::default();
        let parent = LayoutCell { depth: 0, x: 2.0 };
        let child = LayoutCell { depth: 1, x: 2.0 };
        let segs = elbow_segments(parent, child, 10.0, PanelOrientation::TopDown, &t);

        assert_eq!(segs[0].left, segs[2].left, "both stubs share a column");
        assert_eq!(
            segs[1].width, t.connector_thickness,
            "the horizontal run is just the joint",
        );
    }

    /// The sideways elbow is the top-down one rotated a quarter turn — two horizontal
    /// stubs on the node centre-rows and a vertical run joining them, contiguous in x
    /// (parent stub right = child stub left = the mid-x).
    #[test]
    fn sideways_elbow_is_three_contiguous_axis_aligned_runs() {
        let t = TfPanelGraphTuning::default();
        let parent = LayoutCell { depth: 0, x: 0.0 };
        let child = LayoutCell { depth: 1, x: 1.0 };
        let segs = elbow_segments(parent, child, 0.0, PanelOrientation::Sideways, &t);

        assert_eq!(
            segs[0].height, t.connector_thickness,
            "parent stub is horizontal"
        );
        assert_eq!(
            segs[2].height, t.connector_thickness,
            "child stub is horizontal"
        );
        assert_eq!(
            segs[1].width, t.connector_thickness,
            "middle run is vertical"
        );

        assert!(
            (segs[0].left + segs[0].width - segs[2].left).abs() < 1e-6,
            "the parent stub, mid-x run, and child stub meet without a gap",
        );
    }

    /// When parent and child share a row the sideways elbow is a straight run right —
    /// both stubs sit on one y and the vertical run collapses to a joint.
    #[test]
    fn sideways_elbow_collapses_to_a_straight_run_when_rows_align() {
        let t = TfPanelGraphTuning::default();
        let parent = LayoutCell { depth: 0, x: 2.0 };
        let child = LayoutCell { depth: 1, x: 2.0 };
        let segs = elbow_segments(parent, child, 10.0, PanelOrientation::Sideways, &t);

        assert_eq!(segs[0].top, segs[2].top, "both stubs share a row");
        assert_eq!(
            segs[1].height, t.connector_thickness,
            "the vertical run is just the joint",
        );
    }

    /// The connector colour tracks the verdict and falls back to neutral without
    /// health, and the three verdict colours are distinct so the states read apart.
    #[test]
    fn edge_colour_tracks_verdict_and_defaults_to_neutral() {
        let c = TfPanelHealthColors::default();
        assert_eq!(edge_color(&None, &c), c.none);
        assert_eq!(edge_color(&Some(health(HealthVerdict::Ok)), &c), c.ok);
        assert_eq!(edge_color(&Some(health(HealthVerdict::Stale)), &c), c.stale);
        assert_eq!(edge_color(&Some(health(HealthVerdict::Dead)), &c), c.dead);

        assert_ne!(c.ok, c.stale);
        assert_ne!(c.stale, c.dead);
        assert_ne!(c.ok, c.dead);
    }

    /// An empty file resolves to the compiled-in defaults and the default orientation.
    #[test]
    fn graph_empty_file_resolves_to_defaults() {
        let (orient, t) = TfPanelGraphTuning::resolve(&TfPanelGraphTuningFile::default()).unwrap();
        let d = TfPanelGraphTuning::default();
        assert_eq!(orient, PanelOrientation::default());
        assert_eq!(t.node_width, d.node_width);
        assert_eq!(t.col_pitch, d.col_pitch);
        assert_eq!(t.node_bg, d.node_bg);
    }

    /// Overrides pass through, the four-tuple packs into an sRGBA colour, and the
    /// orientation string resolves.
    #[test]
    fn graph_overrides_pack_colors_and_orientation() {
        let file = TfPanelGraphTuningFile {
            default_orientation: Some(ORIENTATION_TOP_DOWN.into()),
            col_pitch: Some(200.0),
            node_bg: Some([0.1, 0.2, 0.3, 0.4]),
            ..Default::default()
        };
        let (orient, t) = TfPanelGraphTuning::resolve(&file).unwrap();
        assert_eq!(orient, PanelOrientation::TopDown);
        assert_eq!(t.col_pitch, 200.0);
        assert_eq!(t.node_bg, Color::srgba(0.1, 0.2, 0.3, 0.4));
        // An untouched field keeps its default.
        assert_eq!(t.row_pitch, TfPanelGraphTuning::default().row_pitch);
    }

    /// An unknown orientation string is a config error, not a silent fallback.
    #[test]
    fn graph_rejects_unknown_orientation() {
        let file = TfPanelGraphTuningFile {
            default_orientation: Some("diagonal".into()),
            ..Default::default()
        };
        let err = TfPanelGraphTuning::resolve(&file).unwrap_err();
        assert!(matches!(
            err,
            InteractionTuningError::UnknownOrientation { .. }
        ));
    }

    /// A zero pitch would collapse the layout and is rejected.
    #[test]
    fn graph_rejects_nonpositive_pitch() {
        let file = TfPanelGraphTuningFile {
            col_pitch: Some(0.0),
            ..Default::default()
        };
        assert!(matches!(
            TfPanelGraphTuning::resolve(&file).unwrap_err(),
            InteractionTuningError::NonPositive { .. }
        ));
    }
}
