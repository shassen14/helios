//! Draws the tf panel model as `bevy_ui` nodes under the panel dock.
//!
//! This is the dumb half of the panel: it walks [`TfPanelModel`] and paints it,
//! computing nothing. All interpretation — layout cells, health verdicts, rates —
//! already happened in the gather step, so the renderer only maps a cell to pixels,
//! an elbow to three boxes, and a verdict to a colour. Those three mappings are pure
//! functions ([`node_origin`], [`elbow_segments`], [`edge_color`]), Tier-1 tested
//! with no `App`; the spawn/despawn shell around them is untested ECS glue.
//!
//! The cell→pixel mapping has two orientations, chosen by [`PanelOrientation`]:
//! `TopDown` (root at top, `depth` down, `slot` across) and `Sideways` (root at
//! left, `depth` across, `slot` down). The three pure mappings branch on it; nothing
//! else does. The dock container ([`TfPanelRoot`]) is persistent — a visibility flip,
//! not a respawn
//! — but its contents are rebuilt wholesale each visible frame (despawn-and-respawn),
//! matching the inspector's renderer. The tree is tiny and the panel is gated off
//! unless shown, so a reconciler is not worth its bug surface yet.

use crate::viz::interaction::tf_panel::layout::LayoutCell;
use crate::viz::interaction::tf_panel::model::{AgentGraph, EdgeHealth, HealthVerdict};
use crate::viz::interaction::tf_panel::panel::{TfPanelHeader, TfPanelViewport};
use crate::viz::interaction::tf_panel::TfPanelModel;

use helios_core::frames::FrameId;

use bevy::prelude::*;
use std::collections::HashMap;

// Interim presentation constants, named rather than inlined — no bare magic numbers
// in the node. A later `[tf_panel]` config surface lifts these out of source, the
// same move the camera-rate and `panel.rs` dock consts are headed for.
const NODE_WIDTH: f32 = 120.0;
const NODE_HEIGHT: f32 = 26.0;
/// Centre-to-centre spacing of adjacent slots and depths. Both exceed the node box,
/// so a gutter is left between rows (for the elbow) and between columns.
const COL_PITCH: f32 = 128.0;
const ROW_PITCH: f32 = 58.0;
/// Small top padding inside the canvas, above the root row, so it is not flush
/// against the viewport's top edge. (The agent name lives in the pinned header now,
/// not in the canvas, so no full header row is reserved here.)
const CANVAS_PAD_TOP: f32 = 6.0;
/// Left padding for the `Sideways` layout, so the root column is not flush against
/// the viewport's left edge — the counterpart of `CANVAS_PAD_TOP`.
const CANVAS_PAD_LEFT: f32 = 6.0;
/// Horizontal gap between one agent's band and the next (multi-agent layout).
const BAND_GAP: f32 = 28.0;
const CONNECTOR_THICKNESS: f32 = 2.0;
const NODE_PADDING: f32 = 4.0;
/// Small enough that a leaf name (`sensor.gps.primary`) sits on one line in the box.
const LABEL_FONT_SIZE: f32 = 10.0;
const HEADER_FONT_SIZE: f32 = 13.0;

const NODE_BG: Color = Color::srgba(0.12, 0.13, 0.18, 0.95);
const LABEL_COLOR: Color = Color::srgb(0.85, 0.88, 0.95);
/// Accent for the per-agent header, shared with the inspector's section titles.
const HEADER_COLOR: Color = Color::srgb(0.62, 0.80, 1.0);
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
/// [`elbow_segments`], [`graph_extent`], and how unjoined per-agent bands stack.
/// The layout cells from [`tree_layout`](super::layout::tree_layout) are
/// orientation-free, so neither the geometry pass nor the model knows which way the
/// tree will be drawn.
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

/// Marks everything the renderer spawns, so the previous frame's whole draw can be
/// found and despawned in one query before the next is built. One canvas per rebuild
/// carries this; its node/connector/label children fall with it (recursive despawn).
#[derive(Component)]
pub struct TfPanelContent;

/// Rebuilds the panel's `bevy_ui` contents from [`TfPanelModel`] each visible frame.
///
/// Despawns last frame's canvas wholesale, then — unless the model is empty — spawns
/// a fresh one under [`TfPanelRoot`] and lays every agent graph into it. Agents are
/// placed in side-by-side horizontal bands: since `LayoutCell::slot` is a per-depth
/// ordinal (not a column positioned under the parent), letting two agents share a row
/// would cross their connectors, so each agent's whole graph gets its own `left` base.
pub fn render_tf_panel(
    model: Res<TfPanelModel>,
    orientation: Res<PanelOrientation>,
    header: Query<Entity, With<TfPanelHeader>>,
    viewport: Query<Entity, With<TfPanelViewport>>,
    old: Query<Entity, With<TfPanelContent>>,
    mut commands: Commands,
) {
    let orient = *orientation;
    // Despawn-and-rebuild: drop the previous header text and canvas before drawing.
    for entity in &old {
        commands.entity(entity).despawn();
    }

    // No dock yet (before `spawn_tf_panel`, or a headless test that skips it) means
    // nowhere to hang the content, so there is nothing to do this frame.
    let (Ok(header_entity), Ok(viewport_entity)) = (header.single(), viewport.single()) else {
        return;
    };
    if model.0.is_empty() {
        return;
    }

    // Pinned title: the agent name(s) whose tree this is, so identity survives the
    // tree being scrolled. It hangs in the header, outside the scroll region.
    let names = model
        .0
        .iter()
        .map(|graph| graph.agent.as_str())
        .collect::<Vec<_>>()
        .join(", ");
    let title = commands
        .spawn((
            TfPanelContent,
            Text::new(names),
            TextFont {
                font_size: FontSize::Px(HEADER_FONT_SIZE),
                ..default()
            },
            TextColor(HEADER_COLOR),
        ))
        .id();
    commands.entity(header_entity).add_child(title);

    // First pass: place each graph's band and size the canvas to bound them all. The
    // band `cursor` runs along the cross axis (the one siblings spread on): for
    // `TopDown` that is horizontal, for `Sideways` vertical. The `main` extent is the
    // canvas bound on the other axis. `band_offset` is what `node_origin` adds on the
    // cross axis, so one cursor drives both orientations.
    let mut bands: Vec<(&AgentGraph, f32)> = Vec::new();
    let mut cursor = 0.0;
    let mut canvas_main = 0.0_f32;
    for graph in &model.0 {
        let (width, height) = graph_extent(graph, orient);
        let (cross, main) = match orient {
            PanelOrientation::TopDown => (width, height),
            PanelOrientation::Sideways => (height, width),
        };
        bands.push((graph, cursor));
        cursor += cross + BAND_GAP;
        canvas_main = canvas_main.max(main);
    }
    let canvas_cross = (cursor - BAND_GAP).max(0.0); // drop the trailing gap
    let (canvas_width, canvas_height) = match orient {
        PanelOrientation::TopDown => (canvas_cross, canvas_main),
        PanelOrientation::Sideways => (canvas_main, canvas_cross),
    };

    // The canvas is an in-flow child sized to the content, so the dock's `overflow`
    // clips it; its own children are absolutely placed relative to it.
    let canvas = commands
        .spawn((
            TfPanelContent,
            Node {
                width: Val::Px(canvas_width),
                height: Val::Px(canvas_height),
                ..default()
            },
        ))
        .id();
    commands.entity(viewport_entity).add_child(canvas);

    for (graph, band_offset) in bands {
        spawn_graph(&mut commands, canvas, graph, band_offset, orient);
    }
}

/// The pixel size of one laid-out graph as `(width, height)`. Which axis `depth` and
/// `x` drive flips with orientation: `TopDown` spreads slots across (width) and depth
/// down (height); `Sideways` spreads depth across (width) and slots down (height).
fn graph_extent(graph: &AgentGraph, orient: PanelOrientation) -> (f32, f32) {
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

/// Spawns one agent's connectors and node boxes into the canvas at its band offset.
///
/// Connectors are spawned before the node boxes so the boxes paint over the joins
/// (later `bevy_ui` siblings render on top). An edge whose parent or child was left
/// unplaced by the layout — an orphan or a cycle member — has no cell to anchor to,
/// so its connector is skipped rather than drawn to a bogus origin. The agent's name
/// is not drawn here; it lives in the panel's pinned header.
fn spawn_graph(
    commands: &mut Commands,
    canvas: Entity,
    graph: &AgentGraph,
    band_offset: f32,
    orient: PanelOrientation,
) {
    let cells: HashMap<&FrameId, LayoutCell> =
        graph.nodes.iter().map(|n| (&n.frame, n.cell)).collect();

    for edge in &graph.edges {
        let (Some(&parent), Some(&child)) =
            (cells.get(&edge.parent), cells.get(&edge.child))
        else {
            continue;
        };
        let color = edge_color(&edge.health);
        for seg in elbow_segments(parent, child, band_offset, orient) {
            // A degenerate run (zero or negative extent) draws nothing; skip it so no
            // `Val::Px` ever receives a negative size.
            if seg.width <= 0.0 || seg.height <= 0.0 {
                continue;
            }
            let segment = commands
                .spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        left: Val::Px(seg.left),
                        top: Val::Px(seg.top),
                        width: Val::Px(seg.width),
                        height: Val::Px(seg.height),
                        ..default()
                    },
                    BackgroundColor(color),
                ))
                .id();
            commands.entity(canvas).add_child(segment);
        }
    }

    for node in &graph.nodes {
        let (left, top) = node_origin(node.cell, band_offset, orient);
        let boxed = commands
            .spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Px(left),
                    top: Val::Px(top),
                    width: Val::Px(NODE_WIDTH),
                    height: Val::Px(NODE_HEIGHT),
                    padding: UiRect::all(Val::Px(NODE_PADDING)),
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    overflow: Overflow::clip(),
                    ..default()
                },
                BackgroundColor(NODE_BG),
            ))
            .id();
        let label = commands
            .spawn((
                Text::new(node.label.clone()),
                TextFont {
                    font_size: FontSize::Px(LABEL_FONT_SIZE),
                    ..default()
                },
                TextColor(LABEL_COLOR),
            ))
            .id();
        commands.entity(boxed).add_child(label);
        commands.entity(canvas).add_child(boxed);
    }
}

/// The top-left of a node's box, in canvas pixels. The `band_offset` always shifts the
/// cross axis (the one siblings spread on). Orientation decides which pixel axis
/// `depth` and `slot` drive:
/// - `TopDown`: `depth`→row (`top`), `slot`→column (`left` + band). Root at top.
/// - `Sideways`: `depth`→column (`left`), `slot`→row (`top` + band). Root at left.
///
/// This is the whole cell→pixel mapping — pure, so it is Tier-1 tested.
fn node_origin(cell: LayoutCell, band_offset: f32, orient: PanelOrientation) -> (f32, f32) {
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
fn elbow_segments(
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

/// One connector run as a placed rectangle, ready to become an absolute `Node`. A
/// plain geometry value with no `bevy_ui` in it, so [`elbow_segments`] stays testable.
#[derive(Debug, Clone, Copy, PartialEq)]
struct SegRect {
    left: f32,
    top: f32,
    width: f32,
    height: f32,
}

/// The connector colour for an edge: its verdict colour when it carries health, the
/// neutral colour otherwise. The renderer only matches the enum — the thresholds that
/// produced the verdict ran in the gather step.
fn edge_color(health: &Option<EdgeHealth>) -> Color {
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
    /// column (`left`) and `slot` drives the row (`top` + band). The pixel-level claim
    /// the whole `Sideways` layout rests on.
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
