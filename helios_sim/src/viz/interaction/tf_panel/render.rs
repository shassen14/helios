//! Draws the tf panel model as `bevy_ui` nodes under the panel dock.
//!
//! This is the dumb half of the panel: it walks [`TfPanelModel`] and paints it,
//! computing nothing. All interpretation — layout cells, health verdicts, rates —
//! already happened in the gather step, so the renderer only maps a cell to pixels,
//! an elbow to three boxes, and a verdict to a colour. Those three mappings are pure
//! functions ([`node_origin`], [`elbow_segments`], [`edge_color`]), Tier-1 tested
//! with no `App`; the spawn/despawn shell around them is untested ECS glue.
//!
//! Root-at-top: `depth` grows downward (`top`), `slot` runs across (`left`). The
//! dock container ([`TfPanelRoot`]) is persistent — a visibility flip, not a respawn
//! — but its contents are rebuilt wholesale each visible frame (despawn-and-respawn),
//! matching the inspector's renderer. The tree is tiny and the panel is gated off
//! unless shown, so a reconciler is not worth its bug surface yet.

use crate::viz::interaction::tf_panel::layout::LayoutCell;
use crate::viz::interaction::tf_panel::model::{AgentGraph, EdgeHealth, HealthVerdict};
use crate::viz::interaction::tf_panel::panel::TfPanelRoot;
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
/// Vertical room reserved above a graph's root row for its agent-name header.
const HEADER_HEIGHT: f32 = 22.0;
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
    root: Query<Entity, With<TfPanelRoot>>,
    old: Query<Entity, With<TfPanelContent>>,
    mut commands: Commands,
) {
    // Despawn-and-rebuild: drop the previous canvas (and its children) before drawing.
    for entity in &old {
        commands.entity(entity).despawn();
    }

    // No dock yet (before `spawn_tf_panel`, or a headless test that skips it) means
    // nowhere to hang the content, so there is nothing to do this frame.
    let Ok(root_entity) = root.single() else {
        return;
    };
    if model.0.is_empty() {
        return;
    }

    // First pass: place each graph's band and size the canvas to bound them all.
    let mut bands: Vec<(&AgentGraph, f32)> = Vec::new();
    let mut cursor = 0.0;
    let mut canvas_height = 0.0_f32;
    for graph in &model.0 {
        let (width, height) = graph_extent(graph);
        bands.push((graph, cursor));
        cursor += width + BAND_GAP;
        canvas_height = canvas_height.max(height);
    }
    let canvas_width = (cursor - BAND_GAP).max(0.0); // drop the trailing gap

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
    commands.entity(root_entity).add_child(canvas);

    for (graph, band_left) in bands {
        spawn_graph(&mut commands, canvas, graph, band_left);
    }
}

/// The pixel size of one laid-out graph: wide enough for its rightmost slot, tall
/// enough for its deepest row plus the header. Drives the canvas bound and the next
/// band's offset.
fn graph_extent(graph: &AgentGraph) -> (f32, f32) {
    let max_x = graph.nodes.iter().map(|n| n.cell.x).fold(0.0, f32::max);
    let max_depth = graph.nodes.iter().map(|n| n.cell.depth).max().unwrap_or(0);
    let width = max_x * COL_PITCH + NODE_WIDTH;
    let height = HEADER_HEIGHT + max_depth as f32 * ROW_PITCH + NODE_HEIGHT;
    (width, height)
}

/// Spawns one agent's header, connectors, and node boxes into the canvas.
///
/// Connectors are spawned before the node boxes so the boxes paint over the joins
/// (later `bevy_ui` siblings render on top). An edge whose parent or child was left
/// unplaced by the layout — an orphan or a cycle member — has no cell to anchor to,
/// so its connector is skipped rather than drawn to a bogus origin.
fn spawn_graph(commands: &mut Commands, canvas: Entity, graph: &AgentGraph, band_left: f32) {
    // Header: the agent name, above the root row.
    let header = commands
        .spawn((
            Text::new(graph.agent.as_str()),
            TextFont {
                font_size: FontSize::Px(HEADER_FONT_SIZE),
                ..default()
            },
            TextColor(HEADER_COLOR),
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(band_left),
                top: Val::Px(0.0),
                ..default()
            },
        ))
        .id();
    commands.entity(canvas).add_child(header);

    let cells: HashMap<&FrameId, LayoutCell> =
        graph.nodes.iter().map(|n| (&n.frame, n.cell)).collect();

    for edge in &graph.edges {
        let (Some(&parent), Some(&child)) =
            (cells.get(&edge.parent), cells.get(&edge.child))
        else {
            continue;
        };
        let color = edge_color(&edge.health);
        for seg in elbow_segments(parent, child, band_left) {
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
        let (left, top) = node_origin(node.cell, band_left);
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

/// The top-left of a node's box, in canvas pixels. `depth`→row (`top`), `slot`→column
/// (`left`), plus the agent's band offset. Root-at-top: `depth 0` sits just below the
/// header. This is the whole cell→pixel mapping — pure, so it is Tier-1 tested.
fn node_origin(cell: LayoutCell, band_left: f32) -> (f32, f32) {
    let left = band_left + cell.x * COL_PITCH;
    let top = HEADER_HEIGHT + cell.depth as f32 * ROW_PITCH;
    (left, top)
}

/// The three axis-aligned runs of a parent→child elbow: a vertical stub down from the
/// parent's bottom-centre, a horizontal run across the row gutter at the midline, and
/// a vertical stub down into the child's top-centre. When the two share a column the
/// horizontal run collapses to a single joint and the stubs form one straight drop.
fn elbow_segments(parent: LayoutCell, child: LayoutCell, band_left: f32) -> [SegRect; 3] {
    let (parent_left, parent_top) = node_origin(parent, band_left);
    let (child_left, child_top) = node_origin(child, band_left);

    let parent_cx = parent_left + NODE_WIDTH / 2.0;
    let child_cx = child_left + NODE_WIDTH / 2.0;
    let parent_bottom = parent_top + NODE_HEIGHT;
    let mid_y = (parent_bottom + child_top) / 2.0;
    let half = CONNECTOR_THICKNESS / 2.0;

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
        let (left, top) = node_origin(LayoutCell { depth: 0, x: 0.0 }, 0.0);
        assert_eq!(left, 0.0);
        assert_eq!(top, HEADER_HEIGHT);

        // A fractional column (a centred parent) maps to a fractional pixel offset.
        let (left, top) = node_origin(LayoutCell { depth: 2, x: 1.5 }, 50.0);
        assert_eq!(left, 50.0 + 1.5 * COL_PITCH);
        assert_eq!(top, HEADER_HEIGHT + 2.0 * ROW_PITCH);
    }

    /// An elbow between offset columns is three axis-aligned runs: two one-thickness
    /// verticals on the two node centre-columns and a one-thickness horizontal joining
    /// them, contiguous in y (parent stub bottom = child stub top = the midline).
    #[test]
    fn elbow_is_three_contiguous_axis_aligned_runs() {
        let parent = LayoutCell { depth: 0, x: 0.0 };
        let child = LayoutCell { depth: 1, x: 1.0 };
        let segs = elbow_segments(parent, child, 0.0);

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
        let segs = elbow_segments(parent, child, 10.0);

        assert_eq!(segs[0].left, segs[2].left, "both stubs share a column");
        assert_eq!(
            segs[1].width, CONNECTOR_THICKNESS,
            "the horizontal run is just the joint",
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
