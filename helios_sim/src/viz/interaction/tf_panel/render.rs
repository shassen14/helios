//! Draws the tf panel model as `bevy_ui` nodes under the panel dock.
//!
//! This is the dumb half of the panel: it walks [`TfPanelModel`] and paints it,
//! computing nothing. All interpretation — layout cells, health verdicts, rates —
//! already happened in the gather step, and the cell→pixel geometry lives next door in
//! [`geometry`](super::geometry) ([`node_origin`], [`elbow_segments`], [`graph_extent`],
//! [`edge_color`], all pure and Tier-1 tested). What is left here is untested ECS glue:
//! map each placed rectangle to an absolute `Node`, spawn the boxes and labels, and
//! despawn the previous frame's draw.
//!
//! The dock container ([`TfPanelRoot`](super::panel::TfPanelRoot)) is persistent — a
//! visibility flip, not a respawn — but its contents are rebuilt wholesale each visible
//! frame (despawn-and-respawn), matching the inspector's renderer. The tree is tiny and
//! the panel is gated off unless shown, so a reconciler is not worth its bug surface yet.

use crate::viz::interaction::tf_panel::geometry::{
    edge_color, elbow_segments, graph_extent, node_origin, PanelOrientation, NODE_HEIGHT,
    NODE_WIDTH,
};
use crate::viz::interaction::tf_panel::layout::LayoutCell;
use crate::viz::interaction::tf_panel::model::AgentGraph;
use crate::viz::interaction::tf_panel::panel::{TfPanelHeader, TfPanelViewport};
use crate::viz::interaction::tf_panel::TfPanelModel;

use helios_core::frames::FrameId;

use bevy::prelude::*;
use std::collections::HashMap;

// Interim presentation constants, named rather than inlined — no bare magic numbers in
// the node. A later `[tf_panel]` config surface lifts these out of source, the same move
// the camera-rate consts are headed for. The geometry constants (node size, pitch,
// colours) live beside the geometry fns in `geometry.rs`; these are the ones only the
// spawn shell touches.
/// Horizontal gap between one agent's band and the next (multi-agent layout).
const BAND_GAP: f32 = 28.0;
const NODE_PADDING: f32 = 4.0;
/// Small enough that a leaf name (`sensor.gps.primary`) sits on one line in the box.
const LABEL_FONT_SIZE: f32 = 10.0;
const HEADER_FONT_SIZE: f32 = 13.0;

const NODE_BG: Color = Color::srgba(0.12, 0.13, 0.18, 0.95);
const LABEL_COLOR: Color = Color::srgb(0.85, 0.88, 0.95);
/// Accent for the per-agent header, shared with the inspector's section titles.
const HEADER_COLOR: Color = Color::srgb(0.62, 0.80, 1.0);

/// Marks everything the renderer spawns, so the previous frame's whole draw can be
/// found and despawned in one query before the next is built. One canvas per rebuild
/// carries this; its node/connector/label children fall with it (recursive despawn).
#[derive(Component)]
pub struct TfPanelContent;

/// Rebuilds the panel's `bevy_ui` contents from [`TfPanelModel`] each visible frame.
///
/// Despawns last frame's canvas wholesale, then — unless the model is empty — spawns
/// a fresh one under the viewport and lays every agent graph into it. Agents are placed
/// in side-by-side bands: since a graph's cells are positioned within its own frame,
/// letting two agents share the cross axis would cross their connectors, so each
/// agent's whole graph gets its own band offset.
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
