//! The tf panel's screen-space container: a bounded, corner-docked `bevy_ui`
//! node, shown or hidden by [`TfPanelVisible`]. This is the empty shell; the
//! renderer hangs the frame and edge nodes under [`TfPanelRoot`] in a later
//! step. Presentation lives here, apart from the plugin wiring in `mod.rs`.

use super::TfPanelVisible;

use bevy::prelude::*;

// Interim presentation constants. A later `[tf_panel]` config surface lifts
// these out of source, the way the camera-rate consts are headed; until then
// they are named here rather than inlined — no bare magic numbers in the node.
const PANEL_MARGIN: f32 = 10.0;
const PANEL_WIDTH: f32 = 360.0;
const PANEL_MAX_HEIGHT: Val = Val::Vh(60.0);
const PANEL_PADDING: f32 = 8.0;
/// Semi-opaque dark backing so the empty dock reads against the 3D scene behind it.
const PANEL_BG: Color = Color::srgba(0.05, 0.05, 0.08, 0.85);

/// Marks the panel's UI root, so the visibility sync can find the one node to
/// flip and the renderer can hang the frame and edge nodes under it.
#[derive(Component)]
pub struct TfPanelRoot;

/// Spawns the empty panel dock once, at startup, hidden.
///
/// Anchored bottom-left and bounded: a fixed width with a viewport-relative
/// height cap and clipped overflow, so a large tree clips inside the box instead
/// of overrunning the viewport. Starts `Hidden` to match [`TfPanelVisible`]'s
/// default; `sync_tf_panel_visibility` reveals it on toggle. A bare coloured node
/// needs no font, so — unlike the inspector's text panel — this spawns with no
/// asset-server gate.
pub fn spawn_tf_panel(mut commands: Commands) {
    commands.spawn((
        TfPanelRoot,
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(PANEL_MARGIN),
            bottom: Val::Px(PANEL_MARGIN),
            width: Val::Px(PANEL_WIDTH),
            max_height: PANEL_MAX_HEIGHT,
            padding: UiRect::all(Val::Px(PANEL_PADDING)),
            flex_direction: FlexDirection::Column,
            overflow: Overflow::clip(),
            ..default()
        },
        BackgroundColor(PANEL_BG),
        Visibility::Hidden,
    ));
}

/// Drives the panel dock's visibility from the master toggle.
///
/// [`TfPanelRoot`] is a persistent entity — spawned once and kept — so showing
/// and hiding it is a `Visibility` flip, not a respawn (contrast the inspector,
/// which despawns and rebuilds its panel every frame). Chained after
/// `toggle_tf_panel`, so a keypress that flips [`TfPanelVisible`] takes effect
/// the same frame. The `single_mut` is fallible by design: before
/// `spawn_tf_panel` has run, or in a headless test that skips it, no root exists
/// yet, so a missing one is skipped rather than unwrapped.
pub fn sync_tf_panel_visibility(
    visible: Res<TfPanelVisible>,
    mut root: Query<&mut Visibility, With<TfPanelRoot>>,
) {
    let Ok(mut visibility) = root.single_mut() else {
        return;
    };

    *visibility = if visible.0 {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
}
