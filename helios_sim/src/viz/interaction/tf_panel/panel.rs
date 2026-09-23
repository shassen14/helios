//! The tf panel's screen-space container: a bounded, corner-docked `bevy_ui`
//! node, shown or hidden by [`TfPanelVisible`]. This is the empty shell; the
//! renderer hangs the frame and edge nodes under [`TfPanelRoot`] in a later
//! step. Presentation lives here, apart from the plugin wiring in `mod.rs`.

use super::TfPanelVisible;
use crate::viz::interaction::tuning::{require_positive, InteractionTuningError};

use bevy::input::mouse::{MouseScrollUnit, MouseWheel};
use bevy::prelude::*;
use bevy::ui::RelativeCursorPosition;
use serde::Deserialize;

/// Sparse TOML overrides for the panel dock's screen-space geometry and scroll feel.
/// Every field is optional; anything omitted falls back to the compiled-in
/// [`TfPanelDockTuning::default`]. `viewport_max_height` is authored as a bare number of
/// viewport-height units (vh) and wrapped into a [`Val::Vh`] on resolve.
#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct TfPanelDockTuningFile {
    pub margin: Option<f32>,
    pub width: Option<f32>,
    pub viewport_max_height: Option<f32>,
    pub padding: Option<f32>,
    pub header_gap: Option<f32>,
    pub bg: Option<[f32; 4]>,
    pub line_scroll: Option<f32>,
}

/// Resolved geometry and scroll feel for the dock — one operator preference for the
/// session, read by [`spawn_tf_panel`] and [`scroll_tf_panel`]. Defaults reproduce the
/// values compiled in before the tuning surface existed.
#[derive(Resource, Debug, Clone)]
pub struct TfPanelDockTuning {
    /// Inset of the dock from the top-left corner, px.
    pub margin: f32,
    /// Fixed dock width, px.
    pub width: f32,
    /// Height cap on the *scrolling* part only — the pinned header sits above it and is
    /// always visible, so the dock's total height is the header plus up to this.
    pub viewport_max_height: Val,
    /// Inner padding of the dock, px.
    pub padding: f32,
    /// Gap under the pinned header, separating it from the scrolling tree below, px.
    pub header_gap: f32,
    /// Semi-opaque dark backing so the dock reads against the 3D scene behind it,
    /// `[r, g, b, a]`.
    pub bg: Color,
    /// Pixels scrolled per wheel *line* (a mouse notch). Trackpads report pixels
    /// already, so their deltas pass through unscaled.
    pub line_scroll: f32,
}

impl Default for TfPanelDockTuning {
    fn default() -> Self {
        Self {
            margin: 10.0,
            width: 440.0,
            viewport_max_height: Val::Vh(70.0),
            padding: 8.0,
            header_gap: 6.0,
            bg: Color::srgba(0.05, 0.05, 0.08, 0.85),
            line_scroll: 20.0,
        }
    }
}

impl TfPanelDockTuning {
    /// Overlays sparse overrides onto [`Default`], packing the `[r, g, b, a]` backing
    /// into an sRGBA [`Color`] and the height cap into a [`Val::Vh`], and rejects a
    /// non-positive width, height cap, or scroll step (margins and padding may be zero —
    /// flush is a legitimate look).
    pub(crate) fn resolve(
        overrides: &TfPanelDockTuningFile,
    ) -> Result<Self, InteractionTuningError> {
        let mut t = Self::default();
        if let Some(v) = overrides.margin {
            t.margin = v;
        }
        if let Some(v) = overrides.width {
            t.width = v;
        }
        if let Some(v) = overrides.padding {
            t.padding = v;
        }
        if let Some(v) = overrides.header_gap {
            t.header_gap = v;
        }
        if let Some([r, g, b, a]) = overrides.bg {
            t.bg = Color::srgba(r, g, b, a);
        }
        if let Some(v) = overrides.line_scroll {
            t.line_scroll = v;
        }

        // The height cap is validated as a bare vh number before it is wrapped, since a
        // `Val` can't be range-checked once built.
        let vh = overrides.viewport_max_height.unwrap_or(70.0);
        require_positive("tf_panel.dock.viewport_max_height", vh)?;
        t.viewport_max_height = Val::Vh(vh);

        require_positive("tf_panel.dock.width", t.width)?;
        require_positive("tf_panel.dock.line_scroll", t.line_scroll)?;
        Ok(t)
    }
}

/// Marks the panel's UI root, so the visibility sync can find the one node to flip.
/// The root itself neither scrolls nor holds the tree directly — it stacks the pinned
/// [`TfPanelHeader`] above the scrolling [`TfPanelViewport`].
#[derive(Component)]
pub struct TfPanelRoot;

/// The pinned header strip at the top of the dock: the renderer writes the agent
/// name(s) here, outside the scroll region, so identity stays visible while the tree
/// below is panned away.
#[derive(Component)]
pub struct TfPanelHeader;

/// The scrolling region under the header: the renderer hangs the tree canvas here.
/// This is the node that carries `overflow: scroll`, its `ScrollPosition`, and the
/// hover test — so the header never moves and only the tree pans.
#[derive(Component)]
pub struct TfPanelViewport;

/// Spawns the empty panel dock once, at startup, hidden.
///
/// Anchored top-left and bounded: a fixed width with a viewport-relative height
/// cap and **scrolling** overflow, so a tree larger than the box is reachable by
/// scroll rather than clipped away — a wide sensor fan grows past the right edge,
/// a deep tree past the bottom, and [`scroll_tf_panel`] pans to the rest. Top-left,
/// not bottom-left, because the tree is drawn root-at-top and grows downward and
/// rightward — anchoring that corner lets it grow into the viewport, not off it.
/// `ScrollPosition` holds the pan offset; `RelativeCursorPosition` lets the scroll
/// system act only when the pointer is over this panel. Starts `Hidden` to match
/// [`TfPanelVisible`]'s default. A bare coloured node needs no font, so — unlike the
/// inspector's text panel — this spawns with no asset-server gate.
pub fn spawn_tf_panel(tuning: Res<TfPanelDockTuning>, mut commands: Commands) {
    // The pinned header: fixed at the top of the dock, filled by the renderer.
    let header = commands
        .spawn((
            TfPanelHeader,
            Node {
                width: Val::Percent(100.0),
                flex_direction: FlexDirection::Column,
                margin: UiRect::bottom(Val::Px(tuning.header_gap)),
                ..default()
            },
        ))
        .id();

    // The scrolling region: bounded in height, clips+scrolls its tree canvas. It owns
    // the scroll offset and the hover test so only this part moves.
    let viewport = commands
        .spawn((
            TfPanelViewport,
            Node {
                width: Val::Percent(100.0),
                max_height: tuning.viewport_max_height,
                overflow: Overflow::scroll(),
                ..default()
            },
            ScrollPosition::default(),
            RelativeCursorPosition::default(),
        ))
        .id();

    // The root sizes to header + viewport and never scrolls itself.
    commands
        .spawn((
            TfPanelRoot,
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(tuning.margin),
                top: Val::Px(tuning.margin),
                width: Val::Px(tuning.width),
                padding: UiRect::all(Val::Px(tuning.padding)),
                flex_direction: FlexDirection::Column,
                overflow: Overflow::clip(),
                ..default()
            },
            BackgroundColor(tuning.bg),
            Visibility::Hidden,
        ))
        .add_child(header)
        .add_child(viewport);
}

/// Pans the panel with the mouse wheel while the pointer is over it.
///
/// The dock's `overflow` is `scroll`, which clips its content but does not move on
/// its own; this feeds the wheel into its [`ScrollPosition`]. Only fires when the
/// cursor is over the panel (via [`RelativeCursorPosition`]) so the wheel still
/// zooms the 3D camera everywhere else. A plain wheel scrolls vertically (the common
/// case, a deep or many-sensored tree); holding `Shift` redirects it to horizontal,
/// which is how a wide sensor fan is reached without a horizontal wheel. Line-unit
/// deltas (a notch mouse) are scaled to pixels; trackpad pixel deltas pass through.
/// The offset is clamped at zero; Bevy clamps the upper bound to the content size.
pub fn scroll_tf_panel(
    tuning: Res<TfPanelDockTuning>,
    mut wheel: MessageReader<MouseWheel>,
    keys: Res<ButtonInput<KeyCode>>,
    mut viewport: Query<(&RelativeCursorPosition, &mut ScrollPosition), With<TfPanelViewport>>,
) {
    let Ok((cursor, mut scroll)) = viewport.single_mut() else {
        return;
    };
    if !cursor.cursor_over {
        return;
    }

    let mut dx = 0.0;
    let mut dy = 0.0;
    for event in wheel.read() {
        let scale = match event.unit {
            MouseScrollUnit::Line => tuning.line_scroll,
            MouseScrollUnit::Pixel => 1.0,
        };
        dx += event.x * scale;
        dy += event.y * scale;
    }

    // Shift turns a vertical wheel into a horizontal pan, so a mouse with only a
    // vertical wheel can still reach a wide fan.
    if keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight) {
        dx += dy;
        dy = 0.0;
    }

    scroll.0.x = (scroll.0.x - dx).max(0.0);
    scroll.0.y = (scroll.0.y - dy).max(0.0);
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
