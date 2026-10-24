//! The tf panel's two actions: showing and hiding the panel, and swapping its
//! layout. Each id sits beside the system that reacts to it.

use crate::viz::interaction::{
    actions::{
        handle::{ActionHandle, ActionId},
        registry::ActionRegistry,
    },
    sampling::ActionState,
    tf_panel::{geometry::PanelOrientation, TfPanelVisible},
};

use bevy::prelude::*;

/// The action that shows and hides the tf panel.
pub const TOGGLE_TF_PANEL: ActionId = ActionId("viz.toggle_tf_panel");

/// The action that swaps the tf panel between its sideways and top-down layouts.
pub const TOGGLE_TF_PANEL_ORIENTATION: ActionId = ActionId("viz.toggle_tf_panel_orientation");

/// Flips the panel's master visibility when `viz.toggle_tf_panel` fires.
///
/// Mirrors `toggle_tf_overlay`: the action handle can't change after startup, so
/// it is resolved once and cached in a `Local`. The `expect` is a startup-time
/// assertion, not a runtime path — the action is declared unconditionally in
/// `register_viz_actions`, so its absence is a wiring bug rather than a condition
/// to handle.
pub(crate) fn toggle_tf_panel(
    registry: Res<ActionRegistry>,
    state: Res<ActionState>,
    mut panel: ResMut<TfPanelVisible>,
    mut handle: Local<Option<ActionHandle>>,
) {
    let h = *handle.get_or_insert_with(|| registry.handle(TOGGLE_TF_PANEL).expect("registered"));

    if state.is_active(h) {
        panel.0 = !panel.0;
    }
}

/// Flips [`PanelOrientation`] between `Sideways` and `TopDown` when
/// `viz.toggle_tf_panel_orientation` fires, so the two layouts swap on one live tree
/// without a rebuild. Mirrors [`toggle_tf_panel`]: the handle can't change after
/// startup, so it is resolved once into a `Local`, and the `expect` is a startup-time
/// assertion — the action is declared unconditionally in `register_viz_actions`.
pub(super) fn toggle_tf_panel_orientation(
    registry: Res<ActionRegistry>,
    state: Res<ActionState>,
    mut orientation: ResMut<PanelOrientation>,
    mut handle: Local<Option<ActionHandle>>,
) {
    let h = *handle.get_or_insert_with(|| {
        registry
            .handle(TOGGLE_TF_PANEL_ORIENTATION)
            .expect("registered")
    });

    if state.is_active(h) {
        *orientation = match *orientation {
            PanelOrientation::Sideways => PanelOrientation::TopDown,
            PanelOrientation::TopDown => PanelOrientation::Sideways,
        };
    }
}
