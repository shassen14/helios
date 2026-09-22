//! The 2D tf topology/health panel: a screen-space `bevy_ui` dock showing an
//! agent's estimated transform tree. This module is its wiring — the master
//! visibility toggle and the plugin that installs it. The pure hierarchy
//! geometry lives in [`layout`], the dock container and its show/hide in
//! [`panel`]; the renderer that fills the dock arrives with a later step.

pub mod layout;
pub mod panel;

use crate::{
    prelude::AppState,
    viz::{
        interaction::{
            actions::{
                handle::{ActionHandle, ActionId},
                registry::ActionRegistry,
            },
            sampling::ActionState,
        },
        VizSet,
    },
};

use bevy::prelude::*;

/// Installs the tf panel: its visibility resource, the one-shot dock spawn, and
/// the per-frame toggle-then-apply pair.
pub struct TfPanelPlugin;

impl Plugin for TfPanelPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TfPanelVisible>();
        app.add_systems(Startup, panel::spawn_tf_panel);
        // Toggle before sync in one chain, so a keypress flips `TfPanelVisible`
        // and the dock's `Visibility` updates the same frame, not a frame late.
        app.add_systems(
            Update,
            (toggle_tf_panel, panel::sync_tf_panel_visibility)
                .chain()
                .in_set(VizSet::Live)
                .run_if(in_state(AppState::Running)),
        );
    }
}

/// The panel's master on/off, flipped by [`toggle_tf_panel`] and applied to the
/// dock by [`panel::sync_tf_panel_visibility`]. Off by default — the panel is
/// opt-in, like the 3D tf overlay.
#[derive(Default, Resource)]
pub struct TfPanelVisible(pub bool);

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
    let h = *handle.get_or_insert_with(|| {
        registry
            .handle(ActionId("viz.toggle_tf_panel"))
            .expect("registered")
    });

    if state.is_active(h) {
        panel.0 = !panel.0;
    }
}

#[cfg(test)]
mod tests {
    use super::panel::{sync_tf_panel_visibility, TfPanelRoot};
    use super::{TfPanelPlugin, TfPanelVisible};

    use crate::prelude::AppState;

    use bevy::prelude::*;
    use bevy::state::app::StatesPlugin;

    /// Tier-3 wiring guard: the plugin must initialise the master resource and
    /// spawn exactly one, hidden, panel root at startup. Dropping the
    /// `init_resource` or the `Startup` spawn still compiles; this catches it. The
    /// app sits in a non-`Running` state so the `Update` chain is gated off — the
    /// guard then needs no `ActionRegistry`/`ActionState`, staying scoped to the
    /// startup wiring it checks.
    #[test]
    fn plugin_inits_resource_and_spawns_hidden_dock() {
        let mut app = App::new();
        app.add_plugins(StatesPlugin);
        app.insert_state(AppState::AssetLoading);
        app.add_plugins(TfPanelPlugin);

        app.update();

        assert!(
            !app.world().resource::<TfPanelVisible>().0,
            "the panel is off by default",
        );

        let mut roots = app
            .world_mut()
            .query_filtered::<&Visibility, With<TfPanelRoot>>();
        let spawned: Vec<Visibility> = roots.iter(app.world()).copied().collect();
        assert_eq!(spawned.len(), 1, "startup spawns exactly one panel root");
        assert_eq!(spawned[0], Visibility::Hidden, "the dock starts hidden");
    }

    /// The show/hide logic in isolation: `sync_tf_panel_visibility` must drive the
    /// root's `Visibility` from the resource each frame, in both directions. This
    /// is the step-6 behaviour without the action plumbing — spawn a root, flip
    /// the resource, run the one system, watch the `Visibility` follow.
    #[test]
    fn sync_drives_dock_visibility_from_resource() {
        let mut app = App::new();
        app.insert_resource(TfPanelVisible(true));
        let root = app.world_mut().spawn((TfPanelRoot, Visibility::Hidden)).id();
        app.add_systems(Update, sync_tf_panel_visibility);

        app.update();
        assert_eq!(
            *app.world()
                .get::<Visibility>(root)
                .expect("root has a Visibility"),
            Visibility::Visible,
            "a set resource reveals the dock",
        );

        app.world_mut().resource_mut::<TfPanelVisible>().0 = false;
        app.update();
        assert_eq!(
            *app.world()
                .get::<Visibility>(root)
                .expect("root has a Visibility"),
            Visibility::Hidden,
            "clearing the resource hides the dock",
        );
    }
}
