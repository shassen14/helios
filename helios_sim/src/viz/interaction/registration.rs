//! Viz-domain action declarations.
//!
//! One registration system per plugin is the rule: this owns every action the
//! viz layer defines, registered next to the feature that reacts to it. The
//! camera and teleop plugins bring their own `register_*` systems into the same
//! [`InteractionSet::Registration`](super::InteractionSet::Registration) set
//! rather than extending this one — so no single file lists every app-wide
//! action, and adding a viz action never touches camera or teleop code.

use crate::viz::{
    interaction::{
        actions::{
            handle::{ActionMetadata, InputKind},
            registry::ActionRegistry,
        },
        tf_panel::{TOGGLE_TF_PANEL, TOGGLE_TF_PANEL_ORIENTATION},
    },
    live::{
        bounding_boxes::TOGGLE_BOUNDING_BOXES, colliders::TOGGLE_COLLIDERS, map::TOGGLE_MAP,
        point_cloud::TOGGLE_POINT_CLOUD, tf::TOGGLE_TF,
    },
};

use bevy::{ecs::system::ResMut, input::keyboard::KeyCode};

/// The group every viz-layer action is listed under.
pub const VIZ_GROUP: &str = "viz";

/// Register every viz-layer action into the shared [`ActionRegistry`].
///
/// Runs once at `Startup` in
/// [`InteractionSet::Registration`](super::InteractionSet::Registration), after
/// [`ActionRegistryPlugin`](super::ActionRegistryPlugin) has inserted the empty
/// registry. Declaring a viz action is one `register` call here; the returned
/// handle is intentionally dropped until a later commit wires key sampling to
/// it.
pub(crate) fn register_viz_actions(mut registry: ResMut<ActionRegistry>) {
    registry.register(
        TOGGLE_MAP,
        ActionMetadata {
            label: "Toggle map",
            group: VIZ_GROUP,
            kind: InputKind::Button,
            default_key: KeyCode::KeyM,
        },
    );
    registry.register(
        TOGGLE_TF,
        ActionMetadata {
            label: "Toggle tf overlay",
            group: VIZ_GROUP,
            kind: InputKind::Button,
            default_key: KeyCode::KeyT,
        },
    );
    registry.register(
        TOGGLE_TF_PANEL,
        ActionMetadata {
            label: "Toggle tf panel",
            group: VIZ_GROUP,
            kind: InputKind::Button,
            default_key: KeyCode::KeyG,
        },
    );
    registry.register(
        TOGGLE_TF_PANEL_ORIENTATION,
        ActionMetadata {
            label: "Toggle tf panel orientation",
            group: VIZ_GROUP,
            kind: InputKind::Button,
            default_key: KeyCode::KeyO,
        },
    );
    registry.register(
        TOGGLE_COLLIDERS,
        ActionMetadata {
            label: "Toggle colliders",
            group: VIZ_GROUP,
            kind: InputKind::Button,
            default_key: KeyCode::KeyC,
        },
    );
    registry.register(
        TOGGLE_BOUNDING_BOXES,
        ActionMetadata {
            label: "Toggle bounding boxes",
            group: VIZ_GROUP,
            kind: InputKind::Button,
            default_key: KeyCode::KeyB,
        },
    );
    registry.register(
        TOGGLE_POINT_CLOUD,
        ActionMetadata {
            label: "Toggle point cloud",
            group: VIZ_GROUP,
            kind: InputKind::Button,
            default_key: KeyCode::KeyL,
        },
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    use bevy::prelude::*;

    /// Tier-3 wiring guard: the failure this catches is someone dropping the
    /// `add_systems(Startup, register_viz_actions…)` line — the code still
    /// compiles, but no viz action is ever declared. The test stands up only the
    /// registry resource and this one system, deliberately *not* the whole
    /// `ActionRegistryPlugin`: that plugin also boots the keybinding loader and
    /// the sampler, which pull in `Cli`, `KeyBindings`, and `ButtonInput` — none
    /// of them the thing under test, and all of them a reason for this guard to
    /// fail for the wrong reason.
    #[test]
    fn register_viz_actions_declares_toggle_map_at_startup() {
        let mut app = App::new();
        app.init_resource::<ActionRegistry>();
        app.add_systems(Startup, register_viz_actions);

        // The first `update()` runs the `Startup` schedule exactly once.
        app.update();

        let registry = app.world().resource::<ActionRegistry>();
        assert!(
            registry.handle(TOGGLE_MAP).is_some(),
            "register_viz_actions must declare viz.toggle_map at Startup",
        );
    }

    /// Tier-3 wiring guard: `toggle_colliders` looks up its handle on the first
    /// frame and panics if the action was never declared.
    #[test]
    fn register_viz_actions_declares_toggle_colliders_at_startup() {
        let mut app = App::new();
        app.init_resource::<ActionRegistry>();
        app.add_systems(Startup, register_viz_actions);

        app.update();

        let registry = app.world().resource::<ActionRegistry>();
        assert!(
            registry.handle(TOGGLE_COLLIDERS).is_some(),
            "register_viz_actions must declare viz.toggle_colliders at Startup",
        );
    }

    /// Tier-3 wiring guard: `toggle_bounding_boxes` looks up its handle on the
    /// first frame and panics if the action was never declared.
    #[test]
    fn register_viz_actions_declares_toggle_bounding_boxes_at_startup() {
        let mut app = App::new();
        app.init_resource::<ActionRegistry>();
        app.add_systems(Startup, register_viz_actions);

        app.update();

        let registry = app.world().resource::<ActionRegistry>();
        assert!(
            registry.handle(TOGGLE_BOUNDING_BOXES).is_some(),
            "register_viz_actions must declare viz.toggle_bounding_boxes at Startup",
        );
    }
}
