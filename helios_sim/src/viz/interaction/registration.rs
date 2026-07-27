//! Viz-domain action declarations.
//!
//! One registration system per plugin is the rule: this owns every action the
//! viz layer defines, registered next to the feature that reacts to it. The
//! camera and teleop plugins bring their own `register_*` systems into the same
//! [`InteractionSet::Registration`](super::InteractionSet::Registration) set
//! rather than extending this one — so no single file lists every app-wide
//! action, and adding a viz action never touches camera or teleop code.

use bevy::{ecs::system::ResMut, input::keyboard::KeyCode};

use crate::viz::interaction::actions::{
    handle::{ActionId, ActionMetadata, InputKind},
    registry::ActionRegistry,
};

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
        ActionId("viz.toggle_map"),
        ActionMetadata {
            label: "Toggle map",
            group: "viz",
            kind: InputKind::Button,
            default_key: KeyCode::KeyM,
        },
    );
}

#[cfg(test)]
mod tests {
    use super::register_viz_actions;

    use crate::viz::interaction::actions::handle::ActionId;
    use crate::viz::interaction::actions::registry::ActionRegistry;

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
            registry.handle(ActionId("viz.toggle_map")).is_some(),
            "register_viz_actions must declare viz.toggle_map at Startup",
        );
    }
}
