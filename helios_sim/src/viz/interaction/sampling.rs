//! Per-frame input sampling — the keyboard read through the binding table into
//! the set of actions active this frame.
//!
//! This is the live half of the action system, the counterpart to the static
//! [`registry`](super::actions::registry). Each frame `sample_actions` walks
//! the registry, looks up each action's resolved key, and asks `ButtonInput`
//! whether it fired — an edge for a `Button`, a held state for an `Axis` —
//! collecting the winners into [`ActionState`]. Consumers then ask
//! `ActionState::is_active(handle)` and never touch a `KeyCode`: that is the
//! whole point of the indirection — a rebind or a different input source changes
//! what lands in [`ActionState`], not a single line of any consumer.

use crate::viz::interaction::{
    actions::{
        handle::{ActionHandle, InputKind},
        registry::ActionRegistry,
    },
    keybindings::resolve::KeyBindings,
};

use bevy::prelude::*;
use std::collections::HashSet;

/// The actions firing this frame, identified by handle.
///
/// Rebuilt from scratch every frame by `sample_actions`, so it is always the
/// current frame's truth and never carries a stale press. Keyed by
/// [`ActionHandle`] rather than by name, so a consumer holding a handle resolves
/// "is my action active?" in a single set lookup, with no string to re-hash.
#[derive(Resource, Default)]
pub struct ActionState {
    active: HashSet<ActionHandle>,
}

impl ActionState {
    /// Whether `handle`'s action fired this frame — the one question a consumer
    /// asks. It learns nothing about which key or device produced the action.
    pub fn is_active(&self, handle: ActionHandle) -> bool {
        self.active.contains(&handle)
    }
}

#[cfg(test)]
impl ActionState {
    /// Test-only constructor: an [`ActionState`] with exactly the given handles
    /// marked active, so a consumer can be exercised without running the sampler
    /// or fabricating keyboard input.
    pub(crate) fn from_active(handles: impl IntoIterator<Item = ActionHandle>) -> Self {
        Self {
            active: handles.into_iter().collect(),
        }
    }
}

/// Sample every registered action's bound key into [`ActionState`] for this
/// frame.
///
/// Runs in `InteractionSet::Sampling`, ordered before the viz consumers so they
/// read this frame's state rather than last frame's. A `Button` action is
/// sampled on the press edge (`just_pressed`), an `Axis` on the held state
/// (`pressed`) — the distinction the action declared through its [`InputKind`].
/// The set is cleared and refilled each call, so a released key simply stops
/// appearing next frame.
pub(crate) fn sample_actions(
    registry: Res<ActionRegistry>,
    bindings: Res<KeyBindings>,
    keys: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<ActionState>,
) {
    state.active.clear();
    for (handle, _id, meta) in registry.iter() {
        let key = bindings.key_for(handle);
        let fired = match meta.kind {
            InputKind::Button => keys.just_pressed(key),
            InputKind::Axis => keys.pressed(key),
        };

        if fired {
            state.active.insert(handle);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::viz::interaction::actions::handle::{ActionId, ActionMetadata};

    fn button(default_key: KeyCode) -> ActionMetadata {
        ActionMetadata {
            label: "test",
            group: "test",
            kind: InputKind::Button,
            default_key,
        }
    }

    /// A registry with one `Button` action bound to `key`, plus the aligned
    /// binding table. The returned handle is that action's.
    fn single_button(key: KeyCode) -> (ActionRegistry, KeyBindings, ActionHandle) {
        let mut registry = ActionRegistry::default();
        let handle = registry.register(ActionId("viz.toggle_map"), button(key));
        let bindings = KeyBindings::from_action_keys(vec![key]);
        (registry, bindings, handle)
    }

    /// A minimal `App` holding the sampler and its inputs, with `pressed` (if
    /// any) held down on the keyboard resource.
    fn sampling_app(
        registry: ActionRegistry,
        bindings: KeyBindings,
        pressed: Option<KeyCode>,
    ) -> App {
        let mut app = App::new();
        let mut keys = ButtonInput::<KeyCode>::default();
        if let Some(key) = pressed {
            keys.press(key);
        }
        app.insert_resource(registry);
        app.insert_resource(bindings);
        app.insert_resource(keys);
        app.init_resource::<ActionState>();
        app.add_systems(Update, sample_actions);
        app
    }

    #[test]
    fn pressing_a_bound_key_activates_its_action() {
        let (registry, bindings, handle) = single_button(KeyCode::KeyM);
        let mut app = sampling_app(registry, bindings, Some(KeyCode::KeyM));

        app.update();

        assert!(app.world().resource::<ActionState>().is_active(handle));
    }

    #[test]
    fn an_unrelated_key_activates_nothing() {
        let (registry, bindings, handle) = single_button(KeyCode::KeyM);
        // A different key is down; the action's own key is not.
        let mut app = sampling_app(registry, bindings, Some(KeyCode::KeyT));

        app.update();

        assert!(!app.world().resource::<ActionState>().is_active(handle));
    }

    #[test]
    fn a_spent_edge_clears_the_action_next_frame() {
        let (registry, bindings, handle) = single_button(KeyCode::KeyM);
        let mut app = sampling_app(registry, bindings, Some(KeyCode::KeyM));

        app.update();
        assert!(app.world().resource::<ActionState>().is_active(handle));

        // Bevy retires edge state between frames; without an input plugin we do
        // it by hand. The `Button` is no longer "just pressed", so the rebuilt
        // set drops it — proof `ActionState` is never sticky.
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .clear();
        app.update();
        assert!(!app.world().resource::<ActionState>().is_active(handle));
    }
}
