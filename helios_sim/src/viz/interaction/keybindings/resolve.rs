//! The pure heart of keybinding resolution: registry + overrides → one
//! complete, conflict-free key for every action.
//!
//! This is where the "no invisible defaults" invariant is made literal. The
//! input overrides are sparse — only the actions a user chose to rebind — but
//! the resolved [`KeyBindings`] covers *every* registered action, filling each
//! gap from the registry's `default_key`. So the defaults become visible in the
//! resolved set (and thus in the config dump) even though the file never named
//! them. The resolution logic takes plain data and returns plain data — no
//! `App`, no filesystem — which is what keeps it unit-testable.

use crate::viz::interaction::actions::handle::{ActionHandle, ActionId};
use crate::viz::interaction::actions::registry::ActionRegistry;

use bevy::prelude::*;
use std::collections::HashMap;
use std::fmt::Display;

/// Every action's resolved key, indexed by `ActionHandle.0`.
///
/// Built by `resolve_bindings` in registration order, so the same index that
/// identifies an action in the registry identifies its key here — the sampling
/// system reads this table by handle without re-hashing a name each frame.
/// Complete by construction: one entry per registered action, never sparse.
#[derive(Resource, Debug)]
pub struct KeyBindings {
    /// Resolved key per action; `by_action[i]` is the key for `ActionHandle(i)`.
    by_action: Vec<KeyCode>,
}

impl KeyBindings {
    /// The resolved key for `handle`. Infallible: the table has one entry per
    /// registered action and a handle can only name a registered action, so its
    /// index is always in range — the same guarantee that lets the registry's
    /// `metadata` skip a bounds check.
    pub(crate) fn key_for(&self, handle: ActionHandle) -> KeyCode {
        self.by_action[handle.index()]
    }
}

#[cfg(test)]
impl KeyBindings {
    /// Test-only constructor: a binding table straight from a per-handle key
    /// vec, skipping resolution. `keys[i]` is the key for `ActionHandle(i)`, so
    /// callers align it with the order they registered actions in.
    pub(crate) fn from_action_keys(keys: Vec<KeyCode>) -> Self {
        Self { by_action: keys }
    }
}

/// A keybinding config that can't be honored — always a startup misconfiguration,
/// so the loader turns it into a startup panic rather than degrading silently.
#[derive(Debug)]
pub enum BindingError {
    /// Two actions resolved to the same key. Which action "wins" would be
    /// arbitrary, so this is rejected rather than guessed.
    Conflict {
        key: KeyCode,
        first: ActionId,
        second: ActionId,
    },
    /// An override named an action that was never registered — typically a
    /// typo in the config. Ignoring it would be an invisible failure.
    UnknownAction(String),
}

impl Display for BindingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BindingError::Conflict { key, first, second } => write!(
                f,
                "key {key:?} is bound to both '{}' and '{}'",
                first.0, second.0
            ),
            BindingError::UnknownAction(name) => {
                write!(f, "keybinding names unknown action '{}'", name)
            }
        }
    }
}

/// Resolve every registered action to a key, applying overrides over defaults.
///
/// Two passes: first reject any override that names an unregistered action
/// (fail fast on a typo before building anything), then walk the registry in
/// order, taking each action's override if present or its `default_key`
/// otherwise, and reject the first key claimed by two actions. On success the
/// returned [`KeyBindings`] has exactly one entry per action, aligned to handle
/// index.
pub(super) fn resolve_bindings(
    registry: &ActionRegistry,
    overrides: &HashMap<String, KeyCode>,
) -> Result<KeyBindings, BindingError> {
    for name in overrides.keys() {
        if registry.handle_by_name(name).is_none() {
            return Err(BindingError::UnknownAction(name.clone()));
        }
    }

    let mut by_action: Vec<KeyCode> = Vec::new();
    let mut seen: HashMap<KeyCode, ActionId> = HashMap::new();

    for (_, id, meta) in registry.iter() {
        let key = overrides.get(id.0).copied().unwrap_or(meta.default_key);

        if let Some(&first) = seen.get(&key) {
            return Err(BindingError::Conflict {
                key,
                first,
                second: id,
            });
        }

        seen.insert(key, id);
        by_action.push(key);
    }

    Ok(KeyBindings { by_action })
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::viz::interaction::actions::handle::{ActionMetadata, InputKind};

    fn button(default_key: KeyCode) -> ActionMetadata {
        ActionMetadata {
            label: "test",
            group: "test",
            kind: InputKind::Button,
            default_key,
        }
    }

    /// Two actions registered in a known order; index 0 is `viz.toggle_map`,
    /// index 1 is `camera.zoom_in`.
    fn two_action_registry() -> ActionRegistry {
        let mut registry = ActionRegistry::default();
        registry.register(ActionId("viz.toggle_map"), button(KeyCode::KeyM));
        registry.register(ActionId("camera.zoom_in"), button(KeyCode::Equal));
        registry
    }

    #[test]
    fn defaults_fill_every_action_when_no_overrides() {
        let registry = two_action_registry();

        let bindings = resolve_bindings(&registry, &HashMap::new()).expect("resolves");

        // Each action falls back to its registered default, in order.
        assert_eq!(bindings.by_action, vec![KeyCode::KeyM, KeyCode::Equal]);
    }

    #[test]
    fn override_replaces_only_the_named_action() {
        let registry = two_action_registry();
        let overrides = HashMap::from([("viz.toggle_map".to_string(), KeyCode::KeyT)]);

        let bindings = resolve_bindings(&registry, &overrides).expect("resolves");

        // First action rebound to T; the untouched second keeps its default.
        assert_eq!(bindings.by_action, vec![KeyCode::KeyT, KeyCode::Equal]);
    }

    #[test]
    fn resolved_set_covers_every_registered_action() {
        let registry = two_action_registry();

        let bindings = resolve_bindings(&registry, &HashMap::new()).expect("resolves");

        // Complete, never sparse: one key per action regardless of overrides.
        assert_eq!(bindings.by_action.len(), registry.iter().count());
    }

    #[test]
    fn two_actions_on_one_key_is_a_conflict() {
        let registry = two_action_registry();
        // Rebind camera.zoom_in onto M, which viz.toggle_map already holds.
        let overrides = HashMap::from([("camera.zoom_in".to_string(), KeyCode::KeyM)]);

        let err = resolve_bindings(&registry, &overrides).expect_err("should conflict");

        assert!(matches!(
            err,
            BindingError::Conflict {
                key: KeyCode::KeyM,
                ..
            }
        ));
    }

    #[test]
    fn override_naming_unknown_action_is_rejected() {
        let registry = two_action_registry();
        let overrides = HashMap::from([("viz.nonexistent".to_string(), KeyCode::KeyX)]);

        let err = resolve_bindings(&registry, &overrides).expect_err("should reject unknown");

        assert!(matches!(err, BindingError::UnknownAction(name) if name == "viz.nonexistent"));
    }
}
