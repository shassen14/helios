//! The action catalog: every registered action plus a name→handle index.
//!
//! Domain plugins populate an [`ActionRegistry`] at startup, each registering
//! its own actions next to the code that uses them. Consumers then look actions
//! up by [`ActionHandle`]; the keybinding loader looks them up by name (the
//! names it reads from a config file are runtime strings). The registry holds
//! only descriptions and default keys — never live input — so "what an action
//! is" and "which key is bound" resolve in one visible place instead of being
//! scattered through the systems that react to them.
//!
//! The table is append-only: actions are declared once, never removed, so a
//! handle's index stays valid for the life of the registry.

use super::handle::{ActionHandle, ActionId, ActionMetadata};

use bevy::ecs::resource::Resource;
use std::collections::HashMap;

/// The registered actions, keyed for the two ways they get looked up.
#[derive(Resource, Default)]
pub struct ActionRegistry {
    /// The action table. `ActionHandle(i)` indexes `entries[i]`.
    entries: Vec<ActionEntry>,
    /// name → handle. Keyed on the raw `&'static str` (not [`ActionId`]) so a
    /// lookup with a runtime `&str` — a name read from a keybinding file —
    /// matches on the string's *contents* via `&'static str: Borrow<str>`, with
    /// no allocation and no scan.
    by_name: HashMap<&'static str, ActionHandle>,
}

impl ActionRegistry {
    /// Register an action and return a handle for O(1) lookups.
    ///
    /// Panics on a duplicate id. Registration happens once at startup, and two
    /// actions claiming one name is a configuration bug — silently returning the
    /// first would drop the second's metadata and hand back a handle to the
    /// wrong action. A startup panic is the right failure; a runtime one would
    /// not be.
    pub fn register(&mut self, id: ActionId, meta: ActionMetadata) -> ActionHandle {
        assert!(
            !self.by_name.contains_key(id.0),
            "duplicate action id registered: {:?}",
            id.0
        );

        let handle = ActionHandle(self.entries.len());

        self.entries.push(ActionEntry { id, meta });
        self.by_name.insert(id.0, handle);

        handle
    }

    /// Resolve a compile-time id to its handle, or `None` if unregistered.
    pub fn handle(&self, id: ActionId) -> Option<ActionHandle> {
        self.handle_by_name(id.0)
    }

    /// Resolve a name that arrived as a runtime string — e.g. a key binding
    /// read from config — to its handle. Matches on string contents, so no
    /// `ActionId` (and thus no `'static` name) is needed to look one up.
    pub fn handle_by_name(&self, name: &str) -> Option<ActionHandle> {
        self.by_name.get(name).copied()
    }

    /// The static description behind a handle. Infallible: a handle can only
    /// come from [`register`](Self::register) and the table never shrinks, so
    /// its index is always in range.
    pub fn metadata(&self, handle: ActionHandle) -> &ActionMetadata {
        &self.entries[handle.0].meta
    }

    /// Every action in registration order, as `(handle, id, metadata)`. The
    /// keybinding loader walks this to seed each action's default key into the
    /// resolved config, so the defaults are visible rather than hidden in code.
    pub fn iter(&self) -> impl Iterator<Item = (ActionHandle, ActionId, &ActionMetadata)> {
        self.entries
            .iter()
            .enumerate()
            .map(|(i, e)| (ActionHandle(i), e.id, &e.meta))
    }
}

/// One row of the action table: an id kept beside its metadata, so [`iter`]
/// can report the name a handle stands for.
///
/// [`iter`]: ActionRegistry::iter
struct ActionEntry {
    id: ActionId,
    meta: ActionMetadata,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::viz::interaction::actions::handle::InputKind;

    use bevy::input::keyboard::KeyCode;

    fn button(label: &'static str, group: &'static str, key: KeyCode) -> ActionMetadata {
        ActionMetadata {
            label,
            group,
            kind: InputKind::Button,
            default_key: key,
        }
    }

    #[test]
    fn register_returns_handle_resolvable_by_id_and_by_name() {
        let mut reg = ActionRegistry::default();
        let handle = reg.register(
            ActionId("viz.toggle_map"),
            button("Toggle map", "viz", KeyCode::KeyM),
        );

        // Both lookup paths land on the same handle: the code-side id and the
        // runtime string a keybinding file would carry.
        assert_eq!(reg.handle(ActionId("viz.toggle_map")), Some(handle));
        assert_eq!(reg.handle_by_name("viz.toggle_map"), Some(handle));
    }

    #[test]
    fn unregistered_name_resolves_to_none() {
        let reg = ActionRegistry::default();
        assert_eq!(reg.handle(ActionId("viz.nonexistent")), None);
        assert_eq!(reg.handle_by_name("viz.nonexistent"), None);
    }

    #[test]
    fn metadata_round_trips_through_handle() {
        let mut reg = ActionRegistry::default();
        let handle = reg.register(
            ActionId("camera.zoom_in"),
            ActionMetadata {
                label: "Zoom in",
                group: "camera",
                kind: InputKind::Axis,
                default_key: KeyCode::Equal,
            },
        );

        let meta = reg.metadata(handle);
        assert_eq!(meta.label, "Zoom in");
        assert_eq!(meta.group, "camera");
        assert_eq!(meta.kind, InputKind::Axis);
        assert_eq!(meta.default_key, KeyCode::Equal);
    }

    #[test]
    fn handles_are_distinct_and_iter_reports_registration_order() {
        let mut reg = ActionRegistry::default();
        let first = reg.register(
            ActionId("viz.toggle_map"),
            button("Toggle map", "viz", KeyCode::KeyM),
        );
        let second = reg.register(
            ActionId("camera.zoom_in"),
            button("Zoom in", "camera", KeyCode::Equal),
        );

        assert_ne!(first, second);

        let seen: Vec<_> = reg.iter().map(|(handle, id, _)| (handle, id)).collect();
        assert_eq!(
            seen,
            vec![
                (first, ActionId("viz.toggle_map")),
                (second, ActionId("camera.zoom_in")),
            ],
        );
    }

    #[test]
    #[should_panic(expected = "duplicate action id")]
    fn duplicate_id_panics() {
        let mut reg = ActionRegistry::default();
        reg.register(
            ActionId("viz.toggle_map"),
            button("Toggle map", "viz", KeyCode::KeyM),
        );
        // Same id again — a startup misconfiguration, must not silently dedupe.
        reg.register(
            ActionId("viz.toggle_map"),
            button("Toggle map again", "viz", KeyCode::KeyN),
        );
    }
}
