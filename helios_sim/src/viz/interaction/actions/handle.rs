//! Action identity and description — the static half of an input action.
//!
//! An *action* is a named thing a user can do ("toggle the map", "zoom in"),
//! deliberately decoupled from the input that triggers it. A consumer holds an
//! [`ActionHandle`] and asks only "is this active?"; it never learns whether a
//! keyboard, a rebound key, or some future input source produced it. That
//! indirection is the whole point — the same action survives a rebind, and the
//! input source can change without touching a single consumer.
//!
//! This module owns the parts of an action that never change at runtime: its
//! id, its human description, how it is sampled, and the key it binds to when
//! nothing overrides it. The *live* pressed/released state is not here — it is
//! read fresh each frame by the system that samples input.

use bevy::input::keyboard::KeyCode;

/// Stable, namespaced name of an action, e.g. `"camera.orbit"`.
///
/// Always a compile-time literal — actions are declared in code, never invented
/// at runtime — so it borrows `'static` and stays `Copy`. The newtype keeps an
/// action name from being silently swapped with any other string, and gives the
/// `"namespace.verb"` convention one documented home.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct ActionId(pub &'static str);

/// How an action's input is sampled each frame — the difference between a
/// press and a hold, which decides whether a consumer reads a rising edge or a
/// held state.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum InputKind {
    /// Fires once, on the press edge. Read with `just_pressed`. Toggles and
    /// mode switches.
    Button,
    /// Active for as long as the input is held. Read with `pressed`. Continuous
    /// motion — pan, zoom, throttle.
    Axis,
}

/// An action's static description: everything about it *except* its live state.
///
/// `default_key` is the key the action binds to when no override exists. It is
/// strictly an *input* to keybinding resolution — nothing reads it to decide
/// live behavior, or a rebind would be silently ignored.
#[derive(Clone, Debug)]
pub struct ActionMetadata {
    /// Human-facing name, for a menu or legend.
    pub label: &'static str,
    /// Grouping bucket for presentation, e.g. `"camera"`, `"viz"`.
    pub group: &'static str,
    /// Edge vs. held sampling.
    pub kind: InputKind,
    /// The binding used when nothing overrides it.
    pub default_key: KeyCode,
}

/// Opaque token identifying a registered action.
///
/// Indexes the registry's action table, so a consumer resolves an action's
/// state in O(1) without re-hashing its name every frame. Only
/// [`ActionRegistry::register`] can mint one, and the table is append-only, so
/// the index is always valid — a handle never dangles.
///
/// [`ActionRegistry::register`]: super::registry::ActionRegistry::register
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct ActionHandle(pub(super) usize);

impl ActionHandle {
    /// The handle's position in the registry table, for indexing the parallel
    /// tables built against it — `KeyBindings` and the per-frame active set.
    ///
    /// Exposes *reading* the index without exposing *minting* a handle: the
    /// field stays crate-private so only [`ActionRegistry::register`] can create
    /// one, keeping every handle backed by a real entry.
    ///
    /// [`ActionRegistry::register`]: super::registry::ActionRegistry::register
    pub(crate) fn index(&self) -> usize {
        self.0
    }
}
