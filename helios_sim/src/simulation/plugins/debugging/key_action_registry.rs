// helios_sim/src/simulation/plugins/debugging/key_action_registry.rs
//
// KeyActionRegistry: a runtime list of registered debug toggle actions.
// Each plugin appends its own actions at startup via a startup system.
// `handle_debug_keybindings` and `update_legend_text` iterate the registry
// so they are automatically correct for any active profile.

use bevy::prelude::*;

/// Which field of `DebugVisualizationConfig` a `KeyAction` controls.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DebugToggle {
    Pose,
    Covariance,
    PointCloud,
    Velocity,
    ErrorLine,
    PathTrail,
    OccupancyGrid,
    TfFrames,
    PlannedPath,
    Legend,
}

/// A single registered debug key action.
#[derive(Debug, Clone)]
pub struct KeyAction {
    /// Stable identifier used in `[debug.keybindings]` TOML overrides.
    pub id: &'static str,
    /// The key that triggers this action (possibly overridden from TOML).
    pub bound_key: KeyCode,
    /// Human-readable label shown in the legend.
    pub label: &'static str,
    /// Which `DebugVisualizationConfig` field to toggle.
    pub toggle: DebugToggle,
}

/// Global registry of all active debug key bindings.
/// Populated by startup systems in each capability plugin.
#[derive(Resource, Default)]
pub struct KeyActionRegistry(pub Vec<KeyAction>);

// ---------------------------------------------------------------------------
// Key-name → KeyCode parser (covers the keys used in default assignments)
// ---------------------------------------------------------------------------

/// Parse a key name string (as written in TOML) into a `KeyCode`.
/// Returns `None` for unrecognised names.
pub fn parse_key_code(name: &str) -> Option<KeyCode> {
    match name.to_uppercase().as_str() {
        "H" => Some(KeyCode::KeyH),
        "F1" => Some(KeyCode::F1),
        "F2" => Some(KeyCode::F2),
        "F3" => Some(KeyCode::F3),
        "F4" => Some(KeyCode::F4),
        "F5" => Some(KeyCode::F5),
        "F6" => Some(KeyCode::F6),
        "F7" => Some(KeyCode::F7),
        "F8" => Some(KeyCode::F8),
        "F9" => Some(KeyCode::F9),
        "F10" => Some(KeyCode::F10),
        "F11" => Some(KeyCode::F11),
        "F12" => Some(KeyCode::F12),
        _ => None,
    }
}
