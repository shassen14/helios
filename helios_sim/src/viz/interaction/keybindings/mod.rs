//! The IO shell around keybinding resolution: read the config file, hand its
//! overrides to [`resolve`], and insert the resolved [`KeyBindings`] resource.
//!
//! All the logic worth asserting lives in [`resolve`]; this module only touches
//! the filesystem and the `World`, so it stays a thin, untested edge.
//!
//! [`KeyBindings`]: resolve::KeyBindings

use crate::{cli::Cli, viz::interaction::actions::registry::ActionRegistry};

use bevy::prelude::*;
use figment::{
    providers::{Format, Toml},
    Figment,
};
use serde::Deserialize;
use std::collections::HashMap;

pub mod resolve;

/// Location of the keybinding file under `--config-root`. A single fixed file
/// for now; selecting between keybinding profiles would arrive as a scenario
/// reference, resolved into the config dump like any other prefab.
const KEYBINDINGS_FILE: &str = "sim/keybindings/default.toml";

/// The on-disk keybinding config: action name → key, deserialized straight from
/// TOML.
///
/// Sparse by design — it lists only the actions a user rebinds; everything else
/// falls back to registry defaults during resolution. The key strings are
/// Bevy `KeyCode` variant names (`"KeyM"`), parsed by Bevy's own serde impl, so
/// the key vocabulary has no hand-maintained table to drift.
#[derive(Deserialize)]
struct KeyBindingsFile {
    #[serde(default)]
    bindings: HashMap<String, KeyCode>,
}

/// Load and resolve keybindings at startup, inserting [`KeyBindings`].
///
/// Runs in `InteractionSet::KeyBinding`, after all action registration, so it
/// sees a fully populated registry. A missing file is not an error — figment
/// treats it as empty, so every action simply takes its default. A malformed
/// file or an unsatisfiable config (unknown action, key conflict) is a startup
/// misconfiguration and panics.
///
/// [`KeyBindings`]: resolve::KeyBindings
pub(crate) fn load_keybindings(
    cli: Res<Cli>,
    registry: Res<ActionRegistry>,
    mut commands: Commands,
) {
    let path = cli.config_root.join(KEYBINDINGS_FILE);

    let file: KeyBindingsFile = Figment::new()
        .merge(Toml::file(&path))
        .extract()
        .expect("keybindings TOML failed to parse");

    match resolve::resolve_bindings(&registry, &file.bindings) {
        Ok(b) => commands.insert_resource(b),
        Err(e) => panic!("keybinding config: {e}"),
    }
}
