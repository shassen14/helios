// helios_sim/src/simulation/config/catalog.rs

//! This module defines the `PrefabCatalog` resource and a plugin to load
//! all component definitions from disk at startup.

use bevy::prelude::*;
use figment::{
    providers::{Format, Toml},
    value::Value,
    Figment,
};
use std::{collections::HashMap, path::Path};
use walkdir::WalkDir;

/// A Bevy resource that holds the entire parsed catalog of prefabs.
/// The key is a namespace string (e.g., "vehicles.ackermann_sedan") and
/// the value is the raw, parsed TOML data for that component.
#[derive(Resource, Default, Debug)]
pub struct PrefabCatalog(pub HashMap<String, Value>);

/// A startup system that walks the `assets/catalog` directory, parses every
/// `.toml` file, and populates the `PrefabCatalog` resource.
pub fn load_catalog_from_disk(mut catalog: ResMut<PrefabCatalog>) {
    let catalog_path = Path::new("assets/catalog");
    if !catalog_path.exists() {
        warn!(
            "Catalog directory not found at {:?}, no prefabs will be loaded.",
            catalog_path
        );
        return;
    }

    info!("Loading prefab catalog from: {:?}", catalog_path);

    for entry in WalkDir::new(catalog_path)
        .into_iter()
        .filter_map(Result::ok)
        .filter(|e| {
            !e.file_type().is_dir() && e.path().extension().map_or(false, |ext| ext == "toml")
        })
    {
        let path = entry.path();
        // Create a key like "vehicles.ackermann_sedan" from the path.
        let key = path
            .strip_prefix(catalog_path)
            .unwrap()
            .with_extension("")
            .to_string_lossy()
            .replace(std::path::MAIN_SEPARATOR, ".");

        match Figment::new().merge(Toml::file(path)).extract::<Value>() {
            Ok(data) => {
                info!("Loaded catalog item: '{}'", key);
                catalog.0.insert(key, data);
            }
            Err(e) => {
                error!("Failed to load catalog item from {:?}: {}", path, e);
            }
        }
    }
}
