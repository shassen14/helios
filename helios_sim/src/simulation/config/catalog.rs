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

use crate::cli::Cli;

/// A Bevy resource that holds the entire parsed catalog of prefabs.
/// The key is a namespace string (e.g., "vehicles.ackermann_sedan") and
/// the value is the raw, parsed TOML data for that component.
#[derive(Resource, Default, Debug)]
pub struct PrefabCatalog(pub HashMap<String, Value>);

/// Sub-directories under `config_root` that contain prefab definitions.
/// Keys are generated relative to `config_root` so `entities/vehicles/foo.toml`
/// becomes `entities.vehicles.foo` and `runtime/catalog/estimators/ekf.toml`
/// becomes `runtime.catalog.estimators.ekf`.
///
/// Scenarios (`sim/scenarios/`) and raw fixture files are excluded — they are
/// not prefabs and are not referenced via `{ from = "..." }`.
const CATALOG_ROOTS: &[&str] = &[
    "entities",
    "runtime/catalog",
    "runtime/profiles",
    "sim/catalog",
];

/// Populates `PrefabCatalog` by walking all vocabulary-partitioned catalog
/// sub-directories under `config_root`. Keys are relative to `config_root`.
pub fn load_catalog_from_disk(mut catalog: ResMut<PrefabCatalog>, cli: Res<Cli>) {
    let config_root = &cli.config_root;

    for sub_dir in CATALOG_ROOTS {
        let dir_path = config_root.join(sub_dir);
        if !dir_path.exists() {
            warn!("Catalog sub-directory not found at {:?}, skipping.", dir_path);
            continue;
        }

        info!("Loading catalog entries from: {:?}", dir_path);

        for entry in WalkDir::new(&dir_path)
            .into_iter()
            .filter_map(Result::ok)
            .filter(|e| {
                !e.file_type().is_dir()
                    && e.path().extension().is_some_and(|ext| ext == "toml")
            })
        {
            let path = entry.path();
            let key = path
                .strip_prefix(config_root)
                .unwrap_or(path)
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
}
