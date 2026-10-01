//! Resolves the scenario's layout at startup, loads each prefab's `.glb`,
//! and spawns the objects when the scene is built, failing the run with
//! every error when the layout or its assets are invalid.

use super::assets::{load_object_assets, ObjectAssets};
use super::error::error_report;
use super::resolved::ResolvedWorldLayout;
use super::spawn::{remove_collider_parts_when_ready, spawn_world_objects};
use crate::config::LoadedWorldLayout;
use crate::core::app_state::AssetLoadSet;
use crate::prelude::*;

/// Resolves the scenario's layout, when it has one, loads its prefabs, and
/// spawns its objects.
pub struct WorldLayoutPlugin;

impl Plugin for WorldLayoutPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ObjectAssets>()
            .add_systems(
                OnEnter(AppState::AssetLoading),
                // Chained so loading sees the resolved layout inserted.
                (resolve_world_layout, load_object_assets)
                    .chain()
                    .in_set(AssetLoadSet::Kickoff),
            )
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                spawn_world_objects.in_set(SceneBuildSet::ProcessWorldObjects),
            )
            .add_observer(remove_collider_parts_when_ready);
    }
}

/// Resolves the loaded layout into [`ResolvedWorldLayout`], panicking with
/// every error when it is invalid: a scene missing some of its objects
/// answers a question nobody asked.
fn resolve_world_layout(mut commands: Commands, loaded: Option<Res<LoadedWorldLayout>>) {
    let Some(loaded) = loaded else {
        return;
    };
    match ResolvedWorldLayout::resolve(&loaded) {
        Ok(resolved) => {
            info!(
                "[WorldLayout] '{}': {} placements of {} prefabs",
                resolved.name(),
                resolved.placements().len(),
                resolved.prefabs().len()
            );
            commands.insert_resource(resolved);
        }
        Err(errors) => panic!(
            "{}",
            error_report(&invalid_layout(&loaded.layout.name), &errors)
        ),
    }
}

/// The heading of a rejected layout's report.
fn invalid_layout(name: &str) -> String {
    format!("world layout `{name}` is invalid")
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::config::structs::{BodyKind, ObjectPlacement, ObjectPrefab, World, WorldLayout};
    use crate::config::{load_world_layout, read_catalog, PrefabCatalog};
    use crate::world::layout::{read_scene, PrefabGeometry};

    use bevy::ecs::system::RunSystemOnce;
    use figment::{
        providers::{Format, Toml},
        value::Value,
        Figment,
    };
    use std::collections::{BTreeMap, HashMap};
    use std::path::{Path, PathBuf};

    /// A one-placement layout, loaded through the real config loader.
    fn loaded_layout() -> LoadedWorldLayout {
        let value = |src: &str| -> Value {
            Figment::new()
                .merge(Toml::string(src))
                .extract()
                .expect("test TOML is valid")
        };
        let catalog = PrefabCatalog(HashMap::from([
            (
                "sim.catalog.worlds.test".to_string(),
                value(
                    "name = \"test\"\n[[objects]]\nname = \"cone_a\"\n\
                     prefab = \"entities.objects.traffic_cone\"\nposition = [0.0, 0.0, 0.0]\n",
                ),
            ),
            (
                "runtime.catalog.semantic_classes.default".to_string(),
                value("[[class]]\nname = \"unlabeled\"\nid = 0\n[[class]]\nname = \"traffic_cone\"\nid = 15\n"),
            ),
            (
                "entities.objects.traffic_cone".to_string(),
                value("mesh = \"objects/traffic_cone.glb\"\nclass = \"traffic_cone\"\n"),
            ),
        ]));
        let world: World = value(
            "[layout]\nfrom = \"sim.catalog.worlds.test\"\n\
             [semantic_classes]\nfrom = \"runtime.catalog.semantic_classes.default\"\n",
        )
        .deserialize()
        .expect("test world is valid");

        load_world_layout(&world, &catalog).expect("test references resolve")
    }

    #[test]
    fn plugin_resolves_a_loaded_layout_into_a_resource() {
        let mut app = App::new();
        app.insert_resource(loaded_layout());
        app.world_mut()
            .run_system_once(resolve_world_layout)
            .expect("system runs");

        let resolved = app.world().resource::<ResolvedWorldLayout>();
        assert_eq!(resolved.placements().len(), 1);
    }

    #[test]
    fn plugin_does_nothing_without_a_layout() {
        let mut app = App::new();
        app.world_mut()
            .run_system_once(resolve_world_layout)
            .expect("system runs");

        assert!(app.world().get_resource::<ResolvedWorldLayout>().is_none());
    }

    /// The repository's `configs/` tree.
    fn config_root() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../configs")
    }

    #[test]
    fn every_catalog_file_parses() {
        // At run time a catalog file that fails to parse is only logged, and
        // any reference to it then reads as missing. This keeps that from
        // reaching a run unnoticed.
        let (_, failures) = read_catalog(&config_root());

        assert!(
            failures.is_empty(),
            "catalog files failed to parse: {failures:#?}"
        );
    }

    #[test]
    fn every_scenario_world_loads_and_resolves() {
        // Reads only each scenario's `[world]`: agents are checked by the
        // scenarios' own runs, and must not fail this test for other reasons.
        // Test fixtures are included, so a broken world fails here rather
        // than as a run that was meant to fail for another reason.
        let (catalog, _) = read_catalog(&config_root());
        let scenarios = SCENARIO_DIRS
            .iter()
            .flat_map(|dir| walkdir::WalkDir::new(config_root().join(dir)))
            .filter_map(Result::ok)
            .filter(|e| e.path().extension().is_some_and(|ext| ext == "toml"));

        let mut checked = 0;
        for scenario in scenarios {
            let path = scenario.path();
            let world: World = Figment::new()
                .merge(Toml::file(path))
                .focus("world")
                .extract()
                .unwrap_or_else(|e| panic!("{path:?}: [world] does not parse: {e}"));

            let loaded =
                load_world_layout(&world, &catalog).unwrap_or_else(|e| panic!("{path:?}: {e:#?}"));
            if let Err(errors) = ResolvedWorldLayout::resolve(&loaded) {
                panic!(
                    "{path:?}: {}",
                    error_report(&invalid_layout(&loaded.layout.name), &errors)
                );
            }
            checked += 1;
        }
        assert!(checked > 0, "no scenarios found under {:?}", config_root());
    }

    #[test]
    fn world_names_are_unique() {
        // Two scenes declaring one name would give their objects the same
        // instance IDs across datasets.
        let (catalog, _) = read_catalog(&config_root());
        let mut seen: HashMap<String, &str> = HashMap::new();

        for (key, value) in &catalog.0 {
            if !key.starts_with(WORLDS_CATALOG_PREFIX) {
                continue;
            }
            let layout: WorldLayout = value
                .deserialize()
                .unwrap_or_else(|e| panic!("{key}: not a world layout: {e}"));
            if let Some(other) = seen.insert(layout.name.clone(), key) {
                panic!("`{other}` and `{key}` both declare name `{}`", layout.name);
            }
        }
    }

    #[test]
    fn every_object_prefab_resolves_against_the_default_catalog() {
        // A prefab no scenario places yet is otherwise never checked: a
        // misspelt class would surface only when someone first uses it.
        let (catalog, _) = read_catalog(&config_root());
        let classes = catalog
            .0
            .get(DEFAULT_CLASSES_KEY)
            .expect("the default class catalog exists")
            .deserialize()
            .expect("the default class catalog is valid");

        let mut prefabs = BTreeMap::new();
        let mut objects = Vec::new();
        for (key, value) in &catalog.0 {
            if !key.starts_with(OBJECTS_CATALOG_PREFIX) {
                continue;
            }
            let prefab: ObjectPrefab = value
                .deserialize()
                .unwrap_or_else(|e| panic!("{key}: not an object prefab: {e}"));
            objects.push(ObjectPlacement {
                name: format!("object_{}", objects.len()),
                prefab: key.clone(),
                position: [0.0; 3],
                orientation_degrees: [0.0; 3],
                scale: [1.0; 3],
                body: BodyKind::Static,
            });
            prefabs.insert(key.clone(), prefab);
        }
        assert!(!objects.is_empty(), "no object prefabs found");

        let loaded = LoadedWorldLayout {
            layout: WorldLayout {
                name: "every_prefab".to_string(),
                objects,
            },
            prefabs,
            taxonomy: classes,
        };
        if let Err(errors) = ResolvedWorldLayout::resolve(&loaded) {
            panic!(
                "{}",
                error_report(&invalid_layout(&loaded.layout.name), &errors)
            );
        }
    }

    #[test]
    fn every_object_prefab_mesh_derives_geometry() {
        // At run time a broken asset fails the scenario that first places
        // it; this finds it when it is exported.
        let (catalog, _) = read_catalog(&config_root());

        let mut checked = 0;
        for (key, value) in &catalog.0 {
            if !key.starts_with(OBJECTS_CATALOG_PREFIX) {
                continue;
            }
            let prefab: ObjectPrefab = value
                .deserialize()
                .unwrap_or_else(|e| panic!("{key}: not an object prefab: {e}"));
            let path = crate::asset_root().join(&prefab.mesh);
            let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("{key}: {path:?}: {e}"));
            let gltf =
                gltf::Gltf::from_slice(&bytes).unwrap_or_else(|e| panic!("{key}: {path:?}: {e}"));
            let roots = read_scene(&gltf).unwrap_or_else(|e| panic!("{key}: {path:?}: {e}"));
            if let Err(errors) = PrefabGeometry::derive(&roots) {
                let errors: Vec<String> = errors.iter().map(ToString::to_string).collect();
                panic!("{key}: {path:?}: {}", errors.join("; "));
            }
            checked += 1;
        }
        assert!(checked > 0, "no object prefabs found");
    }

    /// Where scenarios live under `configs/`: the runnable ones and the test
    /// fixtures.
    const SCENARIO_DIRS: [&str; 2] = ["sim/scenarios", "test/scenarios"];

    /// Catalog keys of object prefabs, from `configs/entities/objects/`.
    const OBJECTS_CATALOG_PREFIX: &str = "entities.objects.";

    /// The catalog key of the default semantic class catalog.
    const DEFAULT_CLASSES_KEY: &str = "runtime.catalog.semantic_classes.default";

    /// Catalog keys of world files, from `configs/sim/catalog/worlds/`.
    const WORLDS_CATALOG_PREFIX: &str = "sim.catalog.worlds.";
}
