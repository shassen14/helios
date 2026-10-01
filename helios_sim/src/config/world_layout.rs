//! Follows a scenario's world references into the catalog: the layout, the
//! semantic class catalog, and every prefab the layout places.
//!
//! References are looked up whole, never merged: nothing written beside a
//! `from` can edit what it points at. The result is still pure data, exactly
//! as authored; the world's loader checks it and builds runtime types.

use crate::config::structs::{CatalogRef, ObjectPrefab, World, WorldLayout};
use crate::config::PrefabCatalog;

use helios_core::interchange::perception::semantic_class::SemanticTaxonomy;

use bevy::prelude::Resource;
use figment::value::Value;
use serde::de::DeserializeOwned;
use std::collections::{BTreeMap, BTreeSet};

/// The scenario fields holding the references, as error messages name them.
const LAYOUT_FIELD: &str = "world.layout";
const SEMANTIC_CLASSES_FIELD: &str = "world.semantic_classes";

/// A scenario's layout with everything it references, loaded but not yet
/// checked.
#[derive(Resource, Debug, Clone)]
pub struct LoadedWorldLayout {
    pub layout: WorldLayout,
    /// Each prefab key the layout uses, once, however many placements share it.
    pub prefabs: BTreeMap<String, ObjectPrefab>,
    pub taxonomy: SemanticTaxonomy,
}

/// Loads the layout and class catalog `world` references, and the prefab of
/// every placement.
///
/// Both references are required: a scenario with no layout has no ground,
/// and its agents would fall with no message. Every failure is collected,
/// each naming the field and key.
pub fn load_world_layout(
    world: &World,
    catalog: &PrefabCatalog,
) -> Result<LoadedWorldLayout, Vec<String>> {
    let mut errors = Vec::new();

    let taxonomy = match &world.semantic_classes {
        Some(r) => lookup::<SemanticTaxonomy>(SEMANTIC_CLASSES_FIELD, r, catalog, &mut errors),
        None => {
            errors.push(format!(
                "{SEMANTIC_CLASSES_FIELD} is missing: every scenario names its class catalog; \
                 add `[{SEMANTIC_CLASSES_FIELD}] from = \"runtime.catalog.semantic_classes.<name>\"`"
            ));
            None
        }
    };

    let Some(layout_ref) = &world.layout else {
        errors.push(format!(
            "{LAYOUT_FIELD} is missing: without a layout there is no ground and \
             agents fall; add `[{LAYOUT_FIELD}] from = \"sim.catalog.worlds.<name>\"` \
             (`open_field` is bare ground)"
        ));
        return Err(errors);
    };

    let Some(layout) = lookup::<WorldLayout>(LAYOUT_FIELD, layout_ref, catalog, &mut errors) else {
        return Err(errors);
    };

    // Each prefab is looked up once, however many placements use it, so a
    // broken prefab is one error naming its first placement, not one per use.
    let mut prefabs = BTreeMap::new();
    let mut failed_keys = BTreeSet::new();
    for placement in &layout.objects {
        let key = &placement.prefab;
        if prefabs.contains_key(key) || failed_keys.contains(key) {
            continue;
        }
        let field = format!("{LAYOUT_FIELD} placement `{}`: prefab", placement.name);
        let reference = CatalogRef { from: key.clone() };
        match lookup::<ObjectPrefab>(&field, &reference, catalog, &mut errors) {
            Some(prefab) => {
                prefabs.insert(key.clone(), prefab);
            }
            None => {
                failed_keys.insert(key.clone());
            }
        }
    }

    match taxonomy {
        Some(taxonomy) if errors.is_empty() => Ok(LoadedWorldLayout {
            layout,
            prefabs,
            taxonomy,
        }),
        _ => Err(errors),
    }
}

/// Looks `reference` up and deserializes it as `T`, recording a failure in
/// `errors` rather than returning it, so the caller keeps collecting.
fn lookup<T: DeserializeOwned>(
    field: &str,
    reference: &CatalogRef,
    catalog: &PrefabCatalog,
    errors: &mut Vec<String>,
) -> Option<T> {
    let key = &reference.from;
    let Some(value) = catalog.0.get(key) else {
        errors.push(format!(
            "{field}: `{key}` is not in the catalog (a catalog file that failed \
             to parse is reported earlier in the log and also shows up this way)"
        ));
        return None;
    };
    match Value::deserialize::<T>(value) {
        Ok(parsed) => Some(parsed),
        Err(e) => {
            errors.push(format!("{field}: `{key}` is invalid: {e}"));
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use figment::{
        providers::{Format, Toml},
        Figment,
    };
    use std::collections::HashMap;

    const CLASSES: &str = r#"
        [[class]]
        name = "unlabeled"
        id   = 0

        [[class]]
        name = "traffic_cone"
        id   = 15
    "#;

    const CONE: &str = r#"
        mesh  = "objects/traffic_cone.glb"
        class = "traffic_cone"
    "#;

    const YARD: &str = r#"
        name = "yard"

        [[objects]]
        name     = "cone_a"
        prefab   = "entities.objects.traffic_cone"
        position = [1.0, 0.0, 0.0]

        [[objects]]
        name     = "cone_b"
        prefab   = "entities.objects.traffic_cone"
        position = [2.0, 0.0, 0.0]
    "#;

    fn value(src: &str) -> Value {
        Figment::new()
            .merge(Toml::string(src))
            .extract()
            .expect("test TOML is valid")
    }

    /// A catalog of TOML strings, parsed as the disk loader parses files.
    fn catalog(entries: &[(&str, &str)]) -> PrefabCatalog {
        PrefabCatalog(
            entries
                .iter()
                .map(|(key, src)| (key.to_string(), value(src)))
                .collect::<HashMap<_, _>>(),
        )
    }

    fn yard_catalog() -> PrefabCatalog {
        catalog(&[
            ("sim.catalog.worlds.yard", YARD),
            ("runtime.catalog.semantic_classes.default", CLASSES),
            ("entities.objects.traffic_cone", CONE),
        ])
    }

    /// The scenario's `[world]` block, parsed through the real `World`.
    fn world(src: &str) -> World {
        Value::deserialize(&value(src)).expect("test world is valid")
    }

    const YARD_WORLD: &str = r#"
        [layout]
        from = "sim.catalog.worlds.yard"

        [semantic_classes]
        from = "runtime.catalog.semantic_classes.default"
    "#;

    #[test]
    fn loads_layout_catalog_and_each_prefab_once() {
        let loaded = load_world_layout(&world(YARD_WORLD), &yard_catalog()).unwrap();

        assert_eq!(loaded.layout.name, "yard");
        assert_eq!(loaded.layout.objects.len(), 2);
        let keys: Vec<&str> = loaded.prefabs.keys().map(String::as_str).collect();
        assert_eq!(keys, ["entities.objects.traffic_cone"]);
    }

    #[test]
    fn world_without_references_names_both_missing_fields() {
        // An empty `[world]` used to load as a world with no ground.
        let errors = load_world_layout(&world(""), &yard_catalog()).unwrap_err();

        assert_eq!(errors.len(), 2, "{errors:?}");
        assert!(errors[0].contains(SEMANTIC_CLASSES_FIELD), "{errors:?}");
        assert!(errors[1].contains(LAYOUT_FIELD), "{errors:?}");
        assert!(errors[1].contains("is missing"), "{errors:?}");
    }

    #[test]
    fn class_catalog_without_layout_is_rejected() {
        let errors = load_world_layout(
            &world(
                r#"
                [semantic_classes]
                from = "runtime.catalog.semantic_classes.default"
                "#,
            ),
            &yard_catalog(),
        )
        .unwrap_err();

        assert_eq!(errors.len(), 1, "{errors:?}");
        assert!(errors[0].contains(LAYOUT_FIELD), "{errors:?}");
    }

    #[test]
    fn layout_without_class_catalog_is_rejected() {
        let errors = load_world_layout(
            &world(
                r#"
                [layout]
                from = "sim.catalog.worlds.yard"
                "#,
            ),
            &yard_catalog(),
        )
        .unwrap_err();

        assert_eq!(errors.len(), 1, "{errors:?}");
        assert!(errors[0].contains(SEMANTIC_CLASSES_FIELD), "{errors:?}");
    }

    #[test]
    fn missing_layout_names_field_and_key() {
        let errors = load_world_layout(
            &world(&YARD_WORLD.replace("worlds.yard", "worlds.yrad")),
            &yard_catalog(),
        )
        .unwrap_err();

        assert!(errors[0].contains(LAYOUT_FIELD), "{errors:?}");
        assert!(errors[0].contains("sim.catalog.worlds.yrad"), "{errors:?}");
    }

    #[test]
    fn reference_to_the_wrong_kind_of_file_is_rejected() {
        let errors = load_world_layout(
            &world(&YARD_WORLD.replace("sim.catalog.worlds.yard", "entities.objects.traffic_cone")),
            &yard_catalog(),
        )
        .unwrap_err();

        assert!(errors[0].contains("is invalid"), "{errors:?}");
    }

    #[test]
    fn missing_prefab_is_reported_once_naming_its_first_placement() {
        let catalog = catalog(&[
            ("sim.catalog.worlds.yard", YARD),
            ("runtime.catalog.semantic_classes.default", CLASSES),
        ]);

        let errors = load_world_layout(&world(YARD_WORLD), &catalog).unwrap_err();

        assert_eq!(errors.len(), 1, "{errors:?}");
        assert!(errors[0].contains("cone_a"), "{errors:?}");
        assert!(
            errors[0].contains("entities.objects.traffic_cone"),
            "{errors:?}"
        );
    }

    #[test]
    fn broken_class_catalog_is_reported_beside_a_missing_layout() {
        let catalog = catalog(&[(
            "runtime.catalog.semantic_classes.default",
            "[[class]]\nname = \"car\"\nid = 9\n",
        )]);

        let errors = load_world_layout(
            &world(
                r#"
                [semantic_classes]
                from = "runtime.catalog.semantic_classes.default"
                "#,
            ),
            &catalog,
        )
        .unwrap_err();

        assert_eq!(errors.len(), 2, "{errors:?}");
        assert!(errors[0].contains("unlabeled"), "{errors:?}");
    }

    #[test]
    fn every_failure_is_collected() {
        // No class catalog and a missing prefab: both reported in one run.
        let catalog = catalog(&[("sim.catalog.worlds.yard", YARD)]);

        let errors = load_world_layout(
            &world(
                r#"
                [layout]
                from = "sim.catalog.worlds.yard"
                "#,
            ),
            &catalog,
        )
        .unwrap_err();

        assert_eq!(errors.len(), 2, "{errors:?}");
    }

    #[test]
    fn key_beside_a_world_reference_is_rejected() {
        let result = Value::deserialize::<World>(&value(
            r#"
            [layout]
            from = "sim.catalog.worlds.yard"
            name = "other"
            "#,
        ));

        assert!(result.is_err());
    }
}
