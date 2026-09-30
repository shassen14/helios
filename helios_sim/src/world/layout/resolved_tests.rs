//! Tests for layout resolution: each load-time check, the rotation
//! convention, and prefab sharing. Layouts go through the real config loader.

use super::*;

use crate::config::structs::World;
use crate::config::{load_world_layout, PrefabCatalog};

use approx::assert_relative_eq;
use figment::{
    providers::{Format, Toml},
    value::Value,
    Figment,
};

const CLASSES: &str = r#"
    [[class]]
    name = "unlabeled"
    id   = 0

    [[class]]
    name = "traffic_cone"
    id   = 15

    [[class]]
    name = "crate"
    id   = 16
"#;

const CONE: &str = r#"
    mesh    = "objects/traffic_cone.glb"
    class   = "traffic_cone"
    mass_kg = 1.2
"#;

const CRATE: &str = r#"
    mesh    = "objects/crate_1m.glb"
    class   = "crate"
    mass_kg = 20.0
"#;

const YARD: &str = r#"
    name = "yard"

    [[objects]]
    name     = "cone_a"
    prefab   = "entities.objects.traffic_cone"
    position = [1.0, 2.0, 0.0]

    [[objects]]
    name     = "cone_b"
    prefab   = "entities.objects.traffic_cone"
    position = [3.0, 2.0, 0.0]

    [[objects]]
    name     = "crate_a"
    prefab   = "entities.objects.crate_1m"
    position = [5.0, 3.0, 0.0]
    body     = "dynamic"
"#;

fn value(src: &str) -> Value {
    Figment::new()
        .merge(Toml::string(src))
        .extract()
        .expect("test TOML is valid")
}

/// Loads `layout` with the given prefab files through the real config
/// loader, so these tests see exactly what a scenario would produce.
fn load(layout: &str, prefabs: &[(&str, &str)]) -> LoadedWorldLayout {
    let mut entries = vec![
        ("sim.catalog.worlds.test".to_string(), value(layout)),
        (
            "runtime.catalog.semantic_classes.default".to_string(),
            value(CLASSES),
        ),
    ];
    for (key, src) in prefabs {
        entries.push((key.to_string(), value(src)));
    }
    let world: World = Value::deserialize(&value(
        r#"
        [layout]
        from = "sim.catalog.worlds.test"

        [semantic_classes]
        from = "runtime.catalog.semantic_classes.default"
        "#,
    ))
    .expect("test world is valid");

    load_world_layout(&world, &PrefabCatalog(entries.into_iter().collect()))
        .expect("test references resolve")
        .expect("a layout was named")
}

fn yard() -> LoadedWorldLayout {
    load(
        YARD,
        &[
            ("entities.objects.traffic_cone", CONE),
            ("entities.objects.crate_1m", CRATE),
        ],
    )
}

fn resolve_err(loaded: &LoadedWorldLayout) -> Vec<LayoutError> {
    ResolvedWorldLayout::resolve(loaded).expect_err("layout should be rejected")
}

/// One placement of the cone, with `extra` lines added to it.
fn one_cone(extra: &str) -> LoadedWorldLayout {
    load(
        &format!(
            r#"
            name = "test"

            [[objects]]
            name     = "cone_a"
            prefab   = "entities.objects.traffic_cone"
            position = [0.0, 0.0, 0.0]
            {extra}
            "#
        ),
        &[("entities.objects.traffic_cone", CONE)],
    )
}

fn placement<'a>(layout: &'a ResolvedWorldLayout, name: &str) -> &'a ResolvedPlacement {
    object(layout, name).0
}

fn object<'a>(
    layout: &'a ResolvedWorldLayout,
    name: &str,
) -> (&'a ResolvedPlacement, &'a ResolvedPrefab) {
    layout
        .objects()
        .find(|(p, _)| p.name == name)
        .expect("placement exists")
}

#[test]
fn valid_layout_resolves() {
    let layout = ResolvedWorldLayout::resolve(&yard()).unwrap();

    assert_eq!(layout.name(), "yard");
    assert_eq!(layout.placements().len(), 3);
    let (cone, cone_prefab) = object(&layout, "cone_a");
    assert_eq!(cone_prefab.class.id(), 15);
    assert_eq!(cone_prefab.key, "entities.objects.traffic_cone");
    assert_eq!(layout.instance_id(cone), "yard/cone_a");
    assert_eq!(cone.pose.translation.vector, Vector3::new(1.0, 2.0, 0.0));
    assert_eq!(cone.scale, Vector3::new(1.0, 1.0, 1.0));
    assert_eq!(cone.body, ResolvedBody::Static);
}

#[test]
fn placements_of_one_prefab_share_it() {
    let layout = ResolvedWorldLayout::resolve(&yard()).unwrap();

    assert_eq!(layout.prefabs().len(), 2);
    let (_, a) = object(&layout, "cone_a");
    let (_, b) = object(&layout, "cone_b");
    assert!(std::ptr::eq(a, b));
}

#[test]
fn placements_keep_file_order() {
    let layout = ResolvedWorldLayout::resolve(&yard()).unwrap();

    let names: Vec<&str> = layout
        .placements()
        .iter()
        .map(|p| p.name.as_str())
        .collect();
    assert_eq!(names, ["cone_a", "cone_b", "crate_a"]);
}

#[test]
fn dynamic_body_carries_the_prefab_mass() {
    let layout = ResolvedWorldLayout::resolve(&yard()).unwrap();

    assert_eq!(
        placement(&layout, "crate_a").body,
        ResolvedBody::Dynamic { mass_kg: 20.0 }
    );
}

#[test]
fn static_prefab_needs_no_mass() {
    let massless = CONE.replace("mass_kg = 1.2", "");
    let loaded = load(
        YARD,
        &[
            ("entities.objects.traffic_cone", &massless),
            ("entities.objects.crate_1m", CRATE),
        ],
    );

    assert!(ResolvedWorldLayout::resolve(&loaded).is_ok());
}

#[test]
fn yaw_90_turns_east_to_north() {
    let layout =
        ResolvedWorldLayout::resolve(&one_cone("orientation_degrees = [0.0, 0.0, 90.0]")).unwrap();

    let forward = placement(&layout, "cone_a").pose.rotation * Vector3::x();
    assert_relative_eq!(forward, Vector3::y(), epsilon = 1e-12);
}

#[test]
fn yaw_is_applied_before_pitch_about_the_turned_axes() {
    // Yaw 90 faces the object north; pitch 30 then tips its nose 30°
    // down about its own (turned) y axis. Applying the angles in the
    // other order would leave the nose pointing somewhere else, which a
    // yaw-only test cannot detect.
    let layout =
        ResolvedWorldLayout::resolve(&one_cone("orientation_degrees = [0.0, 30.0, 90.0]")).unwrap();

    let forward = placement(&layout, "cone_a").pose.rotation * Vector3::x();
    let pitch = 30f64.to_radians();
    assert_relative_eq!(
        forward,
        Vector3::new(0.0, pitch.cos(), -pitch.sin()),
        epsilon = 1e-12
    );
}

#[test]
fn rejects_layout_name_not_snake_case() {
    let loaded = load(
        &YARD.replace("name = \"yard\"", "name = \"Yard\""),
        &[
            ("entities.objects.traffic_cone", CONE),
            ("entities.objects.crate_1m", CRATE),
        ],
    );

    assert_eq!(
        resolve_err(&loaded),
        [LayoutError::LayoutNameNotSnakeCase {
            name: "Yard".to_string()
        }]
    );
}

#[test]
fn rejects_placement_name_not_snake_case() {
    let loaded = load(
        &YARD.replace("\"crate_a\"", "\"crates/a\""),
        &[
            ("entities.objects.traffic_cone", CONE),
            ("entities.objects.crate_1m", CRATE),
        ],
    );

    assert_eq!(
        resolve_err(&loaded),
        [LayoutError::PlacementNameNotSnakeCase {
            placement: "crates/a".to_string()
        }]
    );
}

#[test]
fn rejects_duplicate_placement_name() {
    let loaded = load(
        &YARD.replace("\"cone_b\"", "\"cone_a\""),
        &[
            ("entities.objects.traffic_cone", CONE),
            ("entities.objects.crate_1m", CRATE),
        ],
    );

    assert_eq!(
        resolve_err(&loaded),
        [LayoutError::DuplicatePlacement {
            placement: "cone_a".to_string()
        }]
    );
}

#[test]
fn rejects_zero_or_negative_scale() {
    for scale in ["[1.0, 1.0, 0.0]", "[-1.0, 1.0, 1.0]"] {
        let errors = resolve_err(&one_cone(&format!("scale = {scale}")));

        assert!(
            matches!(errors[..], [LayoutError::NonPositiveScale { .. }]),
            "{scale}: {errors:?}"
        );
    }
}

#[test]
fn rejects_values_that_are_not_finite() {
    let errors = resolve_err(&one_cone("orientation_degrees = [0.0, 0.0, nan]"));

    assert_eq!(
        errors,
        [LayoutError::NotFinite {
            placement: "cone_a".to_string(),
            field: "orientation_degrees",
        }]
    );
}

#[test]
fn rejects_mesh_that_is_not_a_relative_glb() {
    for mesh in ["objects/traffic_cone.gltf", "/abs/traffic_cone.glb"] {
        let cone = CONE.replace("objects/traffic_cone.glb", mesh);
        let loaded = load(
            "name = \"test\"\n[[objects]]\nname = \"cone_a\"\n\
             prefab = \"entities.objects.traffic_cone\"\nposition = [0.0, 0.0, 0.0]\n",
            &[("entities.objects.traffic_cone", &cone)],
        );

        let errors = resolve_err(&loaded);
        assert!(
            matches!(errors[..], [LayoutError::MeshNotRelativeGlb { .. }]),
            "{mesh}: {errors:?}"
        );
    }
}

#[test]
fn rejects_unknown_class_listing_valid_ones() {
    let loaded = load(
        YARD,
        &[
            (
                "entities.objects.traffic_cone",
                &CONE.replace("\"traffic_cone\"", "\"cone\""),
            ),
            ("entities.objects.crate_1m", CRATE),
        ],
    );

    let errors = resolve_err(&loaded);
    assert_eq!(errors.len(), 1, "one error per prefab, not per placement");
    let message = errors[0].to_string();
    assert!(
        message.contains("entities.objects.traffic_cone"),
        "{message}"
    );
    assert!(
        message.contains("crate, traffic_cone, unlabeled"),
        "{message}"
    );
}

#[test]
fn rejects_non_positive_mass() {
    let loaded = load(
        YARD,
        &[
            ("entities.objects.traffic_cone", CONE),
            ("entities.objects.crate_1m", &CRATE.replace("20.0", "0.0")),
        ],
    );

    assert_eq!(
        resolve_err(&loaded),
        [LayoutError::InvalidMass {
            prefab: "entities.objects.crate_1m".to_string(),
            mass_kg: 0.0,
        }]
    );
}

#[test]
fn rejects_dynamic_without_mass() {
    let loaded = load(
        YARD,
        &[
            ("entities.objects.traffic_cone", CONE),
            (
                "entities.objects.crate_1m",
                &CRATE.replace("mass_kg = 20.0", ""),
            ),
        ],
    );

    assert_eq!(
        resolve_err(&loaded),
        [LayoutError::DynamicWithoutMass {
            placement: "crate_a".to_string(),
            prefab: "entities.objects.crate_1m".to_string(),
        }]
    );
}

#[test]
fn rejects_dynamic_without_collider() {
    let loaded = load(
        YARD,
        &[
            ("entities.objects.traffic_cone", CONE),
            (
                "entities.objects.crate_1m",
                &format!("{CRATE}\ncollides = false"),
            ),
        ],
    );

    assert_eq!(
        resolve_err(&loaded),
        [LayoutError::DynamicWithoutCollider {
            placement: "crate_a".to_string(),
            prefab: "entities.objects.crate_1m".to_string(),
        }]
    );
}

#[test]
fn reports_every_broken_placement() {
    let loaded = load(
        &YARD.replace("\"cone_a\"", "\"Cone A\"").replace(
            "position = [3.0, 2.0, 0.0]",
            "position = [3.0, 2.0, 0.0]\nscale = [0.0, 1.0, 1.0]",
        ),
        &[
            ("entities.objects.traffic_cone", CONE),
            ("entities.objects.crate_1m", CRATE),
        ],
    );

    assert_eq!(resolve_err(&loaded).len(), 2);
}

#[test]
fn prefab_missing_from_a_hand_built_layout_is_an_error() {
    // A generator builds `LoadedWorldLayout` itself; a placement naming a
    // prefab it forgot to include must fail, not panic.
    let mut loaded = yard();
    loaded.prefabs.remove("entities.objects.crate_1m");

    assert_eq!(
        resolve_err(&loaded),
        [LayoutError::UnknownPrefab {
            placement: "crate_a".to_string(),
            prefab: "entities.objects.crate_1m".to_string(),
        }]
    );
}
