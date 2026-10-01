//! The world layout as written: prefab files describing a kind of object,
//! world files placing named instances of them, and the references a
//! scenario uses to pick one of each catalog entry.
//!
//! Pure data, exactly as authored (names, degrees). Class names, prefab
//! keys and orientations are resolved and checked by the world's loader.

use serde::Deserialize;
use std::path::PathBuf;

/// A kind of object, stored in `configs/entities/objects/<name>.toml`: its
/// geometry file and the facts Blender does not hold. Placed any number of
/// times by a [`WorldLayout`].
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct ObjectPrefab {
    /// Path to the `.glb` holding the visual mesh and any `col_` collider
    /// parts, relative to the Bevy asset root.
    pub mesh: PathBuf,
    /// Semantic class name, looked up in the scenario's class catalog.
    pub class: String,
    /// Mass in kg of one placed object. Required only when a placement
    /// makes it dynamic, and dynamic placements are never scaled, so this
    /// is the mass at the modelled size.
    #[serde(default)]
    pub mass_kg: Option<f64>,
    /// Whether the object has a physics collider. `false` for things
    /// agents pass through (foliage); such an object cannot be dynamic.
    #[serde(default = "default_collides")]
    pub collides: bool,
}

fn default_collides() -> bool {
    true
}

/// One placed instance of a prefab in a [`WorldLayout`].
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct ObjectPlacement {
    /// Unique within the layout; with the layout's name it forms the
    /// object's instance ID.
    pub name: String,
    /// Catalog key of an [`ObjectPrefab`], e.g. `"entities.objects.crate_1m"`.
    pub prefab: String,
    /// Position of the object's origin in ENU world frame
    /// [x_east, y_north, z_up], meters. Required: a forgotten position
    /// would stack objects at the origin.
    pub position: [f64; 3],
    /// Orientation [roll, pitch, yaw] in **degrees**, right-handed. Applied
    /// about the object's own axes as it turns: yaw about up (positive turns
    /// east toward north), then pitch about its turned y (positive tips the
    /// nose down), then roll about its turned x. Defaults to [0, 0, 0].
    #[serde(default)]
    pub orientation_degrees: [f64; 3],
    /// Per-axis scale along the object's own axes. Defaults to [1, 1, 1],
    /// the only scale a dynamic placement may have.
    #[serde(default = "default_scale")]
    pub scale: [f64; 3],
    /// Whether physics moves the object. Defaults to `static`.
    #[serde(default)]
    pub body: BodyKind,
}

fn default_scale() -> [f64; 3] {
    [1.0, 1.0, 1.0]
}

/// How physics treats a placed object. The placement decides, not the
/// prefab: the same crate can be fixed scenery or pushable.
#[derive(Debug, Deserialize, Clone, Copy, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum BodyKind {
    /// Never moves; other bodies collide with it.
    #[default]
    Static,
    /// Moved by gravity and contacts; needs a mass and a collider.
    Dynamic,
}

/// A scene: named placements of prefabs, stored in
/// `configs/sim/catalog/worlds/<name>.toml`.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct WorldLayout {
    /// The scene's identity and the prefix of every instance ID in it.
    /// Declared rather than taken from the file path, so moving the file
    /// does not change any ID.
    pub name: String,
    #[serde(default)]
    pub objects: Vec<ObjectPlacement>,
}

/// A reference to one catalog entry, used whole: `{ from = "<key>" }`.
///
/// Unlike an agent's `from`, nothing can be written beside the key to
/// override what it points at.
#[derive(Debug, Deserialize, Clone, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct CatalogRef {
    pub from: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    use figment::{
        providers::{Format, Toml},
        value::Value,
        Figment,
    };

    /// Parses TOML the way the prefab catalog does (file to `Value`), then
    /// deserializes the `Value`, so the tests see the loader's errors.
    fn parse<T: for<'de> Deserialize<'de>>(src: &str) -> Result<T, String> {
        let value: Value = Figment::new()
            .merge(Toml::string(src))
            .extract()
            .expect("test TOML is valid");
        value.deserialize::<T>().map_err(|e| e.to_string())
    }

    #[test]
    fn prefab_defaults_to_colliding_with_no_mass() {
        let prefab: ObjectPrefab = parse(
            r#"
            mesh  = "objects/traffic_cone.glb"
            class = "traffic_cone"
            "#,
        )
        .unwrap();

        assert_eq!(prefab.mesh, PathBuf::from("objects/traffic_cone.glb"));
        assert_eq!(prefab.class, "traffic_cone");
        assert_eq!(prefab.mass_kg, None);
        assert!(prefab.collides);
    }

    #[test]
    fn prefab_rejects_unknown_field() {
        let err = parse::<ObjectPrefab>(
            r#"
            mesh     = "objects/crate_1m.glb"
            class    = "crate"
            mass     = 20.0
            "#,
        )
        .unwrap_err();

        assert!(err.contains("mass"), "error should name the field: {err}");
    }

    #[test]
    fn placement_defaults() {
        let placement: ObjectPlacement = parse(
            r#"
            name     = "crate_a"
            prefab   = "entities.objects.crate_1m"
            position = [5.0, 3.0, 0.0]
            "#,
        )
        .unwrap();

        assert_eq!(placement.orientation_degrees, [0.0, 0.0, 0.0]);
        assert_eq!(placement.scale, [1.0, 1.0, 1.0]);
        assert_eq!(placement.body, BodyKind::Static);
    }

    #[test]
    fn placement_accepts_integer_coordinates() {
        let placement: ObjectPlacement = parse(
            r#"
            name     = "crate_a"
            prefab   = "entities.objects.crate_1m"
            position = [5, 3, 0]
            "#,
        )
        .unwrap();

        assert_eq!(placement.position, [5.0, 3.0, 0.0]);
    }

    #[test]
    fn placement_requires_name_and_position() {
        let no_name = parse::<ObjectPlacement>(
            r#"
            prefab   = "entities.objects.crate_1m"
            position = [0.0, 0.0, 0.0]
            "#,
        )
        .unwrap_err();
        let no_position = parse::<ObjectPlacement>(
            r#"
            name   = "crate_a"
            prefab = "entities.objects.crate_1m"
            "#,
        )
        .unwrap_err();

        assert!(no_name.contains("name"), "{no_name}");
        assert!(no_position.contains("position"), "{no_position}");
    }

    #[test]
    fn body_is_static_or_dynamic() {
        let dynamic: ObjectPlacement = parse(
            r#"
            name     = "crate_a"
            prefab   = "entities.objects.crate_1m"
            position = [0.0, 0.0, 0.0]
            body     = "dynamic"
            "#,
        )
        .unwrap();
        assert_eq!(dynamic.body, BodyKind::Dynamic);

        // Kinematic bodies are not built; the error lists what is.
        let err = parse::<ObjectPlacement>(
            r#"
            name     = "crate_a"
            prefab   = "entities.objects.crate_1m"
            position = [0.0, 0.0, 0.0]
            body     = "kinematic"
            "#,
        )
        .unwrap_err();
        assert!(
            err.contains("static") && err.contains("dynamic"),
            "error should list the valid bodies: {err}"
        );
    }

    #[test]
    fn layout_parses_a_world_file() {
        let layout: WorldLayout = parse(
            r#"
            name = "yard"

            [[objects]]
            name     = "ground"
            prefab   = "entities.objects.ground_100m"
            position = [0.0, 0.0, 0.0]

            [[objects]]
            name     = "crate_a"
            prefab   = "entities.objects.crate_1m"
            position = [5.0, 3.0, 0.0]
            body     = "dynamic"
            "#,
        )
        .unwrap();

        assert_eq!(layout.name, "yard");
        let names: Vec<&str> = layout.objects.iter().map(|o| o.name.as_str()).collect();
        assert_eq!(names, ["ground", "crate_a"]);
    }

    #[test]
    fn layout_requires_a_name() {
        let err = parse::<WorldLayout>(
            r#"
            [[objects]]
            name     = "crate_a"
            prefab   = "entities.objects.crate_1m"
            position = [0.0, 0.0, 0.0]
            "#,
        )
        .unwrap_err();

        assert!(err.contains("name"), "{err}");
    }

    #[test]
    fn catalog_ref_rejects_a_key_beside_from() {
        // The guard against `from + override`: a layout reference with
        // objects written beside it must fail, not silently add them.
        let err = parse::<CatalogRef>(
            r#"
            from = "sim.catalog.worlds.yard"
            name = "other_yard"
            "#,
        )
        .unwrap_err();

        assert!(err.contains("name"), "error should name the field: {err}");
    }
}
