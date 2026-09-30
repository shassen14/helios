use super::*;
use crate::config::structs::{BodyKind, ObjectPlacement, ObjectPrefab, WorldLayout};
use crate::config::LoadedWorldLayout;
use crate::core::components::BoundingBox3D;
use crate::world::layout::AssetNode;

use helios_core::interchange::perception::semantic_class::SemanticTaxonomy;

use avian3d::prelude::SimpleCollider;
use bevy::ecs::system::RunSystemOnce;
use figment::{
    providers::{Format, Toml},
    Figment,
};
use nalgebra::{Matrix4, Point3};
use std::collections::BTreeMap;
use std::sync::Arc;

const TOLERANCE: f32 = 1e-5;
const CRATE: &str = "entities.objects.crate_1m";
const CONE: &str = "entities.objects.traffic_cone";

/// A crate placed dynamic, turned to face north and stretched along each of
/// its axes differently, and a cone that does not collide.
fn yard() -> ResolvedWorldLayout {
    let taxonomy: SemanticTaxonomy = Figment::new()
        .merge(Toml::string(
            "[[class]]\nname = \"unlabeled\"\nid = 0\n\
             [[class]]\nname = \"crate\"\nid = 3\n\
             [[class]]\nname = \"traffic_cone\"\nid = 15\n",
        ))
        .extract()
        .expect("test catalog is valid");
    let prefab = |mesh: &str, class: &str, mass_kg, collides| ObjectPrefab {
        mesh: mesh.into(),
        class: class.to_string(),
        mass_kg,
        collides,
    };
    let loaded = LoadedWorldLayout {
        layout: WorldLayout {
            name: "yard".to_string(),
            objects: vec![
                ObjectPlacement {
                    name: "crate_a".to_string(),
                    prefab: CRATE.to_string(),
                    position: [1.0, 2.0, 0.0],
                    orientation_degrees: [0.0, 0.0, 90.0],
                    scale: [1.0, 2.0, 3.0],
                    body: BodyKind::Dynamic,
                },
                ObjectPlacement {
                    name: "cone_a".to_string(),
                    prefab: CONE.to_string(),
                    position: [0.0; 3],
                    orientation_degrees: [0.0; 3],
                    scale: [1.0; 3],
                    body: BodyKind::Static,
                },
            ],
        },
        prefabs: BTreeMap::from([
            (
                CRATE.to_string(),
                prefab("objects/crate_1m.glb", "crate", Some(20.0), true),
            ),
            (
                CONE.to_string(),
                prefab("objects/traffic_cone.glb", "traffic_cone", None, false),
            ),
        ]),
        taxonomy,
    };
    ResolvedWorldLayout::resolve(&loaded).expect("test layout resolves")
}

/// A 1 m cube standing on its origin, as glTF stores it (y up).
fn cube_on_origin() -> PrefabGeometry {
    let mut corners = Vec::with_capacity(8);
    for x in [-0.5, 0.5] {
        for y in [0.0, 1.0] {
            for z in [-0.5, 0.5] {
                corners.push(Point3::new(x, y, z));
            }
        }
    }
    let root = AssetNode {
        name: "crate".to_string(),
        local: Matrix4::identity(),
        positions: corners,
        children: Vec::new(),
    };
    PrefabGeometry::derive(&[root]).expect("cube derives")
}

/// Prepared prefabs in the layout's prefab order, each a cube; the scene
/// handles are placeholders, since nothing renders here.
fn prepared(layout: &ResolvedWorldLayout) -> Vec<PreparedPrefab> {
    layout
        .prefabs()
        .iter()
        .map(|_| PreparedPrefab {
            scene: Handle::default(),
            geometry: cube_on_origin(),
        })
        .collect()
}

/// Spawns `layout` into a fresh app and returns it.
fn spawned(layout: ResolvedWorldLayout) -> App {
    let mut app = App::new();
    let prefabs = prepared(&layout);
    app.world_mut()
        .run_system_once(move |mut commands: Commands| {
            spawn_objects(&mut commands, &layout, &prefabs).expect("objects spawn");
        })
        .expect("system runs");
    app
}

fn object(app: &mut App, instance_id: &str) -> Entity {
    let mut query = app.world_mut().query::<(Entity, &ObjectInstanceId)>();
    query
        .iter(app.world())
        .find(|(_, id)| id.0 == instance_id)
        .map(|(entity, _)| entity)
        .unwrap_or_else(|| panic!("`{instance_id}` was spawned"))
}

fn assert_vec3_eq(actual: Vec3, expected: Vec3) {
    assert!(
        actual.abs_diff_eq(expected, TOLERANCE),
        "{actual} != {expected}"
    );
}

#[test]
fn each_placement_is_one_labelled_body() {
    let layout = yard();
    let crate_class = layout.taxonomy().class("crate").unwrap();
    let mut app = spawned(layout);
    let crate_a = object(&mut app, "yard/crate_a");
    let world = app.world();

    assert_eq!(world.get::<Name>(crate_a).unwrap().as_str(), "yard/crate_a");
    assert_eq!(
        world.get::<ObjectClass>(crate_a),
        Some(&ObjectClass(crate_class))
    );
    assert_eq!(world.get::<WorldObjectType>(crate_a).unwrap().0, CRATE);
    assert_eq!(world.get::<RigidBody>(crate_a), Some(&RigidBody::Dynamic));
    assert_eq!(world.get::<Mass>(crate_a), Some(&Mass(20.0)));
    assert!(world.get::<Collider>(crate_a).is_some());
}

#[test]
fn body_is_posed_in_enu_and_never_scaled() {
    let mut app = spawned(yard());
    let crate_a = object(&mut app, "yard/crate_a");
    let transform = app.world().get::<Transform>(crate_a).unwrap();

    // ENU (1, 2, 0) is Bevy (1, 0, −2); yaw 90 turns the crate's forward
    // from east (Bevy +X) to north (Bevy −Z).
    assert_vec3_eq(transform.translation, Vec3::new(1.0, 0.0, -2.0));
    assert_vec3_eq(transform.rotation * Vec3::X, Vec3::NEG_Z);
    assert_eq!(transform.scale, Vec3::ONE);
}

#[test]
fn visual_child_carries_the_scale_in_bevy_axes() {
    let mut app = spawned(yard());
    let crate_a = object(&mut app, "yard/crate_a");
    let world = app.world();
    let children = world.get::<Children>(crate_a).expect("has a visual child");
    let visual = children[0];

    assert!(world.get::<ObjectVisual>(visual).is_some());
    assert!(world.get::<WorldAssetRoot>(visual).is_some());
    // Object scale (forward 1, left 2, up 3) is Bevy (x 1, y up 3, z 2).
    assert_vec3_eq(
        world.get::<Transform>(visual).unwrap().scale,
        Vec3::new(1.0, 3.0, 2.0),
    );
}

#[test]
fn bounding_box_is_scaled_and_matches_the_box_collider() {
    let mut app = spawned(yard());
    let crate_a = object(&mut app, "yard/crate_a");
    let world = app.world();
    let bbox = world.get::<BoundingBox3D>(crate_a).unwrap();

    // The cube stands on its origin, stretched 3× up and 2× left: its centre
    // is 1.5 m up, and left maps to Bevy z.
    assert_vec3_eq(bbox.centre, Vec3::new(0.0, 1.5, 0.0));
    assert_vec3_eq(bbox.half_extents, Vec3::new(0.5, 1.5, 1.0));

    let aabb = world
        .get::<Collider>(crate_a)
        .unwrap()
        .aabb(Vec3::ZERO, Quat::IDENTITY);
    assert_vec3_eq(aabb.min, bbox.centre - bbox.half_extents);
    assert_vec3_eq(aabb.max, bbox.centre + bbox.half_extents);
}

#[test]
fn static_object_without_collision_has_no_collider_or_mass() {
    let mut app = spawned(yard());
    let cone_a = object(&mut app, "yard/cone_a");
    let world = app.world();

    assert_eq!(world.get::<RigidBody>(cone_a), Some(&RigidBody::Static));
    assert!(world.get::<Collider>(cone_a).is_none());
    assert!(world.get::<Mass>(cone_a).is_none());
    assert!(world.get::<BoundingBox3D>(cone_a).is_some());
}

#[test]
fn a_missing_prefab_is_reported_not_skipped() {
    let layout = yard();
    let mut app = App::new();
    let errors = app
        .world_mut()
        .run_system_once(move |mut commands: Commands| {
            spawn_objects(&mut commands, &layout, &[]).expect_err("nothing was prepared")
        })
        .expect("system runs");

    assert_eq!(errors.len(), 2, "{errors:?}");
}

#[test]
fn colliders_are_shared_per_prefab_and_scale() {
    let geometry = cube_on_origin();
    let mut cache = ColliderCache::default();
    let mut get = |prefab, scale: [f64; 3]| {
        cache
            .get(prefab, &geometry, &Vector3::from(scale))
            .expect("collider builds")
    };

    let first = get(0, [1.0, 2.0, 1.0]);
    let same = get(0, [1.0, 2.0, 1.0]);
    let other_scale = get(0, [2.0, 1.0, 1.0]);
    let other_prefab = get(1, [1.0, 2.0, 1.0]);

    let shares = |a: &Collider, b: &Collider| Arc::ptr_eq(&a.shape().0, &b.shape().0);
    assert!(shares(&first, &same));
    assert!(!shares(&first, &other_scale));
    assert!(!shares(&first, &other_prefab));
}

#[test]
fn collider_parts_are_removed_from_the_rendered_scene() {
    // visual ── cone ── cone.mesh
    //        ├─ col_cone ── col_cone.mesh
    //        └─ group ── col_base
    let mut app = App::new();
    let world = app.world_mut();
    let visual = world.spawn(ObjectVisual).id();
    let named = |world: &mut World, name: &str, parent: Entity| {
        world
            .spawn((Name::new(name.to_string()), ChildOf(parent)))
            .id()
    };
    let cone = named(world, "cone", visual);
    let cone_mesh = named(world, "cone.mesh", cone);
    let col_cone = named(world, "col_cone", visual);
    let col_cone_mesh = named(world, "col_cone.mesh", col_cone);
    let group = named(world, "group", visual);
    let col_base = named(world, "col_base", group);

    app.world_mut()
        .run_system_once(
            move |children: Query<&Children>, names: Query<&Name>, mut commands: Commands| {
                remove_collider_parts(visual, &children, &names, &mut commands);
            },
        )
        .expect("system runs");

    let world = app.world();
    for kept in [cone, cone_mesh, group] {
        assert!(world.get_entity(kept).is_ok(), "{kept} was kept");
    }
    for removed in [col_cone, col_cone_mesh, col_base] {
        assert!(world.get_entity(removed).is_err(), "{removed} was removed");
    }
}
