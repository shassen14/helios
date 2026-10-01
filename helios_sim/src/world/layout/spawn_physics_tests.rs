//! Spawned objects under real physics: Avian steps a bare app, with no asset
//! server, so each prefab's geometry is built in code. These check that the
//! collider sits where the object's box says, which only shows once bodies
//! rest on, hit and are hit by one another.

use super::*;
use crate::config::structs::{BodyKind, ObjectPlacement, ObjectPrefab, WorldLayout};
use crate::config::LoadedWorldLayout;
use crate::world::layout::AssetNode;

use helios_core::interchange::perception::semantic_class::SemanticTaxonomy;

use avian3d::prelude::{Gravity, LinearVelocity, PhysicsPlugins, SpatialQuery, SpatialQueryFilter};
use bevy::ecs::system::RunSystemOnce;
use bevy::time::TimeUpdateStrategy;
use figment::{
    providers::{Format, Toml},
    Figment,
};
use nalgebra::{Matrix4, Point3};
use std::collections::BTreeMap;
use std::time::Duration;

/// Avian's default fixed step.
const STEP: Duration = Duration::from_micros(15_625);
const GRAVITY_M_S2: f32 = 9.81;
/// Resting contacts sink by a few millimetres under Avian's default solver.
const REST_TOLERANCE_M: f32 = 0.02;
const RAY_TOLERANCE_M: f32 = 1e-3;
/// Float round-off on a body that did not move (metres, and quaternion
/// components).
const UNMOVED_TOLERANCE: f32 = 1e-5;
/// A tilt left by settling, not by tipping over (radians).
const UPRIGHT_TOLERANCE_RAD: f32 = 1e-2;

const GROUND: &str = "entities.objects.ground";
const CRATE: &str = "entities.objects.crate";
const WALL: &str = "entities.objects.wall";

/// A box in glTF axes (x forward, y up, z right), as an exported `.glb`
/// stores it.
fn box_geometry(min: [f64; 3], max: [f64; 3]) -> PrefabGeometry {
    let mut corners = Vec::with_capacity(8);
    for x in [min[0], max[0]] {
        for y in [min[1], max[1]] {
            for z in [min[2], max[2]] {
                corners.push(Point3::new(x, y, z));
            }
        }
    }
    let root = AssetNode {
        name: "box".to_string(),
        local: Matrix4::identity(),
        positions: corners,
        triangles: Vec::new(),
        children: Vec::new(),
    };
    PrefabGeometry::derive(&[root]).expect("box derives")
}

/// The geometry of each test prefab, shaped like its asset: a 40 m ground
/// slab with its top at 0, a 1 m crate and a 4 m wall standing on their
/// origins.
fn geometry(prefab: &str) -> PrefabGeometry {
    match prefab {
        GROUND => box_geometry([-20.0, -1.0, -20.0], [20.0, 0.0, 20.0]),
        CRATE => box_geometry([-0.5, 0.0, -0.5], [0.5, 1.0, 0.5]),
        WALL => box_geometry([-2.0, 0.0, -0.1], [2.0, 2.0, 0.1]),
        other => panic!("no test geometry for `{other}`"),
    }
}

/// A layout of `placements` on the ground.
fn layout(placements: Vec<ObjectPlacement>) -> ResolvedWorldLayout {
    let taxonomy: SemanticTaxonomy = Figment::new()
        .merge(Toml::string(
            "[[class]]\nname = \"unlabeled\"\nid = 0\n\
             [[class]]\nname = \"terrain\"\nid = 3\n\
             [[class]]\nname = \"wall\"\nid = 5\n\
             [[class]]\nname = \"crate\"\nid = 7\n",
        ))
        .extract()
        .expect("test catalog is valid");
    let prefab = |class: &str, mass_kg| ObjectPrefab {
        mesh: format!("objects/{class}.glb").into(),
        class: class.to_string(),
        mass_kg,
        collides: true,
    };
    let mut objects = vec![place("ground", GROUND, [0.0; 3])];
    objects.extend(placements);
    let loaded = LoadedWorldLayout {
        layout: WorldLayout {
            name: "physics".to_string(),
            objects,
        },
        prefabs: BTreeMap::from([
            (GROUND.to_string(), prefab("terrain", None)),
            (CRATE.to_string(), prefab("crate", Some(20.0))),
            (WALL.to_string(), prefab("wall", None)),
        ]),
        taxonomy,
    };
    ResolvedWorldLayout::resolve(&loaded).expect("test layout resolves")
}

/// A static, unrotated, unscaled placement.
fn place(name: &str, prefab: &str, position: [f64; 3]) -> ObjectPlacement {
    ObjectPlacement {
        name: name.to_string(),
        prefab: prefab.to_string(),
        position,
        orientation_degrees: [0.0; 3],
        scale: [1.0; 3],
        body: BodyKind::Static,
    }
}

/// An app stepping Avian one fixed step per update, with `layout` spawned
/// and gravity pointing down Bevy's y.
fn physics_app(layout: ResolvedWorldLayout) -> App {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        AssetPlugin::default(),
        bevy::mesh::MeshPlugin,
        PhysicsPlugins::default(),
    ))
    .insert_resource(TimeUpdateStrategy::ManualDuration(STEP))
    .insert_resource(Gravity(Vec3::NEG_Y * GRAVITY_M_S2));
    // `App::run` would call these; stepping by hand must, since Avian adds
    // some of its resources only when plugins finish.
    app.finish();
    app.cleanup();

    let prefabs: Vec<PreparedPrefab> = layout
        .prefabs()
        .iter()
        .map(|prefab| PreparedPrefab {
            scene: Handle::default(),
            geometry: geometry(&prefab.key),
        })
        .collect();
    app.world_mut()
        .run_system_once(move |mut commands: Commands| {
            spawn_objects(&mut commands, &layout, &prefabs).expect("objects spawn");
        })
        .expect("system runs");
    app
}

fn step(app: &mut App, seconds: f32) {
    let steps = (seconds / STEP.as_secs_f32()).ceil() as usize;
    for _ in 0..steps {
        app.update();
    }
}

fn object(app: &mut App, name: &str) -> Entity {
    let instance_id = format!("physics/{name}");
    let mut query = app.world_mut().query::<(Entity, &ObjectInstanceId)>();
    query
        .iter(app.world())
        .find(|(_, id)| id.0 == instance_id)
        .map(|(entity, _)| entity)
        .unwrap_or_else(|| panic!("`{instance_id}` was spawned"))
}

fn transform(app: &App, entity: Entity) -> Transform {
    *app.world()
        .get::<Transform>(entity)
        .expect("has a transform")
}

#[test]
fn dropped_crate_comes_to_rest_upright_on_the_ground() {
    let crate_a = ObjectPlacement {
        body: BodyKind::Dynamic,
        ..place("crate_a", CRATE, [0.0, 0.0, 1.0])
    };
    let mut app = physics_app(layout(vec![crate_a]));
    let crate_a = object(&mut app, "crate_a");

    step(&mut app, 3.0);

    // The crate's origin is its base, so resting on the ground's top face
    // puts it at height 0 (Bevy y).
    let rest = transform(&app, crate_a);
    assert!(
        rest.translation.y.abs() < REST_TOLERANCE_M,
        "crate rests at height {}",
        rest.translation.y
    );
    assert!(
        rest.rotation.angle_between(Quat::IDENTITY) < UPRIGHT_TOLERANCE_RAD,
        "crate tipped: {:?}",
        rest.rotation
    );
}

#[test]
fn static_wall_stops_a_dynamic_body_and_does_not_move() {
    // A wall running north–south at 4 m east, and a crate sliding east into
    // it at 5 m/s.
    let wall = ObjectPlacement {
        orientation_degrees: [0.0, 0.0, 90.0],
        ..place("wall", WALL, [4.0, 0.0, 0.0])
    };
    let crate_a = ObjectPlacement {
        body: BodyKind::Dynamic,
        ..place("crate_a", CRATE, [0.0, 0.0, 0.0])
    };
    let mut app = physics_app(layout(vec![wall, crate_a]));
    let wall = object(&mut app, "wall");
    let crate_a = object(&mut app, "crate_a");
    let wall_before = transform(&app, wall);
    app.world_mut()
        .entity_mut(crate_a)
        .insert(LinearVelocity(Vec3::X * 5.0));

    step(&mut app, 2.0);

    // Avian writes a static body's pose back through its own types, so the
    // last float digit may change; a moved wall would be off by far more.
    let wall_after = transform(&app, wall);
    assert!(
        wall_after
            .translation
            .abs_diff_eq(wall_before.translation, UNMOVED_TOLERANCE)
            && wall_after
                .rotation
                .abs_diff_eq(wall_before.rotation, UNMOVED_TOLERANCE),
        "wall moved from {wall_before:?} to {wall_after:?}"
    );
    // The wall's west face is at 3.9 m and the crate is 0.5 m deep.
    let crate_east = transform(&app, crate_a).translation.x;
    assert!(
        crate_east < 3.4 + REST_TOLERANCE_M,
        "crate went through the wall to x = {crate_east}"
    );
}

#[test]
fn ray_hits_a_placed_wall_where_its_box_says() {
    // The wall runs north–south at 5 m east, stretched to 8 m long, so it
    // spans 4 m either side of the x axis. Scale is along the wall's own
    // length (its forward, x), which yaw 90 turns north.
    let wall = ObjectPlacement {
        orientation_degrees: [0.0, 0.0, 90.0],
        scale: [2.0, 1.0, 1.0],
        ..place("wall", WALL, [5.0, 0.0, 0.0])
    };
    let mut app = physics_app(layout(vec![wall]));
    let wall = object(&mut app, "wall");
    step(&mut app, STEP.as_secs_f32());

    // Rays cast east at 1 m up, from 3 m north (inside the stretched
    // wall's reach, outside the unstretched one) and from 5 m north (past
    // its end). ENU (0, n, 1) is Bevy (0, 1, −n).
    let hits = app
        .world_mut()
        .run_system_once(|query: SpatialQuery| {
            [3.0, 5.0].map(|north: f32| {
                query.cast_ray(
                    Vec3::new(0.0, 1.0, -north),
                    Dir3::X,
                    20.0,
                    true,
                    &SpatialQueryFilter::default(),
                )
            })
        })
        .expect("system runs");

    let hit = hits[0].expect("the ray 3 m north hits the wall");
    assert_eq!(hit.entity, wall);
    // The wall is 0.2 m thick, so its west face is 4.9 m east.
    assert!(
        (hit.distance - 4.9).abs() < RAY_TOLERANCE_M,
        "hit at {} m",
        hit.distance
    );
    assert!(hits[1].is_none(), "the ray 5 m north passes the wall's end");
}
