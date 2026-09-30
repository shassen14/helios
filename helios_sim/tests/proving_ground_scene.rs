//! Wiring guard for the world-object pipeline, through the real headless host
//! on the proving ground scenario: every placement is spawned, every object's
//! scene is instanced, and its collider parts are gone from what renders.
//!
//! The asset gate and the scene-ready observer are registrations that compile
//! fine when missing; only a real app, with its asset server and scene
//! spawner, shows them working. Driving to a goal is not checked here: that
//! is `helios_test`'s job (`configs/test/runs/proving_ground.toml`).

use helios_sim::core::components::ObjectInstanceId;
use helios_sim::prelude::*;
use helios_sim::world::ResolvedWorldLayout;

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

const SCENARIO: &str = "sim/scenarios/01_proving_ground.toml";
/// Wall-clock budget for loading the assets and building the scene: about
/// ten times what a debug build takes. A missing asset gate hangs here.
const SCENE_TIMEOUT: Duration = Duration::from_secs(60);

/// Node names inside `traffic_cone.glb`: the visible cone and its collider
/// part.
const CONE_VISUAL_NODE: &str = "traffic_cone";
const CONE_COLLIDER_NODE: &str = "col_cone";
/// The cone's prefab, as the proving ground's placements name it.
const CONE_PREFAB: &str = "entities.objects.traffic_cone";

fn config_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../configs")
}

fn headless_app() -> App {
    let config_root = config_root();
    let cli = Cli {
        scenario: config_root.join(SCENARIO),
        config_root,
        headless: true,
        speed: None,
        seed: None,
    };
    let mut app = App::new();
    app.add_plugins(HeliosHost::new(
        cli,
        Presentation::Headless,
        TimePolicy::FastAsPossible,
    ));
    // `App::run` would call these; stepping by hand must.
    app.finish();
    app.cleanup();
    app
}

/// Every entity's name in the subtree under `root`, root excluded.
fn descendant_names(world: &mut World, root: Entity) -> Vec<String> {
    let mut children = world.query::<&Children>();
    let mut names = world.query::<&Name>();
    let mut stack: Vec<Entity> = children
        .get(world, root)
        .map(|c| c.to_vec())
        .unwrap_or_default();
    let mut found = Vec::new();
    while let Some(entity) = stack.pop() {
        if let Ok(name) = names.get(world, entity) {
            found.push(name.as_str().to_string());
        }
        if let Ok(below) = children.get(world, entity) {
            stack.extend_from_slice(below);
        }
    }
    found
}

/// The placed objects: entity and prefab key.
fn objects(world: &mut World) -> Vec<(Entity, String)> {
    world
        .query_filtered::<(Entity, &WorldObjectType), With<ObjectInstanceId>>()
        .iter(world)
        .map(|(entity, prefab)| (entity, prefab.0.clone()))
        .collect()
}

/// Whether every placed cone's scene has been instanced under it.
fn cones_instanced(world: &mut World) -> bool {
    let cones: Vec<Entity> = objects(world)
        .into_iter()
        .filter(|(_, prefab)| prefab == CONE_PREFAB)
        .map(|(entity, _)| entity)
        .collect();
    !cones.is_empty()
        && cones.into_iter().all(|cone| {
            descendant_names(world, cone)
                .iter()
                .any(|name| name == CONE_VISUAL_NODE)
        })
}

#[test]
fn proving_ground_spawns_every_object_without_collider_parts() {
    let mut app = headless_app();
    let start = Instant::now();
    while !(*app.world().resource::<State<AppState>>().get() == AppState::Running
        && cones_instanced(app.world_mut()))
    {
        assert!(
            start.elapsed() < SCENE_TIMEOUT,
            "the scene was not built within {SCENE_TIMEOUT:?}; state {:?}",
            app.world().resource::<State<AppState>>().get()
        );
        app.update();
    }
    // The observer's despawns are commands, applied by the next update.
    app.update();

    let world = app.world_mut();
    let placements = world.resource::<ResolvedWorldLayout>().placements().len();
    let objects = objects(world);
    assert_eq!(objects.len(), placements, "every placement is spawned");

    for (object, prefab) in objects {
        let names = descendant_names(world, object);
        assert!(
            !names.is_empty(),
            "`{prefab}` at {object} has no instanced scene"
        );
        assert!(
            !names.iter().any(|name| name == CONE_COLLIDER_NODE),
            "`{prefab}` at {object} still renders its collider part: {names:?}"
        );
    }
}
