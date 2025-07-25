use crate::{prelude::*, simulation::core::app_state::AssetLoadSet};
use avian3d::prelude::*;
use bevy::{
    asset::LoadState,
    gltf::{Gltf, GltfMesh},
};

use bevy_fly_camera::{FlyCamera, FlyCameraPlugin};

// --- Resources to track loading state ---

// Resource to hold the handle for the VISUAL scene GLB
#[derive(Resource, Default)]
struct VisualWorldHandle(Handle<Scene>);

// Resource to hold the handle for the COLLIDER mesh GLB
#[derive(Resource, Default)]
struct ColliderWorldHandle(Handle<Gltf>);

pub struct WorldSpawnerPlugin;

impl Plugin for WorldSpawnerPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FlyCameraPlugin);

        app.init_resource::<VisualWorldHandle>()
            .init_resource::<ColliderWorldHandle>();

        app
            // --- STAGE 1: ASSET LOADING ---
            .add_systems(
                OnEnter(AppState::AssetLoading),
                start_world_asset_loading.in_set(AssetLoadSet::Kickoff),
            )
            // --- THE FIX ---
            // This system sets the state, so it should only run IN the AssetLoading state.
            // It should NOT run in subsequent states like SceneBuilding.
            .add_systems(
                Update,
                check_for_world_load_completion
                    .in_set(AssetLoadSet::Check)
                    .run_if(in_state(AppState::AssetLoading)), // <-- Add this run condition
            )
            // --- STAGE 2: SCENE BUILDING ---
            // These systems run once we enter the SceneBuilding state.
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                (
                    // Spawn non-blocking assets like lights and a camera.
                    spawn_lighting_and_camera,
                    // Spawn the visual scene. This needs to run before colliders.
                    spawn_visual_world_scene.in_set(SceneBuildSet::ProcessVehicle),
                    // Spawn the colliders. This runs after the visual scene is in place.
                    spawn_colliders_from_gltf
                        .in_set(SceneBuildSet::Physics) // Place in physics set for good measure
                        .after(spawn_visual_world_scene),
                ),
            );
    }
}

/// This system runs on Update. It checks if loading has started, and if not,
/// it kicks it off. This is a "run-once" pattern.
fn start_world_asset_loading(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    config: Res<ScenarioConfig>,
) {
    // --- Load Visual Scene ---
    let visual_scene_path = config.world.map_file.clone();
    info!(
        "[ASSETS] Starting to load visual world scene: {}",
        visual_scene_path.display()
    );
    // Directly load the scene from the GLB.
    // let scene_handle: Handle<Scene> = asset_server.load(visual_scene_path);
    let scene_handle = asset_server.load(GltfAssetLabel::Scene(0).from_asset(visual_scene_path));
    commands.insert_resource(VisualWorldHandle(scene_handle));

    // --- Load Collider Mesh ---
    let collider_mesh_path_str = config
        .world
        .map_file
        .to_str()
        .unwrap()
        .replace(".glb", "_collider.glb");
    let collider_mesh_path = std::path::PathBuf::from(collider_mesh_path_str);
    info!(
        "[ASSETS] Starting to load collider world mesh: {}",
        collider_mesh_path.display()
    );
    // Load the full GLTF data for the collider file.
    let gltf_handle: Handle<Gltf> = asset_server.load(collider_mesh_path);
    commands.insert_resource(ColliderWorldHandle(gltf_handle));
}

/// Checks if BOTH assets are finished loading before changing the state.
// fn check_for_world_load_completion(
//     mut next_state: ResMut<NextState<AppState>>,
//     asset_server: Res<AssetServer>,
//     visual_handle: Res<VisualWorldHandle>,
//     collider_handle: Res<ColliderWorldHandle>,
// ) {
//     let visual_loaded = matches!(
//         asset_server.get_load_state(&visual_handle.0),
//         Some(LoadState::Loaded)
//     );
//     let collider_loaded = matches!(
//         asset_server.get_load_state(&collider_handle.0),
//         Some(LoadState::Loaded)
//     );

//     if visual_loaded && collider_loaded {
//         next_state.set(AppState::SceneBuilding);
//     }
// }

/// Checks if BOTH assets are finished loading before changing the state.
fn check_for_world_load_completion(
    mut next_state: ResMut<NextState<AppState>>,
    asset_server: Res<AssetServer>,
    visual_handle: Res<VisualWorldHandle>,
    collider_handle: Res<ColliderWorldHandle>,
) {
    println!("hello");
    // The correct way to check if a handle is the default/uninitialized one
    // is to compare it to the default value.
    let default_scene_handle = Handle::<Scene>::default();
    let default_gltf_handle = Handle::<Gltf>::default();

    if visual_handle.0 == default_scene_handle || collider_handle.0 == default_gltf_handle {
        // The real handles haven't been inserted by `start_world_asset_loading` yet.
        // We are still in the first frame of the `AssetLoading` state, so we just wait.
        return;
    }

    // This logic is now safe because we know the handles are valid.
    let visual_loaded = matches!(
        asset_server.get_load_state(&visual_handle.0),
        Some(LoadState::Loaded)
    );
    let collider_loaded = matches!(
        asset_server.get_load_state(&collider_handle.0),
        Some(LoadState::Loaded)
    );

    if visual_loaded && collider_loaded {
        info!("[ASSETS] World visual and collider assets loaded successfully.");
        next_state.set(AppState::SceneBuilding);
    }
}

/// Spawns the visual-only scene into the world.
fn spawn_visual_world_scene(mut commands: Commands, visual_handle: Res<VisualWorldHandle>) {
    info!("[SCENE] Spawning main visual world scene.");

    commands.spawn(SceneRoot(visual_handle.0.clone()));
}

fn spawn_colliders_from_gltf(
    asset_server: Res<AssetServer>,
    gltfs: Res<Assets<Gltf>>,
    gltf_meshes: Res<Assets<GltfMesh>>,
    meshes: Res<Assets<Mesh>>,
    mut commands: Commands,
    gltf_handle_res: Option<Res<ColliderWorldHandle>>,
) {
    let Some(gltf_handle_res) = gltf_handle_res else {
        return;
    };

    let gltf_handle = &gltf_handle_res.0;

    let load_state = asset_server
        .get_load_state(gltf_handle)
        .expect("Load State not available");
    if !load_state.is_loaded() {
        debug!("Waiting for GLTF to finish loading...");
        return;
    }

    let Some(gltf) = gltfs.get(gltf_handle) else {
        warn!("GLTF load state says loaded, but Gltf asset not found.");
        return;
    };

    for (name, gltf_mesh_handle) in &gltf.named_meshes {
        // 1. Get the GltfMesh asset using the first handle.
        let Some(gltf_mesh) = gltf_meshes.get(gltf_mesh_handle) else {
            warn!("GltfMesh for '{}' not loaded yet.", name);
            continue;
        };

        if gltf_mesh.primitives.is_empty() {
            warn!("Mesh '{}' has no primitives.", name);
            continue;
        }

        // 2. The GltfMesh contains a list of primitives. Each primitive
        //    has a handle to the FINAL, processed Mesh asset.
        let mesh_handle = gltf_mesh.primitives[0].mesh.clone();

        // 3. Now use this Handle<Mesh> to get the actual mesh data.
        if let Some(mesh) = meshes.get(&mesh_handle) {
            info!("✅ Creating collider for '{}'", name);
            let collider =
                Collider::trimesh_from_mesh_with_config(mesh, TrimeshFlags::FIX_INTERNAL_EDGES)
                    .expect(&format!("Could not create collider from mesh '{}'", name));

            commands.spawn((
                collider,
                RigidBody::Static,
                Name::new(format!("Collider: {}", name)),
            ));
        } else {
            debug!("Mesh asset not loaded for '{}'", name);
        }
    }

    // ✅ Done — remove marker so we don't run again
    commands.remove_resource::<ColliderWorldHandle>();
}

/// Spawns lights and a camera. (This is fine from before).
fn spawn_lighting_and_camera(mut commands: Commands) {
    // --- Spawn Lighting ---
    commands.spawn(DirectionalLight {
        shadows_enabled: true,
        illuminance: 15_000.0,
        ..default()
    });

    // --- Spawn Camera ---
    // You would add a flycam or other camera controller here.
    // For now, a static camera.
    let camera_transform: Transform =
        Transform::from_xyz(-30.0, 25.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y);
    let bob = camera_transform.rotation.to_euler(EulerRot::YZX);
    commands
        .spawn((
            Camera3d::default(),
            camera_transform,
            GlobalTransform::default(), // Usually added automatically, but good to be explicit
        ))
        .insert(FlyCamera {
            pitch: bob.2,
            yaw: bob.0,
            ..Default::default()
        });
}
