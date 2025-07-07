// src/simulation/plugins/world/spawner.rs

use crate::simulation::core::app_state::AppState;
use crate::simulation::core::config::SimulationConfig;
use avian3d::prelude::*;
use bevy::asset::LoadState;
use bevy::{gltf::GltfMesh, prelude::*};

pub struct WorldSpawnerPlugin;

impl Plugin for WorldSpawnerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            (spawn_world, spawn_lighting_and_camera),
        )
        .add_systems(
            Update,
            spawn_colliders_from_all_meshes
                .run_if(in_state(AppState::SceneBuilding).or(in_state(AppState::Running))),
        );
    }
}

/// Runs during `AssetLoading`. Checks if the primary world scene asset is loaded.
/// If it is, it transitions the app to the `SceneBuilding` state.
pub fn check_for_asset_load(
    mut next_state: ResMut<NextState<AppState>>,
    asset_server: Res<AssetServer>,
    config: Res<SimulationConfig>,
) {
    // let world_handle: Handle<Scene> = asset_server
    //     .get_handle(config.world.map_file.clone())
    //     .expect("Unable to get world handle");

    let world_handle: Handle<Scene> =
        asset_server.load(GltfAssetLabel::Scene(0).from_asset(config.world.map_file.clone()));

    // get_load_state is the modern way to check an asset's status.
    match asset_server.get_load_state(&world_handle) {
        Some(LoadState::Loaded) => {
            info!("World asset loaded. Transitioning to SceneBuilding state.");
            // We've finished loading, so we command the state machine to change.
            next_state.set(AppState::SceneBuilding);
        }
        Some(LoadState::Failed(..)) => {
            error!(
                "World asset failed to load! Check path: {}",
                config.world.map_file.display()
            );
            // You might want to panic or exit here in a real app.
        }
        _ => {
            // Still loading, do nothing and check again next frame.
        }
    }
}

#[derive(Resource)]
struct GltfHandle(pub Handle<Gltf>);

fn spawn_world(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    config: Res<SimulationConfig>,
) {
    // Load GLTF (e.g., terrain.glb in assets/maps/)
    // 1. Start loading the specific mesh from the GLTF file.
    //    The `GltfAssetLabel` is a clean way to pick a specific part of the file.
    let original_file = config.world.map_file.clone();
    let mesh_file = original_file
        .parent()
        .unwrap_or_else(|| original_file.as_path()) // fallback if no parent
        .join(format!(
            "mesh_{}",
            original_file.file_name().unwrap().to_string_lossy()
        ));
    let gltf_handle: Handle<Gltf> = asset_server.load(mesh_file);
    // let gltf_handle: Handle<Gltf> = asset_server.load(config.world.map_file.clone());

    // Visual scene (optional, separate .glb with visual assets)
    let scene_handle: Handle<Scene> =
        asset_server.load(GltfAssetLabel::Scene(0).from_asset(config.world.map_file.clone()));

    commands.spawn((
        SceneRoot(scene_handle),
        RigidBody::Static,
        Name::new("WorldMesh"),
    ));
    commands.insert_resource(GltfHandle(gltf_handle));
}

fn spawn_colliders_from_all_meshes(
    asset_server: Res<AssetServer>,
    gltfs: Res<Assets<Gltf>>,
    gltf_meshes: Res<Assets<GltfMesh>>,
    meshes: Res<Assets<Mesh>>,
    mut commands: Commands,
    gltf_handle_res: Option<Res<GltfHandle>>,
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
        let Some(gltf_mesh) = gltf_meshes.get(gltf_mesh_handle) else {
            warn!("GltfMesh for '{}' not loaded yet.", name);
            continue;
        };

        if gltf_mesh.primitives.is_empty() {
            warn!("Mesh '{}' has no primitives.", name);
            continue;
        }

        // Use first primitive's mesh handle
        let mesh_handle = gltf_mesh.primitives[0].mesh.clone();

        if let Some(mesh) = meshes.get(&mesh_handle) {
            info!("✅ Creating collider for '{}'", name);
            let collider = Collider::trimesh_from_mesh(mesh)
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
    commands.remove_resource::<GltfHandle>();
}

/// **STARTUP SYSTEM 2:** Spawns lights and a camera. Kept separate for clarity.
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
    let camera_transform = Transform::from_xyz(-30.0, 25.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y);
    commands.spawn((
        Camera3d::default(),
        camera_transform,
        GlobalTransform::default(), // Usually added automatically, but good to be explicit
    ));
}

// Extra code that I tried to use at some point

// #[derive(Component)]
// // struct CreateColliderFromMesh(Handle<Mesh>);
// struct CreateColliderFromNamedMesh {
//     gltf_handle: Handle<Gltf>,
//     mesh_name: String,
// }

// #[derive(Component)]
// struct SceneAssets {
//     pub scene_handle: Handle<Scene>,
//     pub mesh_handle: Handle<Mesh>,
// }

// fn print_mesh_names(
//     asset_server: Res<AssetServer>,
//     gltfs: Res<Assets<Gltf>>,
//     config: Res<SimulationConfig>,
// ) {
//     let gltf_handle = asset_server.load(config.world.map_file.clone());
//     debug!("Starting print_mesh");
//     if let Some(gltf) = gltfs.get(&gltf_handle) {
//         debug!("hello");
//         for (name, handle) in gltf.named_meshes.iter() {
//             println!("Mesh: {}", name);
//         }
//     }
// }

// fn spawn_colliders(
//     asset_server: Res<AssetServer>,
//     gltfs: Res<Assets<Gltf>>,
//     gltf_meshes: Res<Assets<GltfMesh>>,
//     meshes: Res<Assets<Mesh>>,
//     mut commands: Commands,
//     gltf_handle: Res<GltfHandle>,
// ) {
//     debug!("spawn_colliders");
//     // if asset_server
//     //     .get_load_state(&gltf_handle.0)
//     //     .expect("Asset Server unable to obtain load state")
//     //     .is_loaded()
//     // {
//     //     debug!("still loading");

//     //     return; // still loading
//     // }
//     let gltf = match gltfs.get(&gltf_handle.0) {
//         Some(g) => g,
//         None => return,
//     };

//     // Use named meshes or index-based access
//     for (name, gltf_mesh_handle) in &gltf.named_meshes {
//         let gltf_mesh = match gltf_meshes.get(gltf_mesh_handle) {
//             Some(m) => m,
//             None => continue,
//         };

//         // Extract the Bevy Mesh handle from the first primitive
//         let mesh_handle: Handle<Mesh> = gltf_mesh.primitives[0].mesh.clone();

//         // Access actual `Mesh` data
//         if let Some(mesh) = meshes.get(&mesh_handle) {
//             info!("Mesh '{}' has {} vertices", name, mesh.count_vertices());

//             // Create collider – for example trimesh for Rapier
//             let collider = Collider::trimesh_from_mesh(mesh); // pseudocode
//             commands.spawn((
//                 collider.expect("Collision not created from mesh"),
//                 RigidBody::Static,
//             ));
//         }
//     }
// }
