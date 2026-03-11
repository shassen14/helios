use std::f64::consts::PI;

use avian3d::prelude::{Collider, RigidBody, TrimeshFlags};
use bevy::{asset::LoadState, gltf::{Gltf, GltfAssetLabel, GltfMesh}, prelude::*};

use crate::prelude::*;
use crate::simulation::core::app_state::AssetLoadSet;
use crate::simulation::core::components::TerrainMedium;
use crate::simulation::core::transforms::enu_iso_to_bevy_transform;
use crate::simulation::plugins::world::objects::WorldObjectAssets;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};

// =========================================================================
// == Resource ==
// =========================================================================

struct TerrainAssetEntry {
    config_idx: usize,
    scene: Handle<Scene>,
    collider: Option<Handle<Gltf>>,
}

/// Loaded asset handles for all terrain tiles. Populated during AssetLoading;
/// consumed during SceneBuilding.
#[derive(Resource, Default)]
pub struct TerrainAssets {
    entries: Vec<TerrainAssetEntry>,
}

impl TerrainAssets {
    pub fn all_loaded(&self, asset_server: &AssetServer) -> bool {
        self.entries.iter().all(|e| {
            let scene_ok = matches!(
                asset_server.get_load_state(&e.scene),
                Some(LoadState::Loaded)
            );
            let col_ok = e.collider.as_ref().map_or(true, |h| {
                matches!(asset_server.get_load_state(h), Some(LoadState::Loaded))
            });
            scene_ok && col_ok
        })
    }
}

// =========================================================================
// == Plugin ==
// =========================================================================

pub struct TerrainPlugin;

impl Plugin for TerrainPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TerrainAssets>()
            .add_systems(
                OnEnter(AppState::AssetLoading),
                start_terrain_asset_loading.in_set(AssetLoadSet::Kickoff),
            )
            .add_systems(
                Update,
                check_all_assets_loaded
                    .in_set(AssetLoadSet::Check)
                    .run_if(in_state(AppState::AssetLoading)),
            )
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                spawn_terrain_tiles.in_set(SceneBuildSet::ProcessWorldObjects),
            );
    }
}

// =========================================================================
// == Systems ==
// =========================================================================

fn start_terrain_asset_loading(
    mut terrain_assets: ResMut<TerrainAssets>,
    config: Res<ScenarioConfig>,
    asset_server: Res<AssetServer>,
) {
    for (idx, terrain) in config.world.terrains.iter().enumerate() {
        let scene = asset_server
            .load(GltfAssetLabel::Scene(0).from_asset(terrain.mesh.clone()));
        info!(
            "[Terrain] Loading visual mesh for tile {}: {:?}",
            idx, terrain.mesh
        );

        let collider = terrain.collider.as_ref().map(|col_path| {
            info!("[Terrain] Loading collider mesh for tile {}: {:?}", idx, col_path);
            asset_server.load::<Gltf>(col_path.clone())
        });

        terrain_assets.entries.push(TerrainAssetEntry {
            config_idx: idx,
            scene,
            collider,
        });
    }

    if config.world.terrains.is_empty() {
        warn!("[Terrain] No [[world.terrains]] defined — scene will have no ground.");
    }
}

/// Central asset readiness gate. Transitions to SceneBuilding only when all
/// terrain tiles AND all world object GLBs have finished loading.
fn check_all_assets_loaded(
    mut next_state: ResMut<NextState<AppState>>,
    asset_server: Res<AssetServer>,
    terrain_assets: Res<TerrainAssets>,
    world_object_assets: Option<Res<WorldObjectAssets>>,
) {
    // Wait until at least one kickoff frame has run.
    if terrain_assets.entries.is_empty() {
        return;
    }

    let terrain_ready = terrain_assets.all_loaded(&asset_server);
    let objects_ready = world_object_assets
        .map(|a| a.all_loaded(&asset_server))
        .unwrap_or(true);

    if terrain_ready && objects_ready {
        info!("[Terrain] All world assets loaded. Transitioning to SceneBuilding.");
        next_state.set(AppState::SceneBuilding);
    }
}

fn spawn_terrain_tiles(
    mut commands: Commands,
    config: Res<ScenarioConfig>,
    terrain_assets: Res<TerrainAssets>,
    gltfs: Res<Assets<Gltf>>,
    gltf_meshes: Res<Assets<GltfMesh>>,
    meshes: Res<Assets<Mesh>>,
) {
    for entry in &terrain_assets.entries {
        let terrain_cfg = &config.world.terrains[entry.config_idx];
        let transform = terrain_transform(terrain_cfg.position, terrain_cfg.orientation_degrees);

        // Spawn the visual scene entity.
        commands.spawn((
            Name::new(format!("terrain/tile_{}", entry.config_idx)),
            SceneRoot(entry.scene.clone()),
            transform,
            TerrainMedium(terrain_cfg.medium.clone()),
        ));

        // Spawn trimesh colliders from the collider GLTF.
        if let Some(ref col_handle) = entry.collider {
            let Some(gltf) = gltfs.get(col_handle) else {
                warn!("[Terrain] Collider GLTF not found for tile {}.", entry.config_idx);
                continue;
            };
            spawn_trimesh_colliders_from_gltf(
                &mut commands,
                gltf,
                &gltf_meshes,
                &meshes,
                &transform,
                entry.config_idx,
            );
        }

        info!(
            "[Terrain] Spawned tile {} (medium: '{}') at ENU ({:.1}, {:.1}, {:.1})",
            entry.config_idx,
            terrain_cfg.medium,
            terrain_cfg.position[0],
            terrain_cfg.position[1],
            terrain_cfg.position[2],
        );
    }
}

// =========================================================================
// == Helpers ==
// =========================================================================

fn terrain_transform(position: [f64; 3], orientation_degrees: [f64; 3]) -> Transform {
    let roll  = orientation_degrees[0] * PI / 180.0;
    let pitch = orientation_degrees[1] * PI / 180.0;
    let yaw   = orientation_degrees[2] * PI / 180.0;
    let translation = Translation3::new(position[0], position[1], position[2]);
    let rotation = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
    enu_iso_to_bevy_transform(&Isometry3::from_parts(translation, rotation))
}

fn spawn_trimesh_colliders_from_gltf(
    commands: &mut Commands,
    gltf: &Gltf,
    gltf_meshes: &Assets<GltfMesh>,
    meshes: &Assets<Mesh>,
    _parent_transform: &Transform,
    tile_idx: usize,
) {
    for (name, gltf_mesh_handle) in &gltf.named_meshes {
        let Some(gltf_mesh) = gltf_meshes.get(gltf_mesh_handle) else {
            warn!("[Terrain] GltfMesh '{}' not loaded.", name);
            continue;
        };
        if gltf_mesh.primitives.is_empty() {
            continue;
        }
        let mesh_handle = gltf_mesh.primitives[0].mesh.clone();
        let Some(mesh) = meshes.get(&mesh_handle) else {
            warn!("[Terrain] Mesh data for '{}' not available.", name);
            continue;
        };
        match Collider::trimesh_from_mesh_with_config(mesh, TrimeshFlags::FIX_INTERNAL_EDGES) {
            Some(collider) => {
                commands.spawn((
                    collider,
                    RigidBody::Static,
                    Name::new(format!("terrain_col/tile_{}/{}", tile_idx, name)),
                ));
                info!("[Terrain] Created collider for mesh '{}'", name);
            }
            None => {
                warn!("[Terrain] Could not create trimesh collider for '{}'", name);
            }
        }
    }
}
