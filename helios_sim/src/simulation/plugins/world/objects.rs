use std::collections::HashMap;

use avian3d::prelude::RigidBody;
use bevy::{
    asset::LoadState,
    gltf::{Gltf, GltfAssetLabel, GltfMesh},
    prelude::*,
};

use crate::prelude::*;
use crate::simulation::config::structs::world_object::WorldObjectPrefab;
use crate::simulation::config::PrefabCatalog;
use crate::simulation::core::app_state::AssetLoadSet;
use crate::simulation::core::components::{BoundingBox3D, SemanticLabel, WorldObjectType};
use super::object_helpers::{
    build_collider, placement_to_bevy_transform, spawn_object_trimesh_colliders, GltfObjectMeta,
};

// =========================================================================
// == Resource ==
// =========================================================================

/// Holds loaded visual scene handles and parsed prefab data for all unique
/// world object types referenced by the scenario. Populated during AssetLoading;
/// consumed during SceneBuilding.
#[derive(Resource, Default)]
pub struct WorldObjectAssets {
    pub prefabs: HashMap<String, WorldObjectPrefab>,
    pub scenes: HashMap<String, Handle<Scene>>,
    pub collider_gltfs: HashMap<String, Handle<Gltf>>,
}

impl WorldObjectAssets {
    pub fn all_loaded(&self, asset_server: &AssetServer) -> bool {
        let scenes_ok = self
            .scenes
            .values()
            .all(|h| matches!(asset_server.get_load_state(h), Some(LoadState::Loaded)));
        let colliders_ok = self
            .collider_gltfs
            .values()
            .all(|h| matches!(asset_server.get_load_state(h), Some(LoadState::Loaded)));
        scenes_ok && colliders_ok
    }
}

// =========================================================================
// == Plugin ==
// =========================================================================

pub struct WorldObjectPlugin;

impl Plugin for WorldObjectPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<WorldObjectAssets>()
            .add_systems(
                OnEnter(AppState::AssetLoading),
                start_object_asset_loading.in_set(AssetLoadSet::Kickoff),
            )
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                spawn_world_objects.in_set(SceneBuildSet::ProcessWorldObjects),
            )
            .add_systems(
                Update,
                read_object_gltf_extras.run_if(in_state(AppState::Running)),
            );
    }
}

// =========================================================================
// == Systems ==
// =========================================================================

fn start_object_asset_loading(
    mut assets: ResMut<WorldObjectAssets>,
    config: Res<ScenarioConfig>,
    catalog: Res<PrefabCatalog>,
    asset_server: Res<AssetServer>,
) {
    for placement in &config.world.objects {
        if assets.prefabs.contains_key(&placement.prefab) {
            continue;
        }

        let Some(raw_value) = catalog.0.get(&placement.prefab) else {
            error!(
                "[WorldObjects] Prefab '{}' not found in catalog.",
                placement.prefab
            );
            continue;
        };

        let prefab: WorldObjectPrefab =
            match figment::value::Value::deserialize::<WorldObjectPrefab>(raw_value) {
                Ok(p) => p,
                Err(e) => {
                    error!(
                        "[WorldObjects] Failed to parse prefab '{}': {}",
                        placement.prefab, e
                    );
                    continue;
                }
            };

        if let Some(ref mesh_path) = prefab.visual_mesh {
            let handle = asset_server.load(GltfAssetLabel::Scene(0).from_asset(mesh_path.clone()));
            info!("[WorldObjects] Loading visual mesh for '{}'", placement.prefab);
            assets.scenes.insert(placement.prefab.clone(), handle);
        }

        if let Some(ref col_path) = prefab.collider_mesh {
            let col_handle: Handle<Gltf> = asset_server.load(col_path.clone());
            info!("[WorldObjects] Loading collider mesh for '{}'", placement.prefab);
            assets.collider_gltfs.insert(placement.prefab.clone(), col_handle);
        }

        assets.prefabs.insert(placement.prefab.clone(), prefab);
    }
}

fn spawn_world_objects(
    mut commands: Commands,
    config: Res<ScenarioConfig>,
    assets: Res<WorldObjectAssets>,
    gltfs: Res<Assets<Gltf>>,
    gltf_meshes: Res<Assets<GltfMesh>>,
    meshes: Res<Assets<Mesh>>,
) {
    for (idx, placement) in config.world.objects.iter().enumerate() {
        let Some(prefab) = assets.prefabs.get(&placement.prefab) else {
            error!(
                "[WorldObjects] Skipping placement #{} — prefab '{}' was not loaded.",
                idx, placement.prefab
            );
            continue;
        };

        let transform =
            placement_to_bevy_transform(placement.position, placement.orientation_degrees);
        let scaled_transform = transform.with_scale(Vec3::from(placement.scale));

        let mut entity_cmds = commands.spawn((
            Name::new(format!("world_object/{}/{}", placement.prefab, idx)),
            scaled_transform,
            WorldObjectType(placement.prefab.clone()),
            SemanticLabel {
                label: prefab.label.clone(),
                class_id: prefab.class_id,
            },
        ));

        if let Some(scene_handle) = assets.scenes.get(&placement.prefab) {
            entity_cmds.insert(SceneRoot(scene_handle.clone()));
        }

        if let Some(bb) = prefab.bounding_box {
            entity_cmds.insert(BoundingBox3D {
                half_extents: Vec3::new(bb[0] * 0.5, bb[1] * 0.5, bb[2] * 0.5),
            });
        }

        if assets.collider_gltfs.contains_key(&placement.prefab) {
            // Trimesh colliders spawned below.
        } else if let Some(ref col_cfg) = prefab.collider {
            match build_collider(col_cfg) {
                Ok(collider) => {
                    entity_cmds.insert((RigidBody::Static, collider));
                }
                Err(msg) => {
                    warn!("[WorldObjects] Collider for '{}' skipped: {}", placement.prefab, msg);
                }
            }
        }

        info!(
            "[WorldObjects] Spawned '{}' at ENU ({:.1}, {:.1}, {:.1})",
            placement.prefab, placement.position[0], placement.position[1], placement.position[2],
        );

        if let Some(col_handle) = assets.collider_gltfs.get(&placement.prefab) {
            if let Some(gltf) = gltfs.get(col_handle) {
                spawn_object_trimesh_colliders(
                    &mut commands,
                    gltf,
                    &gltf_meshes,
                    &meshes,
                    scaled_transform,
                    &placement.prefab,
                    idx,
                );
            }
        }
    }
}

fn read_object_gltf_extras(
    mut commands: Commands,
    extras_query: Query<(&GltfExtras, &ChildOf), Added<GltfExtras>>,
    parent_query: Query<Option<&ChildOf>>,
    world_obj_query: Query<(), With<WorldObjectType>>,
    label_query: Query<(), With<SemanticLabel>>,
) {
    for (extras, first_parent) in &extras_query {
        let mut current: Entity = first_parent.parent();
        for _ in 0..5 {
            if world_obj_query.contains(current) {
                if label_query.contains(current) {
                    break;
                }
                if let Ok(meta) = serde_json::from_str::<GltfObjectMeta>(&extras.value) {
                    if let Some(label) = meta.label {
                        commands.entity(current).insert(SemanticLabel {
                            label,
                            class_id: meta.class_id.unwrap_or(0),
                        });
                    }
                }
                break;
            }
            match parent_query.get(current) {
                Ok(Some(p)) => current = p.parent(),
                _ => break,
            }
        }
    }
}
