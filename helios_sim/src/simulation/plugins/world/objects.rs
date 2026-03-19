use std::collections::HashMap;
use std::f64::consts::PI;

use avian3d::prelude::{Collider, RigidBody, TrimeshFlags};
use bevy::{
    asset::LoadState,
    gltf::{Gltf, GltfAssetLabel, GltfMesh},
    prelude::*,
};

use crate::prelude::*;
use crate::simulation::config::structs::world_object::{WorldObjectCollider, WorldObjectPrefab};
use crate::simulation::config::PrefabCatalog;
use crate::simulation::core::app_state::AssetLoadSet;
use crate::simulation::core::components::{BoundingBox3D, SemanticLabel, WorldObjectType};
use crate::simulation::core::transforms::EnuWorldPose;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};

// =========================================================================
// == Resource ==
// =========================================================================

/// Holds loaded visual scene handles and parsed prefab data for all unique
/// world object types referenced by the scenario. Populated during AssetLoading;
/// consumed during SceneBuilding.
#[derive(Resource, Default)]
pub struct WorldObjectAssets {
    /// Parsed prefab definitions keyed by catalog key (e.g. `"objects.stop_sign"`).
    pub prefabs: HashMap<String, WorldObjectPrefab>,
    /// Loaded Scene handles for objects that have a `visual_mesh`. Keyed by catalog key.
    pub scenes: HashMap<String, Handle<Scene>>,
    /// Loaded GLTF handles for objects that use `collider_mesh`. Keyed by catalog key.
    pub collider_gltfs: HashMap<String, Handle<Gltf>>,
}

impl WorldObjectAssets {
    /// Returns true when all registered scene handles have finished loading.
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

/// Resolves all unique prefab types from the catalog and kicks off GLB loading.
/// Runs once in `AssetLoading / Kickoff`.
fn start_object_asset_loading(
    mut assets: ResMut<WorldObjectAssets>,
    config: Res<ScenarioConfig>,
    catalog: Res<PrefabCatalog>,
    asset_server: Res<AssetServer>,
) {
    for placement in &config.world.objects {
        if assets.prefabs.contains_key(&placement.prefab) {
            continue; // Already resolved this prefab type.
        }

        let Some(raw_value) = catalog.0.get(&placement.prefab) else {
            error!(
                "[WorldObjects] Prefab '{}' not found in catalog. \
                 Check that configs/catalog/objects/<name>.toml exists.",
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
            info!(
                "[WorldObjects] Loading visual mesh for '{}': {:?}",
                placement.prefab, mesh_path
            );
            assets.scenes.insert(placement.prefab.clone(), handle);
        }

        if let Some(ref col_path) = prefab.collider_mesh {
            let col_handle: Handle<Gltf> = asset_server.load(col_path.clone());
            info!(
                "[WorldObjects] Loading collider mesh for '{}': {:?}",
                placement.prefab, col_path
            );
            assets
                .collider_gltfs
                .insert(placement.prefab.clone(), col_handle);
        }

        assets.prefabs.insert(placement.prefab.clone(), prefab);
    }
}

/// Spawns all world object entities. Runs once in `SceneBuilding / ProcessWorldObjects`.
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

        let entity_name = format!("world_object/{}/{}", placement.prefab, idx);

        let mut entity_cmds = commands.spawn((
            Name::new(entity_name),
            scaled_transform,
            WorldObjectType(placement.prefab.clone()),
            SemanticLabel {
                label: prefab.label.clone(),
                class_id: prefab.class_id,
            },
        ));

        // Visual mesh.
        if let Some(scene_handle) = assets.scenes.get(&placement.prefab) {
            entity_cmds.insert(SceneRoot(scene_handle.clone()));
        }

        // Semantic bounding box.
        if let Some(bb) = prefab.bounding_box {
            entity_cmds.insert(BoundingBox3D {
                half_extents: Vec3::new(bb[0] * 0.5, bb[1] * 0.5, bb[2] * 0.5),
            });
        }

        // Physics: prefer collider_mesh (trimesh from GLB) over primitive [collider].
        if assets.collider_gltfs.contains_key(&placement.prefab) {
            // Spawned below via spawn_object_trimesh_colliders.
        } else if let Some(ref col_cfg) = prefab.collider {
            match build_collider(col_cfg) {
                Ok(collider) => {
                    entity_cmds.insert((RigidBody::Static, collider));
                }
                Err(msg) => {
                    warn!(
                        "[WorldObjects] Collider for '{}' skipped: {}",
                        placement.prefab, msg
                    );
                }
            }
        }

        info!(
            "[WorldObjects] Spawned '{}' at ENU ({:.1}, {:.1}, {:.1})",
            placement.prefab, placement.position[0], placement.position[1], placement.position[2],
        );

        // Trimesh colliders from a dedicated collision GLB.
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

#[derive(serde::Deserialize)]
struct GltfObjectMeta {
    label: Option<String>,
    class_id: Option<u32>,
}

/// Reads GLTF extras embedded by Blender custom properties and attaches
/// `SemanticLabel` to the parent world object entity if not already set.
/// Runs async in Update since SceneRoot instantiation is deferred.
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
                    break; // TOML-provided label takes precedence.
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

// =========================================================================
// == Helpers ==
// =========================================================================

/// Converts an ENU position + RPY orientation (degrees) to a Bevy `Transform`.
/// Uses `enu_iso_to_bevy_transform` so the axis conversion is always centralised.
fn placement_to_bevy_transform(position: [f64; 3], orientation_degrees: [f64; 3]) -> Transform {
    let roll = orientation_degrees[0] * PI / 180.0;
    let pitch = orientation_degrees[1] * PI / 180.0;
    let yaw = orientation_degrees[2] * PI / 180.0;

    let translation = Translation3::new(position[0], position[1], position[2]);
    let rotation = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
    let iso = Isometry3::from_parts(translation, rotation);
    Transform::from(EnuWorldPose(iso))
}

fn spawn_object_trimesh_colliders(
    commands: &mut Commands,
    gltf: &Gltf,
    gltf_meshes: &Assets<GltfMesh>,
    meshes: &Assets<Mesh>,
    transform: Transform,
    prefab_key: &str,
    instance_idx: usize,
) {
    for (name, gltf_mesh_handle) in &gltf.named_meshes {
        let Some(gltf_mesh) = gltf_meshes.get(gltf_mesh_handle) else {
            continue;
        };
        if gltf_mesh.primitives.is_empty() {
            continue;
        }
        let mesh_handle = gltf_mesh.primitives[0].mesh.clone();
        let Some(mesh) = meshes.get(&mesh_handle) else {
            continue;
        };
        if let Some(collider) =
            Collider::trimesh_from_mesh_with_config(mesh, TrimeshFlags::FIX_INTERNAL_EDGES)
        {
            commands.spawn((
                collider,
                RigidBody::Static,
                transform,
                Name::new(format!(
                    "world_object_col/{}/{}/{}",
                    prefab_key, instance_idx, name
                )),
            ));
        }
    }
}

/// Builds an Avian3D `Collider` from a `WorldObjectCollider` config.
fn build_collider(cfg: &WorldObjectCollider) -> Result<Collider, String> {
    match cfg.shape.as_str() {
        "box" => {
            let he = cfg
                .half_extents
                .ok_or("'box' collider requires `half_extents`")?;
            Ok(Collider::cuboid(he[0], he[1], he[2]))
        }
        "sphere" => {
            let r = cfg.radius.ok_or("'sphere' collider requires `radius`")?;
            Ok(Collider::sphere(r))
        }
        "capsule" => {
            let r = cfg.radius.ok_or("'capsule' collider requires `radius`")?;
            let hh = cfg
                .half_height
                .ok_or("'capsule' collider requires `half_height`")?;
            // Avian `capsule(height, radius)` where height is the cylinder section only.
            Ok(Collider::capsule(hh * 2.0, r))
        }
        "cylinder" => {
            let r = cfg.radius.ok_or("'cylinder' collider requires `radius`")?;
            let hh = cfg
                .half_height
                .ok_or("'cylinder' collider requires `half_height`")?;
            Ok(Collider::cylinder(hh * 2.0, r))
        }
        other => Err(format!(
            "Unknown collider shape '{}'. Use box/sphere/capsule/cylinder.",
            other
        )),
    }
}
