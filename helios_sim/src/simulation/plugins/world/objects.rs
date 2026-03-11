use std::collections::HashMap;
use std::f64::consts::PI;

use avian3d::prelude::{Collider, RigidBody};
use bevy::{asset::LoadState, gltf::GltfAssetLabel, prelude::*};

use crate::prelude::*;
use crate::simulation::config::PrefabCatalog;
use crate::simulation::config::structs::world_object::{WorldObjectCollider, WorldObjectPrefab};
use crate::simulation::core::app_state::AssetLoadSet;
use crate::simulation::core::components::{BoundingBox3D, SemanticLabel, WorldObjectType};
use crate::simulation::core::transforms::enu_iso_to_bevy_transform;
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
}

impl WorldObjectAssets {
    /// Returns true when all registered scene handles have finished loading.
    pub fn all_loaded(&self, asset_server: &AssetServer) -> bool {
        self.scenes.values().all(|h| {
            matches!(asset_server.get_load_state(h), Some(LoadState::Loaded))
        })
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

        let prefab: WorldObjectPrefab = match figment::value::Value::deserialize::<WorldObjectPrefab>(raw_value) {
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
            let handle = asset_server
                .load(GltfAssetLabel::Scene(0).from_asset(mesh_path.clone()));
            info!(
                "[WorldObjects] Loading visual mesh for '{}': {:?}",
                placement.prefab, mesh_path
            );
            assets.scenes.insert(placement.prefab.clone(), handle);
        }

        assets.prefabs.insert(placement.prefab.clone(), prefab);
    }
}

/// Spawns all world object entities. Runs once in `SceneBuilding / ProcessWorldObjects`.
fn spawn_world_objects(
    mut commands: Commands,
    config: Res<ScenarioConfig>,
    assets: Res<WorldObjectAssets>,
) {
    for (idx, placement) in config.world.objects.iter().enumerate() {
        let Some(prefab) = assets.prefabs.get(&placement.prefab) else {
            error!(
                "[WorldObjects] Skipping placement #{} — prefab '{}' was not loaded.",
                idx, placement.prefab
            );
            continue;
        };

        let transform = placement_to_bevy_transform(placement.position, placement.orientation_degrees);
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

        // Physics collider.
        if let Some(ref col_cfg) = prefab.collider {
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
            placement.prefab,
            placement.position[0],
            placement.position[1],
            placement.position[2],
        );
    }
}

// =========================================================================
// == Helpers ==
// =========================================================================

/// Converts an ENU position + RPY orientation (degrees) to a Bevy `Transform`.
/// Uses `enu_iso_to_bevy_transform` so the axis conversion is always centralised.
fn placement_to_bevy_transform(position: [f64; 3], orientation_degrees: [f64; 3]) -> Transform {
    let roll  = orientation_degrees[0] * PI / 180.0;
    let pitch = orientation_degrees[1] * PI / 180.0;
    let yaw   = orientation_degrees[2] * PI / 180.0;

    let translation = Translation3::new(position[0], position[1], position[2]);
    let rotation = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
    let iso = Isometry3::from_parts(translation, rotation);
    enu_iso_to_bevy_transform(&iso)
}

/// Builds an Avian3D `Collider` from a `WorldObjectCollider` config.
fn build_collider(cfg: &WorldObjectCollider) -> Result<Collider, String> {
    match cfg.shape.as_str() {
        "box" => {
            let he = cfg.half_extents.ok_or("'box' collider requires `half_extents`")?;
            Ok(Collider::cuboid(he[0], he[1], he[2]))
        }
        "sphere" => {
            let r = cfg.radius.ok_or("'sphere' collider requires `radius`")?;
            Ok(Collider::sphere(r))
        }
        "capsule" => {
            let r  = cfg.radius.ok_or("'capsule' collider requires `radius`")?;
            let hh = cfg.half_height.ok_or("'capsule' collider requires `half_height`")?;
            // Avian `capsule(height, radius)` where height is the cylinder section only.
            Ok(Collider::capsule(hh * 2.0, r))
        }
        "cylinder" => {
            let r  = cfg.radius.ok_or("'cylinder' collider requires `radius`")?;
            let hh = cfg.half_height.ok_or("'cylinder' collider requires `half_height`")?;
            Ok(Collider::cylinder(hh * 2.0, r))
        }
        other => Err(format!("Unknown collider shape '{}'. Use box/sphere/capsule/cylinder.", other)),
    }
}
