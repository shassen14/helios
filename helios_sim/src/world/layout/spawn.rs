//! Spawns the layout's objects: one rigid body per placement, carrying its
//! ground-truth labels, with its prefab's scene as a scaled visual child.
//!
//! The body is posed but never scaled: the collider is built at the
//! placement's scale instead, since Avian's own scaling of a hull is wrong
//! for a non-uniform scale. Only the visual child carries the scale.

use super::assets::ObjectAssets;
use super::error::error_report;
use super::geometry::{is_collider_node, GeometryError, PrefabGeometry};
use super::gltf_nodes::read_scene;
use super::resolved::{ResolvedBody, ResolvedWorldLayout};
use crate::core::components::{ObjectClass, ObjectInstanceId, WorldObjectType};
use crate::core::transforms::{per_axis_to_bevy, transform_bevy_to_bevy_transform, ToBevy};

use helios_core::spatial::conventions::Enu;
use helios_core::spatial::transforms::Transform as CoreTransform;

use avian3d::prelude::{Collider, Mass, RigidBody};
use bevy::gltf::Gltf;
use bevy::prelude::*;
use bevy::world_serialization::{WorldAsset, WorldAssetRoot, WorldInstanceReady};
use nalgebra::{Isometry3, Vector3};
use std::collections::HashMap;
use std::fmt;
use std::path::PathBuf;

/// Spawns every placement of the scenario's layout, when it has one,
/// panicking with every error when a prefab's asset cannot be used.
pub(super) fn spawn_world_objects(
    mut commands: Commands,
    layout: Option<Res<ResolvedWorldLayout>>,
    assets: Res<ObjectAssets>,
    gltfs: Res<Assets<Gltf>>,
) {
    let Some(layout) = layout else {
        return;
    };
    let result = prepare_prefabs(&layout, &assets, &gltfs)
        .and_then(|prefabs| spawn_objects(&mut commands, &layout, &prefabs));
    if let Err(errors) = result {
        let heading = format!("world layout `{}` cannot be spawned", layout.name());
        panic!("{}", error_report(&heading, &errors));
    }
    info!(
        "[WorldLayout] '{}': spawned {} objects",
        layout.name(),
        layout.placements().len()
    );
}

/// What every placement of one prefab shares: the scene it renders and the
/// geometry it collides with and is labelled by.
struct PreparedPrefab {
    scene: Handle<WorldAsset>,
    geometry: PrefabGeometry,
}

/// Derives each loaded prefab's geometry, in the order of the layout's
/// prefabs, or returns every problem found.
fn prepare_prefabs(
    layout: &ResolvedWorldLayout,
    assets: &ObjectAssets,
    gltfs: &Assets<Gltf>,
) -> Result<Vec<PreparedPrefab>, Vec<SpawnError>> {
    let mut prepared = Vec::with_capacity(layout.prefabs().len());
    let mut errors = Vec::new();
    for (index, prefab) in layout.prefabs().iter().enumerate() {
        let error = |problem| SpawnError::Prefab {
            prefab: prefab.key.clone(),
            mesh: prefab.mesh.clone(),
            problem,
        };
        let Some(gltf) = assets.gltf(index).and_then(|handle| gltfs.get(handle)) else {
            errors.push(error(PrefabProblem::NotLoaded));
            continue;
        };
        match prepare_prefab(gltf) {
            Ok(ready) => prepared.push(ready),
            Err(problems) => errors.extend(problems.into_iter().map(error)),
        }
    }
    if errors.is_empty() {
        Ok(prepared)
    } else {
        Err(errors)
    }
}

/// Reads one loaded `.glb`: the scene to render is the one the geometry is
/// derived from (the default scene, else the first).
fn prepare_prefab(gltf: &Gltf) -> Result<PreparedPrefab, Vec<PrefabProblem>> {
    let source = gltf
        .source
        .as_ref()
        .ok_or_else(|| vec![PrefabProblem::NoSource])?;
    let scene = gltf
        .default_scene
        .clone()
        .or_else(|| gltf.scenes.first().cloned())
        .ok_or_else(|| vec![PrefabProblem::Geometry(GeometryError::NoScene)])?;
    let roots = read_scene(source).map_err(|e| vec![PrefabProblem::Geometry(e)])?;
    let geometry = PrefabGeometry::derive(&roots).map_err(|errors| {
        errors
            .into_iter()
            .map(PrefabProblem::Geometry)
            .collect::<Vec<_>>()
    })?;
    Ok(PreparedPrefab { scene, geometry })
}

/// Spawns one body per placement. `prefabs` is in the order of the layout's
/// prefabs, as [`prepare_prefabs`] returns it.
fn spawn_objects(
    commands: &mut Commands,
    layout: &ResolvedWorldLayout,
    prefabs: &[PreparedPrefab],
) -> Result<(), Vec<SpawnError>> {
    let mut colliders = ColliderCache::default();
    let mut errors = Vec::new();

    for (placement, prefab) in layout.objects() {
        let index = placement.prefab_index();
        let Some(prepared) = prefabs.get(index) else {
            errors.push(SpawnError::Prefab {
                prefab: prefab.key.clone(),
                mesh: prefab.mesh.clone(),
                problem: PrefabProblem::NotLoaded,
            });
            continue;
        };
        let collider = if prefab.collides {
            match colliders.get(index, &prepared.geometry, &placement.scale) {
                Ok(collider) => Some(collider),
                Err(error) => {
                    errors.push(SpawnError::Collider {
                        placement: placement.name.clone(),
                        error,
                    });
                    continue;
                }
            }
        } else {
            None
        };

        let instance_id = layout.instance_id(placement);
        let mut object = commands.spawn((
            Name::new(instance_id.clone()),
            ObjectInstanceId(instance_id),
            ObjectClass(prefab.class),
            WorldObjectType(prefab.key.clone()),
            prepared
                .geometry
                .bounds()
                .scaled(&placement.scale)
                .in_bevy_frame(),
            object_transform(&placement.pose),
            Visibility::default(),
            rigid_body(placement.body),
        ));
        if let Some(collider) = collider {
            object.insert(collider);
        }
        if let ResolvedBody::Dynamic { mass_kg } = placement.body {
            // Overrides the mass Avian would derive from the collider's
            // volume; the inertia is rescaled to match.
            object.insert(Mass(mass_kg as f32));
        }
        object.with_child((
            ObjectVisual,
            WorldAssetRoot(prepared.scene.clone()),
            Transform::from_scale(visual_scale(&placement.scale)),
        ));
    }

    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
}

/// Colliders built once per distinct (prefab, scale) and shared: a clone
/// shares the shape, so 50 crates of one size hold one hull.
#[derive(Default)]
struct ColliderCache {
    built: HashMap<(usize, [u64; 3]), Collider>,
}

impl ColliderCache {
    fn get(
        &mut self,
        prefab_index: usize,
        geometry: &PrefabGeometry,
        scale: &Vector3<f64>,
    ) -> Result<Collider, GeometryError> {
        let key = (prefab_index, [scale.x, scale.y, scale.z].map(f64::to_bits));
        if let Some(collider) = self.built.get(&key) {
            return Ok(collider.clone());
        }
        let collider = geometry.collider(scale)?;
        self.built.insert(key, collider.clone());
        Ok(collider)
    }
}

/// Why a layout's objects cannot be spawned.
#[derive(Debug)]
enum SpawnError {
    Prefab {
        prefab: String,
        mesh: PathBuf,
        problem: PrefabProblem,
    },
    /// The collider could not be built at this placement's scale.
    Collider {
        placement: String,
        error: GeometryError,
    },
}

impl fmt::Display for SpawnError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Prefab {
                prefab,
                mesh,
                problem,
            } => write!(f, "prefab `{prefab}` ({}): {problem}", mesh.display()),
            Self::Collider { placement, error } => write!(f, "placement `{placement}`: {error}"),
        }
    }
}

/// What is wrong with one prefab's loaded `.glb`.
#[derive(Debug)]
enum PrefabProblem {
    NotLoaded,
    /// Loaded without the glTF document, so its geometry cannot be read.
    NoSource,
    Geometry(GeometryError),
}

impl fmt::Display for PrefabProblem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotLoaded => write!(f, "the file is not loaded"),
            Self::NoSource => write!(
                f,
                "the file was loaded without its glTF source; it must be loaded \
                 with `include_source`"
            ),
            Self::Geometry(error) => write!(f, "{error}"),
        }
    }
}

/// Marks the scene entity under a placed object, whose `col_` nodes are
/// removed once it has spawned.
#[derive(Component)]
pub(super) struct ObjectVisual;

/// Removes the collider parts from an object's rendered scene as soon as it
/// has spawned. They are despawned, not hidden: a hidden entity still costs
/// render extraction and may still be picked, and the collider was already
/// built from the glTF document.
pub(super) fn remove_collider_parts_when_ready(
    ready: On<WorldInstanceReady>,
    visuals: Query<(), With<ObjectVisual>>,
    children: Query<&Children>,
    names: Query<&Name>,
    mut commands: Commands,
) {
    if visuals.contains(ready.entity) {
        remove_collider_parts(ready.entity, &children, &names, &mut commands);
    }
}

/// Despawns every `col_` node under `root`, with its subtree.
fn remove_collider_parts(
    root: Entity,
    children: &Query<&Children>,
    names: &Query<&Name>,
    commands: &mut Commands,
) {
    let mut stack: Vec<Entity> = Vec::new();
    if let Ok(direct) = children.get(root) {
        stack.extend_from_slice(direct);
    }
    while let Some(entity) = stack.pop() {
        if names
            .get(entity)
            .is_ok_and(|name| is_collider_node(name.as_str()))
        {
            commands.entity(entity).despawn();
        } else if let Ok(below) = children.get(entity) {
            stack.extend_from_slice(below);
        }
    }
}

/// The placed object's pose in Bevy, at unit scale. Object axes cross as
/// ENU (see the geometry module), so the pose is an ENU-to-ENU transform.
fn object_transform(pose: &Isometry3<f64>) -> Transform {
    transform_bevy_to_bevy_transform(CoreTransform::<Enu, Enu>::from_isometry(*pose).to_bevy())
}

/// The visual's scale in Bevy axes, from the scale along the object's axes.
fn visual_scale(scale: &Vector3<f64>) -> Vec3 {
    let scale = per_axis_to_bevy::<Enu>(*scale).cast::<f32>();
    Vec3::new(scale.x, scale.y, scale.z)
}

fn rigid_body(body: ResolvedBody) -> RigidBody {
    match body {
        ResolvedBody::Static => RigidBody::Static,
        ResolvedBody::Dynamic { .. } => RigidBody::Dynamic,
    }
}

#[cfg(test)]
#[path = "spawn_tests.rs"]
mod tests;
