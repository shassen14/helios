// helios_sim/src/simulation/plugins/world/object_helpers.rs
//
// Private helpers for world object spawning.

use std::f64::consts::PI;

use avian3d::prelude::{Collider, RigidBody, TrimeshFlags};
use bevy::{
    gltf::{Gltf, GltfMesh},
    prelude::*,
};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};

use crate::simulation::config::structs::world_object::WorldObjectCollider;
use crate::simulation::core::transforms::EnuWorldPose;

#[derive(serde::Deserialize)]
pub struct GltfObjectMeta {
    pub label: Option<String>,
    pub class_id: Option<u32>,
}

/// Converts an ENU position + RPY orientation (degrees) to a Bevy `Transform`.
pub fn placement_to_bevy_transform(position: [f64; 3], orientation_degrees: [f64; 3]) -> Transform {
    let roll = orientation_degrees[0] * PI / 180.0;
    let pitch = orientation_degrees[1] * PI / 180.0;
    let yaw = orientation_degrees[2] * PI / 180.0;

    let translation = Translation3::new(position[0], position[1], position[2]);
    let rotation = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
    let iso = Isometry3::from_parts(translation, rotation);
    Transform::from(EnuWorldPose(iso))
}

pub fn spawn_object_trimesh_colliders(
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
pub fn build_collider(cfg: &WorldObjectCollider) -> Result<Collider, String> {
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
