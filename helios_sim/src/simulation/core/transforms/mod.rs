// helios_sim/src/simulation/core/transforms/mod.rs
//! Coordinate frame conversion helpers and the runtime TF tree.
//!
//! Provides typed frame newtypes ([`EnuBodyPose`], [`EnuWorldPose`], [`FluLocalPose`],
//! [`EnuVector`], [`FluVector`]) with `From` impls for converting between ENU/FLU and
//! Bevy coordinate systems. The [`TfTree`] resource and its update systems are here too.
//!
//! All axis-swap logic is centralized in `bevy_bridge.rs`. Never perform manual axis swaps
//! (e.g. `v.y = physics.z`) anywhere else in the codebase — every such swap is a latent bug.

mod bevy_bridge;
mod constants;
mod frame_types;

pub use frame_types::{EnuBodyPose, EnuVector, EnuWorldPose, FluLocalPose, FluVector};

use bevy::prelude::{GlobalTransform, *};
use helios_core::prelude::{FrameHandle, TfProvider};
use nalgebra::Isometry3;
use std::collections::HashMap;
use std::sync::Arc;

// =========================================================================
// == TF Tree Infrastructure (The "Service") ==
// =========================================================================

/// A Bevy component to mark any entity that should be tracked by the TF system.
#[derive(Component)]
pub struct TrackedFrame;

/// A single named frame's pose snapshot, published to TopicBus after each
/// physics step so Foxglove can plot any frame's trajectory over time.
#[derive(Clone, Debug)]
pub struct TfFramePose {
    /// Elapsed simulation time (seconds) when this snapshot was taken.
    pub sim_time: f64,
    /// Name of this frame (e.g. "truck/imu").
    pub frame_name: Arc<str>,
    /// Name of the parent frame; "world" if this is a root frame.
    pub parent_frame: Arc<str>,
    // World pose in ENU
    pub pos_x: f64,
    pub pos_y: f64,
    pub pos_z: f64,
    pub quat_x: f64,
    pub quat_y: f64,
    pub quat_z: f64,
    pub quat_w: f64,
    // Parent-relative pose (ENU for root frames, FLU for body-mounted sensors)
    pub local_pos_x: f64,
    pub local_pos_y: f64,
    pub local_pos_z: f64,
    pub local_quat_x: f64,
    pub local_quat_y: f64,
    pub local_quat_z: f64,
    pub local_quat_w: f64,
}

/// The Bevy resource that holds the complete transform graph for a single frame.
#[derive(Resource, Default, Debug)]
pub struct TfTree {
    // For fast, internal lookups using the entity ID.
    transforms_to_world: HashMap<Entity, Isometry3<f64>>,

    // Parent-relative poses in ENU (FLU for sensor children).
    local_transforms: HashMap<Entity, Isometry3<f64>>,

    // Parent entity for each tracked frame; None = world root.
    parent_map: HashMap<Entity, Option<Entity>>,

    // For user-facing API calls, configuration, and debugging.
    name_to_entity: HashMap<Arc<str>, Entity>,

    /// Maps a Bevy Entity ID back to its human-readable name for debugging/logging.
    pub entity_to_name: HashMap<Entity, Arc<str>>,

    /// Elapsed simulation time at the last rebuild.
    pub sim_time: f64,
}

impl TfTree {
    /// Looks up the world pose of a frame by its name.
    pub fn lookup_by_name(&self, frame_name: &str) -> Option<Isometry3<f64>> {
        let entity = self.name_to_entity.get(frame_name)?;
        self.transforms_to_world.get(entity).copied()
    }

    /// Looks up the world pose of a frame by its Entity ID.
    pub fn lookup_by_entity(&self, entity: Entity) -> Option<Isometry3<f64>> {
        self.transforms_to_world.get(&entity).copied()
    }

    /// Looks up the parent-relative pose of a frame by its Entity ID.
    pub fn lookup_local_by_entity(&self, entity: Entity) -> Option<Isometry3<f64>> {
        self.local_transforms.get(&entity).copied()
    }

    /// Returns an iterator over all tracked frames: `(entity, world_iso, local_iso, parent_entity)`.
    pub fn iter_frames(
        &self,
    ) -> impl Iterator<Item = (Entity, Isometry3<f64>, Isometry3<f64>, Option<Entity>)> + '_ {
        self.transforms_to_world
            .iter()
            .map(|(&entity, &world_iso)| {
                let local_iso = self
                    .local_transforms
                    .get(&entity)
                    .copied()
                    .unwrap_or(world_iso);
                let parent = self.parent_map.get(&entity).copied().flatten();
                (entity, world_iso, local_iso, parent)
            })
    }

    pub fn get_transform_by_name(
        &self,
        from_frame: &str,
        to_frame: &str,
    ) -> Option<Isometry3<f64>> {
        let pose_from_world = self.lookup_by_name(from_frame)?;
        let pose_to_world = self.lookup_by_name(to_frame)?;
        Some(pose_from_world.inverse() * pose_to_world)
    }
}

impl TfProvider for TfTree {
    fn get_transform(
        &self,
        from_frame: FrameHandle,
        to_frame: FrameHandle,
    ) -> Option<Isometry3<f64>> {
        let from_entity = Entity::from_bits(from_frame.0);
        let to_entity = Entity::from_bits(to_frame.0);
        let pose_from_world = self.transforms_to_world.get(&from_entity)?;
        let pose_to_world = self.transforms_to_world.get(&to_entity)?;
        Some(pose_from_world.inverse() * pose_to_world)
    }

    fn world_pose(&self, frame: FrameHandle) -> Option<Isometry3<f64>> {
        let entity = Entity::from_bits(frame.0);
        self.transforms_to_world.get(&entity).copied()
    }
}

// ---------------------------------------------------------------------------
// Private helper
// ---------------------------------------------------------------------------

/// Resolves the parent-relative (local) pose for a tracked entity.
fn resolve_local_iso(
    tf_tree: &TfTree,
    all_transforms: &Query<&GlobalTransform>,
    child_of: Option<&ChildOf>,
    world_iso: Isometry3<f64>,
) -> Isometry3<f64> {
    let Some(parent_entity) = child_of.map(|c| c.parent()) else {
        return world_iso;
    };
    let parent_world = tf_tree
        .transforms_to_world
        .get(&parent_entity)
        .copied()
        .or_else(|| {
            all_transforms
                .get(parent_entity)
                .ok()
                .map(|t| EnuBodyPose::from(t).0)
        });
    parent_world
        .map(|pw| pw.inverse() * world_iso)
        .unwrap_or(world_iso)
}

// =========================================================================
// == Systems ==
// =========================================================================

/// Handles structural changes to the TF graph: entities gaining or losing `TrackedFrame`.
///
/// Runs in `SimulationSet::Precomputation`.
pub fn tf_tree_structural_system(
    mut tf_tree: ResMut<TfTree>,
    added_query: Query<(Entity, &GlobalTransform, Option<&ChildOf>), Added<TrackedFrame>>,
    all_transforms: Query<&GlobalTransform>,
    mut removed: RemovedComponents<TrackedFrame>,
) {
    for entity in removed.read() {
        tf_tree.transforms_to_world.remove(&entity);
        tf_tree.local_transforms.remove(&entity);
        tf_tree.parent_map.remove(&entity);
    }

    if added_query.is_empty() {
        return;
    }

    // Pass 1: world pose + parent link for every newly tracked entity.
    for (entity, gt, child_of) in &added_query {
        tf_tree
            .transforms_to_world
            .insert(entity, EnuBodyPose::from(gt).0);
        tf_tree
            .parent_map
            .insert(entity, child_of.map(|c| c.parent()));
    }

    // Pass 2: local pose — requires pass 1 complete so parent world poses are present.
    for (entity, _, child_of) in &added_query {
        let world_iso = tf_tree.transforms_to_world[&entity];
        let local_iso = resolve_local_iso(&tf_tree, &all_transforms, child_of, world_iso);
        tf_tree.local_transforms.insert(entity, local_iso);
    }
}

/// Incrementally updates world and local poses for any `TrackedFrame` whose
/// `GlobalTransform` changed since this system last ran.
///
/// Registered **twice** in the schedule:
/// - `SimulationSet::Precomputation` — captures first-tick initialization.
/// - `SimulationSet::StateSync` — runs immediately after Avian3D's physics step.
pub fn tf_tree_incremental_update_system(
    mut tf_tree: ResMut<TfTree>,
    changed_query: Query<
        (Entity, &GlobalTransform, Option<&ChildOf>),
        (With<TrackedFrame>, Changed<GlobalTransform>),
    >,
    all_transforms: Query<&GlobalTransform>,
    time: Res<Time>,
) {
    if changed_query.is_empty() {
        return;
    }

    // Pass 1: world poses + parent links.
    for (entity, gt, child_of) in &changed_query {
        tf_tree
            .transforms_to_world
            .insert(entity, EnuBodyPose::from(gt).0);
        tf_tree
            .parent_map
            .insert(entity, child_of.map(|c| c.parent()));
    }

    // Pass 2: local poses — parent world poses already updated in pass 1.
    for (entity, _, child_of) in &changed_query {
        let world_iso = tf_tree.transforms_to_world[&entity];
        let local_iso = resolve_local_iso(&tf_tree, &all_transforms, child_of, world_iso);
        tf_tree.local_transforms.insert(entity, local_iso);
    }

    tf_tree.sim_time = time.elapsed_secs_f64();
}

/// System that runs ONCE to build the static name-to-entity mappings.
pub fn build_static_tf_maps(
    mut tf_tree: ResMut<TfTree>,
    query: Query<(Entity, &Name), With<TrackedFrame>>,
) {
    info!("Building static TF name maps...");
    tf_tree.name_to_entity.clear();
    tf_tree.entity_to_name.clear();

    for (entity, name) in &query {
        let frame_name: Arc<str> = Arc::from(name.as_str());
        tf_tree.name_to_entity.insert(frame_name.clone(), entity);
        tf_tree.entity_to_name.insert(entity, frame_name);
    }
}
