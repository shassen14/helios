//! Coordinate frame conversion helpers and the runtime TF tree.
//!
//! The vector/point boundary crosses through core's typed frame algebra: the
//! [`Bevy`] frame marker turns each axis swap into a real `Rotation`, and the
//! [`ToBevy`] / [`FromBevy`] traits carry any core frame across — dispatched on
//! the frame type — handing back typed `Point<Bevy>` / `FreeVector<Bevy>` values
//! that the cast helpers ([`point_bevy_to_vec3`] etc.) copy to `bevy::Vec3` at
//! the render edge. Poses cross the same way through [`ToBevy`] / [`FromBevy`] on
//! `Transform<From, To>`, with the [`transform_bevy_to_bevy_transform`] /
//! [`bevy_transform_to_transform_bevy`] casts at the `bevy::Transform` edge. The
//! [`TfTree`] resource and its update systems are here too.
//!
//! All axis-swap logic is centralized in `bevy_bridge.rs`. Never perform manual axis swaps
//! (e.g. `v.y = physics.z`) anywhere else in the codebase — every such swap is a latent bug.

mod bevy_bridge;
mod body_twist;

pub use bevy_bridge::{
    bevy_transform_to_transform_bevy, freevector_bevy_to_vec3, point_bevy_to_vec3,
    transform_bevy_to_bevy_transform, vec3_to_freevector_bevy, vec3_to_point_bevy, Bevy, FromBevy,
    ToBevy,
};
pub use body_twist::enu_twist_to_body_flu;

use bevy::prelude::{GlobalTransform, *};
use helios_core::frames::conventions::{Enu, Flu};
use helios_core::frames::transforms::{Convention, ErasedTransform, Transform as CoreTransform};
use helios_core::frames::FrameId;
use nalgebra::Isometry3;
use std::collections::HashMap;
use std::sync::Arc;

// =========================================================================
// == TF Tree Infrastructure (The "Service") ==
// =========================================================================

/// A Bevy component to mark any entity that should be tracked by the TF system.
#[derive(Component)]
pub struct TrackedFrame(pub Convention);

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

/// One tracked frame's complete state, held as a single value so a frame is
/// inserted and removed atomically. Storing the world pose, parent-relative
/// local pose, axis convention, and parent link together removes the drift
/// class that four parallel `HashMap<Entity, _>` invited — nothing to keep in
/// lockstep — and mirrors the "isometry plus convention as one value" shape the
/// [`ErasedTransform`] boundary already settled on.
#[derive(Debug)]
pub struct FrameNode {
    /// World pose, ENU. In sim this is Bevy's already-propagated global pose;
    /// [`TfTree::erased`] composes world poses rather than walking parents.
    world: Isometry3<f64>,
    /// Pose relative to `parent` (ENU for root frames, FLU for body-mounted
    /// sensors); equals `world` for a frame with no parent.
    local: Isometry3<f64>,
    /// Axis convention of this frame, carried through to `ErasedTransform` so a
    /// convention mismatch fails loudly at the typed crossing.
    convention: Convention,
    /// Parent entity; `None` for a world root.
    parent: Option<Entity>,
}

/// The Bevy resource that holds the complete transform graph for a single frame.
#[derive(Resource, Default, Debug)]
pub struct TfTree {
    entity_to_frame_node: HashMap<Entity, FrameNode>,

    // For user-facing API calls, configuration, and debugging.
    name_to_entity: HashMap<Arc<str>, Entity>,

    /// Maps a Bevy Entity ID back to its human-readable name for debugging/logging.
    pub entity_to_name: HashMap<Entity, Arc<str>>,

    /// Elapsed simulation time at the last rebuild.
    pub sim_time: f64,
}

impl TfTree {
    /// Resolves the transform between two frames at the tree's *current* state.
    ///
    /// The tree is latest-only: one pose per frame (the most recent physics
    /// step), no history and no time index, so every query is answered for
    /// "now" — which is why there is no time argument here and
    /// `SimRuntime::get_transform` discards its `at`. A buffered, interpolating
    /// tree (the tf2 model: binary-search the bracketing samples, slerp/lerp to
    /// the requested time, error rather than extrapolate out of range) is
    /// deferred until the estimated map->odom edge is what first makes
    /// "now != at" observable.
    pub fn erased(&self, from: FrameId, to: FrameId) -> Option<ErasedTransform> {
        let (from_pose, from_conv) = self.resolve(from)?;

        let (to_pose, to_conv) = self.resolve(to)?;

        let iso = from_pose.inverse() * to_pose;

        Some(ErasedTransform::from_parts(iso, from_conv, to_conv))
    }

    /// Looks up the world pose of a frame by its name.
    pub fn lookup_by_name(&self, frame_name: &str) -> Option<Isometry3<f64>> {
        let entity = self.name_to_entity.get(frame_name)?;
        Some(self.entity_to_frame_node.get(entity)?.world)
    }

    /// Looks up the world pose of a frame by its Entity ID.
    pub fn lookup_by_entity(&self, entity: Entity) -> Option<Isometry3<f64>> {
        Some(self.entity_to_frame_node.get(&entity)?.world)
    }

    /// Looks up the parent-relative pose of a frame by its Entity ID.
    pub fn lookup_local_by_entity(&self, entity: Entity) -> Option<Isometry3<f64>> {
        Some(self.entity_to_frame_node.get(&entity)?.local)
    }

    /// Returns an iterator over all tracked frames: `(entity, world_iso, local_iso, parent_entity)`.
    pub fn iter_frames(
        &self,
    ) -> impl Iterator<Item = (Entity, Isometry3<f64>, Isometry3<f64>, Option<Entity>)> + '_ {
        self.entity_to_frame_node
            .iter()
            .map(|(&entity, node)| (entity, node.world, node.local, node.parent))
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

    fn resolve(&self, frame: FrameId) -> Option<(Isometry3<f64>, Convention)> {
        match frame {
            FrameId::World => Some((Isometry3::identity(), Convention::Enu)),
            FrameId::Map(_) => Some((Isometry3::identity(), Convention::Enu)),
            // Odom is the estimator's reference frame. With no map→odom
            // correction yet, it is coincident with world (identity, ENU); the
            // world→odom drift lives in the estimate values, not this edge.
            FrameId::Odom(_) => Some((Isometry3::identity(), Convention::Enu)),
            FrameId::Body(handle) | FrameId::Sensor(handle) => {
                let entity = Entity::from_bits(handle.0);
                let node = self.entity_to_frame_node.get(&entity)?;

                Some((node.world, node.convention))
            }
        }
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
        .entity_to_frame_node
        .get(&parent_entity)
        .map(|node| node.world)
        .or_else(|| {
            all_transforms.get(parent_entity).ok().map(|t| {
                let pose: CoreTransform<Flu, Enu> =
                    bevy_transform_to_transform_bevy(t.compute_transform()).from_bevy();
                pose.into_inner()
            })
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
    added_query: Query<
        (Entity, &GlobalTransform, Option<&ChildOf>, &TrackedFrame),
        Added<TrackedFrame>,
    >,
    all_transforms: Query<&GlobalTransform>,
    mut removed: RemovedComponents<TrackedFrame>,
) {
    for entity in removed.read() {
        tf_tree.entity_to_frame_node.remove(&entity);
    }

    if added_query.is_empty() {
        return;
    }

    // Pass 1: world pose, convention, and parent link for every newly tracked
    // entity. `local` is seeded to `world` and overwritten in pass 2, which
    // runs for every added entity — the seed never survives.
    for (entity, gt, child_of, tracked) in &added_query {
        let world: CoreTransform<Flu, Enu> =
            bevy_transform_to_transform_bevy(gt.compute_transform()).from_bevy();
        let world_iso = world.into_inner();
        tf_tree.entity_to_frame_node.insert(
            entity,
            FrameNode {
                world: world_iso,
                local: world_iso,
                convention: tracked.0,
                parent: child_of.map(|c| c.parent()),
            },
        );
    }

    // Pass 2: local pose — requires pass 1 complete so parent world poses are present.
    for (entity, _, child_of, _) in &added_query {
        let world_iso = tf_tree.entity_to_frame_node[&entity].world;
        let local_iso = resolve_local_iso(&tf_tree, &all_transforms, child_of, world_iso);
        if let Some(node) = tf_tree.entity_to_frame_node.get_mut(&entity) {
            node.local = local_iso;
        }
    }
}

/// Incrementally updates world and local poses for any `TrackedFrame` whose
/// `GlobalTransform` changed since this system last ran.
///
/// Registered **twice** in the schedule:
/// - `SimulationSet::Precomputation` — captures first-tick initialization.
/// - `SimulationSet::StateSync` — runs immediately after Avian3D's physics step.
#[allow(clippy::type_complexity)]
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

    // Pass 1: world poses + parent links. The structural system owns node
    // creation; this system only updates. `get_mut` therefore leaves the
    // convention untouched (a fresh insert would wipe it every tick) and skips
    // any entity the structural system has not created yet — on the tick a
    // frame first appears, the structural system creates its node fully from
    // the same `GlobalTransform`, so the end state is correct in whichever
    // order the two run within `Precomputation`.
    for (entity, gt, child_of) in &changed_query {
        let world: CoreTransform<Flu, Enu> =
            bevy_transform_to_transform_bevy(gt.compute_transform()).from_bevy();
        if let Some(node) = tf_tree.entity_to_frame_node.get_mut(&entity) {
            node.world = world.into_inner();
            node.parent = child_of.map(|c| c.parent());
        }
    }

    // Pass 2: local poses — parent world poses already updated in pass 1.
    for (entity, _, child_of) in &changed_query {
        let Some(world_iso) = tf_tree.entity_to_frame_node.get(&entity).map(|n| n.world) else {
            continue;
        };
        let local_iso = resolve_local_iso(&tf_tree, &all_transforms, child_of, world_iso);
        if let Some(node) = tf_tree.entity_to_frame_node.get_mut(&entity) {
            node.local = local_iso;
        }
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
