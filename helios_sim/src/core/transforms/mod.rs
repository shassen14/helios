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
    bevy_transform_to_transform_bevy, frame_triad_to_bevy, freevector_bevy_to_vec3,
    per_axis_to_bevy, point_bevy_to_vec3, transform_bevy_to_bevy_transform,
    vec3_to_freevector_bevy, vec3_to_point_bevy, Bevy, FromBevy, ToBevy,
};
pub use body_twist::enu_twist_to_body_flu;

use bevy::prelude::{GlobalTransform, *};
use helios_core::spatial::conventions::{Enu, Flu};
use helios_core::spatial::transforms::{
    Convention, EdgeEnds, ErasedTransform, Transform as CoreTransform,
};
use helios_core::spatial::FrameId;
use nalgebra::Isometry3;
use std::collections::HashMap;

// =========================================================================
// == TF Tree Infrastructure (The "Service") ==
// =========================================================================

/// Marks an entity whose pose the TF tree tracks, carrying the frame's
/// coordinate identity and axis convention as one value.
///
/// The [`FrameId`] is the tree's key. Every spawner stamps the same
/// `(agent, leaf)` the runtime builds that frame's `MeasurementModel` for —
/// the leaf is the installation's fit key for a sensor, the mount's name for
/// a topology mount, `base_link`/`odom` for the spine — so a reading, its tf
/// edge, and the filter's model all name one identity.
///
/// The convention comes from the frame's owner (the device kind, the
/// topology, the estimate relay) and only names the frame's axes. The pose
/// already carries them: an installation's rotation is the device's actual
/// axes in the body, and the tree reads every pose back through the same FLU
/// basis its spawner placed it with, so a non-FLU device needs no other
/// conversion. The tag rides along to the [`ErasedTransform`] boundary so a
/// reader expecting other axes fails loudly at the typed crossing.
#[derive(Component)]
pub struct TrackedFrame {
    pub id: FrameId,
    pub convention: Convention,
}

impl TrackedFrame {
    pub fn new(id: FrameId, convention: Convention) -> Self {
        Self { id, convention }
    }
}

/// One tracked frame's resolved state: its world pose (ENU) and axis
/// convention. [`TfTree::erased`] composes world poses rather than walking
/// parents, so a node needs no parent link — the world pose is Bevy's
/// already-propagated global transform, captured each physics step.
#[derive(Debug)]
pub struct FrameNode {
    world: Isometry3<f64>,
    convention: Convention,
}

/// The Bevy resource that holds the complete transform graph, keyed directly by
/// [`FrameId`].
///
/// The hot path ([`erased`](TfTree::erased) → [`resolve`](TfTree::resolve)) is a
/// single `HashMap<FrameId, _>` lookup per frame. `entity_to_frame` is a cold
/// reverse index touched only when a `TrackedFrame` is removed: a despawned
/// `Entity` no longer carries its `FrameId` component, so the eviction path
/// needs the entity → frame mapping to know which node to drop.
///
/// Keying by `FrameId` means **one entity per `(agent, leaf)`** — two entities
/// stamped with the same `FrameId` collide and the second overwrites the first.
/// This already holds: one `base_link` per agent, and channel names are unique
/// per agent (the pipeline build refuses a name naming two channels).
#[derive(Resource, Default, Debug)]
pub struct TfTree {
    frame_to_node: HashMap<FrameId, FrameNode>,

    /// Cold `Entity → FrameId` index, touched only on structural add/remove so a
    /// despawned entity's node can be evicted after its `FrameId` component is
    /// gone.
    entity_to_frame: HashMap<Entity, FrameId>,

    /// The world's datum: the global frame at the scene origin, whose pose is
    /// the identity. `None` until the world layout resolves.
    datum: Option<FrameId>,

    /// Elapsed simulation time at the last rebuild.
    pub sim_time: f64,
}

impl TfTree {
    /// Roots the tree at the world's datum, the frame every tracked pose is
    /// measured from.
    pub fn set_datum(&mut self, datum: FrameId) {
        self.datum = Some(datum);
    }

    /// The world's datum, once the world layout has resolved.
    pub fn datum(&self) -> Option<&FrameId> {
        self.datum.as_ref()
    }

    /// The pose of `from` expressed in `to`, as [`TfProvider`](helios_core::spatial::tf::TfProvider) orders a
    /// lookup, at the tree's *current* state.
    ///
    /// The tree is latest-only: one pose per frame (the most recent physics
    /// step), no history and no time index, so every query is answered for
    /// "now" — which is why there is no time argument here. The estimated tree
    /// the pipeline reads is a separate, time-buffered `TfBuffer` (the tf2 model:
    /// binary-search the bracketing samples, slerp/lerp to the requested time,
    /// error rather than extrapolate out of range); this truth tree stays
    /// latest-only because sensor generation and viz only ever want "now".
    pub fn erased(&self, from: FrameId, to: FrameId) -> Option<ErasedTransform> {
        let (from_pose, from_conv) = self.resolve(from)?;

        let (to_pose, to_conv) = self.resolve(to)?;

        // Each pose is its frame's in the world, so `to`'s inverse after
        // `from`'s is the pose of `from` in `to`.
        let iso = to_pose.inverse() * from_pose;

        Some(ErasedTransform::new(
            iso,
            EdgeEnds {
                child: from_conv,
                parent: to_conv,
            },
        ))
    }

    /// Resolves one frame to its world pose and convention.
    ///
    /// Predicate dispatch over the frame's identity: the world's datum and —
    /// until a localizer publishes a `map -> odom` correction — every agent's
    /// `map`/`odom` root resolve to the identity ENU pose, the world→odom drift
    /// living in the estimate values rather than these edges. Every other frame
    /// (a `base_link`, a sensor) is one lookup into the pose store.
    fn resolve(&self, frame: FrameId) -> Option<(Isometry3<f64>, Convention)> {
        if self.datum.as_ref() == Some(&frame) {
            return Some((Isometry3::identity(), Convention::Enu));
        }

        if frame.is_map() || frame.is_odom() {
            return Some((Isometry3::identity(), Convention::Enu));
        }

        let node = self.frame_to_node.get(&frame)?;

        Some((node.world, node.convention))
    }
}

// =========================================================================
// == Systems ==
// =========================================================================

/// Handles structural changes to the TF graph: entities gaining or losing
/// `TrackedFrame`. Keys the pose store by each frame's stamped [`FrameId`] and
/// records the cold `Entity → FrameId` index the eviction path needs.
///
/// Runs in `SimulationSet::Precomputation`.
pub fn tf_tree_structural_system(
    mut tf_tree: ResMut<TfTree>,
    added_query: Query<(Entity, &GlobalTransform, &TrackedFrame), Added<TrackedFrame>>,
    mut removed: RemovedComponents<TrackedFrame>,
) {
    for entity in removed.read() {
        if let Some(frame) = tf_tree.entity_to_frame.remove(&entity) {
            tf_tree.frame_to_node.remove(&frame);
        }
    }

    for (entity, gt, tracked) in &added_query {
        let world: CoreTransform<Flu, Enu> =
            bevy_transform_to_transform_bevy(gt.compute_transform()).from_bevy();
        let frame = tracked.id.clone();
        tf_tree.entity_to_frame.insert(entity, frame.clone());
        tf_tree.frame_to_node.insert(
            frame,
            FrameNode {
                world: world.into_inner(),
                convention: tracked.convention,
            },
        );
    }
}

/// Incrementally refreshes the world pose of any `TrackedFrame` whose
/// `GlobalTransform` changed since this system last ran.
///
/// Registered **twice** in the schedule:
/// - `SimulationSet::Precomputation` — captures first-tick initialization.
/// - `SimulationSet::StateSync` — runs immediately after Avian3D's physics step.
///
/// The structural system owns node creation; this one only updates, so `get_mut`
/// leaves the convention untouched and skips any entity the structural system
/// has not created yet. On the tick a frame first appears, the structural system
/// (chained before this in `Precomputation`) creates the node fully from the
/// same `GlobalTransform`, so the end state is correct.
pub fn tf_tree_incremental_update_system(
    mut tf_tree: ResMut<TfTree>,
    changed_query: Query<(&GlobalTransform, &TrackedFrame), Changed<GlobalTransform>>,
    time: Res<Time>,
) {
    if changed_query.is_empty() {
        return;
    }

    for (gt, tracked) in &changed_query {
        let world: CoreTransform<Flu, Enu> =
            bevy_transform_to_transform_bevy(gt.compute_transform()).from_bevy();
        if let Some(node) = tf_tree.frame_to_node.get_mut(&tracked.id) {
            node.world = world.into_inner();
        }
    }

    tf_tree.sim_time = time.elapsed_secs_f64();
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::prelude::AgentId;

    const DATUM: &str = "yard";

    fn rooted() -> TfTree {
        let mut tree = TfTree::default();
        tree.set_datum(FrameId::global(DATUM));
        tree
    }

    #[test]
    fn the_datum_resolves_to_the_scene_origin() {
        let datum = FrameId::global(DATUM);
        let transform = rooted()
            .erased(datum.clone(), datum)
            .expect("the datum resolves");
        assert_eq!(transform.isometry(), Isometry3::identity());
    }

    #[test]
    fn a_global_frame_other_than_the_datum_is_not_the_origin() {
        // A fixture or another site's datum is not this world's origin; until
        // something tracks it, it does not resolve.
        let tree = rooted();
        let elsewhere = FrameId::global("other_site");
        assert!(tree.erased(FrameId::global(DATUM), elsewhere).is_none());
    }

    #[test]
    fn nothing_is_the_origin_before_the_world_resolves() {
        let datum = FrameId::global(DATUM);
        assert!(TfTree::default().erased(datum.clone(), datum).is_none());
    }

    #[test]
    fn a_frame_resolves_as_its_pose_in_the_other_tagged_child_then_parent() {
        // A lidar 2 m east of the datum: its pose in the datum is +2 m east,
        // and the datum's in the lidar the inverse.
        let lidar = FrameId::agent(AgentId::new("car"), "lidar");
        let mut tree = rooted();
        tree.frame_to_node.insert(
            lidar.clone(),
            FrameNode {
                world: Isometry3::translation(2.0, 0.0, 0.0),
                convention: Convention::Flu,
            },
        );

        let lidar_in_datum = tree
            .erased(lidar.clone(), FrameId::global(DATUM))
            .expect("both resolve");
        let datum_in_lidar = tree
            .erased(FrameId::global(DATUM), lidar)
            .expect("both resolve");

        assert_eq!(lidar_in_datum.isometry().translation.x, 2.0);
        assert!(lidar_in_datum.typed::<Flu, Enu>().is_ok());
        assert_eq!(datum_in_lidar.isometry().translation.x, -2.0);
        assert!(datum_in_lidar.typed::<Enu, Flu>().is_ok());
    }

    #[test]
    fn an_agent_root_still_resolves_to_the_origin() {
        // Until a localizer anchors `odom`, the truth tree places it at the
        // scene origin, so it relates to the datum by the identity.
        let odom = FrameId::odom(AgentId::new("car"));
        let transform = rooted()
            .erased(FrameId::global(DATUM), odom)
            .expect("odom resolves");
        assert_eq!(transform.isometry(), Isometry3::identity());
    }
}
