use crate::core::transforms::EnuBodyPose;
use crate::prelude::*;
use crate::registry::contexts::{
    CollisionBuildContext, PlantBuildContext, TopologyBuildContext, VisualBuildContext,
};
use crate::registry::embodiment::EmbodimentRegistry;

use avian3d::prelude::RigidBody;

/// Generic body assembly: for each freshly-spawned agent, look up and run its
/// per-axis builders — topology, then plant, then collision, then visual — from
/// the [`EmbodimentRegistry`], keyed on each `[section]`'s `kind`. Topology runs
/// first as the build-dependency root; the plant and visual axes both read the
/// mount frames it declares (by config, since the mount entities are still queued
/// on the shared command buffer during this pass).
///
/// Runs in `ProcessVehicle`. `Without<RigidBody>` makes it idempotent — the
/// topology builder inserts the body, so a built agent is skipped on any re-run,
/// which is what would let agents spawn mid-simulation later. A missing builder or
/// a builder error is a config/wiring fault and panics at spawn (a startup panic,
/// permitted; the running sim never reaches this).
pub(super) fn build_embodiment(
    mut commands: Commands,
    registry: Res<EmbodimentRegistry>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    query: Query<(Entity, &GroundTruthState, &SpawnAgentConfigRequest), Without<RigidBody>>,
) {
    for (entity, ground_truth, request) in &query {
        let vehicle = &request.0.vehicle;
        let start_transform = Transform::from(EnuBodyPose(ground_truth.pose));

        // Topology first — it deposits the body and (later) the mount frames the
        // plant and collision builders read.
        let kind = vehicle.topology.kind_str();
        let Some(builder) = registry.topology(kind) else {
            panic!("no topology builder registered for kind `{kind}`");
        };
        let mut ctx = TopologyBuildContext {
            entity,
            commands: &mut commands,
            config: &vehicle.topology,
            start_transform,
        };
        if let Err(e) = builder(&mut ctx) {
            panic!("topology build failed: {e}");
        }

        // Plant.
        let kind = vehicle.plant.kind_str();
        let Some(builder) = registry.plant(kind) else {
            panic!("no plant builder registered for kind `{kind}`");
        };
        let mut ctx = PlantBuildContext {
            entity,
            commands: &mut commands,
            config: &vehicle.plant,
            actuation: &vehicle.actuation,
            mounts: vehicle.topology.mounts(),
        };
        if let Err(e) = builder(&mut ctx) {
            panic!("plant build failed: {e}");
        }

        // Collision.
        let kind = vehicle.collision.kind_str();
        let Some(builder) = registry.collision(kind) else {
            panic!("no collision builder registered for kind `{kind}`");
        };
        let mut ctx = CollisionBuildContext {
            entity,
            commands: &mut commands,
            config: &vehicle.collision,
        };
        if let Err(e) = builder(&mut ctx) {
            panic!("collision build failed: {e}");
        }

        // Visual — cosmetic only, and last: nothing else reads what it spawns.
        let kind = vehicle.visual.kind_str();
        let Some(builder) = registry.visual(kind) else {
            panic!("no visual builder registered for kind `{kind}`");
        };
        let mut ctx = VisualBuildContext {
            entity,
            commands: &mut commands,
            config: &vehicle.visual,
            mounts: vehicle.topology.mounts(),
            meshes: &mut meshes,
            materials: &mut materials,
        };
        if let Err(e) = builder(&mut ctx) {
            panic!("visual build failed: {e}");
        }
    }
}
