use crate::core::transforms::EnuBodyPose;
use crate::prelude::*;
use crate::registry::contexts::{CollisionBuildContext, PlantBuildContext, TopologyBuildContext};
use crate::registry::embodiment::EmbodimentRegistry;

use avian3d::prelude::RigidBody;

/// Generic body assembly: for each freshly-spawned agent, look up and run its
/// per-axis builders — topology, then plant, then collision — from the
/// [`EmbodimentRegistry`], keyed on each `[section]`'s `kind`. Topology runs first
/// as the build-dependency root; later axes will read frames it deposits.
///
/// Runs in `ProcessVehicle`. `Without<RigidBody>` makes it idempotent — the
/// topology builder inserts the body, so a built agent is skipped on any re-run,
/// which is what would let agents spawn mid-simulation later. A missing builder or
/// a builder error is a config/wiring fault and panics at spawn (a startup panic,
/// permitted; the running sim never reaches this).
pub(super) fn build_embodiment(
    mut commands: Commands,
    registry: Res<EmbodimentRegistry>,
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
    }
}
