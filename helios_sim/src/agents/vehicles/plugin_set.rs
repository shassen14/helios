// HeliosVehiclesPlugin: populates the embodiment registry with the built-in axis
// builders, adds the generic body-assembly dispatch, then the vehicle families.

use super::ackermann::AckermannCarPlugin;
use super::builders::{build_cuboid, build_l0_shim, build_rigid_body_with_mount};
use super::embodiment::build_embodiment;
use crate::agents::vehicles::builders::build_wheeled_primitives;
use crate::agents::vehicles::raycast::RaycastCarPlugin;
use crate::config::structs::{CUBOID, L0_SHIM, RIGID_BODY_WITH_MOUNT, WHEELED_PRIMITIVES};
use crate::core::app_state::{AppState, SceneBuildSet};
use crate::registry::embodiment::EmbodimentRegistry;

use bevy::prelude::*;

/// Adds the embodiment dispatch and every vehicle family.
///
/// Requires `EmbodimentRegistryPlugin` to have run first (it creates the resource
/// this populates); `HeliosSimulationPlugin` adds it earlier in the list.
pub struct HeliosVehiclesPlugin;

impl Plugin for HeliosVehiclesPlugin {
    fn build(&self, app: &mut App) {
        // Register the built-in kinds once, here rather than per family — they are
        // morphology-general (a drone reuses `RigidBodyWithMount`), so no single
        // family owns them.
        {
            let mut registry = app.world_mut().resource_mut::<EmbodimentRegistry>();
            registry.register_topology(RIGID_BODY_WITH_MOUNT, build_rigid_body_with_mount);
            registry.register_plant(L0_SHIM, build_l0_shim);
            registry.register_collision(CUBOID, build_cuboid);
            registry.register_visual(WHEELED_PRIMITIVES, build_wheeled_primitives);
        }

        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            build_embodiment.in_set(SceneBuildSet::ProcessVehicle),
        )
        .add_plugins(AckermannCarPlugin)
        .add_plugins(RaycastCarPlugin);
    }
}
