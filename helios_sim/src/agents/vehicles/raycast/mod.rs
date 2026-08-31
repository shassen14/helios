pub mod plant;
pub mod systems;

use bevy::prelude::*;

use crate::{
    agents::vehicles::raycast::plant::build_raycast_wheels,
    config::structs::RAYCAST_WHEELS,
    prelude::{AppState, SimulationSet},
    registry::embodiment::EmbodimentRegistry,
};

pub struct RaycastCarPlugin;

impl Plugin for RaycastCarPlugin {
    fn build(&self, app: &mut App) {
        let mut registry = app.world_mut().resource_mut::<EmbodimentRegistry>();
        registry.register_plant(RAYCAST_WHEELS, build_raycast_wheels);

        app.add_systems(
            FixedUpdate,
            systems::drive_raycast_cars
                .in_set(SimulationSet::Actuation)
                .run_if(in_state(AppState::Running)),
        );
    }
}
