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

#[cfg(test)]
mod tests {
    use super::*;

    use crate::registry::embodiment::EmbodimentRegistryPlugin;

    /// Guards the plant registration in `build`: the `raycast_wheels` kind is what
    /// lets a profile select this plant, and losing the `register_plant` call would
    /// leave the kind unknown at spawn while still compiling. The registry plugin
    /// must run first — it creates the resource this populates, exactly as
    /// `HeliosSimulationPlugin` orders them.
    #[test]
    fn plugin_registers_the_raycast_wheels_plant() {
        let mut app = App::new();
        app.add_plugins((EmbodimentRegistryPlugin, RaycastCarPlugin));

        let registry = app.world().resource::<EmbodimentRegistry>();
        assert!(
            registry.plant(RAYCAST_WHEELS).is_some(),
            "RaycastCarPlugin must register a builder for the {RAYCAST_WHEELS} plant kind"
        );
    }
}
