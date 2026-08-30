pub mod plant;

use bevy::prelude::*;

use crate::{
    agents::vehicles::raycast::plant::build_raycast_wheels, config::structs::RAYCAST_WHEELS,
    registry::embodiment::EmbodimentRegistry,
};

pub struct RaycastCarPlugin;

impl Plugin for RaycastCarPlugin {
    fn build(&self, app: &mut App) {
        let mut registry = app.world_mut().resource_mut::<EmbodimentRegistry>();
        registry.register_plant(RAYCAST_WHEELS, build_raycast_wheels);
    }
}
