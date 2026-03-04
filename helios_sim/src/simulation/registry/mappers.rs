// helios_sim/src/simulation/registry/mappers.rs
//
// Registers all known mapper implementations with the AutonomyRegistry.
// To add a new mapper:
//   1. Implement Mapper in helios_core.
//   2. Add a register_mapper call below with the factory closure.
//   Zero spawning systems change.

use bevy::prelude::*;
use helios_core::{mapping::{Mapper, NoneMapper}};

use super::{AutonomyRegistry, MapperBuildContext};

pub struct DefaultMappersPlugin;

impl Plugin for DefaultMappersPlugin {
    fn build(&self, app: &mut App) {
        app.world_mut()
            .resource_mut::<AutonomyRegistry>()
            .register_mapper("None", build_none_mapper)
            .register_mapper("OccupancyGrid2D", build_occupancy_grid_mapper);
    }
}

fn build_none_mapper(_ctx: MapperBuildContext) -> Option<Box<dyn Mapper>> {
    Some(Box::new(NoneMapper))
}

fn build_occupancy_grid_mapper(_ctx: MapperBuildContext) -> Option<Box<dyn Mapper>> {
    warn!("AutonomyRegistry: OccupancyGrid2D mapper not yet implemented. Using NoneMapper.");
    Some(Box::new(NoneMapper))
}
