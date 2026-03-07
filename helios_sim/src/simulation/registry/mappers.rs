// helios_sim/src/simulation/registry/mappers.rs
//
// Registers all known mapper implementations with the AutonomyRegistry.
// To add a new mapper:
//   1. Implement Mapper in helios_core.
//   2. Add a register_mapper call below with the factory closure.
//   Zero spawning systems change.

use bevy::prelude::{App, Plugin};
use helios_core::{
    mapping::{Mapper, MapperPoseSource, NoneMapper, OccupancyGridMapper},
    types::FrameHandle,
};

use crate::simulation::config::structs::{MapperConfig, MapperPoseSourceConfig};
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

fn build_none_mapper(_ctx: MapperBuildContext) -> Result<Box<dyn Mapper>, String> {
    Ok(Box::new(NoneMapper))
}

fn build_occupancy_grid_mapper(ctx: MapperBuildContext) -> Result<Box<dyn Mapper>, String> {
    let MapperConfig::OccupancyGrid2D { resolution, width_m, height_m, pose_source, .. } = ctx.mapper_cfg else {
        return Err("Expected OccupancyGrid2D config".to_string());
    };
    let agent_handle = FrameHandle::from_entity(ctx.agent_entity);
    let mapper_pose_source = match pose_source {
        MapperPoseSourceConfig::GroundTruth => MapperPoseSource::GroundTruth,
        MapperPoseSourceConfig::Estimated => MapperPoseSource::Estimated,
    };
    Ok(Box::new(OccupancyGridMapper::new(
        resolution as f64,
        width_m as f64,
        height_m as f64,
        agent_handle,
        mapper_pose_source,
    )))
}
