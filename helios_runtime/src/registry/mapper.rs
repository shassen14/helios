//! Registers built-in mapper factories.

use helios_core::data::envelope::SensorReading;
use helios_core::data::sensor::PointCloud2D;
use helios_core::mapping::MapData;
use helios_core::mapping::{Mapper, OccupancyGridMapper};

use crate::config::MapLayerConfig;
use crate::pipeline::node::PipelineNode;
use crate::pipeline::nodes::occupancy_grid::OccupancyGridNode;
use crate::port::{InternalChannel, SensorChannel};

use super::{contexts::MapperBuildContext, AutonomyRegistry};

pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry.register_mapper("OccupancyGrid2D", build_occupancy_grid_2d);
    registry.register_mapper("None", |_| {
        Err("None mapper produces no pipeline node — omit the map_layer entry instead".to_string())
    });
}

fn build_occupancy_grid_2d(ctx: MapperBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    let MapLayerConfig::OccupancyGrid2D {
        rate,
        resolution,
        scan_channel,
        width_m,
        height_m,
        ..
    } = ctx.config
    else {
        return Err("build_occupancy_grid_2d received wrong config variant".to_string());
    };

    let mapper: Box<dyn Mapper> = Box::new(OccupancyGridMapper::new(
        resolution as f64,
        width_m as f64,
        height_m as f64,
    ));

    let scan_channel =
        SensorChannel::named::<Vec<SensorReading<PointCloud2D>>>(scan_channel.as_str());
    let map_channel = InternalChannel::named::<MapData>("local");

    Ok(Box::new(OccupancyGridNode::new(
        "occupancy_grid",
        mapper,
        ctx.agent_handle,
        scan_channel,
        map_channel,
        Some(rate as f64),
    )))
}
