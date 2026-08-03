//! Registers built-in mapper factories.

use helios_core::data::envelope::SensorReading;
use helios_core::data::sensor::PointCloud2D;
use helios_core::mapping::MapData;
use helios_core::mapping::{Mapper, OccupancyGridMapper};

use super::node::OccupancyGridNode;

use crate::config::MapLayerConfig;
use crate::pipeline::node::PipelineNode;
use crate::port::{InternalChannel, SensorChannel};
use crate::registry::{contexts::MapperBuildContext, AutonomyRegistry};

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
    // The map is published on a channel named by this layer's config-map key, so
    // two layers of one kind don't collide on a single producer slot and a
    // planner selects the layer it consumes by that same key.
    let map_channel = InternalChannel::named::<MapData>(ctx.instance_name.as_str());

    Ok(Box::new(OccupancyGridNode::new(
        ctx.instance_name,
        mapper,
        ctx.agent_handle,
        scan_channel,
        map_channel,
        Some(rate as f64),
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::data::primitives::FrameHandle;

    fn context(instance_name: &str) -> MapperBuildContext {
        MapperBuildContext {
            agent_handle: FrameHandle(0),
            instance_name: instance_name.to_string(),
            config: MapLayerConfig::OccupancyGrid2D {
                rate: 5.0,
                resolution: 0.1,
                scan_channel: "scan".to_string(),
                width_m: 10.0,
                height_m: 10.0,
                pose_source: Default::default(),
            },
        }
    }

    // The node name is the config-map key, not the kind: two `OccupancyGrid2D`
    // layers under distinct keys must yield distinct node identities.
    #[test]
    fn node_name_is_the_config_key_not_the_kind() {
        let local = build_occupancy_grid_2d(context("local")).unwrap();
        let global = build_occupancy_grid_2d(context("global")).unwrap();

        assert_eq!(local.name(), "local");
        assert_eq!(global.name(), "global");
    }

    // The map output channel is named by the layer's config-map key, so two
    // layers of one kind publish to distinct slots instead of colliding on a
    // single hardcoded producer.
    #[test]
    fn map_output_channel_is_named_by_the_config_key() {
        let map_type = std::any::TypeId::of::<MapData>();
        let output_instance = |node: Box<dyn PipelineNode>| -> String {
            node.port_descriptor()
                .outputs
                .iter()
                .find(|key| key.type_id() == map_type)
                .map(|key| key.instance().to_string())
                .expect("occupancy node must declare a MapData output")
        };

        assert_eq!(
            output_instance(build_occupancy_grid_2d(context("local")).unwrap()),
            "local"
        );
        assert_eq!(
            output_instance(build_occupancy_grid_2d(context("global")).unwrap()),
            "global"
        );
    }
}
