//! Registers the `OccupancyGrid2D` kind, which builds an [`OccupancyGridNode`]
//! from a `[nodes.<name>]` entry.

use super::config::OccupancyGridConfig;
use super::node::OccupancyGridNode;

use crate::assembly::{AutonomyRegistry, BuildContext, FactoryOutput};
use crate::port::{InternalChannel, SensorChannel};

use helios_core::interchange::measurement::envelope::SensorReading;
use helios_core::interchange::perception::map::MapData;
use helios_core::mapping::{Mapper, OccupancyGridMapper};
use helios_core::prelude::PointCloud;
use helios_core::spatial::conventions::Flu;

/// The `kind` a profile writes to get an occupancy-grid node.
pub(crate) const OCCUPANCY_GRID_2D_KIND: &str = "OccupancyGrid2D";

/// Adds the `OccupancyGrid2D` kind to `registry`.
pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry
        .register_node(OCCUPANCY_GRID_2D_KIND, build_occupancy_grid_2d)
        .expect("OccupancyGrid2D is registered once, by the default registry")
}

/// Builds the grid node for one entry.
fn build_occupancy_grid_2d(
    config: OccupancyGridConfig,
    ctx: &BuildContext,
) -> Result<FactoryOutput, String> {
    let mapper: Box<dyn Mapper> = Box::new(OccupancyGridMapper::new(
        config.resolution as f64,
        config.width_m as f64,
        config.height_m as f64,
    ));

    let scan_channel =
        SensorChannel::named::<Vec<SensorReading<PointCloud<Flu>>>>(config.scan_channel.as_str());
    // The map is published on a channel named after the node, so two grids
    // don't collide on a single producer slot and a planner selects the grid
    // it consumes by that same name.
    let map_channel = InternalChannel::named::<MapData>(ctx.node_name());

    Ok(FactoryOutput::new(Box::new(OccupancyGridNode::new(
        ctx.node_name(),
        mapper,
        ctx.agent().clone(),
        scan_channel,
        map_channel,
        Some(config.rate as f64),
    ))))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::pipeline::node::PipelineNode;

    use helios_core::prelude::AgentId;

    use std::collections::HashSet;

    /// Builds a grid named `name` from a small valid section.
    fn build(name: &str) -> Box<dyn PipelineNode> {
        let config = OccupancyGridConfig {
            rate: 5.0,
            resolution: 0.1,
            scan_channel: "scan".to_string(),
            width_m: 10.0,
            height_m: 10.0,
            pose_source: Default::default(),
        };
        let channels = HashSet::new();
        let ctx = BuildContext::new(AgentId::new("test_agent"), name, &channels);
        build_occupancy_grid_2d(config, &ctx)
            .expect("a valid section builds")
            .into_node()
    }

    // The node name is the table key, not the kind: two `OccupancyGrid2D`
    // entries under distinct keys must yield distinct node identities.
    #[test]
    fn node_name_is_the_table_key_not_the_kind() {
        assert_eq!(build("local").name(), "local");
        assert_eq!(build("global").name(), "global");
    }

    // The map output channel is named after the node, so two grids publish to
    // distinct slots instead of colliding on a single hardcoded producer.
    #[test]
    fn map_output_channel_is_named_after_the_node() {
        let map_type = std::any::TypeId::of::<MapData>();
        let output_instance = |node: Box<dyn PipelineNode>| -> String {
            node.port_descriptor()
                .outputs()
                .iter()
                .find(|key| key.type_id() == map_type)
                .map(|key| key.instance().to_string())
                .expect("occupancy node must declare a MapData output")
        };

        assert_eq!(output_instance(build("local")), "local");
        assert_eq!(output_instance(build("global")), "global");
    }
}
