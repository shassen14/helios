//! Registers the `AStar` kind, which builds a [`SearchPlannerNode`] from a
//! `[nodes.<name>]` entry.

use super::config::AStarPlannerConfig;
use super::input::DefaultSearchPlannerInputBuilder;
use super::node::SearchPlannerNode;

use crate::assembly::{AutonomyRegistry, BuildContext, FactoryOutput};
use crate::port::InternalChannel;

use helios_core::interchange::path::Path;
use helios_core::interchange::perception::map::MapData;
use helios_core::planning::search::astar::{AStarConfig, AStarPlanner};
use helios_core::planning::SearchPlanner;

/// The `kind` a profile writes to get an A* planner node.
pub(crate) const ASTAR_KIND: &str = "AStar";

/// Adds the `AStar` kind to `registry`.
pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry
        .register_node(ASTAR_KIND, build_astar)
        .expect("AStar is registered once, by the default registry")
}

/// Builds the planner node for one entry. The goal is declared as an outside
/// input: in today's stacks only the host writes it.
fn build_astar(config: AStarPlannerConfig, ctx: &BuildContext) -> Result<FactoryOutput, String> {
    let planner: Box<dyn SearchPlanner> = Box::new(AStarPlanner::new(AStarConfig {
        rate_hz: config.rate as f64,
        arrival_tolerance_m: config.arrival_tolerance_m as f64,
        occupancy_threshold: config.occupancy_threshold,
        max_search_depth: config.max_search_depth,
        enable_path_smoothing: config.enable_path_smoothing,
        replan_on_path_deviation: config.replan_on_path_deviation,
        deviation_tolerance_m: config.deviation_tolerance_m as f64,
        level_key: config.map_channel.clone(),
    }));

    let map_channel = InternalChannel::named::<MapData>(config.map_channel.as_str());
    let input_builder = Box::new(DefaultSearchPlannerInputBuilder::new(
        map_channel,
        &config.goal_channel,
    ));
    // The path is published on a channel named after the node, so two planners
    // don't collide on one producer slot and a follower selects the planner it
    // tracks by that same name.
    let path_channel = InternalChannel::named::<Path>(ctx.node_name());

    let node = SearchPlannerNode::new(ctx.node_name(), planner, input_builder, path_channel);

    Ok(FactoryOutput::new(Box::new(node)).with_outside_input(
        DefaultSearchPlannerInputBuilder::goal_key(&config.goal_channel),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::ChannelKey;

    use helios_core::prelude::{AgentId, PlannerGoal};

    use std::collections::HashSet;

    /// Builds a planner named `name` over the `local` map, reading its goal
    /// from `goal_channel`.
    fn build(name: &str, goal_channel: &str) -> FactoryOutput {
        let config = AStarPlannerConfig {
            rate: 10.0,
            arrival_tolerance_m: 0.5,
            occupancy_threshold: 50,
            max_search_depth: 1000,
            enable_path_smoothing: false,
            replan_on_path_deviation: false,
            deviation_tolerance_m: 0.5,
            map_channel: "local".to_string(),
            goal_channel: goal_channel.to_string(),
        };
        let channels = HashSet::new();
        let ctx = BuildContext::new(AgentId::new("test_agent"), name, &channels);
        build_astar(config, &ctx).expect("a valid section builds")
    }

    // The node name is the table key, not the kind: two `AStar` entries under
    // distinct keys must yield distinct node identities.
    #[test]
    fn node_name_is_the_table_key_not_the_kind() {
        assert_eq!(build("coarse", "mission").into_node().name(), "coarse");
        assert_eq!(build("fine", "mission").into_node().name(), "fine");
    }

    // The path is written on a channel named after the node, and the map is
    // read from the channel the section names.
    #[test]
    fn reads_the_named_map_and_writes_a_path_named_after_the_node() {
        let node = build("local_path", "mission").into_node();
        let descriptor = node.port_descriptor();

        let path: ChannelKey = InternalChannel::named::<Path>("local_path").into();
        let map: ChannelKey = InternalChannel::named::<MapData>("local").into();
        assert_eq!(descriptor.outputs(), [path]);
        assert!(descriptor.required_inputs().any(|key| *key == map));
    }

    // The goal is declared as an outside input on the channel the section
    // names, and it is the same key the node reads.
    #[test]
    fn goal_is_declared_as_the_outside_input_the_node_reads() {
        let output = build("local_path", "waypoints");
        let goal: ChannelKey = InternalChannel::named::<PlannerGoal>("waypoints").into();

        assert_eq!(output.outside_inputs(), std::slice::from_ref(&goal));
        assert!(output
            .into_node()
            .port_descriptor()
            .optional_inputs()
            .any(|key| *key == goal));
    }
}
