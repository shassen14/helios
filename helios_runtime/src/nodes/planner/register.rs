//! Registers built-in search planner factories.

use super::input::DefaultSearchPlannerInputBuilder;
use super::node::SearchPlannerNode;

use crate::config::SearchPlannerConfig;
use crate::pipeline::node::PipelineNode;
use crate::registry::{contexts::SearchPlannerBuildContext, AutonomyRegistry};

use helios_core::planning::astar::{AStarConfig, AStarPlanner};
use helios_core::planning::SearchPlanner;

pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry.register_search_planner("AStar", build_astar);
}

fn build_astar(ctx: SearchPlannerBuildContext) -> Result<Box<dyn PipelineNode>, String> {
    let SearchPlannerConfig::AStar {
        rate,
        arrival_tolerance_m,
        occupancy_threshold,
        max_search_depth,
        enable_path_smoothing,
        replan_on_path_deviation,
        deviation_tolerance_m,
        ..
    } = ctx.config;

    let level_key = ctx.config.get_level_str().to_string();
    let planner: Box<dyn SearchPlanner> = Box::new(AStarPlanner::new(AStarConfig {
        rate_hz: rate as f64,
        arrival_tolerance_m: arrival_tolerance_m as f64,
        occupancy_threshold,
        max_search_depth,
        enable_path_smoothing,
        replan_on_path_deviation,
        deviation_tolerance_m: deviation_tolerance_m as f64,
        level_key,
    }));

    let input_builder = Box::new(DefaultSearchPlannerInputBuilder::new(
        ctx.map_channel,
        ctx.config.get_goal_channel(),
    ));

    Ok(Box::new(SearchPlannerNode::new(
        ctx.instance_name,
        planner,
        input_builder,
        ctx.path_channel,
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::InternalChannel;

    use helios_core::data::primitives::FrameHandle;
    use helios_core::mapping::MapData;
    use helios_core::planning::types::Path;

    fn context(instance_name: &str) -> SearchPlannerBuildContext {
        SearchPlannerBuildContext {
            agent_handle: FrameHandle(0),
            instance_name: instance_name.to_string(),
            config: SearchPlannerConfig::AStar {
                rate: 10.0,
                arrival_tolerance_m: 0.5,
                occupancy_threshold: 50,
                max_search_depth: 1000,
                enable_path_smoothing: false,
                replan_on_path_deviation: false,
                deviation_tolerance_m: 0.5,
                level: "local".to_string(),
                goal_channel: "goal".to_string(),
            },
            map_channel: InternalChannel::named::<MapData>("local"),
            path_channel: InternalChannel::named::<Path>(instance_name),
        }
    }

    // The node name is the config-map key, not the kind: two `AStar` planners
    // under distinct keys must yield distinct node identities.
    #[test]
    fn node_name_is_the_config_key_not_the_kind() {
        let coarse = build_astar(context("coarse")).unwrap();
        let fine = build_astar(context("fine")).unwrap();

        assert_eq!(coarse.name(), "coarse");
        assert_eq!(fine.name(), "fine");
    }
}
