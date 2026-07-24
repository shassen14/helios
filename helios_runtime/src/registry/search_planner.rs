//! Registers built-in search planner factories.

use helios_core::planning::astar::{AStarConfig, AStarPlanner};
use helios_core::planning::SearchPlanner;

use crate::config::SearchPlannerConfig;
use crate::pipeline::builders::planner::DefaultSearchPlannerInputBuilder;
use crate::pipeline::node::PipelineNode;
use crate::pipeline::nodes::search_planner::SearchPlannerNode;

use super::{contexts::SearchPlannerBuildContext, AutonomyRegistry};

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
        "astar",
        planner,
        input_builder,
        ctx.path_channel,
    )))
}
