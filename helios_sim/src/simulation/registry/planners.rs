// helios_sim/src/simulation/registry/planners.rs
//
// Registers all built-in planner factories into AutonomyRegistry.

use bevy::prelude::*;
use helios_core::planning::astar::{AStarConfig, AStarPlanner};

use super::{AutonomyRegistry, PlannerBuildContext};
use crate::simulation::config::structs::PlannerConfig;

pub struct DefaultPlannersPlugin;

impl Plugin for DefaultPlannersPlugin {
    fn build(&self, app: &mut App) {
        let Some(mut registry) = app.world_mut().get_resource_mut::<AutonomyRegistry>() else {
            error!("DefaultPlannersPlugin: AutonomyRegistry not found. Add AutonomyRegistryPlugin first.");
            return;
        };

        // --- AStar ---
        registry.register_planner("AStar", |ctx: PlannerBuildContext| {
            let PlannerConfig::AStar {
                rate,
                arrival_tolerance_m,
                occupancy_threshold,
                max_search_depth,
                enable_path_smoothing,
                replan_on_path_deviation,
                deviation_tolerance_m,
                level,
            } = ctx.planner_cfg;
            let config = AStarConfig {
                rate_hz: rate as f64,
                arrival_tolerance_m: arrival_tolerance_m as f64,
                occupancy_threshold,
                max_search_depth,
                enable_path_smoothing,
                replan_on_path_deviation,
                deviation_tolerance_m: deviation_tolerance_m as f64,
                level_key: level,
            };
            Ok(Box::new(AStarPlanner::new(config)))
        });
    }
}
