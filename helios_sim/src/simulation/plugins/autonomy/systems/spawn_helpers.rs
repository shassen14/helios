// helios_sim/src/simulation/plugins/autonomy/systems/spawn_helpers.rs
//
// Shared helpers for the agent spawning pipeline.

use bevy::prelude::*;
use helios_core::mapping::NoneMapper;
use helios_core::planning::types::PlannerGoal;
use helios_runtime::config::ControllerStateSourceConfig;
use helios_runtime::config::MapperConfig;
use helios_runtime::pipeline::PipelineBuilder;
use helios_runtime::stage::PipelineLevel;

use crate::prelude::*;
use crate::simulation::core::components::{ControllerStateSource, SensorMailbox};
use crate::simulation::plugins::autonomy::components::{
    ControlPipelineComponent, EstimatorComponent, MapperComponent, ModuleTimer,
    PathFollowingComponent, PathFollowingOutputComponent,
};
use crate::simulation::registry::{
    AutonomyRegistry, ControllerBuildContext, MapperBuildContext, PathFollowerBuildContext,
    PlannerBuildContext,
};

pub fn level_from_str(s: &str) -> PipelineLevel {
    match s {
        "global" => PipelineLevel::Global,
        "local" => PipelineLevel::Local,
        other => PipelineLevel::Custom(other.to_string()),
    }
}

/// Builds mapper, controllers, planners, and goal from config; inserts all pipeline
/// components onto the agent entity. The caller provides the `EstimatorComponent`.
pub fn build_separate_pipeline(
    entity: Entity,
    commands: &mut Commands,
    map_cfg: Option<&MapperConfig>,
    agent_config: &AgentConfig,
    registry: &AutonomyRegistry,
    est_component: EstimatorComponent,
) {
    let mut builder = PipelineBuilder::new();

    if let Some(cfg) = map_cfg {
        if let Some(rate) = cfg.get_timer_rate() {
            commands.entity(entity).insert(ModuleTimer::from_hz(rate));
        }
        match registry.build_mapper(
            cfg.get_kind_str(),
            MapperBuildContext {
                agent_entity: entity,
                mapper_cfg: cfg.clone(),
            },
        ) {
            Ok(m) => {
                builder = builder.with_mapper(PipelineLevel::Local, m);
            }
            Err(e) => {
                error!(
                    "Mapper build failed for '{}': {}. Falling back to NoneMapper.",
                    agent_config.name(),
                    e
                );
                builder = builder.with_mapper(PipelineLevel::Local, Box::new(NoneMapper));
            }
        }
    }

    for (_key, ctrl_cfg) in &agent_config.autonomy_stack().controllers {
        let kind = ctrl_cfg.get_kind_str();
        let ctx = ControllerBuildContext {
            agent_entity: entity,
            controller_cfg: ctrl_cfg.clone(),
            agent_config: agent_config.clone(),
            dynamics_factories: registry.clone_dynamics_arc(),
        };
        match registry.build_controller(kind, ctx) {
            Ok(ctrl) => {
                builder = builder.with_controller(PipelineLevel::Local, ctrl);
            }
            Err(e) => {
                error!(
                    "Failed to build controller '{}' for agent '{}': {}",
                    kind,
                    agent_config.name(),
                    e
                );
            }
        }
    }

    for (_key, plan_cfg) in &agent_config.autonomy_stack().planners {
        let kind = plan_cfg.get_kind_str();
        let level = level_from_str(plan_cfg.get_level_str());
        let ctx = PlannerBuildContext {
            agent_entity: entity,
            planner_cfg: plan_cfg.clone(),
            level: level.clone(),
        };
        match registry.build_planner(kind, ctx) {
            Ok(planner) => {
                builder = builder.with_planner(level, planner);
            }
            Err(e) => {
                error!(
                    "Failed to build planner '{}' for agent '{}': {}",
                    kind,
                    agent_config.name(),
                    e
                );
            }
        }
    }

    if let Some(pf_cfg) = &agent_config.autonomy_stack().path_following {
        let kind = pf_cfg.get_kind_str();
        let ctx = PathFollowerBuildContext {
            agent_entity: entity,
            path_following_cfg: pf_cfg.clone(),
        };
        match registry.build_path_follower(kind, ctx) {
            Ok(pf) => {
                builder = builder.with_path_follower(pf);
            }
            Err(e) => {
                error!(
                    "Failed to build path follower '{}' for agent '{}': {}",
                    kind,
                    agent_config.name(),
                    e
                );
            }
        }
    }

    builder = builder.with_goal(PlannerGoal::WorldPose(agent_config.goal_pose.to_isometry()));

    let ctrl_state_source = agent_config
        .autonomy_stack()
        .controllers
        .values()
        .next()
        .map(|c| c.state_source())
        .unwrap_or_default();
    let ctrl_state_source_component = match ctrl_state_source {
        ControllerStateSourceConfig::GroundTruth => ControllerStateSource::GroundTruth,
        ControllerStateSourceConfig::Estimated => ControllerStateSource::Estimated,
    };

    let (_, mapping, path_following, control) = builder.build().into_parts();
    let mut entity_cmd = commands.entity(entity);
    entity_cmd.insert((
        est_component,
        MapperComponent(Box::new(mapping)),
        ControlPipelineComponent(control),
        PathFollowingOutputComponent::default(),
        ctrl_state_source_component,
        SensorMailbox::default(),
    ));
    if let Some(pf) = path_following {
        entity_cmd.insert(PathFollowingComponent(pf));
    }
}
