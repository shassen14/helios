// helios_sim/src/simulation/plugins/autonomy/systems/spawn.rs
//
// Spawning systems for agent autonomy pipelines.
//
// Public: spawn_autonomy_pipeline, spawn_passthrough_pipeline, spawn_odom_frames
// Private: build_separate_pipeline (shared helper), level_from_str

use avian3d::prelude::Gravity;
use bevy::prelude::*;
use helios_core::mapping::NoneMapper;
use helios_core::types::FrameHandle;
use helios_runtime::config::MapperConfig;
use helios_runtime::estimation::GroundTruthPassthrough;
use helios_runtime::pipeline::PipelineBuilder;
use helios_runtime::stage::PipelineLevel;
use helios_runtime::validation::validate_autonomy_config;
use std::collections::HashMap;

use crate::prelude::*;
use crate::simulation::core::components::{AgentTopicNames, ControllerStateSource, SensorMailbox};
use crate::simulation::plugins::autonomy::components::{
    ControlPipelineComponent, EstimatorComponent, MapperComponent, ModuleTimer, OdomFrameOf,
};
use crate::simulation::registry::{
    AutonomyRegistry, ControllerBuildContext, EstimatorBuildContext, MapperBuildContext,
    PlannerBuildContext, SlamBuildContext,
};
use helios_core::planning::types::PlannerGoal;
use helios_runtime::config::ControllerStateSourceConfig;

// =========================================================================
// == Helpers ==
// =========================================================================

fn level_from_str(s: &str) -> PipelineLevel {
    match s {
        "global" => PipelineLevel::Global,
        "local" => PipelineLevel::Local,
        other => PipelineLevel::Custom(other.to_string()),
    }
}

/// Shared helper for the `Separate` world-model path.
///
/// Builds mapper, controllers, planners, and goal from config; inserts all
/// pipeline components onto the agent entity. The caller is responsible for
/// constructing and passing the correct `EstimatorComponent` (real `EstimationCore`
/// or `GroundTruthPassthrough`).
fn build_separate_pipeline(
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

    let goal_iso = agent_config.goal_pose.to_isometry();
    builder = builder.with_goal(PlannerGoal::WorldPose(goal_iso));

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

    let agent_name = agent_config.name();
    let (_, mapping, control) = builder.build().into_parts();
    commands.entity(entity).insert((
        est_component,
        MapperComponent(Box::new(mapping)),
        ControlPipelineComponent(control),
        ctrl_state_source_component,
        SensorMailbox::default(),
        AgentTopicNames {
            active_waypoint: format!("/{}/planning/active_waypoint", agent_name),
            odometry_estimated: format!("/{}/odometry/estimated", agent_name),
            map_global: format!("/{}/map/global", agent_name),
            map_local: format!("/{}/map", agent_name),
        },
    ));
}

// =========================================================================
// == SPAWNING SYSTEMS ==
// =========================================================================

/// Spawns the full autonomy pipeline for agents with real estimation (EKF/UKF).
/// Queries `&Children` to build measurement models from sensor child entities.
pub fn spawn_autonomy_pipeline(
    mut commands: Commands,
    agent_query: Query<(Entity, &SpawnAgentConfigRequest, &Children)>,
    measurement_model_query: Query<&MeasurementModel>,
    gravity: Res<Gravity>,
    registry: Res<AutonomyRegistry>,
) {
    let dynamics_factories = registry.clone_dynamics();
    let capabilities = registry.capabilities();

    for (agent_entity, request, children) in &agent_query {
        let agent_config = &request.0;
        let gravity_magnitude = gravity.0.length() as f64;

        let measurement_models: HashMap<FrameHandle, _> = children
            .iter()
            .filter_map(|child| {
                measurement_model_query
                    .get(child)
                    .ok()
                    .map(|m| (FrameHandle::from_entity(child), m.0.clone()))
            })
            .collect();

        let validation_errors =
            validate_autonomy_config(agent_config.autonomy_stack(), &capabilities);
        if !validation_errors.is_empty() {
            for err in &validation_errors {
                error!(
                    "Config validation failed for agent '{}': {}",
                    agent_config.name(),
                    err
                );
            }
            continue;
        }

        match &agent_config.autonomy_stack().world_model {
            Some(WorldModelConfig::Separate {
                estimator: est_cfg,
                mapper: map_cfg,
            }) => {
                let mut builder = PipelineBuilder::new();

                if let Some(cfg) = est_cfg {
                    let ctx = EstimatorBuildContext {
                        agent_entity,
                        estimator_cfg: cfg.clone(),
                        agent_config: agent_config.clone(),
                        gravity_magnitude,
                        measurement_models,
                        dynamics_factories: dynamics_factories.clone(),
                    };
                    match registry.build_estimator(cfg.get_kind_str(), ctx) {
                        Ok(est) => {
                            builder = builder.with_estimator(est);
                        }
                        Err(e) => {
                            error!(
                                "Cannot build estimator for agent '{}': {}",
                                agent_config.name(),
                                e
                            );
                            continue;
                        }
                    }
                }

                let (estimation, _, _) = builder.build().into_parts();
                let est_component = EstimatorComponent(Box::new(estimation));
                build_separate_pipeline(
                    agent_entity,
                    &mut commands,
                    map_cfg.as_ref(),
                    agent_config,
                    &registry,
                    est_component,
                );
            }

            Some(WorldModelConfig::CombinedSlam { slam: slam_cfg }) => {
                let ctx = SlamBuildContext {
                    agent_entity,
                    slam_cfg: slam_cfg.clone(),
                    agent_config: agent_config.clone(),
                    gravity_magnitude,
                    measurement_models,
                    dynamics_factories: dynamics_factories.clone(),
                };
                match registry.build_slam(slam_cfg.get_kind_str(), ctx) {
                    Ok(system) => {
                        let agent_name = agent_config.name();
                        let ctrl_state_source = agent_config
                            .autonomy_stack()
                            .controllers
                            .values()
                            .next()
                            .map(|c| c.state_source())
                            .unwrap_or_default();
                        let ctrl_state_source_component = match ctrl_state_source {
                            ControllerStateSourceConfig::GroundTruth => {
                                ControllerStateSource::GroundTruth
                            }
                            ControllerStateSourceConfig::Estimated => {
                                ControllerStateSource::Estimated
                            }
                        };
                        let (estimation, mapping, control) = PipelineBuilder::new()
                            .with_slam(system)
                            .build()
                            .into_parts();
                        commands.entity(agent_entity).insert((
                            EstimatorComponent(Box::new(estimation)),
                            MapperComponent(Box::new(mapping)),
                            ControlPipelineComponent(control),
                            ctrl_state_source_component,
                            SensorMailbox::default(),
                            AgentTopicNames {
                                active_waypoint: format!(
                                    "/{}/planning/active_waypoint",
                                    agent_name
                                ),
                                odometry_estimated: format!("/{}/odometry/estimated", agent_name),
                                map_global: format!("/{}/map/global", agent_name),
                                map_local: format!("/{}/map", agent_name),
                            },
                        ));
                    }
                    Err(e) => {
                        error!(
                            "Cannot build SLAM for agent '{}': {}",
                            agent_config.name(),
                            e
                        );
                    }
                }
            }

            None => {
                warn!(
                    "No world model configured for agent '{}'. Skipping.",
                    agent_config.name()
                );
            }
        }
    }
}

/// Spawns the passthrough pipeline for mock-estimator profiles (MappingOnly, PlanningOnly, ControlOnly).
/// Inserts `GroundTruthPassthrough` instead of a real EKF/UKF. Does not query `&Children`.
pub fn spawn_passthrough_pipeline(
    mut commands: Commands,
    agent_query: Query<(Entity, &SpawnAgentConfigRequest)>,
    registry: Res<AutonomyRegistry>,
) {
    let capabilities = registry.capabilities();

    for (agent_entity, request) in &agent_query {
        let agent_config = &request.0;

        let validation_errors =
            validate_autonomy_config(agent_config.autonomy_stack(), &capabilities);
        if !validation_errors.is_empty() {
            for err in &validation_errors {
                error!(
                    "Config validation failed for agent '{}': {}",
                    agent_config.name(),
                    err
                );
            }
            continue;
        }

        match &agent_config.autonomy_stack().world_model {
            Some(WorldModelConfig::Separate {
                estimator: _,
                mapper: map_cfg,
            }) => {
                let est_component = EstimatorComponent(Box::new(GroundTruthPassthrough::default()));
                build_separate_pipeline(
                    agent_entity,
                    &mut commands,
                    map_cfg.as_ref(),
                    agent_config,
                    &registry,
                    est_component,
                );
            }

            Some(WorldModelConfig::CombinedSlam { .. }) => {
                let agent_name = agent_config.name();
                warn!(
                    "Agent '{}' uses CombinedSlam but passthrough profile was requested. \
                     Inserting GT passthrough with empty mapper/control.",
                    agent_name
                );
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
                let (_, mapping, control) = PipelineBuilder::new().build().into_parts();
                commands.entity(agent_entity).insert((
                    EstimatorComponent(Box::new(GroundTruthPassthrough::default())),
                    MapperComponent(Box::new(mapping)),
                    ControlPipelineComponent(control),
                    ctrl_state_source_component,
                    SensorMailbox::default(),
                    AgentTopicNames {
                        active_waypoint: format!("/{}/planning/active_waypoint", agent_name),
                        odometry_estimated: format!("/{}/odometry/estimated", agent_name),
                        map_global: format!("/{}/map/global", agent_name),
                        map_local: format!("/{}/map", agent_name),
                    },
                ));
            }

            None => {
                warn!(
                    "No world model configured for agent '{}'. Skipping.",
                    agent_config.name()
                );
            }
        }
    }
}

pub fn spawn_odom_frames(
    mut commands: Commands,
    agent_query: Query<(Entity, &SpawnAgentConfigRequest), With<EstimatorComponent>>,
) {
    for (agent_entity, request) in &agent_query {
        let agent_name = request.0.name();
        commands.spawn((
            Name::new(format!("{}/odom", agent_name)),
            TrackedFrame,
            Transform::IDENTITY,
            GlobalTransform::IDENTITY,
            OdomFrameOf(agent_entity),
        ));
        info!("[OdomFrame] Spawned odom frame for '{}'", agent_name);
    }
}
