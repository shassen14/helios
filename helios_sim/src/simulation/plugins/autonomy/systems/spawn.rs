// helios_sim/src/simulation/plugins/autonomy/systems/spawn.rs
//
// Spawning systems for agent autonomy pipelines.
//
// Public: spawn_autonomy_pipeline, spawn_passthrough_pipeline, spawn_odom_frames

use bevy::prelude::*;
use helios_core::types::FrameHandle;
use helios_runtime::config::ControllerStateSourceConfig;
use helios_runtime::estimation::GroundTruthPassthrough;
use helios_runtime::pipeline::PipelineBuilder;
use helios_runtime::validation::validate_autonomy_config;
use std::collections::HashMap;

use crate::prelude::*;
use crate::simulation::core::components::{ControllerStateSource, SensorMailbox};
use crate::simulation::plugins::autonomy::components::{
    ControlPipelineComponent, EstimatorComponent, MapperComponent, OdomFrameOf,
    PathFollowingComponent, PathFollowingOutputComponent,
};
use crate::simulation::registry::{AutonomyRegistry, EstimatorBuildContext, SlamBuildContext};

use super::spawn_helpers::build_separate_pipeline;

/// Spawns the full autonomy pipeline for agents with real estimation (EKF/UKF).
pub fn spawn_autonomy_pipeline(
    mut commands: Commands,
    agent_query: Query<(Entity, &SpawnAgentConfigRequest, &Children)>,
    measurement_model_query: Query<&MeasurementModel>,
    gravity: Res<avian3d::prelude::Gravity>,
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

                let (estimation, _, _, _) = builder.build().into_parts();
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
                        let (estimation, mapping, path_following, control) = PipelineBuilder::new()
                            .with_slam(system)
                            .build()
                            .into_parts();
                        let mut entity_cmd = commands.entity(agent_entity);
                        entity_cmd.insert((
                            EstimatorComponent(Box::new(estimation)),
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

/// Spawns the passthrough pipeline for mock-estimator profiles.
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
                warn!(
                    "Agent '{}' uses CombinedSlam but passthrough profile was requested. \
                     Inserting GT passthrough with empty mapper/control.",
                    agent_config.name()
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
                let (_, mapping, path_following, control) =
                    PipelineBuilder::new().build().into_parts();
                let mut entity_cmd = commands.entity(agent_entity);
                entity_cmd.insert((
                    EstimatorComponent(Box::new(GroundTruthPassthrough::default())),
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
