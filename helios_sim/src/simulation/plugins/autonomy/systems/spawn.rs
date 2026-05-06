// helios_sim/src/simulation/plugins/autonomy/systems/spawn.rs
//
// Spawning systems for agent autonomy pipelines.
//
// Public: spawn_autonomy_pipeline, spawn_passthrough_pipeline, spawn_odom_frames

use bevy::prelude::*;
use helios_core::data::primitives::FrameHandle;
use helios_runtime::estimation::GroundTruthPassthrough;
use helios_runtime::pipeline::PipelineBuilder;
use helios_runtime::validation::validate_autonomy_config;
use std::collections::HashMap;

use crate::prelude::*;
use crate::simulation::plugins::autonomy::components::{EstimatorComponent, OdomFrameOf};
use crate::simulation::registry::{AutonomyRegistry, EstimatorBuildContext};

use super::spawn_helpers::build_pipeline_from_stack;

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

        let stack = agent_config.autonomy_stack();
        let mut builder = PipelineBuilder::new();

        if let Some(cfg) = &stack.estimator {
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
        build_pipeline_from_stack(
            agent_entity,
            &mut commands,
            stack,
            agent_config,
            &registry,
            est_component,
        );
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

        let est_component = EstimatorComponent(Box::new(GroundTruthPassthrough::default()));
        build_pipeline_from_stack(
            agent_entity,
            &mut commands,
            agent_config.autonomy_stack(),
            agent_config,
            &registry,
            est_component,
        );
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
