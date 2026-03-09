use avian3d::prelude::Gravity;
use bevy::prelude::*;
use helios_core::mapping::NoneMapper;
use helios_core::types::FrameHandle;
use helios_runtime::pipeline::PipelineBuilder;
use helios_runtime::stage::PipelineLevel;
use std::collections::HashMap;

use crate::prelude::*;
use crate::simulation::config::structs::WorldModelConfig;
use crate::simulation::core::events::BevyMeasurementMessage;
use crate::simulation::core::sim_runtime::SimRuntime;
use crate::simulation::core::topics::TopicBus;
use crate::simulation::core::transforms::{enu_iso_to_bevy_transform, TfTree};
use crate::simulation::plugins::world_model::components::{ModuleTimer, OdomFrameOf};
use crate::simulation::plugins::world_model::WorldModelComponent;
use crate::simulation::registry::{
    AutonomyRegistry, EstimatorBuildContext, MapperBuildContext, SlamBuildContext,
};

// =========================================================================
// == SPAWNING SYSTEM ==
// =========================================================================

pub fn spawn_world_model_modules(
    mut commands: Commands,
    agent_query: Query<(Entity, &SpawnAgentConfigRequest, &Children)>,
    measurement_model_query: Query<&MeasurementModel>,
    gravity: Res<Gravity>,
    registry: Res<AutonomyRegistry>,
) {
    let dynamics_factories = registry.clone_dynamics();

    for (agent_entity, request, children) in &agent_query {
        let agent_config = &request.0;
        let gravity_magnitude = gravity.0.length() as f64;
        let mut entity_commands = commands.entity(agent_entity);

        // Pre-build measurement models keyed by FrameHandle (from child entity bits).
        let measurement_models: HashMap<FrameHandle, _> = children
            .iter()
            .filter_map(|child| {
                measurement_model_query
                    .get(child)
                    .ok()
                    .map(|m| (FrameHandle::from_entity(child), m.0.clone()))
            })
            .collect();

        match &agent_config.autonomy_stack.world_model {
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
                                agent_config.name, e
                            );
                            continue;
                        }
                    }
                }

                if let Some(cfg) = map_cfg {
                    if let Some(rate) = cfg.get_timer_rate() {
                        entity_commands.insert(ModuleTimer::from_hz(rate));
                    }
                    match registry.build_mapper(
                        cfg.get_kind_str(),
                        MapperBuildContext {
                            agent_entity,
                            mapper_cfg: cfg.clone(),
                        },
                    ) {
                        Ok(m) => {
                            builder = builder.with_mapper(PipelineLevel::Local, m);
                        }
                        Err(e) => {
                            error!(
                                "Mapper build failed for '{}': {}. Falling back to NoneMapper.",
                                agent_config.name, e
                            );
                            builder = builder.with_mapper(PipelineLevel::Local, Box::new(NoneMapper));
                        }
                    }
                }

                entity_commands.insert(WorldModelComponent(builder.build()));
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
                        let pipeline = PipelineBuilder::new().with_slam(system).build();
                        entity_commands.insert(WorldModelComponent(pipeline));
                    }
                    Err(e) => {
                        error!(
                            "Cannot build SLAM for agent '{}': {}",
                            agent_config.name, e
                        );
                    }
                }
            }

            None => {
                warn!(
                    "No world model configured for agent '{}'. Skipping.",
                    &agent_config.name
                );
            }
        }
    }
}

// =========================================================================
// == RUNTIME SYSTEMS ==
// =========================================================================

pub fn world_model_event_processor(
    mut agent_query: Query<(Entity, &mut WorldModelComponent)>,
    mut measurement_events: EventReader<BevyMeasurementMessage>,
    tf_tree: Res<TfTree>,
    time: Res<Time>,
) {
    let mut messages: Vec<_> = measurement_events.read().map(|e| e.0.clone()).collect();
    if messages.is_empty() {
        return;
    }
    messages.sort_by(|a, b| a.timestamp.partial_cmp(&b.timestamp).unwrap());

    let runtime = SimRuntime {
        tf: &*tf_tree,
        elapsed_secs: time.elapsed_secs_f64(),
    };

    for message in &messages {
        if let Ok((_, mut wm)) = agent_query.get_mut(message.agent_handle.to_entity()) {
            wm.0.process_measurement(message, &runtime);
        }
    }
}

pub fn world_model_mapping_system(
    mut module_query: Query<(Entity, &mut WorldModelComponent, &mut ModuleTimer)>,
    odom_query: Query<(&OdomFrameOf, Entity)>,
    mut measurement_events: EventReader<BevyMeasurementMessage>,
    time: Res<Time>,
    tf_tree: Res<TfTree>,
) {
    let all_new_messages: Vec<_> = measurement_events.read().map(|e| e.0.clone()).collect();

    let runtime = SimRuntime {
        tf: &*tf_tree,
        elapsed_secs: time.elapsed_secs_f64(),
    };

    // Agent entity → odom world pose (from TfTree).
    let agent_to_odom_iso: std::collections::HashMap<Entity, nalgebra::Isometry3<f64>> =
        odom_query
            .iter()
            .filter_map(|(odom_of, odom_entity)| {
                tf_tree
                    .lookup_by_entity(odom_entity)
                    .map(|iso| (odom_of.0, iso))
            })
            .collect();

    for (agent_entity, mut wm, mut timer) in &mut module_query {
        timer.0.tick(time.delta());

        // Forward new sensor data to mappers every frame (cheap log-odds update).
        wm.0.process_mapper_messages(&all_new_messages, &runtime);

        // On timer fire: push odom pose update so the grid can recenter.
        if timer.0.just_finished() {
            if let Some(&odom_iso) = agent_to_odom_iso.get(&agent_entity) {
                wm.0.process_mapper_pose_update(odom_iso);
            }
        }
    }
}

// =========================================================================
// == ODOM FRAME SYSTEMS ==
// =========================================================================

pub fn spawn_odom_frames(
    mut commands: Commands,
    agent_query: Query<(Entity, &SpawnAgentConfigRequest), With<WorldModelComponent>>,
) {
    for (agent_entity, request) in &agent_query {
        let agent_name = &request.0.name;
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

pub fn update_odom_frames(
    agent_query: Query<&WorldModelComponent>,
    mut odom_query: Query<(&OdomFrameOf, &mut Transform)>,
) {
    for (odom_of, mut transform) in &mut odom_query {
        let Ok(world_model) = agent_query.get(odom_of.0) else {
            continue;
        };

        if let Some(iso) = world_model.0.get_state().and_then(|s| s.get_pose_isometry()) {
            *transform = enu_iso_to_bevy_transform(&iso);
        }
    }
}

pub fn world_model_output_publisher(
    query: Query<(&WorldModelComponent, &Name)>,
    mut topic_bus: ResMut<TopicBus>,
) {
    for (wm, name) in &query {
        if let Some(state) = wm.0.get_state() {
            topic_bus.publish(
                &format!("/{}/odometry/estimated", name.as_str()),
                state.clone(),
            );
        }

        if let Some(map) = wm.0.get_map(&PipelineLevel::Local) {
            if !matches!(map, helios_core::mapping::MapData::None) {
                topic_bus.publish(&format!("/{}/map", name.as_str()), map.clone());
            }
        }
    }
}
