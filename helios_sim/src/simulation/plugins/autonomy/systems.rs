use avian3d::prelude::Gravity;
use bevy::prelude::*;
use helios_core::mapping::NoneMapper;
use helios_core::types::FrameHandle;
use helios_runtime::pipeline::PipelineBuilder;
use helios_runtime::stage::PipelineLevel;
use std::collections::HashMap;

use helios_runtime::validation::validate_autonomy_config;

use crate::prelude::*;
use crate::simulation::config::structs::WorldModelConfig;
use crate::simulation::core::components::{MailboxEntry, SensorMailbox, SensorTopicName};
use crate::simulation::core::events::BevyMeasurementMessage;
use crate::simulation::core::sim_runtime::SimRuntime;
use crate::simulation::core::topics::TopicBus;
use crate::simulation::core::transforms::{enu_iso_to_bevy_transform, TfTree};
use crate::simulation::plugins::autonomy::components::{
    ControlPipelineComponent, EstimatorComponent, MapperComponent, ModuleTimer, OdomFrameOf,
};
use crate::simulation::registry::{
    AutonomyRegistry, ControllerBuildContext, EstimatorBuildContext, MapperBuildContext,
    SlamBuildContext,
};

// =========================================================================
// == SPAWNING SYSTEMS ==
// =========================================================================

pub fn spawn_world_model_modules(
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
                                agent_config.name(),
                                e
                            );
                            builder =
                                builder.with_mapper(PipelineLevel::Local, Box::new(NoneMapper));
                        }
                    }
                }

                for (_key, ctrl_cfg) in &agent_config.autonomy_stack().controllers {
                    let kind = ctrl_cfg.get_kind_str();
                    let ctx = ControllerBuildContext {
                        agent_entity,
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

                let (estimation, mapping, control) = builder.build().into_parts();
                entity_commands.insert((
                    EstimatorComponent(estimation),
                    MapperComponent(mapping),
                    ControlPipelineComponent(control),
                    SensorMailbox::default(),
                ));
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
                        let (estimation, mapping, control) =
                            PipelineBuilder::new().with_slam(system).build().into_parts();
                        entity_commands.insert((
                            EstimatorComponent(estimation),
                            MapperComponent(mapping),
                            ControlPipelineComponent(control),
                            SensorMailbox::default(),
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

// =========================================================================
// == RUNTIME SYSTEMS ==
// =========================================================================

/// Routes `BevyMeasurementMessage` events to per-agent `SensorMailbox` components.
/// Clears each mailbox, fills it with this frame's messages, and sorts by timestamp.
/// Must run before `estimation_system` and `mapping_system`.
pub fn route_sensor_messages(
    mut events: EventReader<BevyMeasurementMessage>,
    mut mailbox_query: Query<&mut SensorMailbox>,
    topic_name_query: Query<&SensorTopicName>,
) {
    // Clear all mailboxes at the start of each frame.
    for mut mailbox in &mut mailbox_query {
        mailbox.entries.clear();
    }

    for event in events.read() {
        let sensor_entity = event.0.sensor_handle.to_entity();
        let topic_name = topic_name_query
            .get(sensor_entity)
            .map(|t| t.0.clone())
            .unwrap_or_default();

        let agent_entity = event.0.agent_handle.to_entity();
        if let Ok(mut mailbox) = mailbox_query.get_mut(agent_entity) {
            mailbox.entries.push(MailboxEntry {
                topic_name,
                message: event.0.clone(),
            });
        }
    }

    // Sort each mailbox by timestamp (ascending) for EKF causality.
    for mut mailbox in &mut mailbox_query {
        mailbox
            .entries
            .sort_by(|a, b| a.message.timestamp.partial_cmp(&b.message.timestamp).unwrap());
    }
}

/// Processes all measurements in each agent's `SensorMailbox` through the estimator.
/// Runs in parallel with `mapping_system` — accesses `EstimatorComponent` only.
pub fn estimation_system(
    mut agent_query: Query<(&mut EstimatorComponent, &SensorMailbox)>,
    tf_tree: Res<TfTree>,
    time: Res<Time>,
) {
    let runtime = SimRuntime {
        tf: &*tf_tree,
        elapsed_secs: time.elapsed_secs_f64(),
    };

    for (mut estimator, mailbox) in &mut agent_query {
        for entry in &mailbox.entries {
            estimator.0.process_measurement(&entry.message, &runtime);
        }
    }
}

/// Forwards each agent's `SensorMailbox` to its mapper and handles timer-gated pose updates.
/// Runs in parallel with `estimation_system` — accesses `MapperComponent` only.
pub fn mapping_system(
    mut module_query: Query<(Entity, &mut MapperComponent, &mut ModuleTimer, &SensorMailbox)>,
    odom_query: Query<(&OdomFrameOf, Entity)>,
    time: Res<Time>,
    tf_tree: Res<TfTree>,
) {
    let runtime = SimRuntime {
        tf: &*tf_tree,
        elapsed_secs: time.elapsed_secs_f64(),
    };

    let agent_to_odom_iso: HashMap<Entity, nalgebra::Isometry3<f64>> = odom_query
        .iter()
        .filter_map(|(odom_of, odom_entity)| {
            tf_tree
                .lookup_by_entity(odom_entity)
                .map(|iso| (odom_of.0, iso))
        })
        .collect();

    for (agent_entity, mut mapper, mut timer, mailbox) in &mut module_query {
        timer.0.tick(time.delta());

        let messages: Vec<_> = mailbox.entries.iter().map(|e| e.message.clone()).collect();
        mapper.0.process_messages(&messages, &runtime);

        if timer.0.just_finished() {
            if let Some(&odom_iso) = agent_to_odom_iso.get(&agent_entity) {
                mapper.0.process_pose_update(odom_iso);
            }
        }
    }
}

// =========================================================================
// == ODOM FRAME SYSTEMS ==
// =========================================================================

pub fn update_odom_frames(
    agent_query: Query<&EstimatorComponent>,
    mut odom_query: Query<(&OdomFrameOf, &mut Transform)>,
) {
    for (odom_of, mut transform) in &mut odom_query {
        let Ok(estimator) = agent_query.get(odom_of.0) else {
            continue;
        };

        if let Some(iso) = estimator.0.get_state().and_then(|s| s.get_pose_isometry()) {
            *transform = enu_iso_to_bevy_transform(&iso);
        }
    }
}

// =========================================================================
// == TELEMETRY SYSTEMS (cold path — Validation phase) ==
// =========================================================================

/// Publishes estimated state and maps to TopicBus.
/// Runs in `SimulationSet::Validation` (cold telemetry path).
pub fn autonomy_telemetry_system(
    query: Query<(&EstimatorComponent, &MapperComponent, &Name)>,
    mut topic_bus: ResMut<TopicBus>,
) {
    for (estimator, mapper, name) in &query {
        if let Some(state) = estimator.0.get_state() {
            topic_bus.publish(
                &format!("/{}/odometry/estimated", name.as_str()),
                state.clone(),
            );
        }

        // Global map: SLAM takes priority over global mappers.
        let global_map = estimator
            .0
            .get_slam_map()
            .or_else(|| mapper.0.get_map(&PipelineLevel::Global));
        if let Some(map) = global_map {
            if !matches!(map, helios_core::mapping::MapData::None) {
                topic_bus.publish(&format!("/{}/map/global", name.as_str()), map.clone());
            }
        }

        if let Some(map) = mapper.0.get_map(&PipelineLevel::Local) {
            if !matches!(map, helios_core::mapping::MapData::None) {
                topic_bus.publish(&format!("/{}/map", name.as_str()), map.clone());
            }
        }
    }
}

/// Publishes each agent's sensor measurements to TopicBus.
/// Runs in `SimulationSet::Validation` (cold telemetry path).
/// Sensor systems no longer touch `TopicBus` directly — all sensor telemetry flows here.
pub fn sensor_telemetry_system(
    query: Query<&SensorMailbox>,
    mut topic_bus: ResMut<TopicBus>,
) {
    for mailbox in &query {
        for entry in &mailbox.entries {
            if !entry.topic_name.is_empty() {
                topic_bus.publish(&entry.topic_name, entry.message.clone());
            }
        }
    }
}
