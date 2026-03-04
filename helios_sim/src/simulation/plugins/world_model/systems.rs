use avian3d::prelude::Gravity;
use bevy::prelude::*;
use helios_core::estimation::StateEstimator;
use helios_core::mapping::{Mapper, NoneMapper};
use helios_core::types::FrameHandle;
use nalgebra::DVector;
use std::collections::HashMap;

use crate::prelude::*;
use crate::simulation::config::structs::{MapperConfig, WorldModelConfig};
use crate::simulation::core::events::BevyMeasurementMessage;
use crate::simulation::core::topics::TopicBus;
use crate::simulation::plugins::world_model::components::{ControlInputCache, ModuleTimer};
use crate::simulation::plugins::world_model::WorldModelComponent;
use crate::simulation::registry::{
    AutonomyRegistry, EstimatorBuildContext, MapperBuildContext, SlamBuildContext,
};

use helios_core::estimation::FilterContext;
use helios_core::messages::ModuleInput;

// =========================================================================
// == SPAWNING SYSTEM ==
// =========================================================================

/// Runs once during scene building. Reads agent configs, queries the
/// AutonomyRegistry by key, and attaches WorldModelComponent + ControlInputCache.
/// No concrete algorithm types are named here.
pub fn spawn_world_model_modules(
    mut commands: Commands,
    agent_query: Query<(Entity, &SpawnAgentConfigRequest, &Children)>,
    measurement_model_query: Query<&MeasurementModel>,
    gravity: Res<Gravity>,
    registry: Res<AutonomyRegistry>,
) {
    // Snapshot the dynamics factory map once. All Plugin::build() calls complete
    // before OnEnter(SceneBuilding), so every registered dynamics factory is present.
    let dynamics_factories = registry.clone_dynamics();

    for (agent_entity, request, children) in &agent_query {
        let agent_config = &request.0;
        let gravity_magnitude = gravity.0.length() as f64;
        let mut entity_commands = commands.entity(agent_entity);

        // --- Pre-build measurement models from child sensor entities (ECS work stays here) ---
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
                // --- Build the estimator via registry ---
                let estimator: Option<Box<dyn StateEstimator>> =
                    if let Some(cfg) = est_cfg {
                        let ctx = EstimatorBuildContext {
                            agent_entity,
                            estimator_cfg: cfg.clone(),
                            agent_config: agent_config.clone(),
                            gravity_magnitude,
                            measurement_models,
                            dynamics_factories: dynamics_factories.clone(),
                        };
                        let result = registry.build_estimator(cfg.get_kind_str(), ctx);
                        if result.is_none() {
                            warn!(
                                "AutonomyRegistry: estimator '{}' returned None for agent '{}'.",
                                cfg.get_kind_str(),
                                &agent_config.name
                            );
                        }
                        result
                    } else {
                        None
                    };

                // --- Build the mapper via registry ---
                let mapper: Box<dyn Mapper> = if let Some(cfg) = map_cfg {
                    // Insert a rate-limited timer if the mapper needs one
                    if let Some(rate) = cfg.get_timer_rate() {
                        entity_commands.insert(ModuleTimer::from_hz(rate));
                    }
                    registry
                        .build_mapper(
                            cfg.get_kind_str(),
                            MapperBuildContext {
                                agent_entity,
                                mapper_cfg: cfg.clone(),
                            },
                        )
                        .unwrap_or_else(|| {
                            warn!(
                                "AutonomyRegistry: mapper '{}' unknown, defaulting to NoneMapper.",
                                cfg.get_kind_str()
                            );
                            Box::new(NoneMapper)
                        })
                } else {
                    Box::new(NoneMapper)
                };

                // --- Insert the final component ---
                if let Some(est) = estimator {
                    let ctrl_dim = est.get_dynamics_model().get_control_dim();
                    if ctrl_dim > 0 {
                        entity_commands.insert(ControlInputCache {
                            u: DVector::zeros(ctrl_dim),
                        });
                    }
                    entity_commands
                        .insert(WorldModelComponent::Separate { estimator: est, mapper });
                }
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
                    Some(system) => {
                        entity_commands.insert(WorldModelComponent::CombinedSlam { system });
                    }
                    None => {
                        warn!(
                            "AutonomyRegistry: SLAM '{}' not yet implemented for agent '{}'.",
                            slam_cfg.get_kind_str(),
                            &agent_config.name
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
// == RUNTIME SYSTEMS (unchanged) ==
// =========================================================================

pub fn world_model_event_processor(
    mut agent_query: Query<(Entity, &mut WorldModelComponent, &mut ControlInputCache)>,
    mut measurement_events: EventReader<BevyMeasurementMessage>,
    tf_tree: Res<TfTree>,
) {
    let mut messages: Vec<_> = measurement_events.read().map(|e| e.0.clone()).collect();
    if messages.is_empty() {
        return;
    }
    messages.sort_by(|a, b| a.timestamp.partial_cmp(&b.timestamp).unwrap());

    let context = FilterContext {
        tf: Some(&*tf_tree),
    };

    for message in messages {
        if let Ok((_, mut module, mut cache)) =
            agent_query.get_mut(message.agent_handle.to_entity())
        {
            if let WorldModelComponent::Separate { estimator, .. } = &mut *module {
                let dt = message.timestamp - estimator.get_state().last_update_timestamp;
                if dt > 1e-9 {
                    estimator.predict(dt, &cache.u, &context);
                }

                let dynamics = estimator.get_dynamics_model();
                if let Some(new_u) = dynamics.get_control_from_measurement(&message.data) {
                    cache.u = new_u;
                } else {
                    estimator.update(&message, &context);
                }
            }
        }
    }
}

pub fn world_model_mapping_system(
    mut module_query: Query<(&mut WorldModelComponent, &mut ModuleTimer)>,
    mut measurement_events: EventReader<BevyMeasurementMessage>,
    time: Res<Time>,
    tf_tree: Res<TfTree>,
) {
    let all_new_messages: Vec<_> = measurement_events.read().collect();
    if all_new_messages.is_empty() {
        return;
    }

    let context = FilterContext {
        tf: Some(&*tf_tree),
    };

    for (mut module, mut timer) in &mut module_query {
        timer.0.tick(time.delta());

        if timer.0.just_finished() {
            if let WorldModelComponent::Separate { estimator, mapper } = &mut *module {
                let pose_update = ModuleInput::PoseUpdate {
                    pose: estimator.get_state(),
                };
                mapper.process(&pose_update, &context);

                for event in &all_new_messages {
                    let sensor_update = ModuleInput::Measurement { message: &event.0 };
                    mapper.process(&sensor_update, &context);
                }
            }
        }
    }
}

pub fn world_model_output_publisher(
    query: Query<&WorldModelComponent>,
    mut topic_bus: ResMut<TopicBus>,
) {
    for module in &query {
        match &*module {
            WorldModelComponent::Separate { estimator, mapper } => {
                let state = estimator.get_state();
                topic_bus.publish("/state/estimated", state.clone());

                let map = mapper.get_map();
                if !matches!(map, helios_core::mapping::MapData::None) {
                    topic_bus.publish("/map", map.clone());
                }
            }
            WorldModelComponent::CombinedSlam { system } => {
                let _ = system;
            }
        }
    }
}
