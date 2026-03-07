use avian3d::prelude::Gravity;
use bevy::prelude::*;
use helios_core::estimation::StateEstimator;
use helios_core::mapping::{Mapper, NoneMapper};
use helios_core::types::FrameHandle;
use nalgebra::DVector;
use std::collections::HashMap;

use crate::prelude::*;
use crate::simulation::config::structs::WorldModelConfig;
use crate::simulation::core::events::BevyMeasurementMessage;
use crate::simulation::core::topics::TopicBus;
use crate::simulation::core::transforms::enu_iso_to_bevy_transform;
use crate::simulation::plugins::world_model::components::{
    ControlInputCache, ModuleTimer, OdomFrameOf,
};
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
                let estimator: Option<Box<dyn StateEstimator>> = if let Some(cfg) = est_cfg {
                    let ctx = EstimatorBuildContext {
                        agent_entity,
                        estimator_cfg: cfg.clone(),
                        agent_config: agent_config.clone(),
                        gravity_magnitude,
                        measurement_models,
                        dynamics_factories: dynamics_factories.clone(),
                    };
                    match registry.build_estimator(cfg.get_kind_str(), ctx) {
                        Ok(est) => Some(est),
                        Err(e) => {
                            error!(
                                "Cannot build world model for agent '{}': {}",
                                agent_config.name, e
                            );
                            continue;
                        }
                    }
                } else {
                    None
                };

                // --- Build the mapper via registry ---
                let mapper: Box<dyn Mapper> = if let Some(cfg) = map_cfg {
                    // Insert a rate-limited timer if the mapper needs one
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
                        Ok(m) => m,
                        Err(e) => {
                            error!(
                                "Mapper build failed for '{}': {}. Falling back to NoneMapper.",
                                agent_config.name, e
                            );
                            Box::new(NoneMapper)
                        }
                    }
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
                    Ok(system) => {
                        entity_commands.insert(WorldModelComponent::CombinedSlam { system });
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
    mut module_query: Query<(Entity, &mut WorldModelComponent, &mut ModuleTimer)>,
    odom_query: Query<(&OdomFrameOf, Entity)>,
    mut measurement_events: EventReader<BevyMeasurementMessage>,
    time: Res<Time>,
    tf_tree: Res<TfTree>,
) {
    // Collect only this tick's messages. Bevy events expire after ~2 Update frames
    // (~33 ms at 60 fps), so we must process them immediately rather than waiting
    // for the mapper's slower output timer.
    let all_new_messages: Vec<_> = measurement_events.read().map(|e| e.0.clone()).collect();

    let context = FilterContext {
        tf: Some(&*tf_tree),
    };

    // Build a map from agent entity → odom world pose (from TfTree).
    // This decouples the mapper from the estimator: the mapper now tracks the
    // odom frame rather than directly reading EKF internal state.
    let agent_to_odom_iso: std::collections::HashMap<Entity, nalgebra::Isometry3<f64>> =
        odom_query
            .iter()
            .filter_map(|(odom_of, odom_entity)| {
                tf_tree
                    .lookup_by_entity(odom_entity)
                    .map(|iso| (odom_of.0, iso))
            })
            .collect();

    for (agent_entity, mut module, mut timer) in &mut module_query {
        timer.0.tick(time.delta());

        if let WorldModelComponent::Separate { mapper, .. } = &mut *module {
            // Always forward sensor data to the mapper as it arrives.
            // The mapper's log-odds update is cheap; cache rebuild is not.
            for message in &all_new_messages {
                mapper.process(&ModuleInput::Measurement { message }, &context);
            }

            // On timer fire: update pose from the odom TF frame and rebuild the cache.
            // Using the odom frame (not the estimator) keeps the mapper decoupled.
            if timer.0.just_finished() {
                if let Some(&odom_iso) = agent_to_odom_iso.get(&agent_entity) {
                    mapper.process(&ModuleInput::PoseUpdate { pose: odom_iso }, &context);
                }
            }
        }
    }
}

// =========================================================================
// == ODOM FRAME SYSTEMS ==
// =========================================================================

/// Spawns a virtual "odom frame" entity for each agent that has a world model.
///
/// Runs in `SceneBuildSet::Finalize` — after `WorldModelComponent` has been
/// attached in `ProcessBaseAutonomy`, so we can filter on it.
/// The odom entity has no physics; its `Transform` is driven each tick
/// by `update_odom_frames`.
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

/// Writes the latest pose estimate into the odom TF frame every tick.
///
/// Runs at the end of `SimulationSet::Estimation`, after the EKF/SLAM has
/// processed all measurements for this tick.
///
/// Any `WorldModelComponent` variant that exposes a pose drives the odom frame:
/// - `Separate { estimator }` → EKF/UKF output via `get_state().get_pose_isometry()`
/// - `CombinedSlam { system }` → will use `SlamSystem::get_pose()` once the
///   trait method is defined (stubbed here until SLAM is implemented).
///
/// If the estimator has not yet converged (pose returns `None`), the odom
/// frame stays at its last known position — it never jumps to the origin.
pub fn update_odom_frames(
    agent_query: Query<&WorldModelComponent>,
    mut odom_query: Query<(&OdomFrameOf, &mut Transform)>,
) {
    for (odom_of, mut transform) in &mut odom_query {
        let Ok(world_model) = agent_query.get(odom_of.0) else {
            continue;
        };

        let pose_enu = match world_model {
            WorldModelComponent::Separate { estimator, .. } => {
                estimator.get_state().get_pose_isometry()
            }
            WorldModelComponent::CombinedSlam { .. } => {
                // TODO: call system.get_pose() once SlamSystem exposes it.
                None
            }
        };

        if let Some(iso) = pose_enu {
            *transform = enu_iso_to_bevy_transform(&iso);
        }
    }
}

pub fn world_model_output_publisher(
    query: Query<(&WorldModelComponent, &Name)>,
    mut topic_bus: ResMut<TopicBus>,
) {
    for (module, name) in &query {
        match &*module {
            WorldModelComponent::Separate { estimator, mapper } => {
                let state = estimator.get_state();
                topic_bus.publish(
                    &format!("/{}/odometry/estimated", name.as_str()),
                    state.clone(),
                );

                let map = mapper.get_map();
                if !matches!(map, helios_core::mapping::MapData::None) {
                    topic_bus.publish(
                        &format!("/{}/map", name.as_str()),
                        map.clone(),
                    );
                }
            }
            WorldModelComponent::CombinedSlam { system } => {
                let _ = system;
            }
        }
    }
}
