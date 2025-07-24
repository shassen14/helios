use avian3d::prelude::Gravity;
use bevy::prelude::*;
use helios_core::estimation::StateEstimator;
use helios_core::frames::layout::STANDARD_INS_STATE_DIM;
use helios_core::mapping::Mapper;
use nalgebra::{DMatrix, DVector};
use std::collections::HashMap;

// Import from the rest of the simulation crate
use crate::prelude::*;
use crate::simulation::config::structs::{EstimatorConfig, MapperConfig, WorldModelConfig};
use crate::simulation::core::events::BevyMeasurementMessage;
use crate::simulation::core::topics::TopicBus;
use crate::simulation::plugins::world_model::components::{ControlInputCache, ModuleTimer};
use crate::simulation::plugins::world_model::WorldModelComponent;

// Import from the core library
use helios_core::{
    estimation::filters::ekf::ExtendedKalmanFilter, estimation::FilterContext,
    frames::layout::standard_ins_state_layout, mapping::NoneMapper,
    models::estimation::dynamics::integrated_imu::IntegratedImuModel, types::FrameHandle,
};

// =========================================================================
// == SPAWNING SYSTEM ==
// =========================================================================

/// This system runs once at startup. It reads the agent's configuration
/// and attaches the appropriate `WorldModelComponent` with a fully instantiated
/// set of algorithms (e.g., an EKF).
pub fn spawn_world_model_modules(
    mut commands: Commands,
    agent_query: Query<(Entity, &SpawnAgentConfigRequest, &Children)>,
    measurement_model_query: Query<&MeasurementModel>,
    gravity: Res<Gravity>,
) {
    for (agent_entity, request, children) in &agent_query {
        let world_model_config = &request.0.autonomy_stack.world_model;
        let mut entity_commands = commands.entity(agent_entity);

        match world_model_config {
            Some(WorldModelConfig::Separate {
                estimator: est_config,
                mapper: map_config,
            }) => {
                // --- Block 1: Create the Estimator (if configured) ---
                // We create the `estimator` variable in this outer scope, initialized to None.
                let estimator: Option<Box<dyn StateEstimator>> = if let Some(config) = est_config {
                    // Now we determine which kind of estimator to build.
                    match config {
                        EstimatorConfig::Ekf(ekf_config) => {
                            info!(
                                "  -> Attaching EKF as Estimator for agent '{}'",
                                &request.0.name
                            );

                            let state_dim = STANDARD_INS_STATE_DIM;
                            let agent_handle = FrameHandle::from_entity(agent_entity);
                            let mut dynamics_model: Option<Box<dyn EstimationDynamics>> = None;
                            let mut q_matrix = DMatrix::<f64>::zeros(state_dim, state_dim);

                            match &ekf_config.dynamics {
                                EkfDynamicsConfig::IntegratedImu(noise_conf) => {
                                    info!(
                                        "  -> EKF for '{}' using IntegratedImu dynamics",
                                        request.0.name
                                    );

                                    // A. Instantiate the correct dynamics model.
                                    dynamics_model = Some(Box::new(IntegratedImuModel {
                                        agent_handle,
                                        gravity_magnitude: gravity.0.length() as f64,
                                    }));

                                    // B. Build the Q matrix using the parameters from the `noise_conf` struct.
                                    let an_var = noise_conf.accel_noise_stddev.powi(2);
                                    let gn_var = noise_conf.gyro_noise_stddev.powi(2);
                                    let ab_var = noise_conf.accel_bias_instability.powi(2);
                                    let gb_var = noise_conf.gyro_bias_instability.powi(2);

                                    // Velocity noise
                                    q_matrix[(3, 3)] = an_var;
                                    q_matrix[(4, 4)] = an_var;
                                    q_matrix[(5, 5)] = an_var;
                                    // Orientation noise (from gyro noise)
                                    // TODO: technically not correct to define directly to quaternion
                                    // but it's practically acceptable approximation for small dt
                                    q_matrix[(6, 6)] = gn_var;
                                    q_matrix[(7, 7)] = gn_var;
                                    q_matrix[(8, 8)] = gn_var;
                                    // Accel bias noise
                                    q_matrix[(10, 10)] = ab_var;
                                    q_matrix[(11, 11)] = ab_var;
                                    q_matrix[(12, 12)] = ab_var;
                                    // Gyro bias noise
                                    q_matrix[(13, 13)] = gb_var;
                                    q_matrix[(14, 14)] = gb_var;
                                    q_matrix[(15, 15)] = gb_var;
                                }
                                EkfDynamicsConfig::AckermannOdometry(noise_conf) => {
                                    warn!("AckermannOdometry dynamics model not yet implemented.");
                                    // When you implement it, the logic would be here.
                                }
                                EkfDynamicsConfig::Quadcopter(_) => {
                                    warn!("Quadcopter dynamics model not yet implemented.");
                                }
                            }

                            // If we successfully created a dynamics model...
                            if let Some(dynamics) = dynamics_model {
                                if dynamics.get_control_dim() > 0 {
                                    entity_commands.insert(ControlInputCache {
                                        u: DVector::zeros(dynamics.get_control_dim()),
                                    });
                                }

                                // --- 2. Build Measurement Model Map ---
                                let mut measurement_models = HashMap::new();
                                for child_entity in children.iter() {
                                    if let Ok(model_component) =
                                        measurement_model_query.get(child_entity)
                                    {
                                        let should_add = match &ekf_config.dynamics {
                                            EkfDynamicsConfig::IntegratedImu(_) => {
                                                !model_component.0.as_any().is::<Imu6DofModel>()
                                            }
                                            _ => true,
                                        };
                                        if should_add {
                                            measurement_models.insert(
                                                FrameHandle::from_entity(child_entity),
                                                model_component.0.clone(),
                                            );
                                        }
                                    }
                                }

                                // --- 3. Compose and Insert the Final Component ---
                                let state_layout = standard_ins_state_layout(agent_handle);
                                let initial_state = FrameAwareState::new(state_layout, 1.0, 0.0);

                                let ekf = ExtendedKalmanFilter::new(
                                    initial_state,
                                    q_matrix,
                                    dynamics,
                                    measurement_models,
                                );
                                // The result of this whole block is `Some(Box<dyn StateEstimator>)`
                                Some(Box::new(ekf))
                            } else {
                                warn!("EKF not created due to some error.");
                                None
                            }
                        }
                        EstimatorConfig::Ukf(_) => {
                            warn!("UKF not yet implemented.");
                            None
                        }
                    }
                } else {
                    None // No estimator configured, so we get None.
                };

                // --- Block 2: Create the Mapper (if configured) ---
                let mapper: Box<dyn Mapper> = if let Some(config) = map_config {
                    match config {
                        MapperConfig::OccupancyGrid2D { rate, .. } => {
                            info!("  -> Attaching OccupancyGridMapper with {rate}Hz timer.");
                            entity_commands.insert(ModuleTimer::from_hz(*rate));
                            // Box::new(OccupancyGridMapper::new(/*...*/))
                            Box::new(NoneMapper) // Placeholder until implemented
                        }
                        MapperConfig::None => Box::new(NoneMapper),
                    }
                } else {
                    // Default to NoneMapper if no mapper section exists.
                    Box::new(NoneMapper)
                };

                // --- Block 3: Insert the Final Component ---
                // Now we check if our `estimator` variable from Block 1 is Some.
                if let Some(est) = estimator {
                    // The `mapper` variable from Block 2 always exists.
                    entity_commands.insert(WorldModelComponent::Separate {
                        estimator: est,
                        mapper,
                    });
                }
            }
            Some(WorldModelConfig::CombinedSlam { .. }) => {
                warn!("SLAM systems not yet implemented.");
            }
            None => {
                warn!("Separate estimator and mapper are none.")
            }
        }
    }
}

pub fn world_model_event_processor(
    mut agent_query: Query<(Entity, &mut WorldModelComponent, &mut ControlInputCache)>,
    mut measurement_events: EventReader<BevyMeasurementMessage>,
    // Get the dependencies for the context directly.
    tf_tree: Res<TfTree>,
) {
    let mut messages: Vec<_> = measurement_events.read().map(|e| e.0.clone()).collect();
    if messages.is_empty() {
        return;
    }
    messages.sort_by(|a, b| a.timestamp.partial_cmp(&b.timestamp).unwrap());

    // --- Build the context once per system run ---
    let context = FilterContext {
        tf: Some(&*tf_tree),
    };

    for message in messages {
        if let Ok((_, mut module, mut cache)) =
            agent_query.get_mut(message.agent_handle.to_entity())
        {
            if let WorldModelComponent::Separate { estimator, .. } = &mut *module {
                // --- Predict-to-Update using the LAST known control input ---
                let dt = message.timestamp - estimator.get_state().last_update_timestamp;
                if dt > 1e-9 {
                    estimator.predict(dt, &cache.u, &context);
                }

                let dynamics = estimator.get_dynamics_model();
                if let Some(new_u) = dynamics.get_control_from_measurement(&message.data) {
                    cache.u = new_u;
                } else {
                    // This is an aiding measurement, run the update step.
                    estimator.update(&message, &context);
                }
            }
        }
    }
}

/// A separate system for a slow mapping process, gated by a timer.
pub fn world_model_mapping_system(
    mut module_query: Query<(&mut WorldModelComponent, &mut ModuleTimer)>,
    mut measurement_events: EventReader<BevyMeasurementMessage>,
    time: Res<Time>,
    tf_tree: Res<TfTree>, // Get dependency directly
) {
    let all_new_messages: Vec<_> = measurement_events.read().collect();
    if all_new_messages.is_empty() {
        return;
    }

    // --- Build the context once per system run ---
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
// --- SYSTEM 3: Output Publisher ---
/// After all processing is done, this system reads the final state from each
/// `WorldModelComponent` and publishes it to the appropriate topics.
pub fn world_model_output_publisher(
    query: Query<&WorldModelComponent>,
    mut topic_bus: ResMut<TopicBus>, // Assuming a TopicBus resource for publishing
) {
    for module in &query {
        match &*module {
            WorldModelComponent::Separate { estimator, mapper } => {
                // Publish the estimated state
                let state = estimator.get_state();
                // info!("state: {}", state.vector);
                topic_bus.publish("/state/estimated", state.clone());

                // Publish the map
                let map = mapper.get_map();
                if !matches!(map, helios_core::mapping::MapData::None) {
                    topic_bus.publish("/map", map.clone());
                }
            }
            WorldModelComponent::CombinedSlam { system } => {
                // Publish state and map from the SLAM system
                let _ = system;
            }
        }
    }
}
