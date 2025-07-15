use avian3d::prelude::Gravity;
use bevy::prelude::*;
use helios_core::estimation::StateEstimator;
use helios_core::mapping::Mapper;
use nalgebra::{DMatrix, DVector};
use std::collections::HashMap;

// Import from our own plugin's modules
use super::types::FrameInputs;

// Import from the rest of the simulation crate
use crate::prelude::*;
use crate::simulation::core::config::{EstimatorConfig, MapperConfig, WorldModelConfig};
use crate::simulation::core::events::BevyMeasurementMessage;
use crate::simulation::core::topics::TopicBus;
use crate::simulation::plugins::world_model::components::{ControlInputCache, ModuleTimer};
use crate::simulation::plugins::world_model::WorldModelComponent;

// Import from the core library
use helios_core::{
    estimation::filters::ekf::ExtendedKalmanFilter,
    estimation::FilterContext,
    frames::layout::standard_ins_state_layout,
    mapping::NoneMapper,
    models::estimation::{dynamics::integrated_imu::IntegratedImuModel, measurement::Measurement},
    types::FrameHandle,
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
            WorldModelConfig::Separate {
                estimator: est_config,
                mapper: map_config,
            } => {
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

                            let agent_handle = FrameHandle::from_entity(agent_entity);

                            let dynamics_model: Box<dyn EstimationDynamics> = match ekf_config
                                .dynamics
                                .as_str()
                            {
                                "IntegratedImu" => Box::new(IntegratedImuModel {
                                    agent_handle,
                                    gravity_magnitude: gravity.0.length() as f64,
                                }),
                                _ => panic!("Unsupported dynamics model: {}", ekf_config.dynamics),
                            };

                            if dynamics_model.get_control_dim() > 0 {
                                entity_commands.insert(ControlInputCache {
                                    u: DVector::zeros(dynamics_model.get_control_dim()),
                                });
                            }

                            let mut measurement_models = HashMap::new();
                            for child_entity in children.iter() {
                                if let Ok(model_component) =
                                    measurement_model_query.get(child_entity)
                                {
                                    let should_add = if ekf_config.dynamics == "IntegratedImu" {
                                        !model_component.0.as_any().is::<Imu6DofModel>()
                                    } else {
                                        true
                                    };
                                    if should_add {
                                        measurement_models.insert(
                                            FrameHandle::from_entity(child_entity),
                                            model_component.0.clone(),
                                        );
                                    }
                                }
                            }

                            let state_layout = standard_ins_state_layout(agent_handle);
                            let initial_state = FrameAwareState::new(state_layout, 1.0, 0.0);
                            let mut q_matrix = DMatrix::zeros(16, 16);
                            for i in 10..16 {
                                q_matrix[(i, i)] = ekf_config.process_noise.powi(2);
                            }

                            let ekf = ExtendedKalmanFilter::new(
                                initial_state,
                                q_matrix,
                                dynamics_model,
                                measurement_models,
                            );

                            // The result of this whole block is `Some(Box<dyn StateEstimator>)`
                            Some(Box::new(ekf))
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
            WorldModelConfig::CombinedSlam { .. } => {
                warn!("SLAM systems not yet implemented.");
            }
        }
    }
}
// pub fn spawn_world_model_modules(
//     mut commands: Commands,
//     agent_query: Query<(Entity, &SpawnAgentConfigRequest, &Children)>,
//     // This query is now more complex. We need the entity's config and its children's configs.
//     // This allows us to find all sensors attached to an agent.
//     measurement_model_query: Query<&MeasurementModel>,
//     gravity: Res<Gravity>,
// ) {
//     for (agent_entity, request, children) in &agent_query {
//         if let WorldModelConfig::Separate {
//             estimator: Some(est_config),
//             ..
//         } = &request.0.autonomy_stack.world_model
//         {
//             if let EstimatorConfig::Ekf(ekf_config) = est_config {
//                 let agent_handle = FrameHandle::from_entity(agent_entity);

//                 // --- 1. Compose the Dynamics Model based on config ---
//                 let dynamics_model: Box<dyn EstimationDynamics> = match ekf_config.dynamics.as_str()
//                 {
//                     "IntegratedImu" => {
//                         info!(
//                             "  -> EKF for '{}' using IntegratedImu dynamics",
//                             request.0.name
//                         );
//                         Box::new(IntegratedImuModel {
//                             agent_handle,
//                             gravity_magnitude: gravity.0.length() as f64,
//                         })
//                     }
//                     // "ConstantAcceleration" => {
//                     //     info!(
//                     //         "  -> EKF for '{}' using ConstantAcceleration dynamics",
//                     //         request.0.name
//                     //     );
//                     //     Box::new(ConstantAccelerationModel {
//                     //         agent_handle,
//                     //         damping: 0.5,
//                     //     })
//                     // }
//                     _ => panic!("Unsupported dynamics model: {}", ekf_config.dynamics),
//                 };

//                 // --- 2. Compose the Measurement Models for Aiding Sensors ---
//                 let mut measurement_models = HashMap::new();
//                 for child_entity in children.iter() {
//                     if let Ok(model_component) = measurement_model_query.get(child_entity) {
//                         let sensor_handle = FrameHandle::from_entity(child_entity);
//                         let model_clone = model_component.0.clone();

//                         // --- The intelligent filtering logic ---
//                         // This logic is based on the *estimator's* needs, not the sensor's type.
//                         let mut should_add = true;
//                         if ekf_config.dynamics == "IntegratedImu" {
//                             // If using the advanced INS model, we have a special rule:
//                             // Do not use the IMU as an aiding measurement.
//                             if model_clone.as_any().is::<Imu6DofModel>()
//                             // || model_clone.as_any().is::<Imu9DofModel>()
//                             {
//                                 should_add = false;
//                             }
//                         }

//                         if should_add {
//                             info!(
//                                 "  -> Adding measurement model from sensor {:?} to EKF.",
//                                 child_entity
//                             );
//                             measurement_models.insert(sensor_handle, model_clone);
//                         }
//                     }
//                 }

//                 // --- 3. Compose the Final Estimator ---
//                 let state_layout = standard_ins_state_layout(agent_handle);
//                 let initial_state = FrameAwareState::new(state_layout, 1.0, 0.0);
//                 let mut q_matrix = DMatrix::zeros(16, 16);
//                 for i in 10..16 {
//                     q_matrix[(i, i)] = ekf_config.process_noise.powi(2);
//                 }

//                 let ekf = ExtendedKalmanFilter::new(
//                     initial_state,
//                     q_matrix,
//                     dynamics_model,
//                     measurement_models,
//                 );

//                 commands
//                     .entity(agent_entity)
//                     .insert(WorldModelComponent::Separate {
//                         estimator: Box::new(ekf),
//                         mapper: Box::new(NoneMapper),
//                     });
//             }
//         }
//     }
// }

// =========================================================================
// == RUNTIME SYSTEMS (THE PIPELINE) ==
// =========================================================================

// --- SYSTEM 1: Input Gatherer ---
pub fn world_model_input_gatherer(
    mut frame_inputs: ResMut<FrameInputs>,
    mut measurement_events: EventReader<BevyMeasurementMessage>,
) {
    frame_inputs.0.clear();
    frame_inputs
        .0
        .extend(measurement_events.read().map(|e| e.0.clone()));
    if frame_inputs.0.len() > 1 {
        frame_inputs
            .0
            .sort_by(|a, b| a.timestamp.partial_cmp(&b.timestamp).unwrap());
    }
}

// --- SYSTEM 2: Module Processor ---
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

// pub fn world_model_processor(
//     mut agent_query: Query<(Entity, &mut WorldModelComponent)>,
//     frame_inputs: Res<FrameInputs>,
//     time: Res<Time>,
//     tf_tree: Res<TfTree>,
// ) {
//     let dt = time.delta_secs_f64();
//     if dt <= 0.0 {
//         return;
//     }
//     let context = FilterContext {
//         tf: Some(&*tf_tree),
//     };

//     for (agent_entity, mut module) in &mut agent_query {
//         if let WorldModelComponent::Separate { estimator, .. } = &mut *module {
//             let dynamics = estimator.get_dynamics_model();
//             let mut u = DVector::zeros(dynamics.get_control_dim());

//             // Find the latest message that the dynamics model claims as a control input.
//             for message in frame_inputs.0.iter().rev() {
//                 if message.agent_handle == FrameHandle::from_entity(agent_entity) {
//                     if let Some(control_vec) = dynamics.get_control_from_measurement(&message.data)
//                     {
//                         u = control_vec;
//                         println!("u: {}", u);
//                         break;
//                     }
//                 }
//             }

//             // PREDICT step is always run.
//             estimator.predict(dt, &u, &context);

//             // UPDATE step is run for all messages.
//             for message in &frame_inputs.0 {
//                 if message.agent_handle == FrameHandle::from_entity(agent_entity) {
//                     estimator.update(message, &context);
//                 }
//             }
//         }
//     }
// }

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
                info!("state: {}", state.vector);
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
