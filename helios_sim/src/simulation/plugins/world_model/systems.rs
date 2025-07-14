use bevy::prelude::*;
use helios_core::estimation::StateEstimator;
use helios_core::mapping::Mapper;
use nalgebra::DMatrix;
use std::collections::HashMap;

// Import from our own plugin's modules
use super::types::{FrameInputs, OwnedModuleInput};

// Import from the rest of the simulation crate
use crate::prelude::*;
use crate::simulation::core::config::{EstimatorConfig, WorldModelConfig};
use crate::simulation::core::events::BevyMeasurementMessage;
use crate::simulation::core::topics::TopicBus;
use crate::simulation::plugins::world_model::WorldModelComponent;

// Import from the core library
use helios_core::{
    estimation::filters::ekf::ExtendedKalmanFilter, estimation::FilterContext, mapping::NoneMapper,
    models::measurement::Measurement, types::FrameHandle,
};

// =========================================================================
// == SPAWNING SYSTEM ==
// =========================================================================

/// This system runs once at startup. It reads the agent's configuration
/// and attaches the appropriate `WorldModelComponent` with a fully instantiated
/// set of algorithms (e.g., an EKF).
pub fn spawn_world_model_modules(
    mut commands: Commands,
    // Query for agents that have a spawn request, and also get their children (for finding sensors).
    agent_query: Query<
        (Entity, &SpawnAgentConfigRequest, &Children),
        Added<SpawnAgentConfigRequest>,
    >,
    // A query to get the `MeasurementModel` component from sensor entities.
    sensor_query: Query<&MeasurementModel>,
) {
    for (agent_entity, request, children) in &agent_query {
        // Get the specific WorldModelConfig for this agent.
        let world_model_config = &request.0.autonomy_stack.world_model;

        // Match on the world model configuration variant (Separate vs. CombinedSlam).
        match world_model_config {
            WorldModelConfig::Separate { estimator, mapper } => {
                // This agent uses a separate Estimator and/or Mapper.

                // --- 1. Instantiate the Estimator ---
                let estimator = if let Some(est_config) = &estimator {
                    // Create the correct estimator based on the "kind" field.
                    match est_config {
                        EstimatorConfig::Ekf(ekf_config) => {
                            info!(
                                "  -> Attaching EKF as Estimator for agent '{}'",
                                &request.0.name
                            );

                            // --- Build all dependencies for the EKF ---
                            let dynamics_model: Box<dyn Dynamics> =
                                match ekf_config.dynamics.as_str() {
                                    "ConstantAcceleration" => Box::new(ConstantAccelerationModel {
                                        agent_handle: FrameHandle::from_entity(agent_entity),
                                    }),
                                    // You could even configure an Ackermann model for your filter here
                                    // if you wanted to, independent of the vehicle.
                                    "Ackermann" => Box::new(AckermannKinematics {
                                        wheelbase: 2.5,
                                        agent_handle: FrameHandle::from_entity(agent_entity),
                                    }),
                                    _ => panic!(
                                        "Unsupported dynamics for EKF: {}",
                                        ekf_config.dynamics
                                    ),
                                };
                            let state_layout = dynamics_model.get_state_layout();
                            let state_dim = state_layout.len();
                            let initial_state = FrameAwareState::new(state_layout, 1.0, 0.0);
                            let q_matrix =
                                DMatrix::identity(state_dim, state_dim) * ekf_config.process_noise;

                            let mut measurement_models: HashMap<FrameHandle, Box<dyn Measurement>> =
                                HashMap::new();
                            for child_entity in children.iter() {
                                if let Ok(model_component) = sensor_query.get(child_entity) {
                                    let model_clone = model_component.0.clone();
                                    measurement_models.insert(
                                        FrameHandle::from_entity(child_entity),
                                        model_clone,
                                    );
                                }
                            }

                            // Instantiate the EKF from helios_core.
                            let ekf = ExtendedKalmanFilter::new(
                                initial_state,
                                q_matrix,
                                dynamics_model,
                                measurement_models,
                            );
                            Some(Box::new(ekf) as Box<dyn StateEstimator>)
                        }
                        EstimatorConfig::Ukf(_) => {
                            // Placeholder for future UKF implementation
                            warn!("UKF not yet implemented.");
                            None
                        }
                    }
                } else {
                    None // No estimator configured.
                };

                // --- 2. Instantiate the Mapper ---
                let mapper = if let Some(map_config) = &mapper {
                    // For now, we only have the NoneMapper placeholder.
                    info!("  -> Attaching NoneMapper for agent '{}'", &request.0.name);
                    Box::new(NoneMapper::default()) as Box<dyn Mapper>
                } else {
                    Box::new(NoneMapper::default()) as Box<dyn Mapper>
                };

                // --- 3. Insert the WorldModelComponent ---
                // Only insert if we actually created an estimator.
                if let Some(estimator) = estimator {
                    commands
                        .entity(agent_entity)
                        .insert(WorldModelComponent::Separate { estimator, mapper });
                }
            }
            WorldModelConfig::CombinedSlam { slam } => {
                let _ = slam;
                // Placeholder for when we implement SLAM systems.
                warn!("SLAM systems not yet implemented.");
            }
        }
    }
}

// =========================================================================
// == RUNTIME SYSTEMS (THE PIPELINE) ==
// =========================================================================

// --- SYSTEM 1: Input Gatherer ---
/// Collects all relevant inputs from the frame (time, measurements) and
/// stores them in the `FrameInputs` resource, sorted chronologically.
pub fn world_model_input_gatherer(
    mut frame_inputs: ResMut<FrameInputs>,
    time: Res<Time>,
    mut measurement_events: EventReader<BevyMeasurementMessage>, // The raw Bevy event
) {
    frame_inputs.0.clear();

    // The first "input" of every frame is the passage of time.
    frame_inputs.0.push(OwnedModuleInput::TimeStep {
        dt: time.delta_secs_f64(),
        current_time: time.elapsed_secs_f64(),
    });

    // Collect all measurement events and sort them by timestamp to ensure correct processing order.
    let mut sorted_events: Vec<_> = measurement_events.read().map(|e| e.0.clone()).collect();
    if !sorted_events.is_empty() {
        sorted_events.sort_by(|a, b| a.timestamp.partial_cmp(&b.timestamp).unwrap());
        for event in sorted_events {
            println!("event: {:?}", event);

            frame_inputs
                .0
                .push(OwnedModuleInput::Measurement { message: event });
        }
    }
}

// --- SYSTEM 2: Module Processor ---
/// Iterates through all entities with a `WorldModelComponent` and feeds them
/// the prepared `FrameInputs` for this frame.
pub fn world_model_processor(
    mut agent_query: Query<(Entity, &mut WorldModelComponent)>,
    frame_inputs: Res<FrameInputs>,
    tf_tree: Res<TfTree>,
) {
    if frame_inputs.0.is_empty() {
        return;
    }

    for (agent_entity, mut module) in &mut agent_query {
        // Build the context for this specific agent.
        let context = FilterContext {
            tf: Some(&*tf_tree),
        };

        match &mut *module {
            WorldModelComponent::Separate { estimator, mapper } => {
                // Loop through every input for this frame in chronological order.
                for owned_input in &frame_inputs.0 {
                    // Check if the measurement belongs to this agent before processing.
                    if let OwnedModuleInput::Measurement { message } = owned_input {
                        if message.agent_handle != FrameHandle::from_entity(agent_entity) {
                            continue;
                        }
                    }

                    info!("owned_input coming into estimator: {:?}", owned_input);

                    // Process the input with the estimator.
                    estimator.process(&owned_input.as_ref(), &context);
                }
                let _ = mapper;
            }
            WorldModelComponent::CombinedSlam { system } => {
                // Logic for SLAM systems would go here.
                let _ = system;
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
                info!("state: {}", state.vector.transpose());
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
