// helios_sim/src/plugins/estimation/ekf.rs

use crate::{
    prelude::*,
    simulation::core::{app_state::SimulationSet, events::BevyMeasurementEvent},
};

use nalgebra::DMatrix;
use std::collections::HashMap;

// =========================================================================
// == EKF Components (The "Adapter" Layer Data Structures) ==
// =========================================================================

/// A Bevy component that holds the core configuration and models for a single
/// Extended Kalman Filter instance.
///
/// This component is attached to the main agent entity and contains the pure,
/// framework-agnostic `Dynamics` trait object.
#[derive(Component)]
pub struct EkfCore {
    /// The agent's dynamics model used for the prediction (`predict`) step.
    /// This is a boxed trait object from `helios_core`.
    pub dynamics: Box<dyn Dynamics>,

    /// The Process Noise Covariance matrix (Q), which models uncertainty in the dynamics.
    pub process_noise_q: DMatrix<f64>,
}

/// A Bevy component that holds the live, changing state of a filter.
///
/// It contains the `FrameAwareState` struct from `helios_core`, which bundles
/// the state vector `x`, the covariance `P`, and other metadata.
#[derive(Component, Debug)]
pub struct FilterState {
    pub state: FrameAwareState,
}

// --- The Bevy Plugin ---
pub struct EkfPlugin;

impl Plugin for EkfPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_ekf_instances.in_set(SceneBuildSet::ProcessBaseAutonomy),
        )
        .add_systems(
            FixedUpdate,
            ekf_adapter_system.in_set(SimulationSet::Estimation),
        );
    }
}

// --- Spawning System ---
fn spawn_ekf_instances(
    mut commands: Commands,
    request_query: Query<(Entity, &SpawnAgentConfigRequest)>,
) {
    for (agent_entity, request) in &request_query {
        let agent_config = &request.0;
        if let Some(EstimatorConfig::Ekf(ekf_config)) =
            agent_config.autonomy_stack.estimators.first()
        {
            info!(
                "  -> Attaching EKF instance '{}' to agent '{}'",
                &ekf_config.name, &agent_config.name
            );

            // The "Factory" logic to build the correct dynamics model.
            let dynamics_model: Box<dyn Dynamics> = match ekf_config.dynamics.as_str() {
                "ConstantAcceleration" => Box::new(ConstantAccelerationModel {
                    agent_handle: FrameHandle::from_entity(agent_entity),
                }),
                _ => {
                    // Default to the vehicle's dynamics
                    if let Vehicle::Ackermann { wheelbase, .. } = &agent_config.vehicle {
                        Box::new(AckermannKinematics {
                            wheelbase: *wheelbase as f64,
                            agent_handle: FrameHandle::from_entity(agent_entity),
                        })
                    } else {
                        panic!("No default dynamics model for this vehicle type!");
                    }
                }
            };

            let state_layout = dynamics_model.get_state_layout();
            let state_dim = state_layout.len();
            let q_matrix = DMatrix::identity(state_dim, state_dim) * ekf_config.process_noise;
            let initial_state = FrameAwareState::new(state_layout, 1.0, 0.0);

            // Insert the components directly onto the main agent entity.
            commands.entity(agent_entity).insert((
                // Name::new(format!("Estimator: {}", ekf_config.name)),
                EkfCore {
                    dynamics: dynamics_model,
                    process_noise_q: q_matrix,
                },
                FilterState {
                    state: initial_state,
                },
            ));
        }
    }
}

// --- Runtime Adapter System ---
fn ekf_adapter_system(
    time: Res<Time>,
    tf_tree: Res<TfTree>, // Get the pre-built transform tree
    mut measurement_events: EventReader<BevyMeasurementEvent>,
    model_query: Query<&MeasurementModel>,
    mut agent_query: Query<(Entity, &mut FilterState, &EkfCore)>,
) {
    // Group events by agent handle
    let mut events_by_agent: HashMap<FrameHandle, Vec<MeasurementEvent>> = HashMap::new();
    for event in measurement_events.read() {
        let pure_data = &event.0;
        events_by_agent
            .entry(pure_data.agent_handle)
            .or_default()
            .push(pure_data.clone());
    }
    for events in events_by_agent.values_mut() {
        events.sort_by(|a, b| a.timestamp.partial_cmp(&b.timestamp).unwrap());
    }

    // Process each agent that has an EKF
    for (agent_entity, mut filter_state, ekf_core) in &mut agent_query {
        let agent_handle = FrameHandle::from_entity(agent_entity);

        // Process all new measurements for this agent
        if let Some(events) = events_by_agent.get(&agent_handle) {
            for event in events {
                // --- PREDICT ---
                let dt = event.timestamp - filter_state.state.last_update_timestamp;
                if dt > 0.0001 {
                    let params = EkfUpdateParams {
                        dynamics: &*ekf_core.dynamics,
                        process_noise_q: &ekf_core.process_noise_q,
                    };
                    filter_state.state = ekf_predict(&filter_state.state, &params, dt);
                }

                // --- UPDATE ---
                if let Ok(model_comp) = model_query.get(event.sensor_handle.to_entity()) {
                    let params = EkfMeasurementParams {
                        model: &*model_comp.0,
                        z: &event.z,
                        context: &*tf_tree,
                    };
                    filter_state.state = ekf_update(&filter_state.state, &params);
                    info!(
                        "EKF for agent {:?} updated with sensor {:?}. New state: {:.3?}",
                        agent_entity,
                        event.sensor_handle.to_entity(),
                        filter_state.state.vector.transpose()
                    );
                }

                filter_state.state.last_update_timestamp = event.timestamp;
            }
        }

        // --- Final prediction step ---
        let final_dt = time.elapsed_secs_f64() - filter_state.state.last_update_timestamp;
        if final_dt > 0.0001 {
            let params = EkfUpdateParams {
                dynamics: &*ekf_core.dynamics,
                process_noise_q: &ekf_core.process_noise_q,
            };
            filter_state.state = ekf_predict(&filter_state.state, &params, final_dt);
        }
    }
}
