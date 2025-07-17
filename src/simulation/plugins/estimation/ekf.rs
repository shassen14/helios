// src/simulation/plugins/estimation/ekf.rs

use bevy::prelude::*;
use nalgebra::{DMatrix, DVector};

use crate::{
    prelude::{AppState, EstimatorConfig, ImuData, TopicBus, TopicReader, TopicTag, Vehicle},
    simulation::{
        core::{
            abstractions::{Dynamics, DynamicsModel, MeasurementModel},
            app_state::SceneBuildSet,
            frames::{FrameAwareState, MeasurementEvent},
            simulation_setup::SpawnAgentConfigRequest,
        },
        models::ackermann::AckermannKinematics,
        utils::integrators::RK4,
    },
};

// --- EKF-SPECIFIC COMPONENT ---
/// A component that marks an entity as having an EKF and holds its core models.
#[derive(Component)]
pub struct EKF {
    /// The agent's dynamics model for the prediction (`predict`) step.
    pub dynamics: Box<dyn Dynamics>,
    /// The Process Noise Covariance matrix (Q), which models uncertainty in the dynamics.
    pub process_noise_q: DMatrix<f64>,
}

/// The component that holds the live, changing state of a filter.
#[derive(Component, Debug)]
pub struct FilterState {
    /// The "smart" state object containing the layout, vector, covariance, and timestamp.
    pub state: FrameAwareState,
}

#[derive(Component, Default)]
pub struct ImuSubscriptions {
    pub readers: Vec<TopicReader<ImuData>>,
}

// Plugin
pub struct EkfPlugin;

impl Plugin for EkfPlugin {
    fn build(&self, app: &mut App) {
        // We add two systems: one for spawning, one for runtime logic.
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_ekf_instances.in_set(SceneBuildSet::ProcessBaseAutonomy),
        )
        .add_systems(
            FixedUpdate,
            ekf_update_engine.run_if(in_state(AppState::Running)),
        );
    }
}

// --- SPAWNING SYSTEM FOR EKF ---
// This system runs once at startup to initialize the EKF on each agent.
fn spawn_ekf_instances(
    mut commands: Commands,
    topic_bus: Res<TopicBus>,
    request_query: Query<(Entity, &SpawnAgentConfigRequest)>,
) {
    for (agent_entity, request) in &request_query {
        let agent_config = &request.0;
        let agent_name = &agent_config.name;

        commands.entity(agent_entity).with_children(|parent| {
            for estimator_config in &agent_config.autonomy_stack.estimators {
                if let EstimatorConfig::Ekf(ekf_config) = estimator_config {
                    info!(
                        "  -> Spawning EKF instance '{}' for agent '{}'",
                        &ekf_config.name, &agent_config.name
                    );

                    // --- THE DYNAMICS MODEL FACTORY LOGIC ---
                    // 1. Get the requested model name from the config string.
                    let model_name = &ekf_config.dynamics;
                    info!("    - EKF requested dynamics model: '{}'", model_name);

                    // 2. Use a simple `match` statement to create the correct model.
                    //    This acts as a simple, local "registry".
                    let dynamics_model: Box<dyn Dynamics> = match model_name.as_str() {
                        "VehicleDefault" | "AckermannKinematics" => {
                            // If the user wants the vehicle's default, we build it here.
                            if let Vehicle::Ackermann { wheelbase, .. } = &agent_config.vehicle {
                                Box::new(AckermannKinematics {
                                    wheelbase: *wheelbase as f64,
                                    agent_entity: agent_entity,
                                })
                            } else {
                                panic!("VehicleDefault requested for non-Ackermann vehicle");
                            }
                        }
                        // "ConstantVelocity" => Box::new(ConstantVelocityModel),
                        // "ConstantAcceleration" => Box::new(ConstantAccelerationModel),
                        // When you add a new model, you just add a new match arm here.
                        _ => panic!(
                            "Unknown dynamics model type requested for EKF: {}",
                            model_name
                        ),
                    };

                    // 3. The rest of the logic is now correct.
                    // let state_layout = dynamics_model.get_state_layout(agent_entity);
                    let state_layout = dynamics_model.get_state_layout();
                    let state_dim = state_layout.len();
                    let q_matrix =
                        DMatrix::identity(state_dim, state_dim) * ekf_config.process_noise;
                    let initial_state = FrameAwareState::new(state_layout, 1.0, 0.0);

                    // --- CREATE IMU SUBSCRIPTIONS ---
                    let mut imu_subscriptions = ImuSubscriptions::default();
                    // Query the topic bus for all IMU topics belonging to this agent.
                    let imu_topic_names = topic_bus.find_topics_by_tag(TopicTag::Imu, agent_name);
                    for topic_name in &imu_topic_names {
                        info!(
                            "    - EKF '{}' subscribing to topic: {}",
                            ekf_config.name, topic_name
                        );
                        imu_subscriptions
                            .readers
                            .push(TopicReader::<ImuData>::new(topic_name));
                    }

                    // 4. Spawn the child entity with the EKF component that NOW HOLDS THE MODEL.
                    parent.spawn((
                        Name::new(format!("Estimator: {}", ekf_config.name)),
                        // The EKF component now stores the specific dynamics model instance it will use.
                        EKF {
                            dynamics: dynamics_model,
                            process_noise_q: q_matrix,
                        },
                        FilterState {
                            state: initial_state,
                        },
                        imu_subscriptions,
                    ));
                }
            }
        });
    }
}

// fn ekf_update_engine(
//     // --- Resources ---
//     time: Res<Time>,
//     mut measurement_events: EventReader<MeasurementEvent>,

//     // --- Queries ---
//     // QUERY 1: A query that can access any MeasurementModel in the world.
//     model_query: Query<&MeasurementModel>,

//     // QUERY 2: The ONLY query we need for the EKF itself.
//     // It finds all entities that have EKF components. These are the CHILD entities.
//     mut ekf_query: Query<(Entity, &mut FilterState, &EKF, &mut ImuSubscriptions)>,
//     // We no longer need a separate parent_query.
// ) {
//     // --- 1. PREDICTION STEP ---
//     let dt = time.delta_secs_f64();
//     if dt > 0.0 {
//         // We can now iterate directly over every EKF instance in the simulation.
//         // No need for parent/child lookups here.
//         for (_ekf_entity, mut filter_state, ekf_params, _imu_subs) in &mut ekf_query {
//             // Get the dynamics model directly from the EKF's own component.
//             let dynamics = &ekf_params.dynamics;
//             let u = DVector::zeros(dynamics.get_control_dim());

//             // Predict state vector
//             let new_x = dynamics.propagate(&filter_state.state.vector, &u, 0.0, dt, &RK4);

//             // Predict covariance
//             let (f_jac, _) = dynamics.calculate_jacobian(&filter_state.state.vector, &u, 0.0);
//             let new_p = &f_jac * &filter_state.state.covariance * f_jac.transpose()
//                 + &ekf_params.process_noise_q;

//             filter_state.state.vector = new_x;
//             filter_state.state.covariance = new_p;
//             filter_state.state.last_update_timestamp = time.elapsed_secs_f64();
//         }
//     }

//     // --- 2. UPDATE STEP ---
//     let events: Vec<MeasurementEvent> = measurement_events.read().cloned().collect();
//     for event in events {
//         // The event tells us which AGENT entity to work on.
//         // This is where we need to find the right EKF child.
//         // This is the one place a hierarchy query is still needed, but we can do it differently.

//         // This is still complex. Let's simplify further. The event should be for a specific EKF instance.
//         // This requires a change in how events are sent.
//         // Let's assume for now the simple case: apply the measurement to ALL EKFs on an agent.

//         // Let's re-introduce the parent query in a limited, correct way for the update step.
//     }
// }

fn ekf_update_engine(
    // --- Resources ---
    time: Res<Time>,
    // We listen for the generic MeasurementEvent sent by any sensor.
    mut measurement_events: EventReader<MeasurementEvent>,

    // --- Queries ---
    // QUERY 1: A query that can access any MeasurementModel component in the world.
    model_query: Query<&MeasurementModel>,

    // It finds all agents that have an EKF and children.
    // NOTE: We now get the EKF components directly from the parent agent.
    // This assumes ONE EKF per agent. If you want multiple, they MUST be children.
    // Let's stick to the child pattern, it's more robust.

    // --- Let's rewrite the queries for the child pattern ---
    // QUERY 1: Find all parent agents that have children and the components
    // the EKF needs from the parent (e.g., the DynamicsModel).
    parent_query: Query<(&DynamicsModel, &Children)>,

    // QUERY 2: A query that can access any EKF component on a child entity.
    mut ekf_query: Query<(&mut FilterState, &EKF)>, // And subscriptions
) {
    // --- 1. PREDICTION STEP ---
    let dt = time.delta_secs_f64();
    if dt > 0.0 {
        // We must iterate through parents and then children to do prediction.
        for (dynamics_model, children) in &parent_query {
            for &child_entity in children {
                // Check if this child is an EKF
                if let Ok((mut filter_state, ekf_params)) = ekf_query.get_mut(child_entity) {
                    // We found an EKF child. Predict it using its parent's dynamics model.
                    let u = DVector::zeros(ekf_params.dynamics.get_control_dim());

                    let new_x = ekf_params.dynamics.propagate(
                        &filter_state.state.vector,
                        &u,
                        0.0,
                        dt,
                        &RK4,
                    );
                    let (f_jac, _) =
                        ekf_params
                            .dynamics
                            .calculate_jacobian(&filter_state.state.vector, &u, 0.0);
                    let new_p = &f_jac * &filter_state.state.covariance * f_jac.transpose()
                        + &ekf_params.process_noise_q;

                    filter_state.state.vector = new_x;
                    filter_state.state.covariance = new_p;
                }
            }
        }
    }

    // --- 2. UPDATE STEP ---
    // Now, process all new measurements that have arrived this frame.
    // This could be made more robust by sorting events by timestamp, but for now this is fine.
    let events: Vec<MeasurementEvent> = measurement_events.read().cloned().collect();

    for event in events {
        // Use `get_mut` to get the specific agent that this measurement is for.
        if let Ok((_dynamics_model, children)) = parent_query.get(event.agent_entity) {
            // And get the specific measurement model from the sensor that sent the event.
            if let Ok(measurement_model) = model_query.get(event.sensor_entity) {
                // We have everything we need to perform one update step.

                for &child_entity in children {
                    if let Ok((mut filter_state, _ekf_params)) = ekf_query.get_mut(child_entity) {
                        // We have found an EKF child to update.
                        // Get Jacobian H and noise R from the generic measurement model.
                        let h_jacobian =
                            measurement_model.0.calculate_jacobian(&filter_state.state);
                        let r_matrix = measurement_model.0.get_r();
                        let z = &event.z; // The measurement vector from the event

                        let p_priori = filter_state.state.covariance.clone();
                        let x_priori = filter_state.state.vector.clone();

                        // --- Standard EKF Update Equations ---
                        // Innovation covariance: S = H * P * H^T + R
                        let s = &h_jacobian * &p_priori * h_jacobian.transpose() + r_matrix;

                        // Kalman Gain: K = P * H^T * S^-1
                        if let Some(s_inv) = s.try_inverse() {
                            let k_gain = &p_priori * h_jacobian.transpose() * s_inv;

                            // Innovation: y = z - h(x)
                            let z_pred =
                                measurement_model.0.predict_measurement(&filter_state.state);
                            let y = z - z_pred;

                            // Update state: x_posteriori = x_priori + K * y
                            filter_state.state.vector = x_priori + &k_gain * y;

                            // Update covariance: P_posteriori = (I - K * H) * P_priori
                            let i_kh = DMatrix::<f64>::identity(
                                filter_state.state.dim(),
                                filter_state.state.dim(),
                            ) - &k_gain * &h_jacobian;
                            filter_state.state.covariance = i_kh * &p_priori;
                        }
                        println!("filter_state: {:?}", filter_state);
                    }
                }
            }
        }
    }
}
