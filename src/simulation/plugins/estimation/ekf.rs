// src/simulation/plugins/estimation/ekf.rs

use crate::prelude::{AppState, AutonomyStack, EstimatorConfig, SensorConfig, StampedMessage};
use crate::simulation::core::app_state::SceneBuildSet;
use crate::simulation::core::dynamics::{Dynamics, DynamicsModel};
use crate::simulation::core::simulation_setup::{SpawnAgentConfigRequest, SpawnRequestAutonomy};
use crate::simulation::core::topics::{ImuData, TopicBus, TopicReader, TopicTag}; // NEW
use crate::simulation::utils::integrators::RK4;
use bevy::prelude::*;
use nalgebra::{DMatrix, DVector, Matrix3, Vector3};
use std::collections::HashMap;

// --- Components ---

#[derive(Component)]
pub struct EKF {
    /// The Process Noise Covariance matrix (Q).
    pub process_noise_covariance: DMatrix<f64>,
    /// Map from a topic name to its Measurement Noise Covariance matrix (R).
    pub measurement_noise_map: HashMap<String, DMatrix<f64>>,
}

#[derive(Component, Default)]
pub struct ImuSubscriptions {
    pub readers: Vec<TopicReader<ImuData>>,
}

#[derive(Component, Debug)]
pub struct EkfState {
    pub state_vector: DVector<f64>,
    pub covariance: DMatrix<f64>,
    /// Timestamp of the last filter update.
    pub last_update_timestamp: f64,
}

impl Default for EkfState {
    fn default() -> Self {
        let state_dim = 3; // e.g., pos, vel, orientation_rads
        Self {
            state_vector: DVector::zeros(state_dim),
            covariance: DMatrix::identity(state_dim, state_dim) * 1.0,
            last_update_timestamp: -1.0, // Start at -1 to ensure first run
        }
    }
}

// --- Plugin and Bevy Constructs ---
pub struct EkfPlugin;

impl Plugin for EkfPlugin {
    fn build(&self, app: &mut App) {
        // This plugin adds two systems to the app.
        app
            // 1. A system to spawn EKF instances during the scene building phase.
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                spawn_ekf_instances.in_set(SceneBuildSet::ProcessRequests),
            )
            // 2. The main runtime system that performs the EKF calculations.
            .add_systems(
                FixedUpdate,
                ekf_main_system.run_if(in_state(AppState::Running)),
            );
    }
}

// --- SPAWNING SYSTEM (Identical in pattern to the IMU spawner) ---
fn spawn_ekf_instances(
    mut commands: Commands,
    topic_bus: Res<TopicBus>,
    // The query now looks for our single, powerful request component.
    request_query: Query<(Entity, &SpawnAgentConfigRequest)>,
) {
    for (agent_entity, request) in &request_query {
        let agent_config = &request.0;
        let agent_name = &agent_config.name;

        // Use with_children to ensure all EKF instances are parented to the agent.
        commands
            .entity(agent_entity)
            .with_children(|parent_builder| {
                // Iterate through the list of estimators from the config.
                for estimator_config in &agent_config.autonomy_stack.estimators {
                    // This system only cares about the `Ekf` variant.
                    // A future `UkfPlugin` would have a system that looks for `EstimatorConfig::Ukf`.
                    if let EstimatorConfig::Ekf(ekf_config) = estimator_config {
                        info!(
                            "  -> Spawning EKF instance '{}' as child of agent '{}'",
                            &ekf_config.name, agent_name
                        );

                        // --- 1. Find Sensor Topics and Create Subscriptions ---
                        let mut imu_subscriptions = ImuSubscriptions::default();
                        let imu_topic_names =
                            topic_bus.find_topics_by_tag(TopicTag::Imu, agent_name);

                        for topic_name in &imu_topic_names {
                            imu_subscriptions
                                .readers
                                .push(TopicReader::<ImuData>::new(topic_name));
                        }
                        // You would repeat this for GPS, etc.

                        // --- 2. Build the Measurement Noise Map (R matrices) ---
                        let mut measurement_noise_map = HashMap::new();
                        for imu_topic_name in &imu_topic_names {
                            if let Some(sensor_name) = imu_topic_name.split('/').last() {
                                // Search the full agent config to find the matching sensor's noise values.
                                for sensor_config in &agent_config.sensors {
                                    if let SensorConfig::Imu(imu_conf) = sensor_config {
                                        if imu_conf.get_name() == sensor_name {
                                            let (accel_std, gyro_std) =
                                                imu_conf.get_noise_stddevs();
                                            let all_stddevs = [
                                                accel_std[0],
                                                accel_std[1],
                                                accel_std[2],
                                                gyro_std[0],
                                                gyro_std[1],
                                                gyro_std[2],
                                            ];
                                            let r_matrix =
                                                DMatrix::from_diagonal(&DVector::from_iterator(
                                                    6,
                                                    all_stddevs
                                                        .iter()
                                                        .map(|&stddev| (stddev as f64).powi(2)),
                                                ));
                                            measurement_noise_map
                                                .insert(imu_topic_name.clone(), r_matrix);
                                            break;
                                        }
                                    }
                                }
                            }
                        }

                        // --- 3. Build the EKF Parameters ---
                        let ekf_params = EKF {
                            process_noise_covariance: DMatrix::identity(9, 9)
                                * ekf_config.process_noise,
                            measurement_noise_map,
                        };

                        // --- 4. Spawn the Child Entity ---
                        // This creates a new entity with all the necessary EKF components.
                        parent_builder.spawn((
                            Name::new(format!("Estimator: {}", ekf_config.name)),
                            ekf_params,
                            EkfState::default(),
                            imu_subscriptions,
                        ));
                    }
                }
            });
    }
}

// --- The Main Orchestrator System ---

/// This single system performs the entire EKF logic in the correct sequence.

// --- RUNTIME SYSTEM ---
// This system runs every physics step to update the EKF state.
fn ekf_main_system(
    // --- Resources ---
    topic_bus: Res<TopicBus>,
    time: Res<Time>, // Use the generic Time, not Time<Fixed> unless you are sure

    // --- Queries (Modern Bevy 0.16+ Parent-Centric Pattern) ---
    // QUERY 1: Find all parent agents that have children and a dynamics model.
    parent_query: Query<(&DynamicsModel, &Children)>,

    // QUERY 2: A query that can access any EKF component in the world.
    mut ekf_query: Query<(&mut EkfState, &mut ImuSubscriptions, &EKF)>,
) {
    // Iterate over all potential parent agents.
    for (dynamics_model, children) in &parent_query {
        // The `children` component is a list of Entity IDs.
        for &child_entity in children {
            // Try to get the EKF components from this child entity.
            if let Ok((mut ekf_state, mut imu_subs, ekf_params)) = ekf_query.get_mut(child_entity) {
                // SUCCESS! We have found an EKF child for the current parent.

                // The full EKF logic (prediction and update steps) goes here.
                // You have everything you need:
                // - dynamics_model (from the parent agent)
                // - ekf_state (the filter's state to update)
                // - imu_subs (to read new sensor data from the TopicBus)
                // - ekf_params (containing Q and R noise matrices)
                // - time (to calculate dt)
                // - topic_bus (to get the actual message data)

                // Example of a simplified update loop:
                // 1. Gather and sort new measurements from all subscriptions.
                // 2. Loop through sorted measurements:
                //    a. Predict state forward to the measurement's timestamp (using dt and dynamics_model).
                //    b. Update state with the measurement data (using the appropriate R matrix from ekf_params.measurement_noise_map).
                // 3. Predict state forward from the last measurement to the current time.
            }
        }
    }
}

// --- Helper Functions (The Reusable Logic) ---

// fn ekf_main_system(
//     topic_bus: Res<TopicBus>,
//     time: Res<Time>,
//     mut query: Query<(
//         Entity,
//         &mut EkfState,
//         &mut ImuSubscriptions, // Needs to be mutable to update the read cursors
//         &EKF,                  // The EKF's static parameters
//         &DynamicsModel,
//     )>,
// ) {
//     // DEBUG 1: Does the system run at all?
//     // This should print every FixedUpdate frame.
//     // If it doesn't, the system is not being added to the schedule correctly in main.rs.
//     // debug!("[DEBUG] ekf_main_system running...");
//     for (entity, mut ekf_state, mut imu_subscriptions, ekf_config, dynamics_model) in
//         query.iter_mut()
//     {
//         // DEBUG 2: Is it finding our EKF entity?
//         // debug!("[DEBUG] Processing EKF for entity {:?}", entity);

//         // Initialize timestamp on the very first run.
//         if ekf_state.last_update_timestamp < 0.0 {
//             ekf_state.last_update_timestamp = time.elapsed_secs_f64();
//         }

//         // --- 1. Gather all new measurements into a unified, sortable list ---
//         #[derive(Clone)]
//         enum Measurement {
//             Imu(StampedMessage<ImuData>, String),
//         }
//         let mut all_new_measurements: Vec<(f64, Measurement)> = Vec::new();

//         for reader in &mut imu_subscriptions.readers {
//             let topic_name = reader.topic_name.clone();
//             if let Some(topic) = topic_bus.get_topic::<ImuData>(&topic_name) {
//                 // The most likely point of failure is here.
//                 // The `read` method advances the cursor. If we read into a temporary
//                 // variable and then don't use it, the messages are "consumed" but never processed.
//                 let messages_for_this_reader: Vec<_> = reader.read(topic).cloned().collect();

//                 // DEBUG 3: Are we finding any new messages for each reader?
//                 if !messages_for_this_reader.is_empty() {
//                     // debug!(
//                     //     "[DEBUG] Found {} new messages on topic '{}'",
//                     //     messages_for_this_reader.len(),
//                     //     topic_name
//                     // );
//                 }

//                 for stamped_msg in messages_for_this_reader {
//                     // `stamped_msg` is now owned
//                     all_new_measurements.push((
//                         stamped_msg.message.timestamp,
//                         Measurement::Imu(stamped_msg, topic_name.clone()),
//                     ));
//                 }
//             } else {
//                 // DEBUG 4: Is the topic name correct?
//                 warn!("WARN: Could not find topic '{}' on the bus.", topic_name);
//             }
//         }
//         // ... gather GPS, etc. here ...

//         // --- 2. If no new data, predict to current time and exit ---
//         if all_new_measurements.is_empty() {
//             // DEBUG 5: This is the most likely reason the system is silent.
//             // It runs, finds no new messages, and exits here.
//             // debug!(
//             //     "[DEBUG] No new measurements found for EKF on entity {:?}. Predicting and exiting.",
//             //     entity
//             // );

//             let dt = time.elapsed_secs_f64() - ekf_state.last_update_timestamp;
//             if dt > 0.001 {
//                 do_ekf_prediction(
//                     &mut ekf_state,
//                     &*dynamics_model.0,
//                     &ekf_config.process_noise_covariance,
//                     dt,
//                 );
//                 ekf_state.last_update_timestamp = time.elapsed_secs_f64();
//             }
//             continue; // Go to the next EKF entity
//         }

//         // --- 3. Sort all measurements by timestamp ---
//         // debug!(
//         //     "[DEBUG] Found a total of {} new measurements. Sorting...",
//         //     all_new_measurements.len()
//         // );
//         all_new_measurements.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

//         // --- 4. Process all new measurements in the correct sequential loop ---
//         // debug!("[DEBUG] Processing sorted measurements...");
//         for (timestamp, measurement) in all_new_measurements {
//             // a. PREDICT from our last known time to this measurement's time
//             let dt = timestamp - ekf_state.last_update_timestamp;
//             if dt > 0.0001 {
//                 // Avoid tiny, meaningless predictions
//                 do_ekf_prediction(
//                     &mut ekf_state,
//                     &*dynamics_model.0,
//                     &ekf_config.process_noise_covariance,
//                     dt,
//                 );
//             }

//             // b. UPDATE with the measurement data
//             match measurement {
//                 Measurement::Imu(imu_msg, topic_name) => {
//                     // Get the correct noise matrix for this specific sensor topic
//                     if let Some(noise_r) = ekf_config.measurement_noise_map.get(&topic_name) {
//                         do_ekf_imu_update(&mut ekf_state, &imu_msg.message, noise_r);
//                     } else {
//                         warn!(
//                             "WARN: EKF has no measurement noise config for topic '{}'",
//                             topic_name
//                         );
//                     }
//                 }
//             }
//             // c. Update our "internal clock"
//             ekf_state.last_update_timestamp = timestamp;
//         }
//         // TODO: This is where you would publish the final state to the FullStateData topic.
//     }
// }

/// Performs the EKF prediction step.
/// This is a plain Rust function, not a Bevy system.
fn do_ekf_prediction(
    ekf_state: &mut EkfState,
    dynamics: &dyn Dynamics,
    process_noise_q: &DMatrix<f64>,
    dt: f64,
) {
    // In a real EKF, you would get the control input `u` that was applied during `dt`.
    // For now, we'll assume a zero control input.
    let u = DVector::zeros(dynamics.get_control_dim());
    let current_time = 0.0; // simplicity

    // --- 1. State Prediction (using the full nonlinear model with a good integrator) ---
    // This is more accurate than Euler integration.
    let integrator = RK4::default();
    let predicted_state =
        dynamics.propagate(&ekf_state.state_vector, &u, current_time, dt, &integrator);

    // --- 2. Covariance Prediction (using the linearized model via the Jacobian) ---
    // Get the Jacobian F = ∂f/∂x, evaluated at the *current* state (before prediction).
    let (f_jacobian, _b_jacobian) =
        dynamics.calculate_jacobian(&ekf_state.state_vector, &u, current_time);

    // Now use the standard linear covariance update formula with the *real* Jacobian.
    let predicted_covariance =
        &f_jacobian * &ekf_state.covariance * f_jacobian.transpose() + process_noise_q;
    // Note: Q is often scaled by dt or dt^2 depending on the noise model

    // --- 3. Update the EKF's state ---
    ekf_state.state_vector = predicted_state;
    ekf_state.covariance = predicted_covariance;
}

/// Performs the EKF update step using an IMU measurement.
/// This is also a plain Rust function.
fn do_ekf_imu_update(
    ekf_state: &mut EkfState,
    imu_data: &ImuData,
    measurement_noise_r: &DMatrix<f64>,
) {
    // Simplified example for updating with angular velocity.
    // A real update would use the full measurement model `z = h(x)`.
    let state_dim = ekf_state.state_vector.nrows();

    // H = Jacobian of measurement model w.r.t. state.
    // This matrix maps the state space to the measurement space.
    // For this example, let's assume we are measuring 3D angular velocity
    // and it corresponds to states 6, 7, and 8 in our state vector.
    let mut h_jacobian = DMatrix::<f64>::zeros(3, state_dim);
    if state_dim >= 9 {
        h_jacobian
            .fixed_view_mut::<3, 3>(0, 6)
            .copy_from(&Matrix3::identity());
    }

    // z = The actual measurement from the sensor.
    let z = imu_data.angular_velocity;

    // z_pred = The predicted measurement from our current state estimate.
    let z_pred = Vector3::new(
        ekf_state.state_vector.get(6).cloned().unwrap_or(0.0),
        ekf_state.state_vector.get(7).cloned().unwrap_or(0.0),
        ekf_state.state_vector.get(8).cloned().unwrap_or(0.0),
    );

    // y = Innovation (the difference between actual and predicted measurement).
    let y = z - z_pred;

    // S = Innovation Covariance: S = H * P * H^T + R
    let s = &h_jacobian * &ekf_state.covariance * h_jacobian.transpose() + measurement_noise_r;

    // K = Kalman Gain: K = P * H^T * S^-1
    if let Some(s_inv) = s.try_inverse() {
        let k = &ekf_state.covariance * h_jacobian.transpose() * s_inv;

        // Update state: x_new = x_pred + K * y
        ekf_state.state_vector += &k * y;

        // Update covariance: P_new = (I - K * H) * P_pred
        let i_kh = DMatrix::<f64>::identity(state_dim, state_dim) - &k * &h_jacobian;
        ekf_state.covariance = i_kh * &ekf_state.covariance;
    }
}
