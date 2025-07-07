// src/simulation/plugins/estimation/ekf.rs

use crate::prelude::{AppState, StampedMessage};
use crate::simulation::core::dynamics::{Dynamics, DynamicsModel};
use crate::simulation::core::topics::{ImuData, TopicBus, TopicReader};
use bevy::prelude::*;
use nalgebra::{DMatrix, DVector, Matrix3, Vector3};
use std::collections::HashMap;

// --- Helper Functions (The Reusable Logic) ---

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

    // F = Jacobian of dynamics w.r.t. state
    // In a real implementation, you'd linearize around the current state.
    // Here we use a simplified placeholder matrix.
    let f_jacobian = DMatrix::<f64>::identity(
        ekf_state.state_vector.nrows(),
        ekf_state.state_vector.nrows(),
    ); // Placeholder

    // Predict state forward: x_pred = x + f(x, u) * dt
    let x_dot = dynamics.get_derivatives(&ekf_state.state_vector, &u, 0.0); // Assuming time t=0 for simplicity
    ekf_state.state_vector += x_dot * dt;

    // Predict covariance forward: P_pred = F * P * F^T + Q
    ekf_state.covariance =
        &f_jacobian * &ekf_state.covariance * f_jacobian.transpose() + process_noise_q * dt;
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

// --- Plugin and Bevy Constructs ---
pub struct EkfPlugin;

impl Plugin for EkfPlugin {
    fn build(&self, app: &mut App) {
        // Just one system, running at high frequency in FixedUpdate.
        app.add_systems(
            FixedUpdate,
            ekf_main_system.run_if(in_state(AppState::Running)),
        );
    }
}

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
        let state_dim = 9; // e.g., pos, vel, orientation_rads
        Self {
            state_vector: DVector::zeros(state_dim),
            covariance: DMatrix::identity(state_dim, state_dim) * 1.0,
            last_update_timestamp: -1.0, // Start at -1 to ensure first run
        }
    }
}

// --- The Main Orchestrator System ---

/// This single system performs the entire EKF logic in the correct sequence.
pub fn ekf_main_system(
    topic_bus: Res<TopicBus>,
    time: Res<Time>,
    mut query: Query<(
        Entity,
        &mut EkfState,
        &mut ImuSubscriptions,
        &EKF,
        &DynamicsModel,
    )>,
) {
    // DEBUG 1: Does the system run at all?
    // This should print every FixedUpdate frame.
    // If it doesn't, the system is not being added to the schedule correctly in main.rs.
    // debug!("[DEBUG] ekf_main_system running...");
    for (entity, mut ekf_state, mut imu_subscriptions, ekf_config, dynamics_model) in
        query.iter_mut()
    {
        // DEBUG 2: Is it finding our EKF entity?
        // debug!("[DEBUG] Processing EKF for entity {:?}", entity);

        // Initialize timestamp on the very first run.
        if ekf_state.last_update_timestamp < 0.0 {
            ekf_state.last_update_timestamp = time.elapsed_secs_f64();
        }

        // --- 1. Gather all new measurements into a unified, sortable list ---
        #[derive(Clone)]
        enum Measurement {
            Imu(StampedMessage<ImuData>, String),
        }
        let mut all_new_measurements: Vec<(f64, Measurement)> = Vec::new();

        for reader in &mut imu_subscriptions.readers {
            let topic_name = reader.topic_name.clone();
            if let Some(topic) = topic_bus.get_topic::<ImuData>(&topic_name) {
                // The most likely point of failure is here.
                // The `read` method advances the cursor. If we read into a temporary
                // variable and then don't use it, the messages are "consumed" but never processed.
                let messages_for_this_reader: Vec<_> = reader.read(topic).cloned().collect();

                // DEBUG 3: Are we finding any new messages for each reader?
                if !messages_for_this_reader.is_empty() {
                    // debug!(
                    //     "[DEBUG] Found {} new messages on topic '{}'",
                    //     messages_for_this_reader.len(),
                    //     topic_name
                    // );
                }

                for stamped_msg in messages_for_this_reader {
                    // `stamped_msg` is now owned
                    all_new_measurements.push((
                        stamped_msg.message.timestamp,
                        Measurement::Imu(stamped_msg, topic_name.clone()),
                    ));
                }
            } else {
                // DEBUG 4: Is the topic name correct?
                warn!("WARN: Could not find topic '{}' on the bus.", topic_name);
            }
        }
        // ... gather GPS, etc. here ...

        // --- 2. If no new data, predict to current time and exit ---
        if all_new_measurements.is_empty() {
            // DEBUG 5: This is the most likely reason the system is silent.
            // It runs, finds no new messages, and exits here.
            // debug!(
            //     "[DEBUG] No new measurements found for EKF on entity {:?}. Predicting and exiting.",
            //     entity
            // );

            let dt = time.elapsed_secs_f64() - ekf_state.last_update_timestamp;
            if dt > 0.001 {
                do_ekf_prediction(
                    &mut ekf_state,
                    &*dynamics_model.0,
                    &ekf_config.process_noise_covariance,
                    dt,
                );
                ekf_state.last_update_timestamp = time.elapsed_secs_f64();
            }
            continue; // Go to the next EKF entity
        }

        // --- 3. Sort all measurements by timestamp ---
        // debug!(
        //     "[DEBUG] Found a total of {} new measurements. Sorting...",
        //     all_new_measurements.len()
        // );
        all_new_measurements.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        // --- 4. Process all new measurements in the correct sequential loop ---
        // debug!("[DEBUG] Processing sorted measurements...");
        for (timestamp, measurement) in all_new_measurements {
            // a. PREDICT from our last known time to this measurement's time
            let dt = timestamp - ekf_state.last_update_timestamp;
            if dt > 0.0001 {
                // Avoid tiny, meaningless predictions
                do_ekf_prediction(
                    &mut ekf_state,
                    &*dynamics_model.0,
                    &ekf_config.process_noise_covariance,
                    dt,
                );
            }

            // b. UPDATE with the measurement data
            match measurement {
                Measurement::Imu(imu_msg, topic_name) => {
                    // Get the correct noise matrix for this specific sensor topic
                    if let Some(noise_r) = ekf_config.measurement_noise_map.get(&topic_name) {
                        do_ekf_imu_update(&mut ekf_state, &imu_msg.message, noise_r);
                    } else {
                        warn!(
                            "WARN: EKF has no measurement noise config for topic '{}'",
                            topic_name
                        );
                    }
                }
            }
            // c. Update our "internal clock"
            ekf_state.last_update_timestamp = timestamp;
        }
        // TODO: This is where you would publish the final state to the FullStateData topic.
    }
}
