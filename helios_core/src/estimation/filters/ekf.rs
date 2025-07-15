// helios_core/src/estimation/filters/ekf.rs

use std::any::Any;
use std::collections::HashMap;

use crate::estimation::{FilterContext, StateEstimator};
use crate::frames::FrameAwareState;
use crate::messages::{MeasurementData, ModuleInput};
use crate::models::estimation::measurement::Measurement;
use crate::prelude::{EstimationDynamics, MeasurementMessage};
// The helper trait for sensors
use crate::types::{Control, FrameHandle};
use crate::utils::integrators::RK4;
use nalgebra::{DMatrix, DVector}; // Assuming you have this

/// A concrete implementation of an Extended Kalman Filter.
pub struct ExtendedKalmanFilter {
    /// The current state of the filter (x, P, t).
    state: FrameAwareState,
    /// The process noise covariance matrix (Q), modeling uncertainty in the dynamics.
    process_noise_q: DMatrix<f64>,
    /// The specific dynamics model this filter will use for prediction.
    dynamics_model: Box<dyn EstimationDynamics>,
    /// Maps a sensor's handle to the model that describes its physics.
    measurement_models: HashMap<FrameHandle, Box<dyn Measurement>>,
}

impl ExtendedKalmanFilter {
    /// Creates a new EKF instance.
    pub fn new(
        initial_state: FrameAwareState,
        process_noise_q: DMatrix<f64>,
        dynamics_model: Box<dyn EstimationDynamics>,
        measurement_models: HashMap<FrameHandle, Box<dyn Measurement>>,
    ) -> Self {
        // Ensure Q has the correct dimensions.
        assert_eq!(initial_state.dim(), process_noise_q.nrows());
        assert_eq!(initial_state.dim(), process_noise_q.ncols());

        Self {
            state: initial_state,
            process_noise_q,
            dynamics_model,
            measurement_models,
        }
    }
}

// --- The Public Trait Implementation ---
impl StateEstimator for ExtendedKalmanFilter {
    /// Predicts the state forward by `dt` using the provided control input `u`.
    fn predict(&mut self, dt: f64, u: &Control, _context: &FilterContext) {
        if dt <= 0.0 {
            return;
        }

        // --- 1. Get current state and dynamics model ---
        let dynamics = &self.dynamics_model;
        let x = &self.state.vector;
        let p = &self.state.covariance;
        let t = self.state.last_update_timestamp;

        // Ensure control input `u` has the correct dimension for the dynamics model.
        let u_sized = if u.nrows() == dynamics.get_control_dim() {
            u
        } else {
            // This case can happen if the control input cache wasn't updated.
            // We fall back to a zero vector to prevent a panic.
            // warn!("Control input dimension mismatch in EKF predict. Using zeros.");
            // A more robust solution might use a thread_local static as before.
            &DVector::zeros(dynamics.get_control_dim())
        };

        // --- 2. Predict the next state vector using numerical integration ---
        // x_pred = f(x, u, t)
        let x_pred = dynamics.propagate(x, u_sized, t, dt, &RK4);

        // --- 3. Linearize the dynamics by calculating the state transition matrix (Jacobian F) ---
        // For an EKF with a discrete time step, we often use an approximation:
        // F_k ≈ I + A * dt
        // But a more accurate way is to use the full matrix exponential if possible,
        // or just the calculated Jacobian `A` from the continuous model. We will use `A`.
        let (a_jac, _b_jac) = dynamics.calculate_jacobian(x, u_sized, t);

        // Discretize the state transition matrix: F ≈ I + A*dt
        let f_k = DMatrix::<f64>::identity(self.state.dim(), self.state.dim()) + a_jac * dt;

        // --- 4. Predict the next covariance matrix ---
        // P_k+1|k = F_k * P_k|k * F_k^T + Q_d
        // We can approximate the discrete process noise Q_d as Q * dt
        let p_pred = &f_k * p * f_k.transpose() + &self.process_noise_q * dt;

        // --- 5. Update the filter's internal state ---
        self.state.vector = x_pred;
        self.state.covariance = p_pred;
        self.state.last_update_timestamp += dt;
    }

    /// Updates the state by fusing an aiding measurement.
    fn update(&mut self, message: &MeasurementMessage, context: &FilterContext) {
        // --- 1. Find the correct measurement model for this sensor ---
        let model = match self.measurement_models.get(&message.sensor_handle) {
            Some(m) => m,
            None => return, // This EKF is not configured to use this sensor.
        };

        // --- 2. Ask the model to predict a measurement based on this message ---
        // `predict_measurement` now returns an Option. If it's `None`, this model
        // cannot handle this message type, and we do nothing.
        let z_pred_option = model.predict_measurement(&self.state, message, context.tf.unwrap());

        if let Some(z_pred) = z_pred_option {
            // --- We have a valid prediction! Now we can proceed. ---

            // First, get the actual measurement vector `z` from the message data.
            // This is safe to unwrap because if predict_measurement succeeded, the data
            // type must be one it understands. A helper function would be cleaner here.
            if let Some(z_slice) = message.data.as_primary_slice() {
                let z = DVector::from_row_slice(z_slice);

                // Dimension safety check
                if z.nrows() != z_pred.nrows() {
                    // In core, we can't log. A good practice is to just return and do nothing.
                    // The filter remains stable, just without the update.
                    return;
                }

                // --- 3. Perform the standard EKF update equations ---
                let h_jac = model.calculate_jacobian(&self.state, context.tf.unwrap());
                let r_mat = model.get_r();

                let y = z - z_pred; // Innovation
                let s = &h_jac * &self.state.covariance * h_jac.transpose() + r_mat; // Innovation covariance

                if let Some(s_inv) = s.try_inverse() {
                    let k_gain = &self.state.covariance * h_jac.transpose() * s_inv; // Kalman Gain

                    self.state.vector += &k_gain * y;
                    let i = DMatrix::<f64>::identity(self.state.dim(), self.state.dim());
                    self.state.covariance = (i - k_gain * &h_jac) * &self.state.covariance;

                    // The timestamp is updated to the measurement's time.
                    self.state.last_update_timestamp = message.timestamp;
                }
            }
        }
        // If predict_measurement returned None, we simply do nothing.
    }

    fn get_state(&self) -> &FrameAwareState {
        &self.state
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn get_dynamics_model(&self) -> &dyn EstimationDynamics {
        &*self.dynamics_model
    }
}
