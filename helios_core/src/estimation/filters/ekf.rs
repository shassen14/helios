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
        let x_old = &self.state.vector;
        let p_old = &self.state.covariance;
        let t_old = self.state.last_update_timestamp;

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
        let x_new = dynamics.propagate(x_old, u_sized, t_old, dt, &RK4);

        // --- 3. Linearize the dynamics by calculating the state transition matrix (Jacobian F) ---
        // For an EKF with a discrete time step, we often use an approximation:
        // F_k ≈ I + A * dt
        // But a more accurate way is to use the full matrix exponential if possible,
        // or just the calculated Jacobian `A` from the continuous model. We will use `A`.
        // --- 2. Calculate State Transition and Noise Input Matrices ---
        // A = ∂f/∂x, B = ∂f/∂u
        let (a_jac, b_jac) = dynamics.calculate_jacobian(x_old, u, t_old);

        // Discretize the state transition matrix: F ≈ I + A*dt
        let f_k = DMatrix::<f64>::identity(self.state.dim(), self.state.dim()) + &a_jac * dt;

        // --- 4. Predict the next covariance matrix ---
        // The correct approach is to apply the process noise `Q` *before* the main propagation.
        // This represents adding uncertainty to the model itself before you predict.
        let p_with_noise = p_old + &self.process_noise_q * dt;

        // Now, propagate this "noisier" covariance forward using the state transition matrix.
        // P_new = F * (P_old + Q*dt) * F^T
        let p_new = &f_k * p_with_noise * f_k.transpose();

        // --- 5. Update the filter's internal state ---
        self.state.vector = x_new;
        self.state.covariance = p_new;
        self.state.last_update_timestamp += dt;

        // --- 5. (Optional but Recommended) Symmetrize Covariance ---
        // Tiny numerical errors can make P slightly non-symmetric. This forces it.
        self.state.covariance = (&self.state.covariance + self.state.covariance.transpose()) * 0.5;
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
            if let Some(z_slice) = message.data.as_primary_slice() {
                let z = DVector::from_row_slice(z_slice);
                if z.nrows() != z_pred.nrows() {
                    return;
                }

                let h_jac = model.calculate_jacobian(&self.state, context.tf.unwrap());
                let r_mat = model.get_r();
                let y = z.clone() - &z_pred;

                // --- Calculate the full Kalman update values once ---
                let s = &h_jac * &self.state.covariance * h_jac.transpose() + r_mat;
                let s_inv_option = s.try_inverse();

                if s_inv_option.is_none() {
                    return; // Can't proceed if S is not invertible.
                }
                let s_inv = s_inv_option.unwrap();
                let k_gain = &self.state.covariance * h_jac.transpose() * s_inv;
                let correction = &k_gain * &y;

                // --- 3. Perform the standard EKF update equations ---
                let h_jac = model.calculate_jacobian(&self.state, context.tf.unwrap());
                let r_mat = model.get_r();

                let y = z.clone() - &z_pred; // Innovation
                                             // Only print for a specific sensor type to avoid log spam.
                match &message.data {
                    crate::messages::MeasurementData::GpsPosition(_) => {
                        println!(
                            "------------------- EKF GPS UPDATE (t={:.3}) -------------------",
                            message.timestamp
                        );
                        println!("Predicted Pos (z_pred): {}", z_pred.transpose());
                        println!("Actual Pos (z):         {}", z.transpose());
                        println!("Innovation (y):         {}", y.transpose());

                        let p_diag_sqrt = self.state.covariance.diagonal().map(|v| v.sqrt());
                        println!(
                            "State Sigma (Position):   [x: {:.3}, y: {:.3}, z: {:.3}]",
                            p_diag_sqrt[0], p_diag_sqrt[1], p_diag_sqrt[2]
                        );

                        // Log the corrections relevant to a GPS update
                        println!(
                            "Bias Correction (Accel):  {}",
                            correction.fixed_rows::<3>(10).transpose()
                        );
                        println!(
                            "-----------------------------------------------------------------"
                        );
                    }
                    crate::messages::MeasurementData::Magnetometer(_) => {
                        println!(
                            "------------------- EKF MAG UPDATE (t={:.3}) -------------------",
                            message.timestamp
                        );
                        println!("Predicted Mag (z_pred): {}", z_pred.transpose());
                        println!("Actual Mag (z):         {}", z.transpose());
                        println!("Innovation (y):         {}", y.transpose());

                        let p_diag_sqrt = self.state.covariance.diagonal().map(|v| v.sqrt());
                        println!(
                            "State Sigma (Quaternion): [x: {:.4}, y: {:.4}, z: {:.4}, w: {:.4}]",
                            p_diag_sqrt[6], p_diag_sqrt[7], p_diag_sqrt[8], p_diag_sqrt[9]
                        );

                        // Log the corrections relevant to a Magnetometer update
                        println!(
                            "Correction (Quaternion):  {}",
                            correction.fixed_rows::<4>(6).transpose()
                        );
                        println!(
                            "-----------------------------------------------------------------"
                        );
                    }
                    _ => {} // No logging for other types
                }

                self.state.vector += correction;
                let i = DMatrix::<f64>::identity(self.state.dim(), self.state.dim());
                self.state.covariance = (i - k_gain * &h_jac) * &self.state.covariance;
                self.state.last_update_timestamp = message.timestamp;
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
