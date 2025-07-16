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

                let y = z.clone() - &z_pred; // Innovation
                                             // Only print for a specific sensor type to avoid log spam.
                if matches!(
                    &message.data,
                    crate::messages::MeasurementData::GpsPosition(_)
                ) {
                    println!(
                        "------------------- EKF GPS UPDATE (t={:.3}) -------------------",
                        message.timestamp
                    );

                    // 1. WHAT THE FILTER THOUGHT (Prediction)
                    // Transpose for easier reading in the console.
                    println!("Predicted Measurement (z_pred): {}", z_pred.transpose());

                    // 2. WHAT THE SENSOR SAW (Measurement)
                    println!("Actual Measurement (z):         {}", z.transpose());

                    // 3. THE SURPRISE (Innovation) - This is the most important value.
                    // Is the filter consistently over/under-estimating?
                    println!("Innovation (y = z - z_pred):    {}", y.transpose());

                    // 4. THE UNCERTAINTY
                    // Get the diagonal of the covariance matrix, which represents the variance
                    // of each state. The sqrt gives the standard deviation (1-sigma).
                    let p_diag_sqrt = self.state.covariance.diagonal().map(|v| v.sqrt());

                    // Let's look at the uncertainty of the position states.
                    println!(
                        "State Sigma (Position):         [x: {:.3}, y: {:.3}, z: {:.3}]",
                        p_diag_sqrt[0], p_diag_sqrt[1], p_diag_sqrt[2]
                    );

                    // And the uncertainty of the bias estimates.
                    println!(
                        "State Sigma (Accel Bias):       [x: {:.4}, y: {:.4}, z: {:.4}]",
                        p_diag_sqrt[10], p_diag_sqrt[11], p_diag_sqrt[12]
                    );
                    println!(
                        "State Sigma (Gyro Bias):        [x: {:.4}, y: {:.4}, z: {:.4}]",
                        p_diag_sqrt[13], p_diag_sqrt[14], p_diag_sqrt[15]
                    );
                }

                let s = &h_jac * &self.state.covariance * h_jac.transpose() + r_mat; // Innovation covariance

                if let Some(s_inv) = s.try_inverse() {
                    let k_gain = &self.state.covariance * h_jac.transpose() * s_inv; // Kalman Gain

                    let correction = &k_gain * &y;

                    self.state.vector += &k_gain * y;
                    let i = DMatrix::<f64>::identity(self.state.dim(), self.state.dim());
                    self.state.covariance = (i - k_gain * &h_jac) * &self.state.covariance;

                    // The timestamp is updated to the measurement's time.
                    self.state.last_update_timestamp = message.timestamp;

                    // What correction is being applied to the biases?
                    println!(
                        "Bias Correction (Accel):        {}",
                        correction.fixed_rows::<3>(10).transpose()
                    );
                }
                println!("-----------------------------------------------------------------");
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
