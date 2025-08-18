// helios_core/src/estimation/filters/ekf.rs

use std::any::Any;
use std::collections::HashMap;

use crate::estimation::{FilterContext, StateEstimator};
use crate::frames::FrameAwareState;
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

        // --- 6. (Optional but Recommended) Symmetrize Covariance ---
        // Tiny numerical errors can make P slightly non-symmetric. This forces it.
        self.state.covariance = (&self.state.covariance + self.state.covariance.transpose()) * 0.5;
    }

    /// Updates the state by fusing an aiding measurement.
    fn update(&mut self, message: &MeasurementMessage, context: &FilterContext) {
        // Iterate through all configured measurement models.
        for model in self.measurement_models.values() {
            // We call the new `get_measurement_vector` method. If it returns `Some`,
            // it means this is the right model and we have our `z` vector.
            if let Some(z) = model.get_measurement_vector(&message.data) {
                // We can now safely unwrap `predict_measurement` because if the model
                // gave us a `z`, it must also be able to give us a `z_pred`.
                let z_pred = model
                    .predict_measurement(&self.state, message, context.tf.unwrap())
                    .expect("Model that provided a `z` vector failed to provide a prediction.");

                if z.nrows() != z_pred.nrows() {
                    return;
                }

                let p_priori = &self.state.covariance;
                let h_jac = model.calculate_jacobian(&self.state, context.tf.unwrap());
                let r_mat = model.get_r();
                let y = z.clone() - &z_pred;

                // --- Calculate the full Kalman update values once ---
                let s = &h_jac * &self.state.covariance * h_jac.transpose() + r_mat;
                let s_inv_option = s.try_inverse();

                // Can't proceed if S is not invertible.
                if s_inv_option.is_none() {
                    return;
                }

                let s_inv = s_inv_option.unwrap();
                let k_gain = &self.state.covariance * h_jac.transpose() * s_inv;
                let correction = &k_gain * &y;

                self.state.vector += correction;
                let i = DMatrix::<f64>::identity(self.state.dim(), self.state.dim());
                // Joseph form is used for numerically stable solution
                let i_kh = &i - &k_gain * &h_jac;
                // P_post = (I - KH)P(I - KH)^T + KRK^T
                let p_post =
                    &i_kh * p_priori * i_kh.transpose() + &k_gain * r_mat * k_gain.transpose();
                self.state.covariance = p_post;
                // --- Enforce Symmetry ---
                // P = 0.5 * (P + P^T)
                self.state.covariance =
                    (&self.state.covariance + self.state.covariance.transpose()) * 0.5;

                // for positivity
                for i in 0..self.state.dim() {
                    if self.state.covariance[(i, i)] < 0.0 {
                        // A variance has become negative, which is physically impossible.
                        // This is a sign of severe filter divergence.
                        // The safest thing to do is reset it to a small positive number.
                        self.state.covariance[(i, i)] = 1e-9;
                    }
                }
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
