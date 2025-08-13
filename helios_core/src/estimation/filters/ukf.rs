use nalgebra::{Cholesky, DMatrix, DVector};
use std::any::Any;
use std::collections::HashMap;

// --- Core Library Imports ---
use crate::estimation::{FilterContext, StateEstimator};
use crate::frames::FrameAwareState;
use crate::models::estimation::measurement::Measurement;
use crate::prelude::{EstimationDynamics, MeasurementMessage};
use crate::types::{Control, FrameHandle};
use crate::utils::integrators::RK4;
/// Configuration parameters for the UKF's sigma point generation.
#[derive(Debug, Clone, Copy)]
pub struct UkfParams {
    pub alpha: f64, // Spreading of sigma points (usually 1e-3)
    pub beta: f64,  // Incorporates prior knowledge of distribution (2.0 is optimal for Gaussian)
    pub kappa: f64, // Secondary scaling parameter (often 0.0 or 3-n)
}

/// A concrete implementation of an Unscented Kalman Filter.
pub struct UnscentedKalmanFilter {
    state: FrameAwareState,
    process_noise_q: DMatrix<f64>,
    dynamics_model: Box<dyn EstimationDynamics>,
    measurement_models: HashMap<FrameHandle, Box<dyn Measurement>>,
    params: UkfParams,

    // --- UKF-specific internal state ---
    /// Weights for calculating the mean from sigma points.
    weights_m: DVector<f64>,
    /// Weights for calculating the covariance from sigma points.
    weights_c: DVector<f64>,
}

impl UnscentedKalmanFilter {
    pub fn new(
        initial_state: FrameAwareState,
        process_noise_q: DMatrix<f64>,
        dynamics_model: Box<dyn EstimationDynamics>,
        measurement_models: HashMap<FrameHandle, Box<dyn Measurement>>,
        params: UkfParams,
    ) -> Self {
        let n = initial_state.dim();
        let lambda = params.alpha.powi(2) * (n as f64 + params.kappa) - n as f64;

        // Calculate the weights for the 2n+1 sigma points.
        let mut weights_m = DVector::from_element(2 * n + 1, 0.5 / (n as f64 + lambda));
        let mut weights_c = DVector::from_element(2 * n + 1, 0.5 / (n as f64 + lambda));
        weights_m[0] = lambda / (n as f64 + lambda);
        weights_c[0] = weights_m[0] + (1.0 - params.alpha.powi(2) + params.beta);

        Self {
            state: initial_state,
            process_noise_q,
            dynamics_model,
            measurement_models,
            params,
            weights_m,
            weights_c,
        }
    }

    /// Generates the `2n+1` sigma points based on the current state and covariance.
    fn generate_sigma_points(&self) -> DMatrix<f64> {
        let n = self.state.dim();
        let lambda = self.params.alpha.powi(2) * (n as f64 + self.params.kappa) - n as f64;
        let mut sigma_points = DMatrix::zeros(n, 2 * n + 1);

        // Cholesky decomposition: P = L * L^T
        // This gives us a "square root" of the covariance matrix.
        // First, perform the decomposition and get the Option.
        let cholesky_option = Cholesky::new(self.state.covariance.clone());

        if let Some(cholesky_result) = cholesky_option {
            let l_matrix = cholesky_result.l_dirty();

            let scale = (n as f64 + lambda).sqrt();
            let scaled_l = l_matrix * scale;

            // First point is the mean.
            sigma_points.column_mut(0).copy_from(&self.state.vector);

            // The other 2n points are spread around the mean.
            for i in 0..n {
                sigma_points
                    .column_mut(i + 1)
                    .copy_from(&(self.state.vector.clone() + scaled_l.column(i)));
                sigma_points
                    .column_mut(i + n + 1)
                    .copy_from(&(self.state.vector.clone() - scaled_l.column(i)));
            }
        } else {
            // If Cholesky fails (P is not positive definite), we can't generate points.
            // A robust fallback is to just return the mean N times.
            for i in 0..(2 * n + 1) {
                sigma_points.column_mut(i).copy_from(&self.state.vector);
            }
        }

        sigma_points
    }
}

impl StateEstimator for UnscentedKalmanFilter {
    fn predict(&mut self, dt: f64, u: &Control, _context: &FilterContext) {
        let n = self.state.dim();

        // --- 1. Generate Sigma Points ---
        let sigma_points = self.generate_sigma_points();

        // --- 2. Propagate each point through the NON-LINEAR dynamics model ---
        let mut propagated_points = DMatrix::zeros(n, 2 * n + 1);
        for i in 0..(2 * n + 1) {
            let point = sigma_points.column(i).into_owned();
            let propagated = self.dynamics_model.propagate(
                &point,
                u,
                self.state.last_update_timestamp,
                dt,
                &RK4,
            );
            propagated_points.column_mut(i).copy_from(&propagated);
        }

        // --- 3. Recover the predicted mean and covariance ---
        // Predicted mean: x_pred = sum(w_m * propagated_point)
        let x_pred = &propagated_points * &self.weights_m;

        // Predicted covariance: P_pred = sum(w_c * (propagated - x_pred) * (propagated - x_pred)^T) + Q
        let mut p_pred = DMatrix::zeros(n, n);
        for i in 0..(2 * n + 1) {
            let diff = propagated_points.column(i) - &x_pred;
            p_pred += self.weights_c[i] * &diff * diff.transpose();
        }
        p_pred += &self.process_noise_q * dt;

        // --- 4. Update the state ---
        self.state.vector = x_pred;
        self.state.covariance = p_pred;
        self.state.last_update_timestamp += dt;
        // Optional but Recommended Symmetrize Covariance
        // Tiny numerical errors can make P slightly non-symmetric. This forces it.
        self.state.covariance = (&self.state.covariance + self.state.covariance.transpose()) * 0.5;
    }

    fn update(&mut self, message: &MeasurementMessage, context: &FilterContext) {
        if let Some(model) = self.measurement_models.get(&message.sensor_handle) {
            // We use predict_measurement just to check if this model handles this data.
            // This call signature needs a slight adjustment to not need the message for the UKF.
            // Let's assume we can get z and a "measurement function" `h`.
            if let Some(z) = message.data.as_primary_slice().map(DVector::from_row_slice) {
                let n = self.state.dim();
                let m = z.nrows();

                // --- 1. Generate new sigma points from the PREDICTED state ---
                let sigma_points = self.generate_sigma_points();

                // --- 2. Propagate points through the NON-LINEAR measurement model ---
                // We need a dummy message for the predict_measurement call.
                let dummy_message = MeasurementMessage {
                    data: message.data.clone(),
                    ..*message
                };

                let mut measurement_points = DMatrix::zeros(m, 2 * n + 1);
                for i in 0..(2 * n + 1) {
                    let mut temp_state = self.state.clone();
                    temp_state.vector.copy_from(&sigma_points.column(i));

                    if let Some(z_point) =
                        model.predict_measurement(&temp_state, &dummy_message, context.tf.unwrap())
                    {
                        measurement_points.column_mut(i).copy_from(&z_point);
                    }
                }

                // --- 3. Recover the predicted measurement and its covariance ---
                let z_pred = &measurement_points * &self.weights_m;
                let mut s_cov = DMatrix::zeros(m, m);
                for i in 0..(2 * n + 1) {
                    let diff = measurement_points.column(i) - &z_pred;
                    s_cov += self.weights_c[i] * &diff * diff.transpose();
                }
                s_cov += model.get_r(); // Add measurement noise R

                // --- 4. Calculate cross-covariance and Kalman Gain ---
                let mut t_cov = DMatrix::zeros(n, m);
                for i in 0..(2 * n + 1) {
                    let diff_x = sigma_points.column(i) - &self.state.vector;
                    let diff_z = measurement_points.column(i) - &z_pred;
                    t_cov += self.weights_c[i] * &diff_x * diff_z.transpose();
                }

                if let Some(s_inv) = s_cov.clone().try_inverse() {
                    let k_gain = t_cov * s_inv;

                    // --- 5. Update state and covariance (same as EKF) ---
                    self.state.vector += &k_gain * (z - z_pred);
                    self.state.covariance -= &k_gain * s_cov * k_gain.transpose();
                    self.state.last_update_timestamp = message.timestamp;

                    // Optional but Recommended Symmetrize Covariance
                    // Tiny numerical errors can make P slightly non-symmetric. This forces it.
                    self.state.covariance =
                        (&self.state.covariance + self.state.covariance.transpose()) * 0.5;
                }
            }
        }
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
