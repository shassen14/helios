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

    // --- Pre-allocated workspace buffers (avoid per-cycle heap allocations) ---
    /// Sigma point matrix reused by `generate_sigma_points`: n × (2n+1).
    sigma_buf: DMatrix<f64>,
    /// Propagated sigma points reused in `predict`: n × (2n+1).
    propagated_buf: DMatrix<f64>,
    /// Predicted covariance buffer reused in `predict`: n × n.
    p_pred_buf: DMatrix<f64>,
    /// Scratch state used in `update` to avoid cloning `self.state` per sigma point.
    scratch_state: FrameAwareState,
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

        let sigma_buf = DMatrix::zeros(n, 2 * n + 1);
        let propagated_buf = DMatrix::zeros(n, 2 * n + 1);
        let p_pred_buf = DMatrix::zeros(n, n);
        let scratch_state = initial_state.clone();

        Self {
            state: initial_state,
            process_noise_q,
            dynamics_model,
            measurement_models,
            params,
            weights_m,
            weights_c,
            sigma_buf,
            propagated_buf,
            p_pred_buf,
            scratch_state,
        }
    }

    /// Fills `sigma_buf` with the `2n+1` sigma points from `state` and `params`.
    /// Takes explicit field borrows to avoid a `&mut self` conflict when the caller
    /// also holds a borrow on another field (e.g. `measurement_models`).
    fn fill_sigma_points(
        sigma_buf: &mut DMatrix<f64>,
        state: &FrameAwareState,
        params: &UkfParams,
    ) {
        let n = state.dim();
        let lambda = params.alpha.powi(2) * (n as f64 + params.kappa) - n as f64;

        // One clone of covariance required for Cholesky ownership.
        let cholesky_option = Cholesky::new(state.covariance.clone());

        if let Some(cholesky_result) = cholesky_option {
            let l_matrix = cholesky_result.l_dirty();
            let scale = (n as f64 + lambda).sqrt();
            let scaled_l = l_matrix * scale;

            sigma_buf.column_mut(0).copy_from(&state.vector);
            for i in 0..n {
                let col_pos = &state.vector + scaled_l.column(i);
                let col_neg = &state.vector - scaled_l.column(i);
                sigma_buf.column_mut(i + 1).copy_from(&col_pos);
                sigma_buf.column_mut(i + n + 1).copy_from(&col_neg);
            }
        } else {
            for i in 0..(2 * n + 1) {
                sigma_buf.column_mut(i).copy_from(&state.vector);
            }
        }
    }
}

impl StateEstimator for UnscentedKalmanFilter {
    fn predict(&mut self, dt: f64, u: &Control, _context: &FilterContext) {
        let n = self.state.dim();

        // --- 1. Generate Sigma Points into self.sigma_buf ---
        Self::fill_sigma_points(&mut self.sigma_buf, &self.state, &self.params);

        // --- 2. Propagate each point through the NON-LINEAR dynamics model ---
        self.propagated_buf.fill(0.0);
        for i in 0..(2 * n + 1) {
            let point = self.sigma_buf.column(i).into_owned();
            let propagated = self.dynamics_model.propagate(
                &point,
                u,
                self.state.last_update_timestamp,
                dt,
                &RK4,
            );
            self.propagated_buf.column_mut(i).copy_from(&propagated);
        }

        // --- 3. Recover the predicted mean and covariance ---
        // Predicted mean: x_pred = sum(w_m * propagated_point)
        let x_pred = &self.propagated_buf * &self.weights_m;

        // Predicted covariance (reuse p_pred_buf to avoid allocation).
        self.p_pred_buf.fill(0.0);
        for i in 0..(2 * n + 1) {
            let diff = self.propagated_buf.column(i) - &x_pred;
            self.p_pred_buf += self.weights_c[i] * &diff * diff.transpose();
        }
        self.p_pred_buf += &self.process_noise_q * dt;

        // --- 4. Update the state ---
        self.state.vector = x_pred;
        self.state.covariance.copy_from(&self.p_pred_buf);
        self.state.last_update_timestamp += dt;

        // Symmetrize covariance in-place (no allocation).
        for i in 0..n {
            for j in (i + 1)..n {
                let avg = (self.state.covariance[(i, j)] + self.state.covariance[(j, i)]) * 0.5;
                self.state.covariance[(i, j)] = avg;
                self.state.covariance[(j, i)] = avg;
            }
        }
    }

    fn update(&mut self, message: &MeasurementMessage, context: &FilterContext) {
        // tf is required for measurement prediction; skip update if unavailable.
        let Some(tf) = context.tf else {
            return;
        };

        for model in self.measurement_models.values() {
            // `get_measurement_vector` returns `Some` only for the matching model.
            if let Some(z) = model.get_measurement_vector(&message.data) {
                let n = self.state.dim();
                let m = z.nrows();

                // --- 1. Generate new sigma points from the PREDICTED state ---
                Self::fill_sigma_points(&mut self.sigma_buf, &self.state, &self.params);

                // --- 2. Propagate points through the NON-LINEAR measurement model ---
                let dummy_message = MeasurementMessage {
                    data: message.data.clone(),
                    ..*message
                };

                // m varies per sensor type — these allocations depend on measurement dimension.
                let mut measurement_points = DMatrix::zeros(m, 2 * n + 1);
                for i in 0..(2 * n + 1) {
                    // Use scratch_state to avoid cloning self.state per sigma point.
                    self.scratch_state
                        .vector
                        .copy_from(&self.sigma_buf.column(i));

                    if let Some(z_point) =
                        model.predict_measurement(&self.scratch_state, &dummy_message, tf)
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
                    let diff_x = self.sigma_buf.column(i) - &self.state.vector;
                    let diff_z = measurement_points.column(i) - &z_pred;
                    t_cov += self.weights_c[i] * &diff_x * diff_z.transpose();
                }

                if let Some(s_inv) = s_cov.clone().try_inverse() {
                    let k_gain = t_cov * s_inv;

                    // --- 5. Update state and covariance ---
                    self.state.vector += &k_gain * (z - z_pred);
                    self.state.covariance -= &k_gain * s_cov * k_gain.transpose();
                    self.state.last_update_timestamp = message.timestamp;

                    // Symmetrize covariance in-place (no allocation).
                    // P * P^T * 0.5
                    let n = self.state.dim();
                    for i in 0..n {
                        for j in (i + 1)..n {
                            let avg = (self.state.covariance[(i, j)]
                                + self.state.covariance[(j, i)])
                                * 0.5;
                            self.state.covariance[(i, j)] = avg;
                            self.state.covariance[(j, i)] = avg;
                        }
                    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimation::FilterContext;
    use crate::frames::{FrameAwareState, FrameId, StateVariable};
    use crate::messages::{MeasurementData, MeasurementMessage};
    use crate::models::estimation::measurement::Measurement;
    use crate::sensor_data;
    use crate::types::{FrameHandle, TfProvider};
    use nalgebra::{DMatrix, DVector, Isometry3, Vector3};
    use std::any::Any;
    use std::collections::HashMap;

    // --- Test Fixtures (mirrored from EKF tests) ---

    struct IdentityTf;

    impl TfProvider for IdentityTf {
        fn get_transform(&self, _from: FrameHandle, _to: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
        fn world_pose(&self, _frame: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
    }

    #[derive(Debug, Clone)]
    struct ConstantVelocity2D;

    impl EstimationDynamics for ConstantVelocity2D {
        fn get_control_dim(&self) -> usize {
            0
        }

        fn get_control_from_measurement(&self, _data: &MeasurementData) -> Option<DVector<f64>> {
            None
        }

        fn get_derivatives(&self, x: &DVector<f64>, _u: &DVector<f64>, _t: f64) -> DVector<f64> {
            let mut xdot = DVector::zeros(4);
            xdot[0] = x[2];
            xdot[1] = x[3];
            xdot
        }
    }

    #[derive(Debug, Clone)]
    struct Position2DMeasurement {
        r: DMatrix<f64>,
    }

    impl Measurement for Position2DMeasurement {
        fn get_measurement_layout(&self) -> Vec<StateVariable> {
            vec![
                StateVariable::Px(FrameId::World),
                StateVariable::Py(FrameId::World),
            ]
        }

        fn get_measurement_vector(&self, data: &MeasurementData) -> Option<DVector<f64>> {
            if let MeasurementData::GpsPosition(gps_position) = data {
                Some(DVector::from_row_slice(&[
                    gps_position.position.x,
                    gps_position.position.y,
                ]))
            } else {
                None
            }
        }

        fn predict_measurement(
            &self,
            state: &FrameAwareState,
            message: &MeasurementMessage,
            _tf: &dyn TfProvider,
        ) -> Option<DVector<f64>> {
            if !matches!(&message.data, MeasurementData::GpsPosition(_)) {
                return None;
            }
            Some(DVector::from_row_slice(&[state.vector[0], state.vector[1]]))
        }

        fn calculate_jacobian(
            &self,
            state: &FrameAwareState,
            _tf: &dyn TfProvider,
        ) -> DMatrix<f64> {
            let n = state.dim();
            let mut h = DMatrix::zeros(2, n);
            h[(0, 0)] = 1.0;
            h[(1, 1)] = 1.0;
            h
        }

        fn get_r(&self) -> &DMatrix<f64> {
            &self.r
        }

        fn as_any(&self) -> &dyn Any {
            self
        }
    }

    const SENSOR: FrameHandle = FrameHandle(1);

    fn make_ukf(initial_px: f64, vx: f64) -> UnscentedKalmanFilter {
        let layout = vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Py(FrameId::World),
            StateVariable::Vx(FrameId::World),
            StateVariable::Vy(FrameId::World),
        ];
        let mut state = FrameAwareState::new(layout, 1.0, 0.0);
        state.vector[0] = initial_px;
        state.vector[2] = vx;

        let q = DMatrix::identity(4, 4) * 0.01;
        let params = UkfParams {
            alpha: 1e-3,
            beta: 2.0,
            kappa: 0.0,
        };
        let r = DMatrix::identity(2, 2) * 0.1;
        let model: Box<dyn Measurement> = Box::new(Position2DMeasurement { r });
        let mut models = HashMap::new();
        models.insert(SENSOR, model);

        UnscentedKalmanFilter::new(state, q, Box::new(ConstantVelocity2D), models, params)
    }

    fn gps_message(x: f64, y: f64, t: f64) -> MeasurementMessage {
        MeasurementMessage {
            agent_handle: FrameHandle::default(),
            sensor_handle: SENSOR,
            timestamp: t,
            data: MeasurementData::GpsPosition(sensor_data::GpsPosition {
                position: Vector3::new(x, y, 0.0),
            }),
        }
    }

    // --- Predict Step Tests ---

    #[test]
    fn predict_advances_position_by_velocity() {
        let mut ukf = make_ukf(0.0, 1.0);
        let u = DVector::zeros(0);
        let ctx = FilterContext::default();

        ukf.predict(1.0, &u, &ctx);

        let px = ukf.get_state().vector[0];
        assert!(
            (px - 1.0).abs() < 0.05,
            "px should advance ≈ vx*dt = 1.0, got {px}"
        );
    }

    #[test]
    fn predict_grows_covariance() {
        let mut ukf = make_ukf(0.0, 1.0);
        let u = DVector::zeros(0);
        let ctx = FilterContext::default();
        let trace_before: f64 = ukf.get_state().covariance.diagonal().sum();

        ukf.predict(1.0, &u, &ctx);

        let trace_after: f64 = ukf.get_state().covariance.diagonal().sum();
        assert!(
            trace_after > trace_before,
            "covariance trace should grow after predict"
        );
    }

    // --- Update Step Tests ---

    #[test]
    fn update_corrects_state_toward_measurement() {
        let mut ukf = make_ukf(0.0, 0.0);
        let tf = IdentityTf;
        let ctx = FilterContext { tf: Some(&tf) };

        ukf.update(&gps_message(5.0, 0.0, 0.1), &ctx);

        let px = ukf.get_state().vector[0];
        assert!(px > 0.0, "state should correct toward measurement (px > 0)");
        assert!(px < 5.0, "state should not overshoot measurement");
    }

    #[test]
    fn update_shrinks_position_uncertainty() {
        let mut ukf = make_ukf(0.0, 0.0);
        let tf = IdentityTf;
        let ctx = FilterContext { tf: Some(&tf) };
        let p00_before = ukf.get_state().covariance[(0, 0)];

        ukf.update(&gps_message(0.0, 0.0, 0.1), &ctx);

        let p00_after = ukf.get_state().covariance[(0, 0)];
        assert!(
            p00_after < p00_before,
            "position variance should shrink after update"
        );
    }

    /// Verifies the P0 fix: `update` must not panic when `context.tf` is `None`.
    #[test]
    fn update_no_tf_does_not_panic() {
        let mut ukf = make_ukf(0.0, 0.0);
        let ctx = FilterContext { tf: None };
        let px_before = ukf.get_state().vector[0];

        // This must not panic (previously called context.tf.unwrap()).
        ukf.update(&gps_message(5.0, 0.0, 0.1), &ctx);

        assert_eq!(
            ukf.get_state().vector[0],
            px_before,
            "update must be skipped when TF is None"
        );
    }

    // --- Convergence Test ---

    #[test]
    fn filter_converges_to_true_position() {
        let mut ukf = make_ukf(0.0, 0.0);
        let tf = IdentityTf;
        let ctx = FilterContext { tf: Some(&tf) };
        let u = DVector::zeros(0);
        let true_px = 3.0_f64;

        for i in 0..50 {
            ukf.predict(0.1, &u, &ctx);
            ukf.update(&gps_message(true_px, 0.0, (i + 1) as f64 * 0.1), &ctx);
        }

        let px = ukf.get_state().vector[0];
        assert!(
            (px - true_px).abs() < 0.1,
            "UKF should converge near {true_px} m, got {px}"
        );
    }
}
