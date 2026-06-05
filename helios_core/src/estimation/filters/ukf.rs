use nalgebra::{Cholesky, DMatrix, DVector};

use crate::estimation::dynamics::EstimationDynamics;
use crate::estimation::measurement::MeasurementModel;
use crate::estimation::{EstimatorInputs, GaussianStateEstimator};
use crate::frames::FrameAwareState;
use crate::ports::TfProvider;
use crate::utils::integrators::RK4;

/// Configuration parameters for the UKF's sigma point generation.
#[derive(Debug, Clone, Copy)]
pub struct UkfParams {
    pub alpha: f64, // Spreading of sigma points (usually 1e-3)
    pub beta: f64,  // Incorporates prior knowledge of distribution (2.0 is optimal for Gaussian)
    pub kappa: f64, // Secondary scaling parameter (often 0.0 or 3-n)
}

/// A concrete implementation of an Unscented Kalman Filter.
///
/// As with the EKF, the filter does not hold a measurement registry. Models and
/// their `R` matrices are supplied per `update` call.
pub struct UnscentedKalmanFilter {
    state: FrameAwareState,
    process_noise_q: DMatrix<f64>,
    dynamics_model: Box<dyn EstimationDynamics>,
    params: UkfParams,

    /// Weights for calculating the mean from sigma points.
    weights_m: DVector<f64>,
    /// Weights for calculating the covariance from sigma points.
    weights_c: DVector<f64>,

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
        params: UkfParams,
    ) -> Self {
        let n = initial_state.dim();
        let lambda = params.alpha.powi(2) * (n as f64 + params.kappa) - n as f64;

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
            params,
            weights_m,
            weights_c,
            sigma_buf,
            propagated_buf,
            p_pred_buf,
            scratch_state,
        }
    }

    fn fill_sigma_points(
        sigma_buf: &mut DMatrix<f64>,
        state: &FrameAwareState,
        params: &UkfParams,
    ) {
        let n = state.dim();
        let lambda = params.alpha.powi(2) * (n as f64 + params.kappa) - n as f64;

        let cholesky_option = Cholesky::new(state.covariance.clone());

        if let Some(cholesky_result) = cholesky_option {
            let l_matrix = cholesky_result.l_dirty();
            let scale = (n as f64 + lambda).sqrt();
            let scaled_l = l_matrix * scale;

            sigma_buf.column_mut(0).copy_from(&state.state.vector);
            for i in 0..n {
                let col_pos = &state.state.vector + scaled_l.column(i);
                let col_neg = &state.state.vector - scaled_l.column(i);
                sigma_buf.column_mut(i + 1).copy_from(&col_pos);
                sigma_buf.column_mut(i + n + 1).copy_from(&col_neg);
            }
        } else {
            for i in 0..(2 * n + 1) {
                sigma_buf.column_mut(i).copy_from(&state.state.vector);
            }
        }
    }
}

impl GaussianStateEstimator for UnscentedKalmanFilter {
    fn predict(&mut self, dt: f64, inputs: &EstimatorInputs) {
        let n = self.state.dim();

        // --- 1. Generate Sigma Points into self.sigma_buf ---
        Self::fill_sigma_points(&mut self.sigma_buf, &self.state, &self.params);

        // --- 2. Propagate each point through the NON-LINEAR dynamics model ---
        self.propagated_buf.fill(0.0);
        for i in 0..(2 * n + 1) {
            let point = self.sigma_buf.column(i).into_owned();
            let propagated = self.dynamics_model.propagate(
                &point,
                &inputs.control,
                self.state.state.timestamp,
                dt,
                &RK4,
            );
            self.propagated_buf.column_mut(i).copy_from(&propagated);
        }

        // --- 3. Recover the predicted mean and covariance ---
        // Predicted mean: x_pred = sum(w_m * propagated_point
        let x_pred = &self.propagated_buf * &self.weights_m;

        // Predicted covariance (reuse p_pred_buf to avoid allocation).
        self.p_pred_buf.fill(0.0);
        for i in 0..(2 * n + 1) {
            let diff = self.propagated_buf.column(i) - &x_pred;
            self.p_pred_buf += self.weights_c[i] * &diff * diff.transpose();
        }
        self.p_pred_buf += &self.process_noise_q * dt;

        // --- 4. Update the state ---
        self.state.state.vector = x_pred;
        self.state.covariance.copy_from(&self.p_pred_buf);
        self.state.state.timestamp += dt;

        // Symmetrize covariance in-place (no allocation).
        for i in 0..n {
            for j in (i + 1)..n {
                let avg = (self.state.covariance[(i, j)] + self.state.covariance[(j, i)]) * 0.5;
                self.state.covariance[(i, j)] = avg;
                self.state.covariance[(j, i)] = avg;
            }
        }
    }

    fn update(
        &mut self,
        z: &DVector<f64>,
        model: &dyn MeasurementModel,
        r: &DMatrix<f64>,
        tf: Option<&dyn TfProvider>,
    ) {
        let m = model.dim();
        if z.nrows() != m || r.nrows() != m || r.ncols() != m {
            return;
        }

        let n = self.state.dim();

        // Regenerate sigma points from the predicted state.
        Self::fill_sigma_points(&mut self.sigma_buf, &self.state, &self.params);

        // Propagate sigma points through the (non-linear) measurement model.
        let mut measurement_points = DMatrix::zeros(m, 2 * n + 1);
        for i in 0..(2 * n + 1) {
            self.scratch_state
                .state
                .vector
                .copy_from(&self.sigma_buf.column(i));

            if let Some(z_point) = model.predict_measurement(&self.scratch_state, tf) {
                if z_point.nrows() == m {
                    measurement_points.column_mut(i).copy_from(&z_point);
                }
            }
        }

        let z_pred = &measurement_points * &self.weights_m;
        let mut s_cov = DMatrix::zeros(m, m);
        for i in 0..(2 * n + 1) {
            let diff = measurement_points.column(i) - &z_pred;
            s_cov += self.weights_c[i] * &diff * diff.transpose();
        }
        s_cov += r;

        let mut t_cov = DMatrix::zeros(n, m);
        for i in 0..(2 * n + 1) {
            let diff_x = self.sigma_buf.column(i) - &self.state.state.vector;
            let diff_z = measurement_points.column(i) - &z_pred;
            t_cov += self.weights_c[i] * &diff_x * diff_z.transpose();
        }

        let Some(s_inv) = s_cov.clone().try_inverse() else {
            return;
        };

        let k_gain = t_cov * s_inv;
        self.state.state.vector += &k_gain * (z - z_pred);
        self.state.covariance -= &k_gain * s_cov * k_gain.transpose();

        for i in 0..n {
            for j in (i + 1)..n {
                let avg = (self.state.covariance[(i, j)] + self.state.covariance[(j, i)]) * 0.5;
                self.state.covariance[(i, j)] = avg;
                self.state.covariance[(j, i)] = avg;
            }
        }
    }

    fn state(&self) -> &FrameAwareState {
        &self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data::primitives::FrameHandle;
    use crate::estimation::measurement::MeasurementModel;
    use crate::estimation::EstimatorInputs;
    use crate::frames::{FrameAwareState, FrameId, StateVariable};
    use crate::ports::TfProvider;
    use nalgebra::{DMatrix, DVector, Isometry3};

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

        fn get_derivatives(&self, x: &DVector<f64>, _u: &DVector<f64>, _t: f64) -> DVector<f64> {
            let mut xdot = DVector::zeros(4);
            xdot[0] = x[2];
            xdot[1] = x[3];
            xdot
        }
    }

    #[derive(Debug, Clone)]
    struct Position2DMeasurement;

    impl MeasurementModel for Position2DMeasurement {
        fn dim(&self) -> usize {
            2
        }

        fn predict_measurement(
            &self,
            state: &FrameAwareState,
            _tf: Option<&dyn TfProvider>,
        ) -> Option<DVector<f64>> {
            Some(DVector::from_row_slice(&[
                state.state.vector[0],
                state.state.vector[1],
            ]))
        }

        fn jacobian(&self, state: &FrameAwareState, _tf: Option<&dyn TfProvider>) -> DMatrix<f64> {
            let n = state.dim();
            let mut h = DMatrix::zeros(2, n);
            h[(0, 0)] = 1.0;
            h[(1, 1)] = 1.0;
            h
        }
    }

    fn make_ukf(initial_px: f64, vx: f64) -> UnscentedKalmanFilter {
        let layout = vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Py(FrameId::World),
            StateVariable::Vx(FrameId::World),
            StateVariable::Vy(FrameId::World),
        ];
        let mut state = FrameAwareState::new(layout, 1.0, 0.0);
        state.state.vector[0] = initial_px;
        state.state.vector[2] = vx;

        let q = DMatrix::identity(4, 4) * 0.01;
        let params = UkfParams {
            alpha: 1e-3,
            beta: 2.0,
            kappa: 0.0,
        };

        UnscentedKalmanFilter::new(state, q, Box::new(ConstantVelocity2D), params)
    }

    fn gps_r() -> DMatrix<f64> {
        DMatrix::identity(2, 2) * 0.1
    }

    fn gps_z(x: f64, y: f64) -> DVector<f64> {
        DVector::from_row_slice(&[x, y])
    }

    #[test]
    fn predict_advances_position_by_velocity() {
        let mut ukf = make_ukf(0.0, 1.0);
        let u = DVector::zeros(0);

        ukf.predict(1.0, &EstimatorInputs { control: u });

        let px = ukf.state().state.vector[0];
        assert!((px - 1.0).abs() < 0.05);
    }

    #[test]
    fn predict_grows_covariance() {
        let mut ukf = make_ukf(0.0, 1.0);
        let u = DVector::zeros(0);
        let trace_before: f64 = ukf.state().covariance.diagonal().sum();

        ukf.predict(1.0, &EstimatorInputs { control: u });

        let trace_after: f64 = ukf.state().covariance.diagonal().sum();
        assert!(trace_after > trace_before);
    }

    #[test]
    fn update_corrects_state_toward_measurement() {
        let mut ukf = make_ukf(0.0, 0.0);
        let tf = IdentityTf;
        let model = Position2DMeasurement;
        let r = gps_r();

        ukf.update(&gps_z(5.0, 0.0), &model, &r, Some(&tf));

        let px = ukf.state().state.vector[0];
        assert!(px > 0.0);
        assert!(px < 5.0);
    }

    #[test]
    fn update_shrinks_position_uncertainty() {
        let mut ukf = make_ukf(0.0, 0.0);
        let tf = IdentityTf;
        let model = Position2DMeasurement;
        let r = gps_r();
        let p00_before = ukf.state().covariance[(0, 0)];

        ukf.update(&gps_z(0.0, 0.0), &model, &r, Some(&tf));

        let p00_after = ukf.state().covariance[(0, 0)];
        assert!(p00_after < p00_before);
    }

    #[test]
    fn filter_converges_to_true_position() {
        let mut ukf = make_ukf(0.0, 0.0);
        let tf = IdentityTf;
        let model = Position2DMeasurement;
        let r = gps_r();
        let u = DVector::zeros(0);
        let true_px = 3.0_f64;

        for _ in 0..50 {
            ukf.predict(0.1, &EstimatorInputs { control: u.clone() });
            ukf.update(&gps_z(true_px, 0.0), &model, &r, Some(&tf));
        }

        let px = ukf.state().state.vector[0];
        assert!((px - true_px).abs() < 0.1);
    }
}
