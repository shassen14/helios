// helios_core/src/estimation/filters/ekf.rs

use crate::estimation::dynamics::EstimationDynamics;
use crate::estimation::measurement::MeasurementModel;
use crate::estimation::{EstimatorInputs, GaussianStateEstimator};
use crate::frames::FrameAwareState;
use crate::ports::TfProvider;
use crate::utils::integrators::RK4;
use nalgebra::{DMatrix, DVector};

/// A concrete implementation of an Extended Kalman Filter.
///
/// Holds only filter-intrinsic state: the current `(x, P, t)`, the process noise
/// `Q`, and the dynamics model. Measurement models and their `R` matrices are
/// supplied per `update` call by the caller — the filter does not maintain a
/// registry of sensors.
pub struct ExtendedKalmanFilter {
    /// The current state of the filter (x, P, t).
    state: FrameAwareState,
    /// The process noise covariance matrix (Q), modeling uncertainty in the dynamics.
    process_noise_q: DMatrix<f64>,
    /// The specific dynamics model this filter will use for prediction.
    dynamics_model: Box<dyn EstimationDynamics>,
}

impl ExtendedKalmanFilter {
    /// Creates a new EKF instance.
    pub fn new(
        initial_state: FrameAwareState,
        process_noise_q: DMatrix<f64>,
        dynamics_model: Box<dyn EstimationDynamics>,
    ) -> Self {
        assert_eq!(initial_state.dim(), process_noise_q.nrows());
        assert_eq!(initial_state.dim(), process_noise_q.ncols());

        Self {
            state: initial_state,
            process_noise_q,
            dynamics_model,
        }
    }

    /// Enforces physical properties on the covariance matrix to prevent divergence.
    /// Called at the end of every predict and update step.
    fn ensure_covariance_health(&mut self) {
        let dim = self.state.dim();
        let p = &mut self.state.covariance;
        let min_variance = 1e-9;

        // 1. Enforce symmetry in-place (no allocation): average each off-diagonal pair.
        // p * p^T * 0.5
        for i in 0..dim {
            for j in (i + 1)..dim {
                let avg = (p[(i, j)] + p[(j, i)]) * 0.5;
                p[(i, j)] = avg;
                p[(j, i)] = avg;
            }
        }

        // 2. Enforce positive diagonal (prevent negative variance).
        for i in 0..dim {
            if p[(i, i)] < min_variance {
                p[(i, i)] = min_variance;
            }
        }

        // 3. Regularization via diagonal add (no identity matrix allocation).
        // Adds a tiny uncertainty preventing "close-minded" filters for numerical stability
        for i in 0..dim {
            p[(i, i)] += 1e-12;
        }
    }
}

impl GaussianStateEstimator for ExtendedKalmanFilter {
    fn predict(&mut self, dt: f64, inputs: &EstimatorInputs) {
        if dt <= 0.0 {
            return;
        }

        // --- 1. Get current state and dynamics model ---
        let dynamics = &self.dynamics_model;
        let x_old = &self.state.state.vector;
        let p_old = &self.state.covariance;
        let t_old = self.state.state.timestamp;

        let u_sized = if inputs.control.nrows() == dynamics.get_control_dim() {
            &inputs.control
        } else {
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
        // --- 2. Calculate State Transition and Noise Input Matrices--
        // A = ∂f/∂x, B = ∂f/∂u
        let (a_jac, _b_jac) = dynamics.calculate_jacobian(x_old, &inputs.control, t_old);

        // F ≈ I + A*dt (no identity allocation).
        let mut f_k = &a_jac * dt;
        for i in 0..self.state.dim() {
            f_k[(i, i)] += 1.0;
        }

        // --- 4. Predict the next covariance matrix ---
        // The correct approach is to apply the process noise `Q` *before* the main propagation.
        // This represents adding uncertainty to the model itself before you predict.
        let p_with_noise = p_old + &self.process_noise_q * dt;

        // Now, propagate this "noisier" covariance forward using the state transition matrix.
        // P_new = F * (P_old + Q*dt) * F^T
        let p_new = &f_k * p_with_noise * f_k.transpose();

        // --- 5. Update the filter's internal state ---
        self.state.state.vector = x_new;
        self.state.covariance = p_new;
        self.state.state.timestamp += dt;

        self.ensure_covariance_health();
        self.state.normalize_quaternion();
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

        let Some(z_pred) = model.predict_measurement(&self.state, tf) else {
            return;
        };
        if z_pred.nrows() != m {
            return;
        }

        let p_priori = &self.state.covariance;
        let h_jac = model.jacobian(&self.state, tf);
        let y = z - &z_pred;

        let s = &h_jac * p_priori * h_jac.transpose() + r;
        let Some(s_inv) = s.try_inverse() else {
            return;
        };

        let k_gain = &self.state.covariance * h_jac.transpose() * s_inv;
        let correction = &k_gain * &y;

        self.state.state.vector += correction;

        // Joseph form: (I - KH) P (I - KH)^T + K R K^T, no identity allocation.
        let mut i_kh = -(&k_gain * &h_jac);
        let n = self.state.dim();
        for i in 0..n {
            i_kh[(i, i)] += 1.0;
        }
        let p_post = &i_kh * p_priori * i_kh.transpose() + &k_gain * r * k_gain.transpose();
        self.state.covariance = p_post;

        self.ensure_covariance_health();
        self.state.normalize_quaternion();
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

    // --- Test Fixtures ---

    struct IdentityTf;

    impl TfProvider for IdentityTf {
        fn get_transform(&self, _from: FrameHandle, _to: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
        fn world_pose(&self, _frame: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
    }

    /// 2D constant-velocity dynamics: state = [px, py, vx, vy].
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

        fn calculate_jacobian(
            &self,
            _x: &DVector<f64>,
            _u: &DVector<f64>,
            _t: f64,
        ) -> (DMatrix<f64>, DMatrix<f64>) {
            let mut a = DMatrix::zeros(4, 4);
            a[(0, 2)] = 1.0;
            a[(1, 3)] = 1.0;
            (a, DMatrix::zeros(4, 0))
        }
    }

    /// 2D position measurement model: z = [px, py].
    /// `R` is held by the test, not the model.
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

    fn make_state_with_velocity(vx: f64) -> FrameAwareState {
        let layout = vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Py(FrameId::World),
            StateVariable::Vx(FrameId::World),
            StateVariable::Vy(FrameId::World),
        ];
        let mut state = FrameAwareState::new(layout, 1.0, 0.0);
        state.state.vector[2] = vx;
        state
    }

    fn make_ekf(initial_px: f64, vx: f64) -> ExtendedKalmanFilter {
        let mut state = make_state_with_velocity(vx);
        state.state.vector[0] = initial_px;
        let q = DMatrix::identity(4, 4) * 0.01;
        ExtendedKalmanFilter::new(state, q, Box::new(ConstantVelocity2D))
    }

    fn gps_r() -> DMatrix<f64> {
        DMatrix::identity(2, 2) * 0.1
    }

    fn gps_z(x: f64, y: f64) -> DVector<f64> {
        DVector::from_row_slice(&[x, y])
    }

    // --- Predict Step Tests ---

    #[test]
    fn predict_advances_position_by_velocity() {
        let mut ekf = make_ekf(0.0, 1.0);
        let u = DVector::zeros(0);

        ekf.predict(1.0, &EstimatorInputs { control: u });

        let px = ekf.state().state.vector[0];
        assert!(
            (px - 1.0).abs() < 0.05,
            "px should advance ≈ vx*dt = 1.0, got {px}"
        );
        assert!(ekf.state().state.vector[1].abs() < 1e-9);
    }

    #[test]
    fn predict_zero_dt_is_noop() {
        let mut ekf = make_ekf(5.0, 2.0);
        let u = DVector::zeros(0);
        let px_before = ekf.state().state.vector[0];

        ekf.predict(0.0, &EstimatorInputs { control: u });

        assert_eq!(ekf.state().state.vector[0], px_before);
    }

    #[test]
    fn predict_grows_covariance() {
        let mut ekf = make_ekf(0.0, 1.0);
        let u = DVector::zeros(0);
        let trace_before: f64 = ekf.state().covariance.diagonal().sum();

        ekf.predict(1.0, &EstimatorInputs { control: u });

        let trace_after: f64 = ekf.state().covariance.diagonal().sum();
        assert!(trace_after > trace_before);
    }

    // --- Update Step Tests ---

    #[test]
    fn update_corrects_state_toward_measurement() {
        let mut ekf = make_ekf(0.0, 0.0);
        let tf = IdentityTf;
        let model = Position2DMeasurement;
        let r = gps_r();

        ekf.update(&gps_z(5.0, 0.0), &model, &r, Some(&tf));

        let px = ekf.state().state.vector[0];
        assert!(px > 0.0, "state should correct toward measurement (px > 0)");
        assert!(px < 5.0, "state should not overshoot measurement");
    }

    #[test]
    fn update_shrinks_position_uncertainty() {
        let mut ekf = make_ekf(0.0, 0.0);
        let tf = IdentityTf;
        let model = Position2DMeasurement;
        let r = gps_r();
        let p00_before = ekf.state().covariance[(0, 0)];

        ekf.update(&gps_z(0.0, 0.0), &model, &r, Some(&tf));

        let p00_after = ekf.state().covariance[(0, 0)];
        assert!(p00_after < p00_before);
    }

    #[test]
    fn update_with_mismatched_r_is_skipped() {
        let mut ekf = make_ekf(0.0, 0.0);
        let tf = IdentityTf;
        let model = Position2DMeasurement;
        // Wrong-sized R (3x3 instead of 2x2) — must be silently skipped.
        let bad_r = DMatrix::identity(3, 3) * 0.1;
        let px_before = ekf.state().state.vector[0];

        ekf.update(&gps_z(5.0, 0.0), &model, &bad_r, Some(&tf));

        assert_eq!(ekf.state().state.vector[0], px_before);
    }

    // --- Convergence Test ---

    #[test]
    fn filter_converges_to_true_position() {
        let mut ekf = make_ekf(0.0, 0.0);
        let tf = IdentityTf;
        let model = Position2DMeasurement;
        let r = gps_r();
        let u = DVector::zeros(0);
        let true_px = 3.0_f64;

        for _ in 0..50 {
            ekf.predict(0.1, &EstimatorInputs { control: u.clone() });
            ekf.update(&gps_z(true_px, 0.0), &model, &r, Some(&tf));
        }

        let px = ekf.state().state.vector[0];
        assert!(
            (px - true_px).abs() < 0.1,
            "EKF should converge near {true_px} m, got {px}"
        );
    }
}
