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
    /// Enforces physical properties on the covariance matrix to prevent divergence.
    /// This should be called at the end of every predict and update step.
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
        // --- 2. Calculate State Transition and Noise Input Matrices--
        // A = ∂f/∂x, B = ∂f/∂u
        let (a_jac, _b_jac) = dynamics.calculate_jacobian(x_old, u, t_old);

        // Discretize the state transition matrix: F ≈ I + A*dt (no identity allocation)
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
        self.state.vector = x_new;
        self.state.covariance = p_new;
        self.state.last_update_timestamp += dt;

        // Force Symmetry and Positivity
        self.ensure_covariance_health();
    }

    /// Updates the state by fusing an aiding measurement.
    fn update(&mut self, message: &MeasurementMessage, context: &FilterContext) {
        // O(1) dispatch: look up the model directly by sensor handle.
        let Some(model) = self.measurement_models.get(&message.sensor_handle) else {
            return;
        };

        let Some(z) = model.get_measurement_vector(&message.data) else {
            return;
        };

        // tf is required for measurement prediction; skip update if unavailable.
        let Some(tf) = context.tf else {
            return;
        };

        let Some(z_pred) = model.predict_measurement(&self.state, message, tf) else {
            return;
        };

        if z.nrows() != z_pred.nrows() {
            return;
        }

        let p_priori = &self.state.covariance;
        let h_jac = model.calculate_jacobian(&self.state, tf);
        let r_mat = model.get_r();
        let y = z - &z_pred;

        // --- Calculate the full Kalman update values once ---
        let s = &h_jac * p_priori * h_jac.transpose() + r_mat;
        let Some(s_inv) = s.try_inverse() else {
            return;
        };

        let k_gain = &self.state.covariance * h_jac.transpose() * s_inv;
        let correction = &k_gain * &y;

        self.state.vector += correction;
        // Joseph form: (I - KH) computed without allocating an identity matrix.
        let mut i_kh = -(&k_gain * &h_jac);
        let n = self.state.dim();
        for i in 0..n {
            i_kh[(i, i)] += 1.0;
        }
        // P_post = (I - KH)P(I - KH)^T + KRK^T
        let p_post = &i_kh * p_priori * i_kh.transpose() + &k_gain * r_mat * k_gain.transpose();
        self.state.covariance = p_post;

        // Enforce Symmetry and Positivity
        self.ensure_covariance_health();

        self.state.last_update_timestamp = message.timestamp;
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
    use crate::types::{FrameHandle, TfProvider};
    use nalgebra::{DMatrix, DVector, Isometry3, Vector3};
    use std::any::Any;
    use std::collections::HashMap;

    // --- Test Fixtures ---

    /// Stub TF provider that returns identity transforms.
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

        fn get_control_from_measurement(&self, _data: &MeasurementData) -> Option<DVector<f64>> {
            None
        }

        fn get_derivatives(&self, x: &DVector<f64>, _u: &DVector<f64>, _t: f64) -> DVector<f64> {
            let mut xdot = DVector::zeros(4);
            xdot[0] = x[2]; // px_dot = vx
            xdot[1] = x[3]; // py_dot = vy
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

    /// 2D position measurement model: z = [px, py] from GpsPosition data.
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
            if let MeasurementData::GpsPosition(v) = data {
                Some(DVector::from_row_slice(&[v[0], v[1]]))
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

    fn make_state_with_velocity(vx: f64) -> FrameAwareState {
        let layout = vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Py(FrameId::World),
            StateVariable::Vx(FrameId::World),
            StateVariable::Vy(FrameId::World),
        ];
        let mut state = FrameAwareState::new(layout, 1.0, 0.0);
        state.vector[2] = vx;
        state
    }

    fn make_ekf(initial_px: f64, vx: f64) -> ExtendedKalmanFilter {
        let mut state = make_state_with_velocity(vx);
        state.vector[0] = initial_px;
        let q = DMatrix::identity(4, 4) * 0.01;
        let r = DMatrix::identity(2, 2) * 0.1;
        let model: Box<dyn Measurement> = Box::new(Position2DMeasurement { r });
        let mut models = HashMap::new();
        models.insert(SENSOR, model);
        ExtendedKalmanFilter::new(state, q, Box::new(ConstantVelocity2D), models)
    }

    fn gps_message(x: f64, y: f64, t: f64) -> MeasurementMessage {
        MeasurementMessage {
            agent_handle: FrameHandle::default(),
            sensor_handle: SENSOR,
            timestamp: t,
            data: MeasurementData::GpsPosition(Vector3::new(x, y, 0.0)),
        }
    }

    // --- Predict Step Tests ---

    #[test]
    fn predict_advances_position_by_velocity() {
        let mut ekf = make_ekf(0.0, 1.0);
        let u = DVector::zeros(0);
        let ctx = FilterContext::default();

        ekf.predict(1.0, &u, &ctx);

        let px = ekf.get_state().vector[0];
        assert!(
            (px - 1.0).abs() < 0.05,
            "px should advance ≈ vx*dt = 1.0, got {px}"
        );
        assert!(ekf.get_state().vector[1].abs() < 1e-9, "py should stay 0");
    }

    #[test]
    fn predict_zero_dt_is_noop() {
        let mut ekf = make_ekf(5.0, 2.0);
        let u = DVector::zeros(0);
        let ctx = FilterContext::default();
        let px_before = ekf.get_state().vector[0];

        ekf.predict(0.0, &u, &ctx);

        assert_eq!(
            ekf.get_state().vector[0],
            px_before,
            "zero dt must not change state"
        );
    }

    #[test]
    fn predict_grows_covariance() {
        let mut ekf = make_ekf(0.0, 1.0);
        let u = DVector::zeros(0);
        let ctx = FilterContext::default();
        let trace_before: f64 = ekf.get_state().covariance.diagonal().sum();

        ekf.predict(1.0, &u, &ctx);

        let trace_after: f64 = ekf.get_state().covariance.diagonal().sum();
        assert!(
            trace_after > trace_before,
            "covariance trace should grow after predict"
        );
    }

    // --- Update Step Tests ---

    #[test]
    fn update_corrects_state_toward_measurement() {
        let mut ekf = make_ekf(0.0, 0.0);
        let tf = IdentityTf;
        let ctx = FilterContext { tf: Some(&tf) };

        // Measurement says position is at [5, 0]; filter starts at [0, 0].
        ekf.update(&gps_message(5.0, 0.0, 0.1), &ctx);

        let px = ekf.get_state().vector[0];
        assert!(px > 0.0, "state should correct toward measurement (px > 0)");
        assert!(px < 5.0, "state should not overshoot measurement");
    }

    #[test]
    fn update_shrinks_position_uncertainty() {
        let mut ekf = make_ekf(0.0, 0.0);
        let tf = IdentityTf;
        let ctx = FilterContext { tf: Some(&tf) };
        let p00_before = ekf.get_state().covariance[(0, 0)];

        ekf.update(&gps_message(0.0, 0.0, 0.1), &ctx);

        let p00_after = ekf.get_state().covariance[(0, 0)];
        assert!(
            p00_after < p00_before,
            "position variance should shrink after update"
        );
    }

    #[test]
    fn update_skipped_without_tf() {
        let mut ekf = make_ekf(0.0, 0.0);
        let ctx = FilterContext { tf: None };
        let px_before = ekf.get_state().vector[0];

        ekf.update(&gps_message(5.0, 0.0, 0.1), &ctx);

        assert_eq!(
            ekf.get_state().vector[0],
            px_before,
            "update must be skipped when TF is None"
        );
    }

    #[test]
    fn update_ignores_unknown_sensor_handle() {
        let mut ekf = make_ekf(0.0, 0.0);
        let tf = IdentityTf;
        let ctx = FilterContext { tf: Some(&tf) };
        let px_before = ekf.get_state().vector[0];

        let msg = MeasurementMessage {
            agent_handle: FrameHandle::default(),
            sensor_handle: FrameHandle(99), // not registered
            timestamp: 0.1,
            data: MeasurementData::GpsPosition(Vector3::new(5.0, 0.0, 0.0)),
        };
        ekf.update(&msg, &ctx);

        assert_eq!(
            ekf.get_state().vector[0],
            px_before,
            "update should be ignored for unregistered sensor handle"
        );
    }

    // --- Convergence Test ---

    #[test]
    fn filter_converges_to_true_position() {
        // Start at px=0, true position is 3.0 m. After repeated predict+update cycles
        // with a GPS measurement, the filter should converge.
        let mut ekf = make_ekf(0.0, 0.0);
        let tf = IdentityTf;
        let ctx = FilterContext { tf: Some(&tf) };
        let u = DVector::zeros(0);
        let true_px = 3.0_f64;

        for i in 0..50 {
            ekf.predict(0.1, &u, &ctx);
            ekf.update(&gps_message(true_px, 0.0, (i + 1) as f64 * 0.1), &ctx);
        }

        let px = ekf.get_state().vector[0];
        assert!(
            (px - true_px).abs() < 0.1,
            "EKF should converge near {true_px} m, got {px}"
        );
    }
}
