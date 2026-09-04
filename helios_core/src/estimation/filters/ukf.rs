use crate::data::ports::TfProvider;
use crate::data::MonotonicTime;
use crate::estimation::dynamics::EstimationDynamics;
use crate::estimation::measurement::MeasurementModel;
use crate::estimation::{EstimatorInputs, GaussianStateEstimator};
use crate::frames::FrameAwareState;
use crate::utils::integrators::RK4;

use nalgebra::{Cholesky, DMatrix, DVector};

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
        let s = initial_state.storage_dim();
        let t = initial_state.tangent_dim();

        let lambda = params.alpha.powi(2) * (t as f64 + params.kappa) - t as f64;

        let mut weights_m = DVector::from_element(2 * t + 1, 0.5 / (t as f64 + lambda));
        let mut weights_c = DVector::from_element(2 * t + 1, 0.5 / (t as f64 + lambda));
        weights_m[0] = lambda / (t as f64 + lambda);
        weights_c[0] = weights_m[0] + (1.0 - params.alpha.powi(2) + params.beta);

        let sigma_buf = DMatrix::zeros(s, 2 * t + 1);
        let propagated_buf = DMatrix::zeros(s, 2 * t + 1);
        let p_pred_buf = DMatrix::zeros(t, t);
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

    /// Fills `sigma_buf` (`s` rows × `2t+1` cols) with sigma points: the mean
    /// plus one pair per tangent axis, straddling the mean along the ± Cholesky
    /// directions of the covariance.
    ///
    /// The two spaces meet here. `t = tangent_dim` sets how many axes the
    /// covariance has, hence how many point pairs (`2t+1` total). Each sigma
    /// point is a POINT in storage space (`s` long); each Cholesky column is a
    /// TANGENT step (`t` long). Straddling is `oplus`, never `+`: the step is
    /// retracted onto the manifold, so it stays correct once a block is curved.
    fn fill_sigma_points(
        sigma_buf: &mut DMatrix<f64>,
        state: &FrameAwareState,
        params: &UkfParams,
    ) {
        let t = state.tangent_dim();

        let lambda = params.alpha.powi(2) * (t as f64 + params.kappa) - t as f64;

        let cholesky_option = Cholesky::new(state.covariance.clone());

        if let Some(cholesky_result) = cholesky_option {
            // Columns of the scaled Cholesky factor are the ± perturbation steps,
            // one per tangent axis. Each column is a tangent vector (length t).
            let l_matrix = cholesky_result.l_dirty();
            let scale = (t as f64 + lambda).sqrt();
            let scaled_l = l_matrix * scale;

            // Column 0 is the mean itself (a point, length s).
            sigma_buf.column_mut(0).copy_from(&state.mean);
            for i in 0..t {
                // Retract the mean by ± the i-th tangent step to get the pair of
                // sigma points straddling it (points, length s).
                let step = scaled_l.column(i);
                let neg_step = -step;

                let sigma_plus = state.schema().oplus(state.mean.as_view(), step);
                let sigma_minus = state
                    .schema()
                    .oplus(state.mean.as_view(), neg_step.as_view());

                sigma_buf.column_mut(i + 1).copy_from(&sigma_plus);
                sigma_buf.column_mut(i + t + 1).copy_from(&sigma_minus);
            }
        } else {
            for i in 0..(2 * t + 1) {
                sigma_buf.column_mut(i).copy_from(&state.mean);
            }
        }
    }
}

impl GaussianStateEstimator for UnscentedKalmanFilter {
    fn predict(&mut self, dt: f64, inputs: &EstimatorInputs) {
        let t = self.state.tangent_dim();

        // --- 1. Generate Sigma Points into self.sigma_buf ---
        Self::fill_sigma_points(&mut self.sigma_buf, &self.state, &self.params);

        // --- 2. Propagate each point through the NON-LINEAR dynamics model ---
        self.propagated_buf.fill(0.0);
        for i in 0..(2 * t + 1) {
            let point = self.sigma_buf.column(i).into_owned();
            let propagated = self.dynamics_model.propagate(
                &point,
                &inputs.control,
                self.state.timestamp,
                dt,
                &RK4,
            );
            self.propagated_buf.column_mut(i).copy_from(&propagated);
        }

        // --- 3. Recover the predicted mean and covariance ---
        // Predicted mean: the weighted average of the propagated points. A point
        // in storage space (length s). Linear averaging is exact while blocks are
        // Euclidean; a curved block needs an iterative mean instead.
        let x_pred = &self.propagated_buf * &self.weights_m;

        // Predicted covariance (t × t), reusing p_pred_buf to avoid allocation.
        // Each propagated point's deviation from the mean is point ⊟ point, a
        // tangent vector (length t) — hence ominus, not `-`. Its weighted outer
        // products accumulate the covariance in tangent space.
        self.p_pred_buf.fill(0.0);
        for i in 0..(2 * t + 1) {
            let deviation = self
                .state
                .schema()
                .ominus(self.propagated_buf.column(i), x_pred.as_view());
            self.p_pred_buf += self.weights_c[i] * &deviation * deviation.transpose();
        }
        self.p_pred_buf += &self.process_noise_q * dt;

        // --- 4. Update the state ---
        self.state.mean = x_pred;
        self.state.covariance.copy_from(&self.p_pred_buf);
        self.state.timestamp += dt;

        // Symmetrize covariance in-place (no allocation).
        for i in 0..t {
            for j in (i + 1)..t {
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
        at: MonotonicTime,
    ) {
        let m = model.dim();
        if z.nrows() != m || r.nrows() != m || r.ncols() != m {
            return;
        }

        let t = self.state.tangent_dim();

        // Regenerate sigma points from the predicted state.
        Self::fill_sigma_points(&mut self.sigma_buf, &self.state, &self.params);

        // Propagate sigma points through the (non-linear) measurement model.
        let mut measurement_points = DMatrix::zeros(m, 2 * t + 1);
        for i in 0..(2 * t + 1) {
            self.scratch_state.mean.copy_from(&self.sigma_buf.column(i));

            if let Some(z_point) = model.predict_measurement(&self.scratch_state, tf, at) {
                if z_point.nrows() == m {
                    measurement_points.column_mut(i).copy_from(&z_point);
                }
            }
        }

        // Predicted measurement and its innovation covariance (m × m). The
        // measurement lives in ordinary Euclidean space, not on the state
        // manifold, so its deviations are plain subtraction — never ominus.
        let z_pred = &measurement_points * &self.weights_m;
        let mut innovation_cov = DMatrix::zeros(m, m);
        for i in 0..(2 * t + 1) {
            let meas_dev = measurement_points.column(i) - &z_pred;
            innovation_cov += self.weights_c[i] * &meas_dev * meas_dev.transpose();
        }
        innovation_cov += r;

        // Cross-covariance between state error and measurement (t × m): how a
        // deviation in tangent space co-varies with one in measurement space.
        // The state side is point ⊟ point (ominus, tangent); the measurement
        // side stays Euclidean. This asymmetry is the whole point.
        let mut cross_cov = DMatrix::zeros(t, m);
        for i in 0..(2 * t + 1) {
            let state_dev = self
                .state
                .schema()
                .ominus(self.sigma_buf.column(i), self.state.mean.as_view());

            let meas_dev = measurement_points.column(i) - &z_pred;
            cross_cov += self.weights_c[i] * &state_dev * meas_dev.transpose();
        }

        // Factor the innovation covariance S and SOLVE for the gain rather than
        // forming S⁻¹: cheaper, better-conditioned, and a failed Cholesky (S not SPD)
        // signals a degenerate sigma-point spread and skips the update — the same
        // guard the old inverse's None branch gave. Clone because S (innovation_cov)
        // is reused in the covariance downdate below.
        let Some(s_chol) = innovation_cov.clone().cholesky() else {
            return;
        };

        // Kalman gain K = P_xz S⁻¹ (t × m), with P_xz the state–measurement cross-
        // covariance (`cross_cov`). S is symmetric, so Kᵀ = S⁻¹ P_xzᵀ: solve
        // S·Kᵀ = cross_covᵀ, then transpose. The correction K·innovation is a tangent
        // vector (length t); retract it onto the mean with ⊞, not `+`.
        let k_gain = s_chol.solve(&cross_cov.transpose()).transpose();
        let correction = &k_gain * (z - z_pred);
        self.state.oplus_assign(&correction);
        self.state.covariance -= &k_gain * innovation_cov * k_gain.transpose();

        for i in 0..t {
            for j in (i + 1)..t {
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
    use crate::data::ports::TfProvider;
    use crate::data::MonotonicTime;
    use crate::estimation::measurement::MeasurementModel;
    use crate::estimation::schema::{MeasurementSchema, StateSchema, StateSchemaBlock};
    use crate::estimation::EstimatorInputs;
    use crate::frames::transforms::{Convention, ErasedTransform};
    use crate::frames::{FrameAwareState, FrameId};
    use crate::state::Quantity;
    use nalgebra::{DMatrix, DVector, Isometry3};

    const AT: MonotonicTime = MonotonicTime(0.0);

    struct IdentityTf;

    impl TfProvider for IdentityTf {
        fn get_transform(
            &self,
            _from: FrameId,
            _to: FrameId,
            _at: MonotonicTime,
        ) -> Option<ErasedTransform> {
            Some(ErasedTransform::from_parts(
                Isometry3::identity(),
                Convention::Flu,
                Convention::Flu,
            ))
        }
    }

    /// 3D constant-velocity dynamics: state = [px, py, pz, vx, vy, vz]. The Z
    /// axis carries no velocity in these tests (vz stays 0), so pz is constant —
    /// the model is 3D only because [`Quantity`] blocks are 3D, not because the
    /// motion is.
    #[derive(Debug, Clone)]
    struct ConstantVelocity3D;

    impl EstimationDynamics for ConstantVelocity3D {
        fn get_control_dim(&self) -> usize {
            0
        }

        fn schema(&self) -> std::sync::Arc<StateSchema> {
            std::sync::Arc::new(StateSchema::compose(vec![
                StateSchemaBlock::new(
                    Quantity::Position(FrameId::World),
                    Convention::Enu,
                    None,
                    DVector::zeros(3),
                    DMatrix::identity(3, 3),
                ),
                StateSchemaBlock::new(
                    Quantity::Velocity(FrameId::World),
                    Convention::Enu,
                    None,
                    DVector::zeros(3),
                    DMatrix::identity(3, 3),
                ),
            ]))
        }

        fn derivatives(&self, x: &DVector<f64>, _u: &DVector<f64>, _t: f64) -> DVector<f64> {
            let mut xdot = DVector::zeros(6);
            xdot[0] = x[3];
            xdot[1] = x[4];
            xdot[2] = x[5];
            xdot
        }
    }

    #[derive(Debug, Clone)]
    struct Position2DMeasurement;

    impl MeasurementModel for Position2DMeasurement {
        fn dim(&self) -> usize {
            2
        }

        // A 2D position observes only x and y. A `Position` quantity is a whole
        // 3-vector, so no honest block yields dim 2 — partial-component
        // measurements are not yet expressible as a schema block. The update
        // tests never ask this mock for a schema, so the gap is recorded, not hit.
        fn schema(&self) -> MeasurementSchema {
            unimplemented!("2D (partial-component) position measurement has no MeasurementSchema block yet")
        }

        fn predict_measurement(
            &self,
            state: &FrameAwareState,
            _tf: Option<&dyn TfProvider>,
            _at: MonotonicTime,
        ) -> Option<DVector<f64>> {
            Some(DVector::from_row_slice(&[state.mean[0], state.mean[1]]))
        }

        fn jacobian(
            &self,
            state: &FrameAwareState,
            _tf: Option<&dyn TfProvider>,
            _at: MonotonicTime,
        ) -> DMatrix<f64> {
            let n = state.storage_dim();
            let mut h = DMatrix::zeros(2, n);
            h[(0, 0)] = 1.0;
            h[(1, 1)] = 1.0;
            h
        }
    }

    fn make_ukf(initial_px: f64, vx: f64) -> UnscentedKalmanFilter {
        // Layout is [px, py, pz, vx, vy, vz]; Vx is index 3.
        let mut state = FrameAwareState::from_schema(ConstantVelocity3D.schema(), 0.0);
        state.mean[0] = initial_px;
        state.mean[3] = vx;

        let q = DMatrix::identity(6, 6) * 0.01;
        let params = UkfParams {
            alpha: 1e-3,
            beta: 2.0,
            kappa: 0.0,
        };

        UnscentedKalmanFilter::new(state, q, Box::new(ConstantVelocity3D), params)
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

        let px = ukf.state().mean[0];
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

        ukf.update(&gps_z(5.0, 0.0), &model, &r, Some(&tf), AT);

        let px = ukf.state().mean[0];
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

        ukf.update(&gps_z(0.0, 0.0), &model, &r, Some(&tf), AT);

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
            ukf.update(&gps_z(true_px, 0.0), &model, &r, Some(&tf), AT);
        }

        let px = ukf.state().mean[0];
        assert!((px - true_px).abs() < 0.1);
    }
}
