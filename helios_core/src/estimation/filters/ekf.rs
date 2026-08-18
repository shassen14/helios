use crate::data::ports::TfProvider;
use crate::data::MonotonicTime;
use crate::estimation::dynamics::EstimationDynamics;
use crate::estimation::measurement::MeasurementModel;
use crate::estimation::{EstimatorInputs, GaussianStateEstimator};
use crate::frames::FrameAwareState;
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
        assert_eq!(initial_state.tangent_dim(), process_noise_q.nrows());
        assert_eq!(initial_state.tangent_dim(), process_noise_q.ncols());

        Self {
            state: initial_state,
            process_noise_q,
            dynamics_model,
        }
    }

    /// Enforces physical properties on the covariance matrix to prevent divergence.
    /// Called at the end of every predict and update step.
    fn ensure_covariance_health(&mut self) {
        // The covariance lives in tangent space (t × t), so every index here is
        // a tangent dimension, not a stored-vector slot.
        let dim = self.state.tangent_dim();
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
        let x_old = &self.state.mean;
        let p_old = &self.state.covariance;
        let t_old = self.state.timestamp;

        let u_sized = if inputs.control.nrows() == dynamics.get_control_dim() {
            &inputs.control
        } else {
            &DVector::zeros(dynamics.get_control_dim())
        };

        // --- 2. Predict the next state vector using numerical integration ---
        // x_pred = f(x, u, t). A point in storage space (length s); the dynamics
        // model owns its own retraction, so this is a direct assignment, not ⊞.
        let x_new = dynamics.propagate(x_old, u_sized, t_old, dt, &RK4);

        // --- 3. Linearize the dynamics by calculating the state transition matrix (Jacobian F) ---
        // For an EKF with a discrete time step, we often use an approximation:
        // F_k ≈ I + A * dt
        // But a more accurate way is to use the full matrix exponential if possible,
        // or just the calculated Jacobian `A` from the continuous model. We will use `A`.
        // --- 2. Calculate State Transition and Noise Input Matrices--
        // A = ∂f/∂x, B = ∂f/∂u
        let (a_jac, _b_jac) = dynamics.jacobian(x_old, &inputs.control, t_old);

        // F ≈ I + A*dt (no identity allocation). F propagates the covariance, so
        // it is a tangent-space map (t × t) — the identity add walks tangent dims.
        let mut f_k = &a_jac * dt;
        for i in 0..self.state.tangent_dim() {
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
        self.state.mean = x_new;
        self.state.covariance = p_new;
        self.state.timestamp += dt;

        self.ensure_covariance_health();
        // Euclidean ⊞ adds the quaternion componentwise, so the mean can drift
        // off the unit sphere; renormalize until the rotation block retracts on
        // its own manifold and keeps itself unit.
        self.state.normalize_quaternion();
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

        let Some(z_pred) = model.predict_measurement(&self.state, tf, at) else {
            return;
        };
        if z_pred.nrows() != m {
            return;
        }

        let p_priori = self.state.covariance.clone();
        let h_jac = model.jacobian(&self.state, tf, at);
        let y = z - &z_pred;

        let s = &h_jac * &p_priori * h_jac.transpose() + r;
        let Some(s_inv) = s.try_inverse() else {
            return;
        };

        // Kalman gain (t × m). The correction K·y is a tangent vector (length t);
        // retract it onto the mean with ⊞, not `+`.
        let k_gain = &self.state.covariance * h_jac.transpose() * s_inv;
        let correction = &k_gain * &y;

        self.state.oplus_assign(&correction);

        // Joseph form: (I - KH) P (I - KH)^T + K R K^T, no identity allocation.
        // All tangent-space (t × t); the identity add walks tangent dims.
        let mut i_kh = -(&k_gain * &h_jac);
        let n = self.state.tangent_dim();
        for i in 0..n {
            i_kh[(i, i)] += 1.0;
        }
        let p_post = &i_kh * p_priori * i_kh.transpose() + &k_gain * r * k_gain.transpose();
        self.state.covariance = p_post;

        self.ensure_covariance_health();
        // ⊞ has already applied the correction; renormalize because the
        // Euclidean quaternion block adds componentwise and can leave the unit
        // sphere. Redundant once the rotation block retracts on its own manifold.
        self.state.normalize_quaternion();
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
    use crate::estimation::EstimatorInputs;
    use crate::frames::transforms::{Convention, ErasedTransform};
    use crate::frames::{FrameAwareState, FrameId, StateVariable};
    use nalgebra::{DMatrix, DVector, Isometry3};

    // --- Test Fixtures ---

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

    /// 2D constant-velocity dynamics: state = [px, py, vx, vy].
    #[derive(Debug, Clone)]
    struct ConstantVelocity2D;

    impl EstimationDynamics for ConstantVelocity2D {
        fn get_control_dim(&self) -> usize {
            0
        }

        fn schema(&self) -> std::sync::Arc<crate::estimation::schema::StateSchema> {
            std::sync::Arc::new(crate::estimation::schema::StateSchema::degenerate(&[
                StateVariable::Px(FrameId::World),
                StateVariable::Py(FrameId::World),
                StateVariable::Vx(FrameId::World),
                StateVariable::Vy(FrameId::World),
            ]))
        }

        fn derivatives(&self, x: &DVector<f64>, _u: &DVector<f64>, _t: f64) -> DVector<f64> {
            let mut xdot = DVector::zeros(4);
            xdot[0] = x[2];
            xdot[1] = x[3];
            xdot
        }

        fn jacobian(
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

    fn make_state_with_velocity(vx: f64) -> FrameAwareState {
        let layout = vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Py(FrameId::World),
            StateVariable::Vx(FrameId::World),
            StateVariable::Vy(FrameId::World),
        ];
        let mut state = FrameAwareState::new(layout, 1.0, 0.0);
        state.mean[2] = vx;
        state
    }

    fn make_ekf(initial_px: f64, vx: f64) -> ExtendedKalmanFilter {
        let mut state = make_state_with_velocity(vx);
        state.mean[0] = initial_px;
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

        let px = ekf.state().mean[0];
        assert!(
            (px - 1.0).abs() < 0.05,
            "px should advance ≈ vx*dt = 1.0, got {px}"
        );
        assert!(ekf.state().mean[1].abs() < 1e-9);
    }

    #[test]
    fn predict_zero_dt_is_noop() {
        let mut ekf = make_ekf(5.0, 2.0);
        let u = DVector::zeros(0);
        let px_before = ekf.state().mean[0];

        ekf.predict(0.0, &EstimatorInputs { control: u });

        assert_eq!(ekf.state().mean[0], px_before);
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

        ekf.update(&gps_z(5.0, 0.0), &model, &r, Some(&tf), AT);

        let px = ekf.state().mean[0];
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

        ekf.update(&gps_z(0.0, 0.0), &model, &r, Some(&tf), AT);

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
        let px_before = ekf.state().mean[0];

        ekf.update(&gps_z(5.0, 0.0), &model, &bad_r, Some(&tf), AT);

        assert_eq!(ekf.state().mean[0], px_before);
    }

    // --- Convergence Test ---

    // --- Golden Trajectory (freeze-forward regression) ---
    //
    // Drives the real `IntegratedImuModel` through the EKF predict loop and
    // freezes the resulting mean and covariance diagonal to the bit. There is no
    // pre-refactor baseline to compare against, so this captures *current*
    // behavior: its job is to fire the moment a change to the schema plumbing,
    // the RK4 propagation, or the covariance update perturbs the trajectory that
    // was not meant to change. When a change legitimately moves these numbers
    // (e.g. the quaternion tangent block dropping 4→3 dims), re-harvest the
    // literals in the same run and commit the new values deliberately.

    /// Builds the frozen 16-state INS EKF and runs a fixed control script,
    /// returning the final `(mean, covariance-diagonal)`. Every input here is a
    /// hardcoded constant so the run is fully deterministic.
    fn run_golden_ins_trajectory() -> (DVector<f64>, DVector<f64>) {
        use crate::data::primitives::FrameHandle;
        use crate::estimation::dynamics::integrated_imu::IntegratedImuModel;
        use nalgebra::Vector3;

        // Distinct per-block variances so a transposed Q or P₀ block cannot hide
        // behind a shared value.
        let model = IntegratedImuModel::new(
            FrameHandle(7),
            Vector3::new(0.0, 0.0, -9.81),
            0.04,     // accel_noise_var
            0.0025,   // gyro_noise_var
            0.0001,   // accel_bias_var
            0.000001, // gyro_bias_var
            0.5,      // pos_var
            0.02,     // ori_var
        );

        let schema = model.schema();
        let q = schema.process_noise().clone();
        let initial_state = FrameAwareState::from_schema(schema, 0.0);
        let mut ekf = ExtendedKalmanFilter::new(initial_state, q, Box::new(model));

        // Constant IMU: 0.5 m/s² forward, gravity-compensated on Z, 0.15 rad/s yaw.
        let control = DVector::from_row_slice(&[0.5, 0.0, 9.81, 0.0, 0.0, 0.15]);
        let inputs = EstimatorInputs { control };

        let dt = 0.02;
        for _ in 0..50 {
            ekf.predict(dt, &inputs);
        }

        let state = ekf.state();
        (state.mean.clone(), state.covariance.diagonal())
    }

    #[test]
    fn golden_ins_trajectory_is_frozen() {
        // Full-precision literals harvested from a captured run. f64's shortest
        // round-trippable text form reparses to the identical bits, so `==` here
        // is an exact comparison, not an approximate one.
        let expected_mean = [
            0.24953160142126202,
            0.01248594503126783,
            0.0,
            0.49812710824531004,
            0.03742974021319026,
            0.0,
            0.0,
            0.0,
            0.07492970727274119,
            0.9971888181122076,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ];
        let expected_cov_diag = [
            6.0144822755614324,
            6.0254566112943975,
            1.7649038960417855,
            33.14751985825398,
            33.22691636994827,
            2.120809565454974,
            0.27204802416636203,
            0.27204802416636203,
            0.2710188494014054,
            0.024084783412809015,
            1.0001000000500073,
            1.0001000000500073,
            1.0001000000500073,
            1.0000010000500095,
            1.0000010000500095,
            1.0000010000500095,
        ];

        let (mean, cov_diag) = run_golden_ins_trajectory();

        for (i, &want) in expected_mean.iter().enumerate() {
            assert_eq!(mean[i], want, "mean[{i}] drifted from frozen golden");
        }
        for (i, &want) in expected_cov_diag.iter().enumerate() {
            assert_eq!(
                cov_diag[i], want,
                "covariance diagonal[{i}] drifted from frozen golden"
            );
        }
    }

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
            ekf.update(&gps_z(true_px, 0.0), &model, &r, Some(&tf), AT);
        }

        let px = ekf.state().mean[0];
        assert!(
            (px - true_px).abs() < 0.1,
            "EKF should converge near {true_px} m, got {px}"
        );
    }

    // --- Config-driven augmentation: construction + inspection ---
    //
    // These prove the composition seam: a config-declared bias block composes
    // onto the frozen 16-state INS schema and yields a constructable EKF whose
    // dimensions, layout, P₀, and Q are the base corner plus the appended block.
    // They inspect the built filter without stepping it; the live-predict tests
    // further down drive the same augmented filter through `predict`.

    /// Magnetometer hard-iron bias tuning for the augmentation tests. Both are
    /// standard deviations (squared into `P₀` / `Q` by the descriptor), chosen
    /// distinct from every base INS block variance so a misplaced value cannot
    /// hide behind a shared number.
    const MAG_BIAS_INIT_UNCERTAINTY: f64 = 0.3;
    const MAG_BIAS_RANDOM_WALK: f64 = 0.02;
    /// Degrees of freedom in one magnetometer hard-iron bias block.
    const MAG_BIAS_DOF: usize = 3;

    /// The frozen 16-state INS model, same tuning as the golden trajectory so its
    /// base schema is the well-exercised one.
    fn ins_model() -> crate::estimation::dynamics::integrated_imu::IntegratedImuModel {
        use crate::data::primitives::FrameHandle;
        use nalgebra::Vector3;

        crate::estimation::dynamics::integrated_imu::IntegratedImuModel::new(
            FrameHandle(7),
            Vector3::new(0.0, 0.0, -9.81),
            0.04,     // accel_noise_var
            0.0025,   // gyro_noise_var
            0.0001,   // accel_bias_var
            0.000001, // gyro_bias_var
            0.5,      // pos_var
            0.02,     // ori_var
        )
    }

    fn mag_bias_block(sensor: FrameId) -> crate::estimation::schema::SchemaBlock {
        crate::estimation::augmentation::augmentation_block(
            crate::estimation::augmentation::MAGNETOMETER_BIAS,
            sensor,
            MAG_BIAS_INIT_UNCERTAINTY,
            MAG_BIAS_RANDOM_WALK,
        )
        .expect("well-formed magnetometer-bias block builds")
    }

    /// Builds an EKF whose state is the INS base augmented with one bias block
    /// per `bias_sensors` entry. Construction only — the returned filter must not
    /// be stepped (see the module note above).
    fn augmented_ins_ekf(bias_sensors: &[FrameId]) -> ExtendedKalmanFilter {
        use std::sync::Arc;

        let model = ins_model();
        let base = model.schema();
        let blocks = bias_sensors.iter().cloned().map(mag_bias_block).collect();
        let augmented = Arc::new(base.extended(blocks));
        let state = FrameAwareState::from_schema(augmented.clone(), 0.0);
        ExtendedKalmanFilter::new(state, augmented.process_noise().clone(), Box::new(model))
    }

    #[test]
    fn augmented_ins_ekf_constructs_and_carries_the_bias_block() {
        use crate::data::primitives::FrameHandle;
        use std::sync::Arc;

        let model = ins_model();
        let base = model.schema();
        let base_dim = base.tangent_dim();
        let sensor = FrameId::Sensor(FrameHandle(9));

        let augmented = Arc::new(base.extended(vec![mag_bias_block(sensor.clone())]));
        let state = FrameAwareState::from_schema(augmented.clone(), 0.0);

        // The construction seam: `ExtendedKalmanFilter::new`'s only assertion is
        // `tangent_dim == Q dims`, which the composed schema satisfies. Reaching
        // the next line without a panic is itself the proof that a 19-dim
        // augmented schema builds a filter.
        let ekf = ExtendedKalmanFilter::new(
            state,
            augmented.process_noise().clone(),
            Box::new(model),
        );
        let state = ekf.state();

        // Dimension grew by exactly the 3-DOF bias block, in both spaces.
        assert_eq!(state.tangent_dim(), base_dim + MAG_BIAS_DOF);
        assert_eq!(state.storage_dim(), base_dim + MAG_BIAS_DOF);

        // The bias variables are appended after the base, tagged with the sensor,
        // and the first sits exactly at the base boundary.
        let layout = state.schema().layout();
        assert_eq!(
            &layout[base_dim..],
            &[
                StateVariable::MagBiasX(sensor.clone()),
                StateVariable::MagBiasY(sensor.clone()),
                StateVariable::MagBiasZ(sensor.clone()),
            ]
        );
        assert_eq!(
            state
                .schema()
                .storage_offset_of(&StateVariable::MagBiasX(sensor.clone())),
            Some(base_dim)
        );

        // P₀ (the state's covariance): base corner byte-identical to the
        // un-augmented schema; bias corner = init_uncertainty² isotropic.
        let p = &state.covariance;
        let base_p = base.initial_covariance();
        for i in 0..base_dim {
            for j in 0..base_dim {
                assert_eq!(p[(i, j)], base_p[(i, j)], "P₀ base corner changed at ({i},{j})");
            }
        }
        for k in 0..MAG_BIAS_DOF {
            let d = base_dim + k;
            assert_eq!(p[(d, d)], MAG_BIAS_INIT_UNCERTAINTY * MAG_BIAS_INIT_UNCERTAINTY);
        }

        // Q (the composed process noise fed to the filter): same story.
        let q = augmented.process_noise();
        let base_q = base.process_noise();
        for i in 0..base_dim {
            for j in 0..base_dim {
                assert_eq!(q[(i, j)], base_q[(i, j)], "Q base corner changed at ({i},{j})");
            }
        }
        for k in 0..MAG_BIAS_DOF {
            let d = base_dim + k;
            assert_eq!(q[(d, d)], MAG_BIAS_RANDOM_WALK * MAG_BIAS_RANDOM_WALK);
        }
    }

    #[test]
    fn augmentation_changes_ekf_dimension_by_block_size() {
        use crate::data::primitives::FrameHandle;

        // Distinct sensors so the two bias blocks are independent, not aliased.
        let s9 = FrameId::Sensor(FrameHandle(9));
        let s10 = FrameId::Sensor(FrameHandle(10));
        let base_dim = ins_model().schema().tangent_dim();

        // Cardinality is a load-time property (a Vec length): each added block
        // shifts the state dimension by exactly its size, none required at build.
        assert_eq!(augmented_ins_ekf(&[]).state().tangent_dim(), base_dim);
        assert_eq!(
            augmented_ins_ekf(&[s9.clone()]).state().tangent_dim(),
            base_dim + MAG_BIAS_DOF
        );
        assert_eq!(
            augmented_ins_ekf(&[s9, s10]).state().tangent_dim(),
            base_dim + 2 * MAG_BIAS_DOF
        );
    }

    // --- Config-driven augmentation: live predict (the sizing proof) ---
    //
    // Because `IntegratedImuModel::{derivatives,jacobian}` size to the input
    // state rather than a fixed 16, a composed 19-dim state propagates through
    // `predict` unchanged. Two properties pin the generalization down: the base
    // sub-vector evolves bit-for-bit like the standalone 16-state filter (the
    // appended block is inert on the base trajectory, since the dynamics reads
    // and writes only base offsets), and the bias block behaves as a random
    // walk — its mean holds at the prior forever and its covariance grows only
    // through its own Q.

    /// Fixed predict script shared by the live-augmentation tests: the same
    /// constant IMU as the golden trajectory (0.5 m/s² forward, gravity-
    /// compensated Z, 0.15 rad/s yaw) over a fixed step count, so the base
    /// evolution being compared against is the well-exercised one.
    const AUG_PREDICT_STEPS: usize = 50;
    const AUG_PREDICT_DT: f64 = 0.02;

    /// Absolute tolerance on the accumulated bias variance. The block-diagonal
    /// covariance recursion makes `P_bias = P₀ + N·Q·dt` exact in real
    /// arithmetic; the only slack is `ensure_covariance_health`'s per-step
    /// 1e-12 diagonal regularization (≈ N·1e-12 total), which this bound clears
    /// by orders of magnitude while staying far below the growth it checks.
    const BIAS_COV_TOL: f64 = 1e-9;

    fn imu_control_script() -> DVector<f64> {
        DVector::from_row_slice(&[0.5, 0.0, 9.81, 0.0, 0.0, 0.15])
    }

    /// The un-augmented 16-state INS EKF, same tuning as [`augmented_ins_ekf`],
    /// for a side-by-side base-trajectory comparison.
    fn base_ins_ekf() -> ExtendedKalmanFilter {
        let model = ins_model();
        let schema = model.schema();
        let state = FrameAwareState::from_schema(schema.clone(), 0.0);
        ExtendedKalmanFilter::new(state, schema.process_noise().clone(), Box::new(model))
    }

    #[test]
    fn augmented_predict_leaves_base_subvector_bit_identical() {
        use crate::data::primitives::FrameHandle;

        let base_dim = ins_model().schema().tangent_dim();
        let sensor = FrameId::Sensor(FrameHandle(9));
        let mut base = base_ins_ekf();
        let mut aug = augmented_ins_ekf(&[sensor]);

        // `predict` only borrows its inputs, so one immutable script drives both.
        let inputs = EstimatorInputs {
            control: imu_control_script(),
        };
        for _ in 0..AUG_PREDICT_STEPS {
            base.predict(AUG_PREDICT_DT, &inputs);
            aug.predict(AUG_PREDICT_DT, &inputs);
        }

        // The 16 base slots reached the exact same bits as the standalone filter:
        // the trailing bias slots never enter the base derivative computation, so
        // augmentation cannot perturb the base trajectory.
        for i in 0..base_dim {
            assert_eq!(
                aug.state().mean[i],
                base.state().mean[i],
                "augmented base slot {i} diverged from the un-augmented filter"
            );
        }

        // Zero-derivative bias states are a random walk whose mean stays put: the
        // prior is zero, so every slot must read exactly zero after the run.
        for k in 0..MAG_BIAS_DOF {
            assert_eq!(
                aug.state().mean[base_dim + k],
                0.0,
                "bias mean slot {k} drifted; a random-walk mean must hold at its prior"
            );
        }
    }

    #[test]
    fn augmented_predict_grows_bias_covariance_by_process_noise() {
        use crate::data::primitives::FrameHandle;

        let base_dim = ins_model().schema().tangent_dim();
        let sensor = FrameId::Sensor(FrameHandle(9));
        let mut aug = augmented_ins_ekf(&[sensor]);

        let inputs = EstimatorInputs {
            control: imu_control_script(),
        };
        for _ in 0..AUG_PREDICT_STEPS {
            aug.predict(AUG_PREDICT_DT, &inputs);
        }

        // The dynamics Jacobian is block-diagonal with an identity bias block
        // (zero bias derivative → zero Jacobian column → F_bias = I), and both P₀
        // and Q are block-diagonal, so the bias covariance never couples to the
        // base and its variance accumulates linearly at Q·dt per step.
        let p0_bias = MAG_BIAS_INIT_UNCERTAINTY * MAG_BIAS_INIT_UNCERTAINTY;
        let q_bias = MAG_BIAS_RANDOM_WALK * MAG_BIAS_RANDOM_WALK;
        let expected = p0_bias + (AUG_PREDICT_STEPS as f64) * q_bias * AUG_PREDICT_DT;

        for k in 0..MAG_BIAS_DOF {
            let d = base_dim + k;
            let got = aug.state().covariance[(d, d)];
            assert!(
                (got - expected).abs() < BIAS_COV_TOL,
                "bias variance {k}: expected P₀ + N·Q·dt = {expected}, got {got}"
            );
        }
    }

    // --- Config-driven augmentation: live update (the observability proof) ---
    //
    // Predict alone leaves the bias a random walk (mean fixed, variance growing).
    // What certifies M2 is that the *update* observes it: a magnetometer reading
    // consistent with a nonzero true bias must both pull the bias estimate off its
    // zero prior toward that truth and shrink its variance. This is only possible
    // because `MagneticFieldModel::predict_measurement` reads the bias slots, so
    // the finite-difference Jacobian grows identity columns over them and the
    // Kalman gain carries a nonzero correction into the appended block.

    /// Isotropic magnetometer measurement variance for the update test — small
    /// relative to the bias prior (`MAG_BIAS_INIT_UNCERTAINTY²`), so a persistent
    /// innovation is attributed largely to the poorly-known bias rather than the
    /// tightly-held attitude.
    const MAG_MEASUREMENT_VAR: f64 = 0.01;
    /// Update count for the convergence test — enough repeated readings for the
    /// bias estimate to move clearly off its prior.
    const MAG_UPDATE_STEPS: usize = 100;

    #[test]
    fn augmented_update_drives_bias_toward_truth() {
        use crate::data::primitives::FrameHandle;
        use crate::estimation::measurement::magnetometer::MagneticFieldModel;
        use nalgebra::Vector3;

        let base_dim = ins_model().schema().tangent_dim();
        let sensor_handle = FrameHandle(9);
        let sensor = FrameId::Sensor(sensor_handle);
        let mut ekf = augmented_ins_ekf(&[sensor.clone()]);

        let model = MagneticFieldModel {
            agent_handle: FrameHandle(7),
            sensor_handle,
            world_magnetic_field: Vector3::new(0.2, 0.4, -0.3),
        };
        let r = DMatrix::identity(3, 3) * MAG_MEASUREMENT_VAR;

        // The reading a stationary sensor with hard-iron bias `b_true` produces:
        // evaluate the model's own prediction on a copy of the initial state whose
        // bias slots hold the truth. Feeding this constant reading back to the
        // filter (whose bias starts at zero) is a persistent, consistent innovation.
        let b_true = Vector3::new(0.15, -0.1, 0.08);
        let mut truth_state = ekf.state().clone();
        truth_state.set_variable(&StateVariable::MagBiasX(sensor.clone()), b_true.x);
        truth_state.set_variable(&StateVariable::MagBiasY(sensor.clone()), b_true.y);
        truth_state.set_variable(&StateVariable::MagBiasZ(sensor.clone()), b_true.z);
        let z = model
            .predict_measurement(&truth_state, Some(&IdentityTf), AT)
            .expect("mag prediction under identity TF is defined");

        let p0_bias = MAG_BIAS_INIT_UNCERTAINTY * MAG_BIAS_INIT_UNCERTAINTY;
        for _ in 0..MAG_UPDATE_STEPS {
            ekf.update(&z, &model, &r, Some(&IdentityTf), AT);
        }

        let state = ekf.state();
        let b_est = Vector3::new(
            state.mean[base_dim],
            state.mean[base_dim + 1],
            state.mean[base_dim + 2],
        );

        // The estimate moved off the zero prior toward the truth: its error is
        // strictly smaller than the initial error (the whole of `b_true`).
        assert!(
            (b_est - b_true).norm() < b_true.norm(),
            "bias estimate {b_est:?} did not move toward truth {b_true:?}"
        );

        // Every bias variance shrank below the prior — the update extracted
        // information about the block, which is the observability claim.
        for k in 0..MAG_BIAS_DOF {
            let d = base_dim + k;
            let var = state.covariance[(d, d)];
            assert!(
                var < p0_bias,
                "bias variance {k} = {var} did not drop below prior {p0_bias}"
            );
        }
    }
}
