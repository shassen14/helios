//! Linearization of dynamics into the discrete tangent-space pair `(F_d, Q_d)`
//! that a Gaussian filter's covariance step `P⁺ = F_d P F_dᵀ + Q_d` needs.
//!
//! [`discrete_linearization`] is the one entry point every filter that needs a
//! dynamics Jacobian calls. Sample-based filters (UKF) push their points
//! through `propagate` and never call it.

use crate::kernel::integrators::Integrator;
use crate::prelude::EstimationDynamics;
use crate::spatial::FrameAwareState;

use nalgebra::{DMatrix, DVector};

/// The discrete covariance-step pair for one predict over `dt`.
pub(crate) struct DiscreteLinearization {
    /// `F_d`, tangent × tangent: carries the prior covariance across the step.
    pub f_d: DMatrix<f64>,
    /// `Q_d`, tangent × tangent: the process noise accumulated over the step.
    pub q_d: DMatrix<f64>,
}

/// The discrete `(F_d, Q_d)` for one step of `dynamics` about `state`.
///
/// `F_d` is the numeric tangent transition ([`tangent_state_transition`]) and
/// `Q_d = Q·dt`, the first-order discretisation of the constant continuous
/// process noise `process_noise_q`. That is the special case of the
/// error-state model with `G_c = I` and `Q_c = Q`. A model's own
/// [`ErrorStateModel`](crate::estimation::dynamics::ErrorStateModel) is not
/// consulted: every model is linearised numerically.
pub(crate) fn discrete_linearization(
    dynamics: &dyn EstimationDynamics,
    state: &FrameAwareState,
    u: &DVector<f64>, // already checked against the input schema
    t: f64,
    dt: f64,
    integrator: &dyn Integrator<f64>,
    process_noise_q: &DMatrix<f64>,
) -> DiscreteLinearization {
    DiscreteLinearization {
        f_d: tangent_state_transition(dynamics, state, u, t, dt, integrator),
        q_d: process_noise_q * dt,
    }
}

/// The discrete tangent-space state-transition matrix `F` (tangent × tangent)
/// linearizing `dynamics` about `state` over the step `dt`.
///
/// This is the **discrete** transition, not a continuous `A`. Because
/// `propagate` already folds in `dt`, the result is `F ≈ I + A·dt` on its own —
/// the caller must **not** add identity.
///
/// The dimension and retraction come from **`state`'s** schema, never
/// `dynamics.schema()`. The two differ under augmentation: the dynamics model is
/// the base process (e.g. a 15-tangent INS) while the state carries the composed,
/// augmented schema (e.g. 18-tangent with a bias block). `F` must be sized by the
/// state that owns `P`, or the covariance update dimensions mismatch. `propagate`
/// stays base-blind — it sizes to its input vector and leaves the appended slots
/// with zero derivative, giving the block an identity transition for free.
///
/// Each column is finite-differenced on the manifold: perturb the mean along
/// tangent basis vector `j` with `oplus`, propagate, then difference the result
/// against the unperturbed next state `y0` through `ominus`. Perturbing in tangent
/// space (rather than nudging stored components) keeps the rotation block's 3-DOF
/// error consistent with its 4-component storage; a raw component bump would walk
/// the quaternion off the unit sphere and mis-scale that block. The step size
/// follows the crate's adaptive rule `ε = 1e-5·(1 + ‖x‖∞)`.
fn tangent_state_transition(
    dynamics: &dyn EstimationDynamics,
    state: &FrameAwareState,
    u: &DVector<f64>, // already checked against the input schema
    t: f64,
    dt: f64,
    integrator: &dyn Integrator<f64>,
) -> DMatrix<f64> {
    let schema = &state.schema;
    let x = &state.mean;
    let n = schema.tangent_dim();

    // The nominal next state. Every column differences against this anchor.
    let y0 = dynamics.propagate(x, u, t, dt, integrator);

    // One step size for the whole Jacobian: a function of x, not the column.
    let eps = 1e-5 * (1.0 + x.amax());

    let mut f_matrix = DMatrix::zeros(n, n);

    for j in 0..n {
        // Bump the j-th tangent coordinate and retract onto the manifold.
        let mut delta = DVector::zeros(n);
        delta[j] = eps;
        let x_pert = schema.oplus(x.as_view(), delta.as_view());

        let y_j = dynamics.propagate(&x_pert, u, t, dt, integrator);

        // Difference in tangent space at the nominal next state.
        f_matrix.set_column(j, &(schema.ominus(y_j.as_view(), y0.as_view()) / eps));
    }

    f_matrix
}

#[cfg(test)]
mod tests {
    use super::{discrete_linearization, tangent_state_transition};
    use crate::estimation::dynamics::integrated_imu::{
        ImuInitialUncertainty, ImuProcessNoise, IntegratedImuModel,
    };
    use crate::estimation::dynamics::ErrorStateModel;
    use crate::estimation::schema::{InputSchema, StateSchema};
    use crate::kernel::integrators::{Integrator, RK4};
    use crate::prelude::EstimationDynamics;
    use crate::prelude::{AgentId, MonotonicTime};
    use crate::spatial::primitives::{Control, State};
    use crate::spatial::FrameAwareState;

    use nalgebra::{DMatrix, DVector, Vector3};
    use std::sync::Arc;

    const DT: f64 = 0.02;

    fn ins_model() -> IntegratedImuModel {
        IntegratedImuModel::new(
            AgentId::new("test_agent"),
            Vector3::new(0.0, 0.0, -9.81),
            ImuProcessNoise {
                accel_noise_var: 0.04,
                gyro_noise_var: 0.0025,
                accel_bias_var: 0.0001,
                gyro_bias_var: 0.000001,
            },
            ImuInitialUncertainty {
                pos_var: 0.5,
                vel_var: 1.0,
                ori_var: 0.02,
                accel_bias_var: 1.0,
                gyro_bias_var: 1.0,
            },
        )
    }

    /// Gravity-compensated, otherwise-still IMU input (control dim is 6).
    fn still_input() -> DVector<f64> {
        DVector::from_row_slice(&[0.0, 0.0, 9.81, 0.0, 0.0, 0.0])
    }

    /// An INS state off the identity: moving, turned, with non-zero biases, so
    /// the transition has rotation and bias coupling rather than a near-identity.
    fn moving_ins_state(model: &IntegratedImuModel) -> FrameAwareState {
        let mut state = FrameAwareState::from_schema(model.schema(), MonotonicTime(0.0));
        let mut delta = DVector::zeros(state.tangent_dim());
        delta.rows_mut(3, 3).copy_from_slice(&[1.5, -0.4, 0.1]); // velocity
        delta.rows_mut(6, 3).copy_from_slice(&[0.3, -0.2, 0.9]); // rotation
        delta.rows_mut(9, 3).copy_from_slice(&[0.05, -0.02, 0.01]); // accel bias
        delta
            .rows_mut(12, 3)
            .copy_from_slice(&[0.002, 0.001, -0.003]); // gyro bias
        state.oplus_assign(&delta);
        state
    }

    /// Claims an analytic error-state model (a deliberately wrong all-zero one)
    /// and otherwise delegates to the INS.
    #[derive(Debug)]
    struct ClaimsAnalytic(IntegratedImuModel);

    impl EstimationDynamics for ClaimsAnalytic {
        fn input_schema(&self) -> Arc<InputSchema> {
            self.0.input_schema()
        }

        fn schema(&self) -> Arc<StateSchema> {
            self.0.schema()
        }

        fn error_state_model(&self, _x: &State, _u: &Control) -> Option<ErrorStateModel> {
            let n = self.0.schema().tangent_dim();
            Some(ErrorStateModel {
                f_c: DMatrix::zeros(n, n),
                g_c: DMatrix::zeros(n, n),
                q_c: DMatrix::zeros(n, n),
            })
        }

        fn propagate(
            &self,
            x: &State,
            u: &Control,
            t: f64,
            dt: f64,
            integrator: &dyn Integrator<f64>,
        ) -> State {
            self.0.propagate(x, u, t, dt, integrator)
        }
    }

    #[test]
    fn ins_tangent_transition_is_fifteen_by_fifteen() {
        // The INS state stores 16 numbers but has only 15 tangent DOF — the SO(3)
        // block is 4 stored / 3 tangent. F is a tangent-space map, so it must be
        // 15×15 (matching the covariance), NOT the 16×16 of the storage Jacobian.
        let model = ins_model();
        let schema = model.schema();
        let state = FrameAwareState::from_schema(schema.clone(), MonotonicTime(0.0));

        let f = tangent_state_transition(&model, &state, &still_input(), 0.0, DT, &RK4);

        assert_eq!(f.nrows(), 15, "F rows = tangent dim");
        assert_eq!(f.ncols(), 15, "F cols = tangent dim");
        assert_eq!(f.nrows(), schema.tangent_dim());

        // Teeth: it is a real transition, not a zero/identity stub. Position
        // integrates velocity over the step, so ∂(next posₓ)/∂(velₓ) ≈ dt.
        assert!(
            (f[(0, 3)] - DT).abs() < 1e-6,
            "position-from-velocity coupling should be ≈ dt, got {}",
            f[(0, 3)]
        );
    }

    #[test]
    fn a_model_without_an_error_state_model_takes_the_numeric_path() {
        // The INS supplies no error-state model, so the entry point must return
        // exactly the finite-difference F and Q·dt the EKF computed inline
        // before it existed; exact equality, not a tolerance, because the
        // proving ground's regression is bit-identical across this change.
        let model = ins_model();
        let state = moving_ins_state(&model);
        let u = still_input();
        assert!(model.error_state_model(&state.mean, &u).is_none());

        let n = state.tangent_dim();
        let q = DMatrix::from_fn(
            n,
            n,
            |i, j| if i == j { 1e-3 * (i + 1) as f64 } else { 0.0 },
        );

        let step = discrete_linearization(&model, &state, &u, 0.0, DT, &RK4, &q);

        assert_eq!(
            step.f_d,
            tangent_state_transition(&model, &state, &u, 0.0, DT, &RK4)
        );
        assert_eq!(step.q_d, &q * DT);
    }

    #[test]
    fn a_supplied_error_state_model_is_not_consulted() {
        // The analytic path is not built, so a model that returns `Some` is
        // still linearised numerically: its (here deliberately all-zero) F_c
        // and Q_c must not reach the covariance step.
        let model = ClaimsAnalytic(ins_model());
        let state = moving_ins_state(&model.0);
        let u = still_input();
        let n = state.tangent_dim();
        let q = DMatrix::identity(n, n) * 1e-3;

        let claimed = discrete_linearization(&model, &state, &u, 0.0, DT, &RK4, &q);
        let numeric = discrete_linearization(&model.0, &state, &u, 0.0, DT, &RK4, &q);

        assert_eq!(claimed.f_d, numeric.f_d);
        assert_eq!(claimed.q_d, numeric.q_d);
        assert_ne!(claimed.f_d, DMatrix::zeros(n, n));
    }
}
