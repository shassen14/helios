//! Numerical linearization of dynamics into a discrete tangent-space state
//! transition matrix, used by the covariance step of Gaussian filters.

use crate::frames::FrameAwareState;
use crate::prelude::EstimationDynamics;
use crate::utils::integrators::Integrator;

use nalgebra::{DMatrix, DVector};

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
pub(crate) fn tangent_state_transition(
    dynamics: &dyn EstimationDynamics,
    state: &FrameAwareState,
    u: &DVector<f64>, // already control-sized (u_sized)
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
    use super::tangent_state_transition;
    use crate::data::primitives::FrameHandle;
    use crate::estimation::dynamics::integrated_imu::{
        ImuInitialUncertainty, ImuProcessNoise, IntegratedImuModel,
    };
    use crate::frames::FrameAwareState;
    use crate::prelude::EstimationDynamics;
    use crate::utils::integrators::RK4;

    use nalgebra::{DVector, Vector3};

    #[test]
    fn ins_tangent_transition_is_fifteen_by_fifteen() {
        // The INS state stores 16 numbers but has only 15 tangent DOF — the SO(3)
        // block is 4 stored / 3 tangent. F is a tangent-space map, so it must be
        // 15×15 (matching the covariance), NOT the 16×16 of the storage Jacobian.
        let model = IntegratedImuModel::new(
            FrameHandle(7),
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
        );
        let schema = model.schema();
        let state = FrameAwareState::from_schema(schema.clone(), 0.0);
        // Gravity-compensated, otherwise-still IMU input (control dim is 6).
        let u = DVector::from_row_slice(&[0.0, 0.0, 9.81, 0.0, 0.0, 0.0]);

        let dt = 0.02;
        let f = tangent_state_transition(&model, &state, &u, 0.0, dt, &RK4);

        assert_eq!(f.nrows(), 15, "F rows = tangent dim");
        assert_eq!(f.ncols(), 15, "F cols = tangent dim");
        assert_eq!(f.nrows(), schema.tangent_dim());

        // Teeth: it is a real transition, not a zero/identity stub. Position
        // integrates velocity over the step, so ∂(next posₓ)/∂(velₓ) ≈ dt.
        assert!(
            (f[(0, 3)] - dt).abs() < 1e-6,
            "position-from-velocity coupling should be ≈ dt, got {}",
            f[(0, 3)]
        );
    }
}
