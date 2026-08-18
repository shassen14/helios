//! Numerical linearization of dynamics into a discrete tangent-space state
//! transition matrix, used by the covariance step of Gaussian filters.

use crate::{prelude::EstimationDynamics, utils::integrators::Integrator};

use nalgebra::{DMatrix, DVector};

/// The discrete tangent-space state-transition matrix `F` (tangent × tangent)
/// linearizing `dynamics` about `x` over the step `dt`.
///
/// This is the **discrete** transition, not a continuous `A`. Because
/// `propagate` already folds in `dt`, the result is `F ≈ I + A·dt` on its own —
/// the caller must **not** add identity.
///
/// Each column is finite-differenced on the manifold: perturb `x` along tangent
/// basis vector `j` with `oplus`, propagate, then difference the result against
/// the unperturbed next state `y0` through `ominus`. Perturbing in tangent space
/// (rather than nudging stored components) is what keeps the rotation block's
/// 3-DOF error consistent with its 4-component storage; a raw component bump
/// would walk the quaternion off the unit sphere and mis-scale that block. The
/// step size follows the crate's adaptive rule `ε = 1e-5·(1 + ‖x‖∞)`.
pub(crate) fn tangent_state_transition(
    dynamics: &dyn EstimationDynamics,
    x: &DVector<f64>, // current storage-space mean
    u: &DVector<f64>, // already control-sized (u_sized)
    t: f64,
    dt: f64,
    integrator: &dyn Integrator<f64>,
) -> DMatrix<f64> {
    let schema = dynamics.schema();
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
