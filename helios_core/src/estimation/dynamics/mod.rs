//! `EstimationDynamics` trait and concrete dynamics models for state estimators.
//!
//! Each model implements `propagate(x, u, t, dt)`, the discrete step the
//! filters call, and `derivatives(x, u, t)`, the continuous-time `ẋ = f(x, u, t)`.
//! A model whose step is a plain integration of `derivatives` implements
//! `propagate` with [`integrate_derivatives`] (prefer RK4). Concrete models:
//! `integrated_imu`.

pub mod integrated_imu;

use crate::estimation::schema::{InputSchema, StateSchema};
use crate::kernel::integrators::Integrator;
use crate::spatial::primitives::{Control, State};
use nalgebra::DMatrix;
use std::fmt::Debug;
use std::sync::Arc;

/// A trait for dynamics models used within state estimators.
///
/// This model's primary responsibility is to propagate a state vector forward
/// in time (`propagate`). The filters linearize it themselves by
/// finite-differencing `propagate`, so a model needs no Jacobian to be filtered.
pub trait EstimationDynamics: Debug + Send + Sync {
    /// The named layout of the input vector `u`: which quantity each row holds,
    /// in which frame and convention. The input builder must supply exactly
    /// this, checked by `check_input_agreement`; `u.nrows()` is its `dim()`. A
    /// model that takes no input returns an empty schema.
    fn input_schema(&self) -> Arc<InputSchema>;

    fn schema(&self) -> Arc<StateSchema>;

    /// Computes the time derivative of the state vector: `x_dot = f(x, u, t)`.
    /// This is the core function describing the system's behavior. The result
    /// has `x.nrows()` rows, zero in any rows past this model's schema; see
    /// `propagate` on augmented states.
    ///
    /// # Arguments
    /// * `x`: Current state vector (`State`, which is `DVector<f64>`).
    /// * `u`: Current control input vector (`Control`, which is `DVector<f64>`).
    /// * `t`: Current simulation time (`Time`, which is `f64`).
    ///
    /// # Returns
    /// The time derivative of the state vector (`State`).
    fn derivatives(&self, x: &State, u: &Control, t: f64) -> State;

    /// (Optional) Calculates the Jacobian matrices of the dynamics function `f(x, u, t)`.
    /// Jacobian A = ∂f/∂x (how state derivatives change with state)
    /// Jacobian B = ∂f/∂u (how state derivatives change with control input)
    /// Useful for linear controllers (LQR — its `B` block) and stability analysis.
    /// The default implementation approximates both by numerical finite
    /// differencing; models with an analytic Jacobian may override it.
    ///
    /// **This is NOT the estimator's linearization.** It is a *continuous*,
    /// *storage-space* `A`/`B` pair. The Gaussian filters propagate covariance
    /// with a *discrete*, *tangent-space* state transition `F` computed by
    /// `filters::linearization::tangent_state_transition`, which finite-differences
    /// `propagate` on the manifold and never calls this method. The two differ in
    /// both time-discretization and coordinate space
    /// (storage vs tangent — a curved block stores more numbers than it has DOF),
    /// so this `A` must not be wired into an EKF/UKF as its `F`.
    ///
    /// # Arguments
    /// * `x`: State vector (`State`) at which to linearize.
    /// * `u`: Control input vector (`Control`) at which to linearize.
    /// * `t`: Simulation time (`Time`).
    ///
    /// # Returns
    /// A tuple `(A, B)` where `A` is an NxN matrix and `B` is an NxM matrix (N=state dim, M=control dim).
    fn jacobian(&self, x: &State, u: &Control, t: f64) -> (DMatrix<f64>, DMatrix<f64>) {
        let n = x.nrows();
        let m = self.input_schema().dim();
        let f0 = self.derivatives(x, u, t);

        let mut a = DMatrix::zeros(n, n);
        for i in 0..n {
            let eps = 1e-5 * (1.0 + x[i].abs());
            let mut x_pert = x.clone();
            x_pert[i] += eps;
            let f_pert = self.derivatives(&x_pert, u, t);
            for j in 0..n {
                a[(j, i)] = (f_pert[j] - f0[j]) / eps;
            }
        }

        let mut b = DMatrix::zeros(n, m);
        for i in 0..m {
            let eps = 1e-5 * (1.0 + u[i].abs());
            let mut u_pert = u.clone();
            u_pert[i] += eps;
            let f_pert = self.derivatives(x, &u_pert, t);
            for j in 0..n {
                b[(j, i)] = (f_pert[j] - f0[j]) / eps;
            }
        }

        (a, b)
    }

    /// Advances the state by one step of `dt`: the discrete process the filters
    /// propagate the mean through and finite-difference for `F`.
    ///
    /// Required, so a discrete model (an odometry increment, `x ⊞ Δ`) states its
    /// step directly. A model with only a continuous `ẋ = f` implements it with
    /// [`integrate_derivatives`].
    ///
    /// Preconditions, checked by the filter before it calls this: `dt > 0`, and
    /// `u.nrows() == self.input_schema().dim()`. An implementation may index `u`
    /// at its input schema's rows without re-checking.
    ///
    /// `x` may be longer than this model's own schema: a filter with
    /// augmentation blocks (a sensor bias, say) appends them after the model's
    /// blocks and passes the whole state. The implementation must return a
    /// vector of `x.nrows()` with the appended rows unchanged, which gives each
    /// augmentation block an identity transition. It reads and writes only its
    /// own blocks at its schema's offsets, and sizes anything it allocates from
    /// `x`, never from its schema (`integrate_derivatives` does this when
    /// `derivatives` sizes `ẋ` from `x` and leaves the appended rows zero).
    ///
    /// # Arguments
    /// * `x`: Current state vector (`State`).
    /// * `u`: Current control input vector (`Control`). Assumed constant over `dt`.
    /// * `t`: Time at the start of the step.
    /// * `dt`: Step length in seconds.
    /// * `integrator`: The integrator to use for any continuous part (e.g. `RK4`).
    ///
    /// # Returns
    /// The state vector at `t + dt` (`State`).
    fn propagate(
        &self,
        x: &State,
        u: &Control,
        t: f64,
        dt: f64,
        integrator: &dyn Integrator<f64>,
    ) -> State;
}

/// Integrates `dynamics.derivatives` over one step of `dt` with `u` held
/// constant: the `propagate` of a model with only a continuous `ẋ = f`.
///
/// Componentwise, so it is exact only for flat blocks. A model with a curved
/// block (an orientation quaternion) must advance that block by its own
/// retraction and use this for the rest, as `IntegratedImuModel` does.
pub fn integrate_derivatives<D: EstimationDynamics + ?Sized>(
    dynamics: &D,
    x: &State,
    u: &Control,
    t: f64,
    dt: f64,
    integrator: &dyn Integrator<f64>,
) -> State {
    let func = |func_x: &State, func_t: f64| -> State { dynamics.derivatives(func_x, u, func_t) };
    integrator.step(&func, x, t, t + dt)
}
