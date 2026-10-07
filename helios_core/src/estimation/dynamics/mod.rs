//! `EstimationDynamics` trait and concrete dynamics models for state estimators.
//!
//! Each model implements `propagate(x, u, t, dt)`, the discrete step the
//! filters call. A model with a continuous `ẋ = f(x, u, t)` also implements
//! [`ContinuousDynamics`] and builds `propagate` from [`integrate_derivatives`]
//! (prefer RK4). A model may supply its own [`ErrorStateModel`] for the
//! filters' covariance step. Concrete models: `integrated_imu`.

pub mod error_state;
pub mod integrated_imu;

pub use error_state::ErrorStateModel;

use crate::estimation::schema::{InputSchema, StateSchema};
use crate::kernel::integrators::Integrator;
use crate::spatial::primitives::{Control, State};
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

    /// The continuous error-state model `(F_c, G_c, Q_c)` linearising this
    /// model about the stored state `x` under input `u`, or `None` to leave the
    /// filter's numeric linearisation in charge.
    ///
    /// Optional; the default returns `None`. A model overrides it to give an
    /// analytic Jacobian, or process noise that depends on the state or input
    /// (odometry noise that grows with distance travelled). It is sized to this
    /// model's own schema, like `propagate`, and is written in that schema's
    /// perturbation convention; see [`ErrorStateModel`]. It takes no time:
    /// estimation dynamics are time-invariant.
    ///
    /// The filters' linearisation does not read it yet: every model is
    /// linearised numerically, so returning `Some` changes no estimate.
    fn error_state_model(&self, _x: &State, _u: &Control) -> Option<ErrorStateModel> {
        None
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

/// A dynamics model with a continuous-time form `ẋ = f(x, u, t)`.
///
/// Separate from [`EstimationDynamics`] because a discrete model (an odometry
/// increment, `x ⊞ Δ`) has a step but no derivative, and the filters only ever
/// call the step. A continuous model implements both and builds its
/// `propagate` from [`integrate_derivatives`].
pub trait ContinuousDynamics: EstimationDynamics {
    /// Computes the time derivative of the state vector: `x_dot = f(x, u, t)`.
    /// The result has `x.nrows()` rows, zero in any rows past this model's
    /// schema; see `propagate` on augmented states.
    ///
    /// # Arguments
    /// * `x`: Current state vector (`State`, which is `DVector<f64>`).
    /// * `u`: Current control input vector (`Control`, which is `DVector<f64>`).
    /// * `t`: Current simulation time (`Time`, which is `f64`).
    ///
    /// # Returns
    /// The time derivative of the state vector (`State`).
    fn derivatives(&self, x: &State, u: &Control, t: f64) -> State;
}

/// Integrates `dynamics.derivatives` over one step of `dt` with `u` held
/// constant: the `propagate` of a model with only a continuous `ẋ = f`.
///
/// Componentwise, so it is exact only for flat blocks. A model with a curved
/// block (an orientation quaternion) must advance that block by its own
/// retraction and use this for the rest, as `IntegratedImuModel` does.
pub fn integrate_derivatives<D: ContinuousDynamics + ?Sized>(
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
