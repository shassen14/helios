// helios_core/src/models/controls/mod.rs
//
// ControlDynamics trait and concrete implementations.
//
// Mirrors the structure of models/estimation/dynamics/ but is intentionally
// simpler: no measurement routing, no sensor coupling.
// Analytical models that also implement EstimationDynamics share the same
// underlying math via a shared private function; the two trait impls just delegate.

pub mod ackermann;

use nalgebra::{DMatrix, DVector};

/// Dynamics model for controllers.
///
/// Mirrors `EstimationDynamics` naming (`get_control_dim`, `get_derivatives`,
/// `calculate_jacobian`) but omits filter-specific concerns:
///   - No `get_control_from_measurement` (controllers never receive raw sensor data)
///   - No `propagate` with an `Integrator` argument (controllers call `get_derivatives`
///     directly inside their own optimization or linearization loops)
///
/// **Two-impl pattern:** An analytical model like `AckermannKinematics` implements
/// both `EstimationDynamics` and `ControlDynamics` via separate `impl` blocks on the
/// same struct, both delegating to the same internal math.
pub trait ControlDynamics: Send + Sync {
    fn get_state_dim(&self) -> usize;
    fn get_control_dim(&self) -> usize;

    /// Computes the time derivative of the state vector: `x_dot = f(x, u, t)`.
    /// Mirrors `EstimationDynamics::get_derivatives`.
    fn get_derivatives(&self, x: &DVector<f64>, u: &DVector<f64>, t: f64) -> DVector<f64>;

    /// Calculates the Jacobian matrices of `f(x, u, t)`.
    /// Returns `(A, B)` where Δx_dot ≈ A·Δx + B·Δu.
    /// Mirrors `EstimationDynamics::calculate_jacobian`.
    ///
    /// Default: first-order finite differences on `get_derivatives()` with
    /// adaptive epsilon `ε = 1e-5 * (1 + |xᵢ|)` for numerical stability.
    fn calculate_jacobian(
        &self,
        x: &DVector<f64>,
        u: &DVector<f64>,
        t: f64,
    ) -> (DMatrix<f64>, DMatrix<f64>) {
        let n = self.get_state_dim();
        let m = self.get_control_dim();
        let f0 = self.get_derivatives(x, u, t);

        let mut a = DMatrix::zeros(n, n);
        for i in 0..n {
            let eps = 1e-5 * (1.0 + x[i].abs());
            let mut x_pert = x.clone();
            x_pert[i] += eps;
            let f_pert = self.get_derivatives(&x_pert, u, t);
            for j in 0..n {
                a[(j, i)] = (f_pert[j] - f0[j]) / eps;
            }
        }

        let mut b = DMatrix::zeros(n, m);
        for i in 0..m {
            let eps = 1e-5 * (1.0 + u[i].abs());
            let mut u_pert = u.clone();
            u_pert[i] += eps;
            let f_pert = self.get_derivatives(x, &u_pert, t);
            for j in 0..n {
                b[(j, i)] = (f_pert[j] - f0[j]) / eps;
            }
        }

        (a, b)
    }
}
