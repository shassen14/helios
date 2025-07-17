// helios_core/src/models/dynamics/mod.rs

use crate::prelude::MeasurementData;
use crate::types::{Control, State};
use crate::utils::integrators::Integrator;
use nalgebra::{DMatrix, DVector};
use std::fmt::Debug;

/// A trait for dynamics models used within state estimators.
///
/// This model's primary responsibilities are to propagate a state vector forward
/// in time (`get_derivatives`) and to provide the necessary Jacobians for
/// linearizing the system, which is essential for filters like the EKF.
pub trait EstimationDynamics: Debug + Send + Sync {
    /// Returns the number of dimensions in the control input vector `u`.
    fn get_control_dim(&self) -> usize;

    /// **The key routing method.** If this model uses the given sensor data as a
    /// driving input, it returns the corresponding control vector `u`.
    /// Otherwise, it returns `None`.
    fn get_control_from_measurement(&self, data: &MeasurementData) -> Option<DVector<f64>>;

    /// Computes the time derivative of the state vector: `x_dot = f(x, u, t)`.
    /// This is the core function describing the system's behavior.
    ///
    /// # Arguments
    /// * `x`: Current state vector (`State`, which is `DVector<f64>`).
    /// * `u`: Current control input vector (`Control`, which is `DVector<f64>`).
    /// * `t`: Current simulation time (`Time`, which is `f64`).
    ///
    /// # Returns
    /// The time derivative of the state vector (`State`).
    fn get_derivatives(&self, x: &State, u: &Control, t: f64) -> State;

    /// (Optional) Calculates the Jacobian matrices of the dynamics function `f(x, u, t)`.
    /// Jacobian A = ∂f/∂x (how state derivatives change with state)
    /// Jacobian B = ∂f/∂u (how state derivatives change with control input)
    /// Useful for linear controllers (LQR), Kalman Filters (EKF), and stability analysis.
    /// Provides a default implementation that panics, forcing implementers to override if needed.
    ///
    /// # Arguments
    /// * `x`: State vector (`State`) at which to linearize.
    /// * `u`: Control input vector (`Control`) at which to linearize.
    /// * `t`: Simulation time (`Time`).
    ///
    /// # Returns
    /// A tuple `(A, B)` where `A` is an NxN matrix and `B` is an NxM matrix (N=state dim, M=control dim).
    fn calculate_jacobian(&self, x: &State, u: &Control, t: f64) -> (DMatrix<f64>, DMatrix<f64>) {
        let _ = t;
        let _ = u;
        let _ = x;
        // Default implementation: Indicate that Jacobians are not implemented for this model.
        // Consider returning Option<(...)> or Result<(...), Error> in a production setting.
        unimplemented!("Jacobian calculation not implemented for this dynamics model.");
    }

    /// Propagates the state forward in time using a numerical integrator.
    /// This method provides a default implementation using the `Integrator` trait.
    /// Specific dynamics models *could* override this if they have an analytical solution
    /// or a specialized integration scheme.
    ///
    /// # Arguments
    /// * `x`: Current state vector (`State`).
    /// * `u`: Current control input vector (`Control`). Assumed constant over `dt`.
    /// * `t`: Current simulation time (`Time`).
    /// * `dt`: Time step duration (`Time`). Must be non-negative.
    /// * `integrator`: A reference to an object implementing the `Integrator` trait (e.g., `RK4`).
    ///
    /// # Returns
    /// The estimated state vector at time `t + dt` (`State`).
    fn propagate(
        &self,
        x: &State,
        u: &Control,
        t: f64,
        dt: f64,
        integrator: &dyn Integrator<f64>,
    ) -> State {
        assert!(dt >= 0.0, "Dynamics::propagate: dt cannot be negative");

        // Ensure control input matches expected dimensions, providing zeros if not.
        // This prevents panics if the controller provides an incorrectly sized vector.
        let u_actual = if u.nrows() == self.get_control_dim() {
            u
        } else {
            // Log a warning or error here in a real application
            // eprintln!("Warning: Control input dimension mismatch for Dynamics propagation. Expected {}, got {}. Using zeros.", self.get_control_dim(), u.nrows());
            // Consider thread-safe logging if needed.
            thread_local! {
                static ZERO_CONTROL: std::cell::RefCell<Control> = std::cell::RefCell::new(Control::zeros(0));
            }
            &ZERO_CONTROL.with(|zc| {
                let mut zc_mut = zc.borrow_mut();
                if zc_mut.nrows() != self.get_control_dim() {
                    *zc_mut = Control::zeros(self.get_control_dim());
                }
                zc_mut.clone() // Borrow the correctly sized zero vector
            })
        };

        // Define the closure f(x, t) for the integrator, capturing the current control input 'u'.
        let func = |func_x: &State, func_t: f64| -> State {
            self.get_derivatives(func_x, u_actual, func_t)
        };

        // Perform the integration step.
        integrator.step(&func, x, t, t + dt)
    }
}

// pub mod ackermann;
// pub mod generic;
pub mod integrated_imu;
