use crate::simulation::utils::integrators::Integrator;
use bevy::prelude::Component;
use nalgebra::{DMatrix, DVector};
use std::fmt::Debug;

pub type State = DVector<f64>;
pub type Control = DVector<f64>;

// --- State Definition ---

/// An enum representing all possible variables that can be part of a state vector.
/// This acts as a "dictionary" for state variables.
/// `#[repr(usize)]` is not strictly needed with our new `get_state_layout` method,
/// but it can be useful for debugging.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StateVariable {
    // --- Cartesian Position (World Frame) ---
    Px, // East or World X
    Py, // North or World Y
    Pz, // Up or World Z

    // --- Linear Velocity (World Frame) ---
    Vx,
    Vy,
    Vz,

    // --- Linear Acceleration (World Frame) ---
    Ax,
    Ay,
    Az,

    // --- Orientation (as Euler angles for simplicity in some models) ---
    Roll,  // Rotation around body's forward axis
    Pitch, // Rotation around body's side-to-side axis
    Yaw,   // Rotation around body's vertical axis

    // --- Orientation (as a Quaternion for robust 3D rotation) ---
    // Note: A 4-element quaternion is needed to represent 3D orientation.
    // The EKF would typically normalize this to ensure it remains a unit quaternion.
    Qx,
    Qy,
    Qz,
    Qw,

    // --- Angular Velocity (Body Frame) ---
    Wx, // Roll rate
    Wy, // Pitch rate
    Wz, // Yaw rate

    // --- Angular Acceleration (Body Frame) ---
    AlphaX, // Roll acceleration
    AlphaY, // Pitch acceleration
    AlphaZ, // Yaw acceleration
}

/// # Dynamics Trait
///
/// Represents the physics or kinematic model of an entity (robot, obstacle, etc.).
/// Defines how the entity's state evolves over time based on control inputs.
/// Implementations should be `Send + Sync` to be safely used across threads by Bevy.
pub trait Dynamics: Debug + Send + Sync {
    /// Returns the complete layout of the state vector for this specific model.
    /// The order of this Vec defines the indices for the state vector `x`.
    /// For example: `vec![StateVariable::Px, StateVariable::Py, StateVariable::Yaw]`
    fn get_state_layout(&self) -> Vec<StateVariable>;

    /// Returns the total number of states (the length of the state vector `x`).
    /// This is provided as a convenience.
    fn get_state_dim(&self) -> usize {
        self.get_state_layout().len()
    }

    /// Returns the number of dimensions in the control input vector `u`.
    fn get_control_dim(&self) -> usize;

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

    /// (Optional) Calculates the feedforward control input required to achieve a desired state derivative.
    /// This attempts to solve `x_dot_desired = f(x, u_ff, t)` for `u_ff`.
    /// This is useful for feedforward control components. Not all models may have a
    /// straightforward or unique inverse.
    ///
    /// # Arguments
    /// * `x`: Current state vector (`State`).
    /// * `x_dot_desired`: The desired time derivative of the state vector (`State`).
    /// * `t`: Current simulation time (`Time`).
    ///
    /// # Returns
    /// * `Some(Control)`: The calculated feedforward control input `u_ff` if solvable.
    /// * `None`: If the feedforward input cannot be determined (e.g., model is not invertible,
    ///   no solution exists, or the feature is not implemented).
    fn calculate_feedforward_input(
        &self,
        x: &State,
        x_dot_desired: &State,
        t: f64,
    ) -> Option<Control> {
        let _ = t;
        let _ = x_dot_desired;
        let _ = x;
        // Default implementation: Feedforward calculation is not supported by default.
        None
    }
}

#[derive(Component)]
pub struct DynamicsModel(pub Box<dyn Dynamics>);

// TODO: Delete later
/// A simple placeholder implementation for testing.
#[derive(Debug)]
pub struct SimpleCarDynamics;

impl Dynamics for SimpleCarDynamics {
    fn get_state_layout(&self) -> Vec<StateVariable> {
        vec![
            StateVariable::Px,
            StateVariable::Py,
            StateVariable::Pz,
            StateVariable::Vx,
            StateVariable::Vy,
            StateVariable::Vz,
            StateVariable::Yaw,
            StateVariable::Pitch,
            StateVariable::Roll,
        ]
    }

    fn get_control_dim(&self) -> usize {
        2 // e.g., throttle, steering_angle
    }

    /// This function would contain the actual physics equations.
    /// For now, it just returns a zero vector.
    fn get_derivatives(&self, _x: &DVector<f64>, _u: &DVector<f64>, _t: f64) -> DVector<f64> {
        DVector::zeros(self.get_state_dim())
    }
}
