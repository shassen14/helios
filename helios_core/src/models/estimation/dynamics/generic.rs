// helios_core/src/models/impl_dynamics/generic.rs

use crate::{
    frames::{FrameId, StateVariable},
    models::dynamics::Dynamics,
    types::FrameHandle,
};
use nalgebra::{DMatrix, DVector};

// --- Constant Acceleration Model ---
// Assumes the object continues to move with a slowly changing acceleration.
// The state includes position, velocity, and acceleration.
#[derive(Debug, Default, Clone)]
pub struct ConstantAccelerationModel {
    /// The handle of the agent this model is for.
    pub agent_handle: FrameHandle,
}

impl Dynamics for ConstantAccelerationModel {
    fn get_state_layout(&self) -> Vec<StateVariable> {
        let body_frame = FrameId::Body(self.agent_handle);
        vec![
            // We define the state in the most convenient frames.
            // Position and Velocity are in the global World frame.
            StateVariable::Px(FrameId::World),
            StateVariable::Py(FrameId::World),
            StateVariable::Pz(FrameId::World),
            StateVariable::Vx(FrameId::World),
            StateVariable::Vy(FrameId::World),
            StateVariable::Vz(FrameId::World),
            // Acceleration is most naturally felt and measured in the Body frame.
            StateVariable::Ax(body_frame.clone()),
            StateVariable::Ay(body_frame.clone()),
            StateVariable::Az(body_frame.clone()),
        ]
    }

    fn get_control_dim(&self) -> usize {
        // This simple model doesn't accept external control inputs.
        // It assumes acceleration changes are random noise (modeled by Q).
        0
    }

    fn get_derivatives(&self, x: &DVector<f64>, _u: &DVector<f64>, _t: f64) -> DVector<f64> {
        let mut x_dot = DVector::zeros(9);

        // d(Position)/dt = Velocity
        x_dot[0] = x[3]; // Px_dot = Vx
        x_dot[1] = x[4]; // Py_dot = Vy
        x_dot[2] = x[5]; // Pz_dot = Vz

        // d(Velocity)/dt = Acceleration
        // NOTE: This is a key simplifying assumption. It assumes body frame == world frame
        // for the purpose of integrating acceleration. A more complex model would
        // require the full orientation state to rotate the body-frame acceleration
        // into the world frame before adding it to world-frame velocity.
        x_dot[3] = x[6]; // Vx_dot ≈ Ax
        x_dot[4] = x[7]; // Vy_dot ≈ Ay
        x_dot[5] = x[8]; // Vz_dot ≈ Az

        // d(Acceleration)/dt = 0 (The "constant" part of the model)
        // The process noise matrix Q will model the uncertainty here.
        x_dot[6] = 0.0;
        x_dot[7] = 0.0;
        x_dot[8] = 0.0;

        x_dot
    }

    fn calculate_jacobian(
        &self,
        _x: &DVector<f64>,
        _u: &DVector<f64>,
        _t: f64,
    ) -> (DMatrix<f64>, DMatrix<f64>) {
        let mut f_jac = DMatrix::zeros(9, 9);

        // d(Px_dot)/d(Vx) = 1
        f_jac[(0, 3)] = 1.0;
        f_jac[(1, 4)] = 1.0;
        f_jac[(2, 5)] = 1.0;

        // d(Vx_dot)/d(Ax) = 1
        f_jac[(3, 6)] = 1.0;
        f_jac[(4, 7)] = 1.0;
        f_jac[(5, 8)] = 1.0;

        (f_jac, DMatrix::zeros(9, 0)) // No control inputs, so B matrix is empty.
    }
}
