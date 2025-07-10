// helios_core/src/estimation/ekf.rs

use crate::abstractions::{Dynamics, Measurement, TfProvider};
use crate::frames::FrameAwareState;
use crate::utils::integrators::RK4;
use nalgebra::{DMatrix, DVector}; // Assuming RK4 is in helios_core::utils

/// A container for parameters needed by the EKF prediction step.
pub struct EkfUpdateParams<'a> {
    pub dynamics: &'a dyn Dynamics,
    pub process_noise_q: &'a DMatrix<f64>,
}

/// A container for parameters needed by the EKF update step.
pub struct EkfMeasurementParams<'a> {
    pub model: &'a dyn Measurement,
    pub z: &'a DVector<f64>,
    pub context: &'a dyn TfProvider,
}

/// PURE FUNCTION: Performs one EKF prediction step.
/// Takes a state and returns the new, predicted state. It has no side effects.
pub fn ekf_predict(
    current_state: &FrameAwareState,
    params: &EkfUpdateParams,
    dt: f64,
) -> FrameAwareState {
    let u = DVector::zeros(params.dynamics.get_control_dim());
    let t = current_state.last_update_timestamp;

    // Predict state vector using the dynamics model
    let new_x = params
        .dynamics
        .propagate(&current_state.vector, &u, t, dt, &RK4);

    // Predict covariance
    let (f_jac, _) = params
        .dynamics
        .calculate_jacobian(&current_state.vector, &u, t);
    let new_p = &f_jac * &current_state.covariance * f_jac.transpose() + params.process_noise_q;

    FrameAwareState {
        layout: current_state.layout.clone(),
        vector: new_x,
        covariance: new_p,
        last_update_timestamp: t + dt,
    }
}

/// PURE FUNCTION: Performs one EKF measurement update step.
/// Takes a state and returns the new, corrected state. It has no side effects.
pub fn ekf_update(
    predicted_state: &FrameAwareState,
    params: &EkfMeasurementParams,
) -> FrameAwareState {
    // The model's methods are responsible for using the context to perform transforms.
    let h_jacobian = params
        .model
        .calculate_jacobian(predicted_state, params.context);
    let z_pred = params
        .model
        .predict_measurement(predicted_state, params.context);
    let r_matrix = params.model.get_r();

    // Standard EKF update equations
    let p_priori = &predicted_state.covariance;
    let x_priori = &predicted_state.vector;
    let y = params.z - z_pred; // Innovation

    let s = &h_jacobian * p_priori * h_jacobian.transpose() + r_matrix;

    if let Some(s_inv) = s.try_inverse() {
        let k_gain = p_priori * h_jacobian.transpose() * s_inv;

        let new_x = x_priori + &k_gain * y;
        let i_kh = DMatrix::<f64>::identity(predicted_state.dim(), predicted_state.dim())
            - &k_gain * &h_jacobian;
        let new_p = i_kh * p_priori;

        return FrameAwareState {
            layout: predicted_state.layout.clone(),
            vector: new_x,
            covariance: new_p,
            last_update_timestamp: predicted_state.last_update_timestamp,
        };
    }

    // If matrix inversion fails, return the predicted state unchanged.
    predicted_state.clone()
}
