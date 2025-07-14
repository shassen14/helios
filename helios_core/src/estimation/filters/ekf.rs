// helios_core/src/estimation/filters/ekf.rs

use std::any::Any;
use std::collections::HashMap;

use crate::estimation::{FilterContext, StateEstimator};
use crate::frames::FrameAwareState;
use crate::messages::{MeasurementData, ModuleInput};
use crate::models::measurement::Measurement;
use crate::prelude::{Dynamics, MeasurementMessage};
// The helper trait for sensors
use crate::types::{Control, FrameHandle};
use crate::utils::integrators::RK4;
use nalgebra::{DMatrix, DVector}; // Assuming you have this

/// A concrete implementation of an Extended Kalman Filter.
pub struct ExtendedKalmanFilter {
    /// The current state of the filter (x, P, t).
    state: FrameAwareState,
    /// The process noise covariance matrix (Q), modeling uncertainty in the dynamics.
    process_noise_q: DMatrix<f64>,

    dynamics: Box<dyn Dynamics>,

    // A map from a sensor's handle to its specific measurement model.
    // This allows the EKF to know how to handle data from different sensors.
    // This map is populated at initialization.
    measurement_models: std::collections::HashMap<crate::types::FrameHandle, Box<dyn Measurement>>,
}

impl ExtendedKalmanFilter {
    /// Creates a new EKF instance.
    pub fn new(
        initial_state: FrameAwareState,
        process_noise_q: DMatrix<f64>,
        dynamics: Box<dyn Dynamics>,
        measurement_models: HashMap<FrameHandle, Box<dyn Measurement>>,
    ) -> Self {
        // Ensure Q has the correct dimensions.
        assert_eq!(initial_state.dim(), process_noise_q.nrows());
        assert_eq!(initial_state.dim(), process_noise_q.ncols());

        Self {
            state: initial_state,
            process_noise_q,
            dynamics,
            measurement_models,
        }
    }

    // --- Private Helper Methods for the EKF Algorithm ---

    /// The internal "predict" step. Advances the state in time.
    fn predict(&mut self, dt: f64, _context: &FilterContext) {
        if dt <= 0.0 {
            return;
        }

        // We need a dynamics model to predict.
        let dynamics = &self.dynamics;

        let x = &self.state.vector;
        let p = &self.state.covariance;
        let t = self.state.last_update_timestamp;

        // A zero-vector control input, as the EKF doesn't use it directly for prediction
        // in this simplified model. A more advanced one might.
        let u = Control::zeros(dynamics.get_control_dim());

        // 1. Predict the state vector using the dynamics model and an integrator.
        let x_pred = dynamics.propagate(x, &u, t, dt, &RK4);

        // 2. Linearize the dynamics by calculating the Jacobian F.
        let (f_jac, _b_jac) = dynamics.calculate_jacobian(x, &u, t);

        // 3. Predict the covariance matrix: P_k+1 = F * P_k * F^T + Q
        let p_pred = &f_jac * p * f_jac.transpose() + &self.process_noise_q * dt; // Scale Q by dt

        // 4. Update the internal state.
        self.state.vector = x_pred;
        self.state.covariance = p_pred;
        self.state.last_update_timestamp += dt;
    }

    /// The internal "update" step. Fuses a measurement.
    fn update(&mut self, event: &MeasurementMessage, context: &FilterContext) {
        // Look up the correct measurement model for the sensor that sent this event.
        let model = match self.measurement_models.get(&event.sensor_handle) {
            Some(m) => m,
            None => {
                // We don't have a model for this sensor, so we ignore its data.
                return;
            }
        };

        // Get the actual measurement vector `z` from the event data.
        let z: DVector<f64> = match &event.data {
            // This is where we convert from the specific static type to a generic DVector.
            MeasurementData::Imu6Dof(v) => DVector::from_row_slice(v.as_slice()),
            MeasurementData::Gps(v) => DVector::from_row_slice(v.as_slice()),
            // The EKF can choose to ignore data types it doesn't understand.
            _ => return,
        };

        // --- Standard EKF Update Equations ---

        // 1. Predict the measurement from our current state: z_hat = h(x)
        let z_pred = model.predict_measurement(&self.state, context.tf.unwrap());

        println!("z: {}", z.transpose());
        println!("z_pred: {}", z_pred.transpose());

        // 2. Calculate the measurement Jacobian H.
        let h_jac = model.calculate_jacobian(&self.state, context.tf.unwrap());

        // 3. Get the measurement noise covariance R.
        let r_mat = model.get_r();

        // 4. Calculate the innovation (y), innovation covariance (S), and Kalman gain (K).
        let y = z - z_pred;
        let s = &h_jac * &self.state.covariance * h_jac.transpose() + r_mat;

        if let Some(s_inv) = s.try_inverse() {
            let k_gain = &self.state.covariance * h_jac.transpose() * s_inv;

            // 5. Update the state vector and covariance matrix.
            self.state.vector += &k_gain * y;
            let i = DMatrix::<f64>::identity(self.state.dim(), self.state.dim());
            self.state.covariance = (i - k_gain * h_jac) * &self.state.covariance;
        }
        // If S is not invertible, the measurement is likely redundant or problematic.
        // We skip the update to maintain filter stability.
    }
}

// --- The Public Trait Implementation ---
impl StateEstimator for ExtendedKalmanFilter {
    fn process(&mut self, input: &ModuleInput, context: &FilterContext) {
        match input {
            ModuleInput::Measurement { message } => {
                // A measurement arrived. We must handle the Predict -> Update sequence.

                // 1. Calculate the time delta since our last update.
                let dt = message.timestamp - self.state.last_update_timestamp;

                // 2. PREDICT: Advance the state to the exact time of the measurement.
                self.predict(dt, context);

                // 3. UPDATE: Now that we're at the correct time, fuse the measurement.
                self.update(message, context);
            }
            ModuleInput::TimeStep { dt, current_time } => {
                // This is a "heartbeat" input. If no measurements arrived,
                // we still need to predict forward to keep the state current.
                // We calculate how much time has passed since our last known update.
                let time_since_last_update = current_time - self.state.last_update_timestamp;
                if time_since_last_update > *dt {
                    // We seem to have missed a measurement, but we predict with what we have
                    self.predict(*dt, context);
                }
                // If a measurement already advanced us past this timestep, do nothing.
            }
            // EKF doesn't use these, but a different estimator might.
            ModuleInput::Control { .. } => {}
            ModuleInput::PoseUpdate { .. } => {}
        }
    }

    fn get_state(&self) -> &FrameAwareState {
        &self.state
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}
