use nalgebra::{DMatrix, DVector, Vector3};
use std::any::Any;
use std::fmt::Debug;

// --- Core Library Imports ---
use crate::frames::{FrameAwareState, FrameId, StateVariable};
use crate::messages::{MeasurementData, MeasurementMessage};
use crate::models::estimation::measurement::Measurement;
use crate::types::TfProvider;

/// A measurement model for a standard GPS sensor that provides 3D position.
///
/// This model relates a 3D position measurement (in the ENU world frame)
/// to the position states (Px, Py, Pz) of the filter's state vector.
#[derive(Debug, Clone)]
pub struct GpsPositionModel {
    /// The 3x3 measurement noise covariance matrix, R.
    /// This matrix represents the uncertainty of the GPS reading itself,
    /// with variances for East, North, and Up on the diagonal.
    pub r_matrix: DMatrix<f64>,

    /// The physical location of the GPS antenna relative to the body's origin,
    /// expressed in the body's coordinate frame. This is a constant.
    pub antenna_offset_body: Vector3<f64>,
}

impl Measurement for GpsPositionModel {
    /// Describes the semantic layout of the measurement vector `z`.
    /// For this model, it's a 3D position in the world frame.
    fn get_measurement_layout(&self) -> Vec<StateVariable> {
        let world_frame = FrameId::World;
        vec![
            StateVariable::Px(world_frame.clone()),
            StateVariable::Py(world_frame.clone()),
            StateVariable::Pz(world_frame.clone()),
        ]
    }

    fn get_measurement_vector(&self, data: &MeasurementData) -> Option<DVector<f64>> {
        if let MeasurementData::GpsPosition(vec) = data {
            Some(DVector::from_row_slice(vec.as_slice()))
        } else {
            None // I am a GPS model, I only handle GpsPosition data.
        }
    }

    /// Predicts the ideal measurement `z_pred` from the filter's state.
    /// It returns `Some` only if the input message contains `GpsPosition` data.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        message: &MeasurementMessage,
        _tf: &dyn TfProvider,
    ) -> Option<DVector<f64>> {
        // This model only cares about `GpsPosition` data. It ignores everything else.
        if !matches!(&message.data, MeasurementData::GpsPosition(_)) {
            return None;
        }

        // --- 1. Get the filter's current estimate of the BODY's state ---
        let body_position_world = match filter_state.get_vector3(&StateVariable::Px(FrameId::World))
        {
            Some(pos) => pos,
            None => return None, // Can't predict if state doesn't have position
        };
        let body_orientation_world = filter_state.get_orientation().unwrap_or_default();

        // --- 2. Calculate the Lever Arm Effect ---
        // We have the antenna's position relative to the body, in the body frame.
        // We need to rotate this offset vector into the world frame.
        let antenna_offset_world = body_orientation_world * self.antenna_offset_body;

        // --- 3. Calculate the final predicted antenna position ---
        // The predicted GPS measurement is the body's position plus the rotated offset.
        let predicted_antenna_position_world = body_position_world + antenna_offset_world;

        Some(DVector::from_row_slice(
            predicted_antenna_position_world.as_slice(),
        ))
    }

    /// Calculates the measurement Jacobian, H.
    /// H maps a change in the full state vector to a change in the predicted measurement.
    /// For this model, H will be a 3x16 matrix.
    /// Calculates the measurement Jacobian, H = ∂h/∂x.
    ///
    /// Because the predicted measurement `z_pred = P_world + R(q) * r_body` now
    /// depends on the orientation `q`, this Jacobian will have non-zero entries
    /// for both the position and orientation states.
    ///
    /// We use numerical differentiation (finite differencing) for a robust and
    /// easy-to-maintain implementation.
    fn calculate_jacobian(
        &self,
        filter_state: &FrameAwareState,
        tf: &dyn TfProvider,
    ) -> DMatrix<f64> {
        let state_dim = filter_state.dim();
        let measurement_dim = 3; // 3D position
        let mut h_jac = DMatrix::zeros(measurement_dim, state_dim);

        // A small perturbation value for the finite difference calculation.
        let epsilon = 1e-8;

        // We need a dummy message to pass to predict_measurement.
        // The data variant must be one this model accepts (`GpsPosition`).
        let dummy_message = MeasurementMessage {
            agent_handle: Default::default(),
            sensor_handle: Default::default(),
            timestamp: 0.0,
            data: MeasurementData::GpsPosition(Default::default()),
        };

        // 1. Calculate the baseline predicted measurement with the current state.
        // If the model can't predict, we can't calculate a Jacobian, so we return zeros.
        let z_base = match self.predict_measurement(filter_state, &dummy_message, tf) {
            Some(z) => z,
            None => return h_jac,
        };

        // 2. Iterate through each element of the state vector.
        for j in 0..state_dim {
            // Create a copy of the state to perturb.
            let mut perturbed_state = filter_state.clone();

            // "Wiggle" the j-th state variable by a small amount epsilon.
            perturbed_state.vector[j] += epsilon;

            // IMPORTANT: If we are perturbing the quaternion part of the state,
            // we must re-normalize it to keep it a valid unit quaternion.
            // We check if the current index `j` is within the quaternion part of our state.
            if j >= 6 && j <= 9 {
                // Assuming Qx, Qy, Qz, Qw are at indices 6, 7, 8, 9
                let mut quat_part = perturbed_state.vector.fixed_rows_mut::<4>(6);
                let norm = quat_part.norm();
                if norm > 1e-9 {
                    quat_part /= norm;
                }
            }

            // Recalculate the prediction with the "wiggled" state.
            if let Some(z_perturbed) =
                self.predict_measurement(&perturbed_state, &dummy_message, tf)
            {
                // Approximate the partial derivative: (f(x + h) - f(x)) / h
                let derivative_column = (z_perturbed - &z_base) / epsilon;

                // Place this column into our Jacobian matrix.
                h_jac.column_mut(j).copy_from(&derivative_column);
            }
        }

        h_jac
    }

    /// Returns the measurement noise covariance matrix, R.
    fn get_r(&self) -> &DMatrix<f64> {
        &self.r_matrix
    }

    /// Allows for dynamic downcasting.
    fn as_any(&self) -> &dyn Any {
        self
    }
}
