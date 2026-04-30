use nalgebra::{DMatrix, DVector};
use std::any::Any;
use std::fmt::Debug;

use crate::frames::{FrameAwareState, FrameId, StateVariable};
use crate::messages::{MeasurementData, MeasurementMessage};
use crate::models::estimation::measurement::Measurement;
use crate::types::{FrameHandle, TfProvider};

#[derive(Debug, Clone)]
pub struct GyroscopeModel {
    pub agent_handle: FrameHandle,
    pub sensor_handle: FrameHandle,
    pub r_matrix: DMatrix<f64>, // 3x3 measurement noise covariance
}

impl Measurement for GyroscopeModel {
    fn get_measurement_layout(&self) -> Vec<StateVariable> {
        let sensor_frame = FrameId::Sensor(self.sensor_handle);
        vec![
            StateVariable::Wx(sensor_frame.clone()),
            StateVariable::Wy(sensor_frame.clone()),
            StateVariable::Wz(sensor_frame.clone()),
        ]
    }

    fn get_measurement_vector(&self, data: &MeasurementData) -> Option<DVector<f64>> {
        if let MeasurementData::AngularVelocity(ang_vel) = data {
            Some(DVector::from_row_slice(ang_vel.value.as_slice()))
        } else {
            None
        }
    }

    /// Predicts the 3-element measurement vector [wx, wy, wz].
    /// It returns `Some` only if the input message contains `AngularVelocity` data.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        message: &MeasurementMessage,
        tf: &dyn TfProvider,
    ) -> Option<DVector<f64>> {
        // --- 1. Check if this model can handle the data ---
        // This model only processes `AngularVelocity` data. It ignores everything else.
        if !matches!(&message.data, MeasurementData::AngularVelocity(_)) {
            return None;
        }

        // --- At this point, we know we should proceed. ---
        let mut z_pred = DVector::zeros(3);
        let body_frame = FrameId::Body(self.agent_handle);

        // --- 2. Get Transform Data ---
        let tf_sensor_from_body = tf
            .get_transform(self.agent_handle, self.sensor_handle)
            .unwrap_or_default();
        let rot_sensor_from_body = tf_sensor_from_body.rotation;

        // --- 3. Extract States from Filter ---
        // Get the filter's current belief about its own state.

        let angular_vel_body = filter_state
            .get_vector3(&StateVariable::Wx(body_frame.clone()))
            .unwrap_or_default();

        // --- 4. Predict Angular Velocity (Gyroscope part) ---
        let predicted_gyro = rot_sensor_from_body.inverse() * angular_vel_body;
        z_pred.fixed_rows_mut::<3>(0).copy_from(&predicted_gyro);

        // Since we successfully processed the data, return the prediction.
        Some(z_pred)
    }

    /// Calculates the Jacobian H using numerical differentiation.
    fn calculate_jacobian(
        &self,
        filter_state: &FrameAwareState,
        tf: &dyn TfProvider,
    ) -> DMatrix<f64> {
        // This numerical implementation is robust and works well here.
        let state_dim = filter_state.dim();
        let mut h_jac = DMatrix::zeros(3, state_dim);
        let epsilon = 1e-8;

        // We need a dummy message to pass to predict_measurement.
        // The data variant just needs to be the one this model accepts.
        let dummy_message = MeasurementMessage {
            agent_handle: self.agent_handle,
            sensor_handle: self.sensor_handle,
            timestamp: 0.0,
            data: MeasurementData::AngularVelocity(Default::default()),
        };

        // The baseline prediction must be valid for the subtraction to work.
        let z_base = match self.predict_measurement(filter_state, &dummy_message, tf) {
            Some(z) => z,
            None => return h_jac, // Should not happen if called correctly
        };

        for j in 0..state_dim {
            let mut perturbed_state = filter_state.clone();
            perturbed_state.vector[j] += epsilon;

            if let Some(z_perturbed) =
                self.predict_measurement(&perturbed_state, &dummy_message, tf)
            {
                let derivative_column = (z_perturbed - &z_base) / epsilon;
                h_jac.column_mut(j).copy_from(&derivative_column);
            }
        }

        h_jac
    }

    fn get_r(&self) -> &DMatrix<f64> {
        &self.r_matrix
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::{FrameAwareState, FrameId, StateVariable};
    use crate::messages::{MeasurementData, MeasurementMessage};
    use crate::sensor_data;
    use crate::types::{FrameHandle, TfProvider};
    use nalgebra::{DMatrix, Isometry3};

    const AGENT: FrameHandle = FrameHandle(1);
    const SENSOR: FrameHandle = FrameHandle(2);

    struct IdentityTf;
    impl TfProvider for IdentityTf {
        fn get_transform(&self, _from: FrameHandle, _to: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
        fn world_pose(&self, _frame: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
    }

    fn make_model() -> GyroscopeModel {
        GyroscopeModel {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            r_matrix: DMatrix::identity(3, 3) * 0.01,
        }
    }

    fn make_state() -> FrameAwareState {
        let body = FrameId::Body(AGENT);
        let world = FrameId::World;
        let layout = vec![
            StateVariable::Px(world.clone()),
            StateVariable::Py(world.clone()),
            StateVariable::Pz(world.clone()),
            StateVariable::Qx(body.clone(), world.clone()),
            StateVariable::Qy(body.clone(), world.clone()),
            StateVariable::Qz(body.clone(), world.clone()),
            StateVariable::Qw(body.clone(), world.clone()),
        ];
        FrameAwareState::new(layout, 1.0, 0.0)
    }

    fn gyro_message() -> MeasurementMessage {
        MeasurementMessage {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            timestamp: 0.0,
            data: MeasurementData::AngularVelocity(Default::default()),
        }
    }

    fn gps_message() -> MeasurementMessage {
        MeasurementMessage {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            timestamp: 0.0,
            data: MeasurementData::GpsPosition(sensor_data::GpsPosition {
                position: nalgebra::Vector3::zeros(),
            }),
        }
    }

    #[test]
    fn measurement_vector_accepts_angular_velocity() {
        let model = make_model();
        let data = MeasurementData::AngularVelocity(sensor_data::AngularVelocity3D {
            value: nalgebra::Vector3::new(0.1, 0.2, 0.3),
        });
        let z = model.get_measurement_vector(&data).unwrap();
        assert_eq!(z.nrows(), 3);
        assert!((z[0] - 0.1).abs() < 1e-12);
        assert!((z[1] - 0.2).abs() < 1e-12);
        assert!((z[2] - 0.3).abs() < 1e-12);
    }

    #[test]
    fn measurement_vector_rejects_linear_acceleration() {
        let model = make_model();
        let data = MeasurementData::LinearAcceleration(Default::default());
        assert!(model.get_measurement_vector(&data).is_none());
    }

    #[test]
    fn predict_rejects_non_gyro_message() {
        let model = make_model();
        let state = make_state();
        let tf = IdentityTf;
        assert!(model
            .predict_measurement(&state, &gps_message(), &tf)
            .is_none());
    }

    #[test]
    fn predict_returns_some_for_gyro_message() {
        let model = make_model();
        let state = make_state();
        let tf = IdentityTf;
        assert!(model
            .predict_measurement(&state, &gyro_message(), &tf)
            .is_some());
    }

    #[test]
    fn jacobian_has_correct_shape() {
        let model = make_model();
        let state = make_state();
        let tf = IdentityTf;
        let h = model.calculate_jacobian(&state, &tf);
        assert_eq!(h.nrows(), 3);
        assert_eq!(h.ncols(), state.dim());
    }
}
