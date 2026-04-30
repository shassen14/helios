use nalgebra::{DMatrix, DVector, Vector3};
use std::any::Any;
use std::fmt::Debug;

use crate::frames::{FrameAwareState, FrameId, StateVariable};
use crate::messages::{MeasurementData, MeasurementMessage};
use crate::models::estimation::measurement::Measurement;
use crate::types::{FrameHandle, TfProvider};

#[derive(Debug, Clone)]
pub struct AccelerometerModel {
    pub agent_handle: FrameHandle,
    pub sensor_handle: FrameHandle,
    pub r_matrix: DMatrix<f64>, // 3x3 measurement noise covariance
    pub gravity_magnitude: f64,
}

impl Measurement for AccelerometerModel {
    fn get_measurement_layout(&self) -> Vec<StateVariable> {
        let sensor_frame = FrameId::Sensor(self.sensor_handle);
        vec![
            StateVariable::Ax(sensor_frame.clone()),
            StateVariable::Ay(sensor_frame.clone()),
            StateVariable::Az(sensor_frame.clone()),
        ]
    }

    fn get_measurement_vector(&self, data: &MeasurementData) -> Option<DVector<f64>> {
        if let MeasurementData::LinearAcceleration(accel) = data {
            Some(DVector::from_row_slice(accel.value.as_slice()))
        } else {
            None
        }
    }

    /// Predicts the 3-element measurement vector [ax, ay, az].
    /// It returns `Some` only if the input message contains `LinearAcceleration` data.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        message: &MeasurementMessage,
        tf: &dyn TfProvider,
    ) -> Option<DVector<f64>> {
        // --- 1. Check if this model can handle the data ---
        // This model only processes `LinearAcceleration` data. It ignores everything else.
        if !matches!(&message.data, MeasurementData::LinearAcceleration(_)) {
            return None;
        }

        // --- At this point, we know we should proceed. ---
        let mut z_pred = DVector::zeros(3);
        let body_frame = FrameId::Body(self.agent_handle);

        // --- 2. Get Transform Data ---
        let tf_sensor_from_body = tf
            .get_transform(self.agent_handle, self.sensor_handle)
            .unwrap_or_default();
        let r_body_to_sensor = tf_sensor_from_body.translation.vector;
        let rot_sensor_from_body = tf_sensor_from_body.rotation;

        // --- 3. Extract States from Filter ---
        // Get the filter's current belief about its own state.
        let linear_accel_body = filter_state
            .get_vector3(&StateVariable::Ax(body_frame.clone()))
            .unwrap_or_default();
        let angular_vel_body = filter_state
            .get_vector3(&StateVariable::Wx(body_frame.clone()))
            .unwrap_or_default();
        let angular_accel_body = filter_state
            .get_vector3(&StateVariable::Alphax(body_frame.clone()))
            .unwrap_or_default();
        let orientation_body_to_world = filter_state.get_orientation().unwrap_or_default();

        // --- 4. Predict Linear Acceleration ---
        let tangential_accel = angular_accel_body.cross(&r_body_to_sensor);
        let centripetal_accel = angular_vel_body.cross(&angular_vel_body.cross(&r_body_to_sensor));
        let total_kinematic_accel_at_sensor =
            linear_accel_body + tangential_accel + centripetal_accel;

        let q_body_from_world = orientation_body_to_world.inverse();
        let gravity_world = Vector3::new(0.0, 0.0, -self.gravity_magnitude);
        let gravity_effect_in_body = q_body_from_world * gravity_world;

        let proper_accel_in_body_frame = total_kinematic_accel_at_sensor - gravity_effect_in_body;
        let predicted_accel = rot_sensor_from_body.inverse() * proper_accel_in_body_frame;
        z_pred.fixed_rows_mut::<3>(0).copy_from(&predicted_accel);

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
            data: MeasurementData::LinearAcceleration(Default::default()),
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

    fn make_model() -> AccelerometerModel {
        AccelerometerModel {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            r_matrix: DMatrix::identity(3, 3) * 0.01,
            gravity_magnitude: 9.81,
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

    fn accel_message() -> MeasurementMessage {
        MeasurementMessage {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            timestamp: 0.0,
            data: MeasurementData::LinearAcceleration(Default::default()),
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
    fn measurement_vector_accepts_linear_acceleration() {
        let model = make_model();
        let data = MeasurementData::LinearAcceleration(sensor_data::LinearAcceleration3D {
            value: nalgebra::Vector3::new(1.0, 2.0, 3.0),
        });
        let z = model.get_measurement_vector(&data).unwrap();
        assert_eq!(z.nrows(), 3);
        assert!((z[0] - 1.0).abs() < 1e-12);
        assert!((z[1] - 2.0).abs() < 1e-12);
        assert!((z[2] - 3.0).abs() < 1e-12);
    }

    #[test]
    fn measurement_vector_rejects_angular_velocity() {
        let model = make_model();
        let data = MeasurementData::AngularVelocity(Default::default());
        assert!(model.get_measurement_vector(&data).is_none());
    }

    #[test]
    fn predict_rejects_non_accel_message() {
        let model = make_model();
        let state = make_state();
        let tf = IdentityTf;
        assert!(model
            .predict_measurement(&state, &gps_message(), &tf)
            .is_none());
    }

    #[test]
    fn predict_returns_some_for_accel_message() {
        let model = make_model();
        let state = make_state();
        let tf = IdentityTf;
        assert!(model
            .predict_measurement(&state, &accel_message(), &tf)
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
