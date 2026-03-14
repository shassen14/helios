use nalgebra::{DMatrix, DVector, Vector3};
use std::any::Any;
use std::fmt::Debug;

use crate::frames::{FrameAwareState, FrameId, StateVariable};
use crate::messages::{MeasurementData, MeasurementMessage};
use crate::models::estimation::measurement::Measurement;
use crate::types::{FrameHandle, TfProvider};

/// A high-fidelity model for a 6-DOF IMU when used as a MEASUREMENT source.
///
/// This model is typically used with simpler dynamics models (like ConstantAcceleration)
/// where the IMU provides corrective information about the agent's acceleration and rotation.
#[derive(Debug, Clone)]
pub struct Imu6DofModel {
    pub agent_handle: FrameHandle,
    pub sensor_handle: FrameHandle,
    pub r_matrix: DMatrix<f64>, // 6x6 measurement noise covariance
    pub gravity_magnitude: f64,
}

impl Measurement for Imu6DofModel {
    fn get_measurement_layout(&self) -> Vec<StateVariable> {
        let sensor_frame = FrameId::Sensor(self.sensor_handle);
        vec![
            StateVariable::Ax(sensor_frame.clone()),
            StateVariable::Ay(sensor_frame.clone()),
            StateVariable::Az(sensor_frame.clone()),
            StateVariable::Wx(sensor_frame.clone()),
            StateVariable::Wy(sensor_frame.clone()),
            StateVariable::Wz(sensor_frame.clone()),
        ]
    }

    fn get_measurement_vector(&self, data: &MeasurementData) -> Option<DVector<f64>> {
        if let MeasurementData::Imu6Dof(vec) = data {
            Some(DVector::from_row_slice(vec.as_slice()))
        } else {
            None
        }
    }

    /// Predicts the 6-element measurement vector [ax, ay, az, wx, wy, wz].
    /// It returns `Some` only if the input message contains `Imu6Dof` data.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        message: &MeasurementMessage,
        tf: &dyn TfProvider,
    ) -> Option<DVector<f64>> {
        // --- 1. Check if this model can handle the data ---
        // This model only processes `Imu6Dof` data. It ignores everything else.
        if !matches!(&message.data, MeasurementData::Imu6Dof(_)) {
            return None;
        }

        // --- At this point, we know we should proceed. ---
        let mut z_pred = DVector::zeros(6);
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

        // --- 4. Predict Angular Velocity (Gyroscope part) ---
        let predicted_gyro = rot_sensor_from_body.inverse() * angular_vel_body;
        z_pred.fixed_rows_mut::<3>(3).copy_from(&predicted_gyro);

        // --- 5. Predict Linear Acceleration (Accelerometer part) ---
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
        let mut h_jac = DMatrix::zeros(6, state_dim);
        let epsilon = 1e-8;

        // We need a dummy message to pass to predict_measurement.
        // The data variant just needs to be the one this model accepts.
        let dummy_message = MeasurementMessage {
            agent_handle: self.agent_handle,
            sensor_handle: self.sensor_handle,
            timestamp: 0.0,
            data: MeasurementData::Imu6Dof(Default::default()),
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
    //! Tests for [`Imu6DofModel`].
    //!
    //! Properties validated:
    //! - Type dispatch: accepts `Imu6Dof`, rejects all other variants.
    //! - `predict_measurement`: returns `None` for non-IMU messages.
    //! - Zero state + identity TF + identity orientation: accelerometer predicts
    //!   [0, 0, g] (proper acceleration counteracting gravity), gyroscope predicts
    //!   [0, 0, 0] (no angular velocity in state).
    //! - Jacobian has correct shape (6 rows × state_dim columns).

    use super::*;
    use crate::frames::layout::standard_ins_state_layout;
    use crate::frames::FrameAwareState;
    use crate::messages::{MeasurementData, MeasurementMessage};
    use crate::types::{FrameHandle, TfProvider};
    use nalgebra::{DMatrix, Isometry3, Vector3};

    const AGENT: FrameHandle = FrameHandle(1);
    const SENSOR: FrameHandle = FrameHandle(2);
    const G: f64 = 9.81;

    /// Stub TF provider that always returns the identity transform.
    struct IdentityTf;
    impl TfProvider for IdentityTf {
        fn get_transform(&self, _from: FrameHandle, _to: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
        fn world_pose(&self, _frame: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
    }

    fn make_model() -> Imu6DofModel {
        Imu6DofModel {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            r_matrix: DMatrix::identity(6, 6) * 0.01,
            gravity_magnitude: G,
        }
    }

    /// Builds the standard 16-element INS state with identity quaternion.
    ///
    /// `FrameAwareState::new` sets Qw = 1.0 automatically.
    fn make_ins_state() -> FrameAwareState {
        FrameAwareState::new(standard_ins_state_layout(AGENT), 1.0, 0.0)
    }

    fn imu_message() -> MeasurementMessage {
        MeasurementMessage {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            timestamp: 0.0,
            data: MeasurementData::Imu6Dof(Default::default()),
        }
    }

    fn gps_message() -> MeasurementMessage {
        MeasurementMessage {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            timestamp: 0.0,
            data: MeasurementData::GpsPosition(Vector3::zeros()),
        }
    }

    // ── Type dispatch ─────────────────────────────────────────────────────────

    #[test]
    fn measurement_vector_accepts_imu6dof() {
        let model = make_model();
        let data = MeasurementData::Imu6Dof(Default::default());
        assert!(model.get_measurement_vector(&data).is_some());
    }

    #[test]
    fn measurement_vector_rejects_gps() {
        let model = make_model();
        let data = MeasurementData::GpsPosition(Vector3::zeros());
        assert!(model.get_measurement_vector(&data).is_none());
    }

    // ── predict_measurement ───────────────────────────────────────────────────

    #[test]
    fn predict_rejects_non_imu_message() {
        let model = make_model();
        let state = make_ins_state();
        let tf = IdentityTf;
        assert!(
            model.predict_measurement(&state, &gps_message(), &tf).is_none(),
            "non-IMU message must yield None"
        );
    }

    #[test]
    fn predict_zero_state_identity_tf_gives_gravity_up() {
        // State: all zeros except Qw = 1 (identity orientation).
        // With identity orientation and zero angular velocity:
        //   - gravity_effect_in_body = inverse(identity) * [0, 0, -g] = [0, 0, -g]
        //   - proper_accel = 0 - [0, 0, -g] = [0, 0, g]
        // The accelerometer should therefore predict [0, 0, g], and the gyroscope [0, 0, 0].
        let model = make_model();
        let state = make_ins_state();
        let tf = IdentityTf;
        let z = model.predict_measurement(&state, &imu_message(), &tf).unwrap();

        assert_eq!(z.nrows(), 6, "IMU measurement vector is 6-dimensional");
        assert!(z[0].abs() < 1e-9, "accel_x should be 0");
        assert!(z[1].abs() < 1e-9, "accel_y should be 0");
        assert!((z[2] - G).abs() < 1e-9, "accel_z should equal gravity ({G})");
        assert!(z[3].abs() < 1e-9, "gyro_x should be 0");
        assert!(z[4].abs() < 1e-9, "gyro_y should be 0");
        assert!(z[5].abs() < 1e-9, "gyro_z should be 0");
    }

    #[test]
    fn jacobian_has_correct_shape() {
        // H must have 6 rows (measurement dim) and state_dim columns.
        let model = make_model();
        let state = make_ins_state();
        let tf = IdentityTf;
        let h = model.calculate_jacobian(&state, &tf);

        assert_eq!(h.nrows(), 6, "H rows = measurement dim");
        assert_eq!(h.ncols(), state.dim(), "H cols = state dim");
    }
}
