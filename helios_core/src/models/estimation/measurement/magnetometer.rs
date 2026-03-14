use std::any::Any;

use crate::{
    frames::{FrameAwareState, FrameId, StateVariable},
    prelude::{Measurement, MeasurementData, MeasurementMessage},
    types::{FrameHandle, TfProvider},
};
use nalgebra::{DMatrix, DVector, Vector3};

/// A measurement model for a 3-axis magnetometer.
///
/// This model relates a measured magnetic field vector to the filter's
/// orientation state, providing an absolute heading reference.
#[derive(Debug, Clone)]
pub struct MagnetometerModel {
    /// A handle to the agent this sensor belongs to (the "body" frame).
    pub agent_handle: FrameHandle,
    /// A handle to the sensor itself (the "sensor" frame).
    pub sensor_handle: FrameHandle,
    /// The 3x3 measurement noise covariance matrix, R.
    pub r_matrix: DMatrix<f64>,
    /// The "true" magnetic field vector in the world (ENU) frame.
    /// For simulation, we define this as a constant pointing North (+Y).
    pub world_magnetic_field: Vector3<f64>,
}

impl Measurement for MagnetometerModel {
    /// Describes the semantic layout of the measurement vector `z`.
    fn get_measurement_layout(&self) -> Vec<StateVariable> {
        let sensor_frame = FrameId::Sensor(self.sensor_handle);
        vec![
            StateVariable::MagX(sensor_frame.clone()),
            StateVariable::MagY(sensor_frame.clone()),
            StateVariable::MagZ(sensor_frame.clone()),
        ]
    }

    fn get_measurement_vector(&self, data: &MeasurementData) -> Option<DVector<f64>> {
        if let MeasurementData::Magnetometer(vec) = data {
            Some(DVector::from_row_slice(vec.as_slice()))
        } else {
            None
        }
    }

    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        message: &MeasurementMessage,
        _tf: &dyn TfProvider,
    ) -> Option<DVector<f64>> {
        // This model only cares about Magnetometer data.
        if !matches!(&message.data, MeasurementData::Magnetometer(_)) {
            return None;
        }

        // Get the filter's current belief about its orientation.
        let orientation_body_to_world = filter_state.get_orientation().unwrap_or_default();

        // --- The Core Logic ---
        // Predict what the magnetometer should see by taking the true world
        // magnetic field and rotating it into the robot's body frame.
        let q_body_from_world = orientation_body_to_world.inverse();
        let predicted_mag_body = q_body_from_world * self.world_magnetic_field;

        Some(DVector::from_row_slice(predicted_mag_body.as_slice()))
    }

    fn calculate_jacobian(
        &self,
        filter_state: &FrameAwareState,
        tf: &dyn TfProvider,
    ) -> DMatrix<f64> {
        let state_dim = filter_state.dim();
        let measurement_dim = 3;
        let mut h_jac = DMatrix::zeros(measurement_dim, state_dim);
        let epsilon = 1e-8;

        let dummy_message = MeasurementMessage {
            agent_handle: self.agent_handle,
            sensor_handle: self.sensor_handle,
            timestamp: 0.0,
            data: MeasurementData::Magnetometer(Default::default()),
        };

        let z_base = match self.predict_measurement(filter_state, &dummy_message, tf) {
            Some(z) => z,
            None => return h_jac,
        };

        for j in 0..state_dim {
            let mut perturbed_state = filter_state.clone();
            perturbed_state.vector[j] += epsilon;

            // No quaternion re-normalisation needed: predict_measurement calls
            // get_orientation(), which calls UnitQuaternion::from_quaternion and
            // normalises unconditionally. The ε perturbation (1e-8) is too small
            // to meaningfully affect the norm anyway.
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
    //! Tests for [`MagnetometerModel`].
    //!
    //! Properties validated:
    //! - Type dispatch: accepts `Magnetometer`, rejects all other variants.
    //! - `predict_measurement`: returns `None` for non-magnetometer messages.
    //! - Identity orientation: predicted field = world field (no rotation applied).
    //! - 90° CCW yaw: world field pointing North appears along body +X axis in
    //!   the sensor frame (since body X is now aligned with North).

    use super::*;
    use crate::frames::{FrameAwareState, FrameId, StateVariable};
    use crate::messages::{MeasurementData, MeasurementMessage};
    use crate::types::{FrameHandle, TfProvider};
    use nalgebra::{DMatrix, Isometry3, UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;

    const AGENT: FrameHandle = FrameHandle(1);
    const SENSOR: FrameHandle = FrameHandle(2);

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

    /// A 4-element orientation-only state layout: [Qx, Qy, Qz, Qw].
    ///
    /// `FrameAwareState::new` sets Qw = 1.0 automatically (identity quaternion).
    fn make_orientation_state() -> FrameAwareState {
        let body = FrameId::Body(AGENT);
        let world = FrameId::World;
        let layout = vec![
            StateVariable::Qx(body.clone(), world.clone()),
            StateVariable::Qy(body.clone(), world.clone()),
            StateVariable::Qz(body.clone(), world.clone()),
            StateVariable::Qw(body.clone(), world.clone()),
        ];
        FrameAwareState::new(layout, 1.0, 0.0)
    }

    /// Sets the quaternion in `state` to a 90° CCW yaw (rotation about +Z in ENU).
    ///
    /// After this rotation body +X points North (+Y_world).
    fn set_yaw_90_ccw(state: &mut FrameAwareState) {
        let q = UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2);
        // Layout: [Qx, Qy, Qz, Qw] at indices 0-3.
        state.vector[0] = q.i;
        state.vector[1] = q.j;
        state.vector[2] = q.k;
        state.vector[3] = q.w;
    }

    fn make_model() -> MagnetometerModel {
        // Magnetic field pointing North (+Y in ENU).
        MagnetometerModel {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            r_matrix: DMatrix::identity(3, 3) * 0.01,
            world_magnetic_field: Vector3::new(0.0, 1.0, 0.0),
        }
    }

    fn mag_message() -> MeasurementMessage {
        MeasurementMessage {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            timestamp: 0.0,
            data: MeasurementData::Magnetometer(Default::default()),
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
    fn measurement_vector_accepts_magnetometer() {
        let model = make_model();
        let data = MeasurementData::Magnetometer(Vector3::new(1.0, 0.5, 0.0));
        let z = model.get_measurement_vector(&data).unwrap();
        assert_eq!(z.nrows(), 3);
        assert!((z[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn measurement_vector_rejects_gps() {
        let model = make_model();
        let data = MeasurementData::GpsPosition(Vector3::zeros());
        assert!(model.get_measurement_vector(&data).is_none());
    }

    // ── predict_measurement ───────────────────────────────────────────────────

    #[test]
    fn predict_rejects_non_magnetometer_message() {
        let model = make_model();
        let state = make_orientation_state();
        let tf = IdentityTf;
        assert!(
            model.predict_measurement(&state, &gps_message(), &tf).is_none(),
            "non-magnetometer message must yield None"
        );
    }

    #[test]
    fn predict_identity_orientation_returns_world_field() {
        // With identity orientation q_body_from_world = identity, so the predicted
        // field in body frame equals the world field exactly.
        let model = make_model();
        let state = make_orientation_state();
        let tf = IdentityTf;
        let z = model.predict_measurement(&state, &mag_message(), &tf).unwrap();

        // world_magnetic_field = [0, 1, 0]; with identity rotation: z_pred = [0, 1, 0].
        assert!(z[0].abs() < 1e-9, "Bx_body = 0 with identity orientation");
        assert!((z[1] - 1.0).abs() < 1e-9, "By_body = 1 (world field is North)");
        assert!(z[2].abs() < 1e-9, "Bz_body = 0 with identity orientation");
    }

    #[test]
    fn predict_90_yaw_rotates_field_to_body_x() {
        // After a 90° CCW yaw, body +X faces North. The North-pointing magnetic
        // field therefore appears along body +X, i.e., z_pred ≈ [1, 0, 0].
        let model = make_model();
        let mut state = make_orientation_state();
        set_yaw_90_ccw(&mut state);
        let tf = IdentityTf;
        let z = model.predict_measurement(&state, &mag_message(), &tf).unwrap();

        assert!((z[0] - 1.0).abs() < 1e-9, "Bx_body ≈ 1 after 90° CCW yaw");
        assert!(z[1].abs() < 1e-9, "By_body ≈ 0 after 90° CCW yaw");
        assert!(z[2].abs() < 1e-9, "Bz_body ≈ 0 (no vertical field component)");
    }
}
