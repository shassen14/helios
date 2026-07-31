use crate::{
    data::primitives::FrameHandle, estimation::measurement::MeasurementModel,
    frames::FrameAwareState, ports::TfProvider,
};
use nalgebra::{DVector, Vector3};

/// A measurement model for a 3-axis magnetometer.
///
/// Maps a measured magnetic field vector to the filter's orientation state,
/// providing an absolute heading reference.
#[derive(Debug, Clone)]
pub struct MagneticFieldModel {
    pub agent_handle: FrameHandle,
    pub sensor_handle: FrameHandle,
    /// The "true" magnetic field vector in the world (ENU) frame.
    pub world_magnetic_field: Vector3<f64>,
}

impl MeasurementModel for MagneticFieldModel {
    fn dim(&self) -> usize {
        3
    }

    /// Predicts the magnetic field in the sensor frame: the known world field is
    /// rotated through the filter's orientation into the body frame, then through
    /// the sensor's mount into the sensor frame. TF is required — the mount
    /// rotation comes from it — so this returns `None` when `tf` is unavailable.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        tf: Option<&dyn TfProvider>,
    ) -> Option<DVector<f64>> {
        let tf = tf?;

        let orientation_body_to_world = filter_state.get_orientation().unwrap_or_default();
        let q_body_from_world = orientation_body_to_world.inverse();
        let predicted_mag_body = q_body_from_world * self.world_magnetic_field;

        let rot_sensor_from_body = tf
            .get_transform(self.agent_handle, self.sensor_handle)
            .unwrap_or_default()
            .rotation;

        Some(DVector::from_row_slice(
            (rot_sensor_from_body.inverse() * predicted_mag_body).as_slice(),
        ))
    }
}

#[cfg(test)]
mod tests {
    //! Tests for [`MagneticFieldModel`].
    //!
    //! - No TF: the sensor mount rotation is required, so the model abstains.
    //! - Identity mount, identity orientation: predicted field = world field.
    //! - Identity mount, 90° CCW yaw: a North-pointing world field appears along
    //!   body +X.
    //! - Rotated mount: the body-frame field is carried on into the sensor frame.

    use super::*;
    use crate::data::primitives::FrameHandle;
    use crate::frames::{FrameAwareState, FrameId, StateVariable};
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;

    const AGENT: FrameHandle = FrameHandle(1);
    const SENSOR: FrameHandle = FrameHandle(2);

    /// Reports one fixed extrinsic — the sensor's pose in body axes — for every
    /// lookup, mirroring what a real TF tree hands the model.
    struct Mount(Isometry3<f64>);

    impl TfProvider for Mount {
        fn get_transform(&self, _from: FrameHandle, _to: FrameHandle) -> Option<Isometry3<f64>> {
            Some(self.0)
        }
        fn world_pose(&self, _frame: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
    }

    fn identity_mount() -> Mount {
        Mount(Isometry3::identity())
    }

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

    fn set_yaw_90_ccw(state: &mut FrameAwareState) {
        let q = UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2);
        state.state.vector[0] = q.i;
        state.state.vector[1] = q.j;
        state.state.vector[2] = q.k;
        state.state.vector[3] = q.w;
    }

    fn make_model() -> MagneticFieldModel {
        MagneticFieldModel {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            world_magnetic_field: Vector3::new(0.0, 1.0, 0.0),
        }
    }

    #[test]
    fn dim_is_three() {
        assert_eq!(make_model().dim(), 3);
    }

    #[test]
    fn predict_without_tf_returns_none() {
        let model = make_model();
        let state = make_orientation_state();
        assert!(model.predict_measurement(&state, None).is_none());
    }

    #[test]
    fn predict_identity_orientation_returns_world_field() {
        let model = make_model();
        let state = make_orientation_state();
        let z = model
            .predict_measurement(&state, Some(&identity_mount()))
            .unwrap();
        assert!(z[0].abs() < 1e-9);
        assert!((z[1] - 1.0).abs() < 1e-9);
        assert!(z[2].abs() < 1e-9);
    }

    #[test]
    fn predict_90_yaw_rotates_field_to_body_x() {
        let model = make_model();
        let mut state = make_orientation_state();
        set_yaw_90_ccw(&mut state);
        let z = model
            .predict_measurement(&state, Some(&identity_mount()))
            .unwrap();
        assert!((z[0] - 1.0).abs() < 1e-9);
        assert!(z[1].abs() < 1e-9);
        assert!(z[2].abs() < 1e-9);
    }

    #[test]
    fn predict_carries_the_field_into_a_rotated_sensor_frame() {
        // Body is level (identity orientation), so the body-frame field is the
        // world field — due north, body +Y. The sensor is yawed +90° about Z
        // relative to the body (its +X axis points along body +Y), so the north
        // field lands on the sensor's +X. This pins the mount rotation: an
        // identity extrinsic would leave the field on +Y.
        let model = make_model();
        let state = make_orientation_state();
        let mount = Mount(Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2),
        ));
        let z = model.predict_measurement(&state, Some(&mount)).unwrap();
        assert!((z[0] - 1.0).abs() < 1e-9);
        assert!(z[1].abs() < 1e-9);
        assert!(z[2].abs() < 1e-9);
    }
}
