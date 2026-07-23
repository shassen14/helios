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

    /// Predicts the magnetic field in the body frame by rotating the known world
    /// field through the filter's orientation. TF is not required.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        _tf: Option<&dyn TfProvider>,
    ) -> Option<DVector<f64>> {
        let orientation_body_to_world = filter_state.get_orientation().unwrap_or_default();
        let q_body_from_world = orientation_body_to_world.inverse();
        let predicted_mag_body = q_body_from_world * self.world_magnetic_field;
        Some(DVector::from_row_slice(predicted_mag_body.as_slice()))
    }
}

#[cfg(test)]
mod tests {
    //! Tests for [`MagneticFieldModel`].
    //!
    //! - Identity orientation: predicted field = world field.
    //! - 90° CCW yaw: a North-pointing world field appears along body +X.

    use super::*;
    use crate::data::primitives::FrameHandle;
    use crate::frames::{FrameAwareState, FrameId, StateVariable};
    use nalgebra::{UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;

    const AGENT: FrameHandle = FrameHandle(1);
    const SENSOR: FrameHandle = FrameHandle(2);

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
    fn predict_identity_orientation_returns_world_field() {
        let model = make_model();
        let state = make_orientation_state();
        let z = model.predict_measurement(&state, None).unwrap();
        assert!(z[0].abs() < 1e-9);
        assert!((z[1] - 1.0).abs() < 1e-9);
        assert!(z[2].abs() < 1e-9);
    }

    #[test]
    fn predict_90_yaw_rotates_field_to_body_x() {
        let model = make_model();
        let mut state = make_orientation_state();
        set_yaw_90_ccw(&mut state);
        let z = model.predict_measurement(&state, None).unwrap();
        assert!((z[0] - 1.0).abs() < 1e-9);
        assert!(z[1].abs() < 1e-9);
        assert!(z[2].abs() < 1e-9);
    }
}
