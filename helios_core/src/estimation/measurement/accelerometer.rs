use nalgebra::{DVector, Vector3};

use crate::data::primitives::FrameHandle;
use crate::estimation::measurement::MeasurementModel;
use crate::frames::{FrameAwareState, FrameId, StateVariable};
use crate::ports::TfProvider;

#[derive(Debug, Clone)]
pub struct AccelerometerModel {
    pub agent_handle: FrameHandle,
    pub sensor_handle: FrameHandle,
    pub gravity_magnitude: f64,
}

impl MeasurementModel for AccelerometerModel {
    fn dim(&self) -> usize {
        3
    }

    /// Predicts the proper acceleration measured by an accelerometer in its sensor frame.
    ///
    /// Returns `None` when `tf` is unavailable — the body→sensor transform is
    /// required to project the predicted acceleration into the sensor frame.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        tf: Option<&dyn TfProvider>,
    ) -> Option<DVector<f64>> {
        let tf = tf?;
        let body_frame = FrameId::Body(self.agent_handle);

        let tf_sensor_from_body = tf
            .get_transform(self.agent_handle, self.sensor_handle)
            .unwrap_or_default();
        let r_body_to_sensor = tf_sensor_from_body.translation.vector;
        let rot_sensor_from_body = tf_sensor_from_body.rotation;

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

        let tangential_accel = angular_accel_body.cross(&r_body_to_sensor);
        let centripetal_accel = angular_vel_body.cross(&angular_vel_body.cross(&r_body_to_sensor));
        let total_kinematic_accel_at_sensor =
            linear_accel_body + tangential_accel + centripetal_accel;

        let q_body_from_world = orientation_body_to_world.inverse();
        let gravity_world = Vector3::new(0.0, 0.0, -self.gravity_magnitude);
        let gravity_effect_in_body = q_body_from_world * gravity_world;

        let proper_accel_in_body_frame = total_kinematic_accel_at_sensor - gravity_effect_in_body;
        let predicted_accel = rot_sensor_from_body.inverse() * proper_accel_in_body_frame;

        let mut z_pred = DVector::zeros(3);
        z_pred.fixed_rows_mut::<3>(0).copy_from(&predicted_accel);
        Some(z_pred)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data::primitives::FrameHandle;
    use crate::frames::{FrameAwareState, FrameId, StateVariable};
    use crate::ports::TfProvider;
    use nalgebra::Isometry3;

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

    #[test]
    fn dim_is_three() {
        assert_eq!(make_model().dim(), 3);
    }

    #[test]
    fn predict_without_tf_returns_none() {
        let model = make_model();
        let state = make_state();
        assert!(model.predict_measurement(&state, None).is_none());
    }

    #[test]
    fn predict_with_tf_returns_some() {
        let model = make_model();
        let state = make_state();
        let tf = IdentityTf;
        assert!(model.predict_measurement(&state, Some(&tf)).is_some());
    }

    #[test]
    fn jacobian_has_correct_shape() {
        let model = make_model();
        let state = make_state();
        let tf = IdentityTf;
        let h = model.jacobian(&state, Some(&tf));
        assert_eq!(h.nrows(), 3);
        assert_eq!(h.ncols(), state.dim());
    }
}
