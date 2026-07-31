use nalgebra::DVector;

use crate::data::primitives::FrameHandle;
use crate::estimation::measurement::MeasurementModel;
use crate::frames::{FrameAwareState, FrameId, StateVariable};
use crate::ports::TfProvider;

/// What the filter believes a rate gyroscope reports: the body's angular
/// velocity, rotated into the sensor frame.
///
/// Unlike [`SpecificForceModel`], this one is a plain frame rotation of a state
/// variable — angular velocity is the same quantity at every point on a rigid
/// body, so a sensor's mounting offset contributes no extra term. Only its
/// orientation matters.
///
/// [`SpecificForceModel`]: crate::estimation::measurement::accelerometer::SpecificForceModel
#[derive(Debug, Clone)]
pub struct AngularRateModel {
    pub agent_handle: FrameHandle,
    pub sensor_handle: FrameHandle,
}

impl MeasurementModel for AngularRateModel {
    fn dim(&self) -> usize {
        3
    }

    /// Predicts angular velocity in the sensor frame.
    ///
    /// Returns `None` when `tf` is unavailable — the body→sensor rotation is
    /// required.
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
        let rot_sensor_from_body = tf_sensor_from_body.rotation;

        let angular_vel_body = filter_state
            .get_vector3(&StateVariable::Wx(body_frame.clone()))
            .unwrap_or_default();

        let predicted_gyro = rot_sensor_from_body.inverse() * angular_vel_body;
        let mut z_pred = DVector::zeros(3);
        z_pred.fixed_rows_mut::<3>(0).copy_from(&predicted_gyro);
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

    fn make_model() -> AngularRateModel {
        AngularRateModel {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
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
