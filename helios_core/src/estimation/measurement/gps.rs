use nalgebra::DVector;

use crate::data::primitives::{FrameHandle, TfProvider};
use crate::estimation::measurement::MeasurementModel;
use crate::frames::{FrameAwareState, FrameId, StateVariable};

/// A measurement model for a standard GPS sensor that provides 3D position.
///
/// Maps a 3D ENU position measurement to the filter's position states
/// `(Px, Py, Pz)`, accounting for the antenna's physical offset from the body
/// origin via the TF tree (same pattern as [`AccelerometerModel`]).
///
/// Note: `R` (measurement noise covariance) is **not** held here. It lives at
/// the call site and is passed per `update`. See `algorithm_family_traits.md` §2.1.
///
/// [`AccelerometerModel`]: crate::estimation::measurement::accelerometer::AccelerometerModel
#[derive(Debug, Clone)]
pub struct GpsPositionModel {
    pub agent_handle: FrameHandle,
    /// Frame handle for the GPS antenna. Used to look up the antenna's offset
    /// from the body origin via the TF tree at prediction time.
    pub sensor_handle: FrameHandle,
}

impl MeasurementModel for GpsPositionModel {
    fn dim(&self) -> usize {
        3
    }

    /// Predicts antenna position in the ENU world frame.
    ///
    /// Requires `tf` to resolve the body→antenna translation. Returns `None`
    /// when `tf` is unavailable — same behaviour as `AccelerometerModel`.
    ///
    /// `predicted = P_world + R(q_body→world) * antenna_offset_body`
    /// where `antenna_offset_body` comes from `tf.get_transform(agent, sensor).translation`.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        tf: Option<&dyn TfProvider>,
    ) -> Option<DVector<f64>> {
        let tf = tf?;
        let body_position_world = filter_state.get_vector3(&StateVariable::Px(FrameId::World))?;
        let body_orientation_world = filter_state.get_orientation().unwrap_or_default();

        let tf_sensor_from_body = tf
            .get_transform(self.agent_handle, self.sensor_handle)
            .unwrap_or_default();
        let antenna_offset_body = tf_sensor_from_body.translation.vector;

        let antenna_offset_world = body_orientation_world * antenna_offset_body;
        let predicted_antenna_position_world = body_position_world + antenna_offset_world;

        Some(DVector::from_row_slice(
            predicted_antenna_position_world.as_slice(),
        ))
    }
}

#[cfg(test)]
mod tests {
    //! Tests for [`GpsPositionModel`].
    //!
    //! Properties validated:
    //! - `predict_measurement` returns `None` when `tf` is `None`.
    //! - With identity TF (antenna at body origin), predicts the body position directly.
    //! - With a non-zero TF translation, adds the rotated lever arm to the body position.
    //! - Default finite-diff Jacobian has the correct shape and identity position columns.

    use super::*;
    use crate::data::primitives::{FrameHandle, TfProvider};
    use crate::frames::{FrameAwareState, FrameId, StateVariable};
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

    const AGENT: FrameHandle = FrameHandle(1);
    const SENSOR: FrameHandle = FrameHandle(2);

    struct FixedTf(Isometry3<f64>);

    impl TfProvider for FixedTf {
        fn get_transform(&self, _from: FrameHandle, _to: FrameHandle) -> Option<Isometry3<f64>> {
            Some(self.0)
        }
        fn world_pose(&self, _frame: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
    }

    fn make_model() -> GpsPositionModel {
        GpsPositionModel {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
        }
    }

    fn make_state(px: f64, py: f64, pz: f64) -> FrameAwareState {
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
        let mut state = FrameAwareState::new(layout, 1.0, 0.0);
        state.state.vector[0] = px;
        state.state.vector[1] = py;
        state.state.vector[2] = pz;
        state
    }

    #[test]
    fn dim_is_three() {
        assert_eq!(make_model().dim(), 3);
    }

    #[test]
    fn predict_without_tf_returns_none() {
        let model = make_model();
        let state = make_state(3.0, 4.0, 5.0);
        assert!(model.predict_measurement(&state, None).is_none());
    }

    #[test]
    fn predict_identity_tf_returns_body_position() {
        let model = make_model();
        let state = make_state(3.0, 4.0, 5.0);
        let tf = FixedTf(Isometry3::identity());
        let z = model.predict_measurement(&state, Some(&tf)).unwrap();
        assert!((z[0] - 3.0).abs() < 1e-9);
        assert!((z[1] - 4.0).abs() < 1e-9);
        assert!((z[2] - 5.0).abs() < 1e-9);
    }

    #[test]
    fn predict_lever_arm_adds_rotated_offset() {
        let model = make_model();
        let state = make_state(1.0, 2.0, 3.0);
        // Antenna is 0.5 m forward, 0.1 m up from body origin; identity orientation.
        let offset = Isometry3::from_parts(
            Translation3::new(0.5, 0.0, 0.1),
            UnitQuaternion::identity(),
        );
        let tf = FixedTf(offset);
        let z = model.predict_measurement(&state, Some(&tf)).unwrap();
        assert!((z[0] - 1.5).abs() < 1e-9);
        assert!((z[1] - 2.0).abs() < 1e-9);
        assert!((z[2] - 3.1).abs() < 1e-9);
    }

    #[test]
    fn jacobian_position_columns_are_identity() {
        let model = make_model();
        let state = make_state(0.0, 0.0, 0.0);
        let tf = FixedTf(Isometry3::identity());
        let h = model.jacobian(&state, Some(&tf));
        assert_eq!(h.nrows(), 3);
        assert_eq!(h.ncols(), state.dim());
        assert!((h[(0, 0)] - 1.0).abs() < 1e-4);
        assert!((h[(1, 1)] - 1.0).abs() < 1e-4);
        assert!((h[(2, 2)] - 1.0).abs() < 1e-4);
    }
}
