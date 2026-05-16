use nalgebra::{DVector, Vector3};

use crate::data::primitives::TfProvider;
use crate::estimation::measurement::MeasurementModel;
use crate::frames::{FrameAwareState, FrameId, StateVariable};

/// A measurement model for a standard GPS sensor that provides 3D position.
///
/// Maps a 3D ENU position measurement to the filter's position states
/// `(Px, Py, Pz)`, accounting for a body-frame antenna lever arm.
///
/// Note: `R` (measurement noise covariance) is **not** held here. It lives at
/// the call site and is passed per `update`. See `algorithm_family_traits.md` §2.1.
#[derive(Debug, Clone)]
pub struct GpsPositionModel {
    /// Physical location of the GPS antenna relative to the body origin, expressed
    /// in the body's coordinate frame.
    pub antenna_offset_body: Vector3<f64>,
}

impl MeasurementModel for GpsPositionModel {
    fn dim(&self) -> usize {
        3
    }

    /// Predicts antenna position in the ENU world frame.
    ///
    /// `predicted = P_world + R(q_body→world) * antenna_offset_body`.
    /// Does not require `tf` — the lever-arm rotation comes from the filter's
    /// own orientation estimate.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        _tf: Option<&dyn TfProvider>,
    ) -> Option<DVector<f64>> {
        let body_position_world = filter_state.get_vector3(&StateVariable::Px(FrameId::World))?;
        let body_orientation_world = filter_state.get_orientation().unwrap_or_default();
        let antenna_offset_world = body_orientation_world * self.antenna_offset_body;
        let predicted_antenna_position_world = body_position_world + antenna_offset_world;
        Some(DVector::from_row_slice(
            predicted_antenna_position_world.as_slice(),
        ))
    }

    // Default finite-diff jacobian is used; explicit impl is not needed because
    // the analytic Jacobian for a lever-arm GPS is short but the finite-diff one
    // is correct and adequate at filter rates.
}

#[cfg(test)]
mod tests {
    //! Tests for [`GpsPositionModel`].
    //!
    //! Properties validated:
    //! - `predict_measurement`: with zero lever arm returns body position directly;
    //!   with a non-zero lever arm adds the rotated offset to the predicted
    //!   antenna position.
    //! - Default Jacobian shape and identity of the position columns (∂z/∂Px ≈ 1).

    use super::*;
    use crate::data::primitives::FrameHandle;
    use crate::frames::{FrameAwareState, FrameId, StateVariable};
    use nalgebra::Vector3;

    const AGENT: FrameHandle = FrameHandle(1);

    /// A minimal 7-element state layout: [Px, Py, Pz, Qx, Qy, Qz, Qw].
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

    fn make_model(offset: Vector3<f64>) -> GpsPositionModel {
        GpsPositionModel {
            antenna_offset_body: offset,
        }
    }

    #[test]
    fn dim_is_three() {
        assert_eq!(make_model(Vector3::zeros()).dim(), 3);
    }

    #[test]
    fn predict_no_lever_arm_returns_body_position() {
        let model = make_model(Vector3::zeros());
        let state = make_state(3.0, 4.0, 5.0);
        let z = model.predict_measurement(&state, None).unwrap();
        assert!((z[0] - 3.0).abs() < 1e-9);
        assert!((z[1] - 4.0).abs() < 1e-9);
        assert!((z[2] - 5.0).abs() < 1e-9);
    }

    #[test]
    fn predict_lever_arm_adds_rotated_offset() {
        let offset = Vector3::new(0.5, 0.0, 0.1);
        let model = make_model(offset);
        let state = make_state(1.0, 2.0, 3.0);
        let z = model.predict_measurement(&state, None).unwrap();
        assert!((z[0] - 1.5).abs() < 1e-9);
        assert!((z[1] - 2.0).abs() < 1e-9);
        assert!((z[2] - 3.1).abs() < 1e-9);
    }

    #[test]
    fn jacobian_position_columns_are_identity() {
        let model = make_model(Vector3::zeros());
        let state = make_state(0.0, 0.0, 0.0);
        let h = model.jacobian(&state, None);
        assert_eq!(h.nrows(), 3);
        assert_eq!(h.ncols(), state.dim());
        assert!((h[(0, 0)] - 1.0).abs() < 1e-4);
        assert!((h[(1, 1)] - 1.0).abs() < 1e-4);
        assert!((h[(2, 2)] - 1.0).abs() < 1e-4);
    }
}
