use nalgebra::{DMatrix, DVector, Vector3};
use std::any::Any;
use std::fmt::Debug;

// --- Core Library Imports ---
use crate::frames::{FrameAwareState, FrameId, StateVariable};
use crate::messages::{MeasurementData, MeasurementMessage};
use crate::models::estimation::measurement::Measurement;
use crate::types::TfProvider;

/// A measurement model for a standard GPS sensor that provides 3D position.
///
/// This model relates a 3D position measurement (in the ENU world frame)
/// to the position states (Px, Py, Pz) of the filter's state vector.
#[derive(Debug, Clone)]
pub struct GpsPositionModel {
    /// The 3x3 measurement noise covariance matrix, R.
    /// This matrix represents the uncertainty of the GPS reading itself,
    /// with variances for East, North, and Up on the diagonal.
    pub r_matrix: DMatrix<f64>,

    /// The physical location of the GPS antenna relative to the body's origin,
    /// expressed in the body's coordinate frame. This is a constant.
    pub antenna_offset_body: Vector3<f64>,
}

impl Measurement for GpsPositionModel {
    /// Describes the semantic layout of the measurement vector `z`.
    /// For this model, it's a 3D position in the world frame.
    fn get_measurement_layout(&self) -> Vec<StateVariable> {
        let world_frame = FrameId::World;
        vec![
            StateVariable::Px(world_frame.clone()),
            StateVariable::Py(world_frame.clone()),
            StateVariable::Pz(world_frame.clone()),
        ]
    }

    fn get_measurement_vector(&self, data: &MeasurementData) -> Option<DVector<f64>> {
        if let MeasurementData::GpsPosition(vec) = data {
            Some(DVector::from_row_slice(vec.as_slice()))
        } else {
            None // I am a GPS model, I only handle GpsPosition data.
        }
    }

    /// Predicts the ideal measurement `z_pred` from the filter's state.
    /// It returns `Some` only if the input message contains `GpsPosition` data.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        message: &MeasurementMessage,
        _tf: &dyn TfProvider,
    ) -> Option<DVector<f64>> {
        // This model only cares about `GpsPosition` data. It ignores everything else.
        if !matches!(&message.data, MeasurementData::GpsPosition(_)) {
            return None;
        }

        // --- 1. Get the filter's current estimate of the BODY's state ---
        let body_position_world = filter_state.get_vector3(&StateVariable::Px(FrameId::World))?;
        let body_orientation_world = filter_state.get_orientation().unwrap_or_default();

        // --- 2. Calculate the Lever Arm Effect ---
        // We have the antenna's position relative to the body, in the body frame.
        // We need to rotate this offset vector into the world frame.
        let antenna_offset_world = body_orientation_world * self.antenna_offset_body;

        // --- 3. Calculate the final predicted antenna position ---
        // The predicted GPS measurement is the body's position plus the rotated offset.
        let predicted_antenna_position_world = body_position_world + antenna_offset_world;

        Some(DVector::from_row_slice(
            predicted_antenna_position_world.as_slice(),
        ))
    }

    /// Calculates the measurement Jacobian, H.
    /// H maps a change in the full state vector to a change in the predicted measurement.
    /// For this model, H will be a 3x16 matrix.
    /// Calculates the measurement Jacobian, H = ∂h/∂x.
    ///
    /// Because the predicted measurement `z_pred = P_world + R(q) * r_body` now
    /// depends on the orientation `q`, this Jacobian will have non-zero entries
    /// for both the position and orientation states.
    ///
    /// We use numerical differentiation (finite differencing) for a robust and
    /// easy-to-maintain implementation.
    fn calculate_jacobian(
        &self,
        filter_state: &FrameAwareState,
        tf: &dyn TfProvider,
    ) -> DMatrix<f64> {
        let state_dim = filter_state.dim();
        let measurement_dim = 3; // 3D position
        let mut h_jac = DMatrix::zeros(measurement_dim, state_dim);

        // A small perturbation value for the finite difference calculation.
        let epsilon = 1e-8;

        // We need a dummy message to pass to predict_measurement.
        // The data variant must be one this model accepts (`GpsPosition`).
        let dummy_message = MeasurementMessage {
            agent_handle: Default::default(),
            sensor_handle: Default::default(),
            timestamp: 0.0,
            data: MeasurementData::GpsPosition(Default::default()),
        };

        // 1. Calculate the baseline predicted measurement with the current state.
        // If the model can't predict, we can't calculate a Jacobian, so we return zeros.
        let z_base = match self.predict_measurement(filter_state, &dummy_message, tf) {
            Some(z) => z,
            None => return h_jac,
        };

        // 2. Iterate through each element of the state vector.
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
                // Approximate the partial derivative: (f(x + h) - f(x)) / h
                let derivative_column = (z_perturbed - &z_base) / epsilon;
                h_jac.column_mut(j).copy_from(&derivative_column);
            }
        }

        h_jac
    }

    /// Returns the measurement noise covariance matrix, R.
    fn get_r(&self) -> &DMatrix<f64> {
        &self.r_matrix
    }

    /// Allows for dynamic downcasting.
    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    //! Tests for [`GpsPositionModel`].
    //!
    //! Properties validated:
    //! - Type dispatch: accepts `GpsPosition`, rejects all other variants.
    //! - `predict_measurement`: returns `None` for non-GPS messages; with zero
    //!   lever arm returns body position directly; with a non-zero lever arm adds
    //!   the rotated offset to the predicted antenna position.
    //! - Jacobian shape and identity of the position columns (∂z/∂Px ≈ 1).

    use super::*;
    use crate::frames::{FrameAwareState, FrameId, StateVariable};
    use crate::messages::{MeasurementData, MeasurementMessage};
    use crate::types::{FrameHandle, TfProvider};
    use nalgebra::{DMatrix, Isometry3, Vector3};

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

    /// A minimal 7-element state layout: [Px, Py, Pz, Qx, Qy, Qz, Qw].
    ///
    /// `FrameAwareState::new` initialises Qw = 1.0 (identity quaternion) automatically.
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
        state.vector[0] = px;
        state.vector[1] = py;
        state.vector[2] = pz;
        state
    }

    fn make_model(offset: Vector3<f64>) -> GpsPositionModel {
        GpsPositionModel {
            r_matrix: DMatrix::identity(3, 3) * 0.1,
            antenna_offset_body: offset,
        }
    }

    fn gps_message(x: f64, y: f64, z: f64) -> MeasurementMessage {
        MeasurementMessage {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            timestamp: 0.1,
            data: MeasurementData::GpsPosition(Vector3::new(x, y, z)),
        }
    }

    fn imu_message() -> MeasurementMessage {
        MeasurementMessage {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            timestamp: 0.1,
            data: MeasurementData::Imu6Dof(Default::default()),
        }
    }

    // ── Type dispatch ─────────────────────────────────────────────────────────

    #[test]
    fn measurement_vector_accepts_gps_position() {
        let model = make_model(Vector3::zeros());
        let data = MeasurementData::GpsPosition(Vector3::new(1.0, 2.0, 3.0));
        let z = model.get_measurement_vector(&data);
        assert!(z.is_some());
        let z = z.unwrap();
        assert_eq!(z.nrows(), 3);
        assert!((z[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn measurement_vector_rejects_imu() {
        let model = make_model(Vector3::zeros());
        let data = MeasurementData::Imu6Dof(Default::default());
        assert!(model.get_measurement_vector(&data).is_none());
    }

    // ── predict_measurement ───────────────────────────────────────────────────

    #[test]
    fn predict_rejects_non_gps_message() {
        let model = make_model(Vector3::zeros());
        let state = make_state(1.0, 2.0, 3.0);
        let tf = IdentityTf;
        let result = model.predict_measurement(&state, &imu_message(), &tf);
        assert!(result.is_none(), "non-GPS message must yield None");
    }

    #[test]
    fn predict_no_lever_arm_returns_body_position() {
        // Zero antenna offset: predicted measurement = body position exactly.
        let model = make_model(Vector3::zeros());
        let state = make_state(3.0, 4.0, 5.0);
        let tf = IdentityTf;
        let z = model
            .predict_measurement(&state, &gps_message(0.0, 0.0, 0.0), &tf)
            .unwrap();

        assert!(
            (z[0] - 3.0).abs() < 1e-9,
            "predicted x should equal body px"
        );
        assert!(
            (z[1] - 4.0).abs() < 1e-9,
            "predicted y should equal body py"
        );
        assert!(
            (z[2] - 5.0).abs() < 1e-9,
            "predicted z should equal body pz"
        );
    }

    #[test]
    fn predict_lever_arm_adds_rotated_offset() {
        // With identity orientation, the body-frame offset is not rotated.
        // Predicted = body_position + antenna_offset_body.
        let offset = Vector3::new(0.5, 0.0, 0.1);
        let model = make_model(offset);
        let state = make_state(1.0, 2.0, 3.0);
        let tf = IdentityTf;
        let z = model
            .predict_measurement(&state, &gps_message(0.0, 0.0, 0.0), &tf)
            .unwrap();

        assert!((z[0] - 1.5).abs() < 1e-9, "predicted x = px + offset_x");
        assert!(
            (z[1] - 2.0).abs() < 1e-9,
            "predicted y = py + offset_y (zero)"
        );
        assert!((z[2] - 3.1).abs() < 1e-9, "predicted z = pz + offset_z");
    }

    // ── Jacobian ─────────────────────────────────────────────────────────────

    #[test]
    fn jacobian_position_columns_are_identity() {
        // With zero lever arm and identity orientation, ∂z/∂Px = 1, ∂z/∂Py = 1,
        // ∂z/∂Pz = 1. The state layout places position at indices 0, 1, 2.
        let model = make_model(Vector3::zeros());
        let state = make_state(0.0, 0.0, 0.0);
        let tf = IdentityTf;
        let h = model.calculate_jacobian(&state, &tf);

        assert_eq!(h.nrows(), 3, "H has 3 rows (measurement dim)");
        assert_eq!(
            h.ncols(),
            state.dim(),
            "H has one column per state variable"
        );
        assert!((h[(0, 0)] - 1.0).abs() < 1e-5, "H(0,0) = ∂z_x/∂Px ≈ 1.0");
        assert!((h[(1, 1)] - 1.0).abs() < 1e-5, "H(1,1) = ∂z_y/∂Py ≈ 1.0");
        assert!((h[(2, 2)] - 1.0).abs() < 1e-5, "H(2,2) = ∂z_z/∂Pz ≈ 1.0");
    }
}
