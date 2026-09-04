use crate::{
    data::{ports::TfProvider, primitives::FrameHandle, MonotonicTime},
    estimation::measurement::MeasurementModel,
    frames::{
        conventions::{Enu, Flu},
        quantities::FreeVector,
        transforms::Rotation,
        FrameAwareState, FrameId,
    },
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
        at: MonotonicTime,
    ) -> Option<DVector<f64>> {
        let tf = tf?;

        let orientation_body_to_world = filter_state
            .orientation::<Flu, Enu>(
                FrameId::Body(self.agent_handle),
                FrameId::Odom(self.agent_handle),
            )
            .map(Rotation::into_inner)
            .unwrap_or_default();
        let q_body_from_world = orientation_body_to_world.inverse();
        let predicted_mag_body = q_body_from_world * self.world_magnetic_field;

        let erased = tf.get_transform(
            FrameId::Body(self.agent_handle),
            FrameId::Sensor(self.sensor_handle),
            at,
        )?;

        let Ok(tf_sensor_from_body) = erased.typed::<Flu, Flu>() else {
            return None;
        };

        let iso = tf_sensor_from_body.into_inner();

        let rot_sensor_from_body = iso.rotation;

        let sensor = FrameId::Sensor(self.sensor_handle);
        let bias = filter_state
            .mag_bias::<Flu>(sensor)
            .map(FreeVector::into_inner)
            .unwrap_or_else(Vector3::zeros);
        let predicted_sensor = rot_sensor_from_body.inverse() * predicted_mag_body + bias;

        Some(DVector::from_row_slice(predicted_sensor.as_slice()))
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
    //! - Bias block present: the estimated hard-iron bias adds into the sensor-
    //!   frame prediction, and the measurement Jacobian carries identity columns
    //!   over the bias slots.
    //! - Bias block absent: an un-augmented state predicts exactly as before —
    //!   the bias read falls back to zero.

    use super::*;
    use crate::data::primitives::FrameHandle;
    use crate::data::MonotonicTime;
    use crate::estimation::schema::{StateSchemaBlock, StateSchema};
    use crate::frames::transforms::{Convention, ErasedTransform};
    use crate::frames::{FrameAwareState, FrameId, StateVariable};
    use crate::manifold::TangentNoise;
    use crate::state::{Component, Quantity};
    use nalgebra::{DMatrix, DVector, Isometry3, Translation3, UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;
    use std::sync::Arc;

    // Isotropic 3-DOF noise; an orientation block cannot be built noiseless, and
    // the value is irrelevant to these prediction tests (none runs a predict step).
    fn noise() -> Option<TangentNoise> {
        Some(TangentNoise::from_variances(DVector::from_element(3, 0.1)).unwrap())
    }

    // The body → World orientation block, seeded to the identity quaternion
    // `[x, y, z, w] = [0, 0, 0, 1]`.
    fn orientation_block() -> StateSchemaBlock {
        StateSchemaBlock::orientation(
            FrameId::Body(AGENT),
            FrameId::Odom(AGENT),
            Convention::Flu,
            Convention::Enu,
            noise(),
            DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
            DMatrix::identity(3, 3),
        )
    }

    const AGENT: FrameHandle = FrameHandle(1);
    const SENSOR: FrameHandle = FrameHandle(2);
    const AT: MonotonicTime = MonotonicTime(0.0);

    /// Reports one fixed extrinsic — the sensor's pose in body axes — for every
    /// lookup, mirroring what a real TF tree hands the model.
    struct Mount(Isometry3<f64>);

    impl TfProvider for Mount {
        fn get_transform(
            &self,
            _from: FrameId,
            _to: FrameId,
            _at: MonotonicTime,
        ) -> Option<ErasedTransform> {
            Some(ErasedTransform::from_parts(
                self.0,
                Convention::Flu,
                Convention::Flu,
            ))
        }
    }

    fn identity_mount() -> Mount {
        Mount(Isometry3::identity())
    }

    fn make_orientation_state() -> FrameAwareState {
        let schema = StateSchema::compose(vec![orientation_block()]);
        FrameAwareState::from_schema(Arc::new(schema), 0.0)
    }

    fn set_yaw_90_ccw(state: &mut FrameAwareState) {
        let q = UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2);
        let (body, world) = (FrameId::Body(AGENT), FrameId::Odom(AGENT));
        state.set_variable(
            &StateVariable::new(
                Quantity::Orientation {
                    from: body.clone(),
                    to: world.clone(),
                },
                Component::X,
            ),
            q.i,
        );
        state.set_variable(
            &StateVariable::new(
                Quantity::Orientation {
                    from: body.clone(),
                    to: world.clone(),
                },
                Component::Y,
            ),
            q.j,
        );
        state.set_variable(
            &StateVariable::new(
                Quantity::Orientation {
                    from: body.clone(),
                    to: world.clone(),
                },
                Component::Z,
            ),
            q.k,
        );
        state.set_variable(
            &StateVariable::new(
                Quantity::Orientation {
                    from: body,
                    to: world,
                },
                Component::W,
            ),
            q.w,
        );
    }

    fn make_model() -> MagneticFieldModel {
        MagneticFieldModel {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            world_magnetic_field: Vector3::new(0.0, 1.0, 0.0),
        }
    }

    /// A state carrying the orientation quaternion followed by a magnetometer
    /// hard-iron bias block for `SENSOR`, the bias initialised to `bias`.
    /// Orientation is identity, so the field prediction isolates the bias term.
    fn make_augmented_state(bias: Vector3<f64>) -> FrameAwareState {
        let sensor = FrameId::Sensor(SENSOR);
        let schema = StateSchema::compose(vec![
            orientation_block(),
            StateSchemaBlock::new(
                Quantity::MagBias(sensor.clone()),
                Convention::Flu,
                noise(),
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
        ]);
        let mut state = FrameAwareState::from_schema(Arc::new(schema), 0.0);
        state.set_variable(
            &StateVariable::new(Quantity::MagBias(sensor.clone()), Component::X),
            bias.x,
        );
        state.set_variable(
            &StateVariable::new(Quantity::MagBias(sensor.clone()), Component::Y),
            bias.y,
        );
        state.set_variable(
            &StateVariable::new(Quantity::MagBias(sensor), Component::Z),
            bias.z,
        );
        state
    }

    #[test]
    fn dim_is_three() {
        assert_eq!(make_model().dim(), 3);
    }

    #[test]
    fn predict_without_tf_returns_none() {
        let model = make_model();
        let state = make_orientation_state();
        assert!(model.predict_measurement(&state, None, AT).is_none());
    }

    #[test]
    fn predict_identity_orientation_returns_world_field() {
        let model = make_model();
        let state = make_orientation_state();
        let z = model
            .predict_measurement(&state, Some(&identity_mount()), AT)
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
            .predict_measurement(&state, Some(&identity_mount()), AT)
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
        let z = model.predict_measurement(&state, Some(&mount), AT).unwrap();
        assert!((z[0] - 1.0).abs() < 1e-9);
        assert!(z[1].abs() < 1e-9);
        assert!(z[2].abs() < 1e-9);
    }

    #[test]
    fn predict_adds_the_estimated_bias_when_the_block_is_present() {
        // An augmented state carries a per-sensor hard-iron bias. With identity
        // orientation and identity mount the field prediction is the world field
        // itself, so the measurement is that field plus the bias, component-wise.
        let model = make_model();
        let bias = Vector3::new(0.1, -0.2, 0.05);
        let state = make_augmented_state(bias);
        let z = model
            .predict_measurement(&state, Some(&identity_mount()), AT)
            .unwrap();
        assert!((z[0] - bias.x).abs() < 1e-9);
        assert!((z[1] - (1.0 + bias.y)).abs() < 1e-9);
        assert!((z[2] - bias.z).abs() < 1e-9);
    }

    #[test]
    fn predict_without_a_bias_block_is_unchanged() {
        // The un-augmented state has no bias slots, so the bias read abstains and
        // the prediction is the bare world field — bit-identical to the model
        // before hard-iron estimation existed. This pins the zero-fallback that
        // keeps the base (16-state) filter's behaviour frozen.
        let model = make_model();
        let state = make_orientation_state();
        let z = model
            .predict_measurement(&state, Some(&identity_mount()), AT)
            .unwrap();
        assert!(z[0].abs() < 1e-9);
        assert!((z[1] - 1.0).abs() < 1e-9);
        assert!(z[2].abs() < 1e-9);
    }

    #[test]
    fn jacobian_carries_identity_over_the_bias_columns() {
        // The bias enters the prediction additively (z = h(x) + b), so ∂z/∂b is
        // the identity. Because the default finite-difference Jacobian perturbs
        // every tangent slot, those identity columns appear with no analytic
        // override — and that is exactly what makes the appended bias block
        // observable to the filter's update. H is tangent-indexed, so the bias
        // columns sit at the block's *tangent* offset (below its storage offset,
        // the orientation block ahead of it spending one fewer tangent than it
        // stores).
        let model = make_model();
        let sensor = FrameId::Sensor(SENSOR);
        let state = make_augmented_state(Vector3::zeros());
        let off = state
            .schema()
            .tangent_offset_of(&StateVariable::new(Quantity::MagBias(sensor), Component::X))
            .expect("augmented state carries the bias block");
        let h = model.jacobian(&state, Some(&identity_mount()), AT);
        for r in 0..3 {
            for c in 0..3 {
                let expected = if r == c { 1.0 } else { 0.0 };
                assert!(
                    (h[(r, off + c)] - expected).abs() < 1e-6,
                    "bias Jacobian block[{r}][{c}] = {}, expected {expected}",
                    h[(r, off + c)]
                );
            }
        }
    }
}
