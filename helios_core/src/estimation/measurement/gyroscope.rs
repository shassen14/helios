use nalgebra::DVector;

use crate::data::ports::TfProvider;
use crate::prelude::AgentId;
use crate::prelude::MonotonicTime;
use crate::estimation::measurement::{MeasurementModel, Prediction, Unavailable};
use crate::estimation::schema::{MeasurementSchema, MeasurementSchemaBlock};
use crate::frames::conventions::Flu;
use crate::frames::quantities::FreeVector;
use crate::frames::transforms::Convention;
use crate::frames::{FrameAwareState, FrameId};
use crate::state::Quantity;

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
    pub agent: AgentId,
    pub sensor: FrameId,
}

impl MeasurementModel for AngularRateModel {
    /// One block: the body's angular velocity resolved in the sensor frame
    /// (FLU) — exactly what `predict_measurement` returns.
    fn schema(&self) -> MeasurementSchema {
        let frame = self.sensor.clone();
        let blocks = vec![MeasurementSchemaBlock::new(
            Quantity::AngularVelocity(frame),
            Convention::Flu,
        )];

        MeasurementSchema::compose(blocks)
    }

    /// Predicts angular velocity in the sensor frame.
    ///
    /// The body→sensor rotation is required, so this returns
    /// [`Prediction::Unavailable`] ([`Unavailable::NoProvider`] with no `tf`,
    /// [`Unavailable::MissingTransform`] when the sensor→base_link edge does not
    /// resolve).
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        tf: Option<&dyn TfProvider>,
        at: MonotonicTime,
    ) -> Prediction {
        let Some(tf) = tf else {
            return Prediction::Unavailable(Unavailable::NoProvider);
        };
        let body_frame = FrameId::base_link(self.agent.clone());

        let Some(erased) = tf.get_transform(
            self.sensor.clone(),
            FrameId::base_link(self.agent.clone()),
            at,
        ) else {
            return Prediction::Unavailable(Unavailable::MissingTransform {
                from: self.sensor.clone(),
                to: FrameId::base_link(self.agent.clone()),
            });
        };

        let Ok(sensor_in_body) = erased.typed::<Flu, Flu>() else {
            return Prediction::Unavailable(Unavailable::ConventionMismatch {
                from: self.sensor.clone(),
                to: FrameId::base_link(self.agent.clone()),
            });
        };

        let iso = sensor_in_body.into_inner();

        // Sensor-in-body rotation maps sensor axes into body axes; its inverse
        // resolves the body's angular velocity into the sensor frame.
        let rot_body_from_sensor = iso.rotation;

        let angular_vel_body = filter_state
            .angular_velocity::<Flu>(body_frame.clone())
            .map(FreeVector::into_inner)
            .unwrap_or_default();

        let predicted_gyro = rot_body_from_sensor.inverse() * angular_vel_body;
        let mut z_pred = DVector::zeros(3);
        z_pred.fixed_rows_mut::<3>(0).copy_from(&predicted_gyro);
        Prediction::Ready(z_pred)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data::ports::TfProvider;
    use crate::prelude::AgentId;
    use crate::prelude::MonotonicTime;
    use crate::estimation::carrier::kinematic_carrier_schema;
    use crate::estimation::schema::{StateSchema, StateSchemaBlock};
    use crate::frames::transforms::{Convention, ErasedTransform};
    use crate::frames::{FrameAwareState, FrameId, StateVariable};
    use crate::manifold::TangentNoise;
    use crate::state::Component;
    use nalgebra::{DMatrix, Isometry3, Translation3, UnitQuaternion};
    use std::f64::consts::FRAC_PI_2;
    use std::sync::Arc;

    fn agent() -> AgentId {
        AgentId::new("test_agent")
    }

    fn sensor() -> FrameId {
        FrameId::sensor(agent(), "imu")
    }

    const AT: MonotonicTime = MonotonicTime(0.0);

    /// A provider that is present but resolves no edges — the shape of the
    /// historical silent-aiding-drop bug (broken tf graph / mislabelled frame).
    struct NoEdgeTf;

    impl TfProvider for NoEdgeTf {
        fn get_transform(
            &self,
            _from: FrameId,
            _to: FrameId,
            _at: MonotonicTime,
        ) -> Option<ErasedTransform> {
            None
        }
    }

    struct IdentityTf;
    impl TfProvider for IdentityTf {
        fn get_transform(
            &self,
            _from: FrameId,
            _to: FrameId,
            _at: MonotonicTime,
        ) -> Option<ErasedTransform> {
            Some(ErasedTransform::from_parts(
                Isometry3::identity(),
                Convention::Flu,
                Convention::Flu,
            ))
        }
    }

    /// Holds the sensor's mount *in body axes* (sensor-in-body). Honours the
    /// canonical [`TfProvider::get_transform`] direction — the stored isometry
    /// for `get_transform(sensor, base_link)`, its inverse for the reverse — so a
    /// swapped argument order resolves the rate through the wrong rotation and
    /// fails the assertion.
    struct MountTf(Isometry3<f64>);

    impl TfProvider for MountTf {
        fn get_transform(
            &self,
            from: FrameId,
            _to: FrameId,
            _at: MonotonicTime,
        ) -> Option<ErasedTransform> {
            let iso = if from.is_sensor() {
                self.0
            } else {
                self.0.inverse()
            };
            Some(ErasedTransform::from_parts(
                iso,
                Convention::Flu,
                Convention::Flu,
            ))
        }
    }

    fn make_model() -> AngularRateModel {
        AngularRateModel {
            agent: agent(),
            sensor: sensor(),
        }
    }

    // A composed kinematic state. The gyroscope reads *body*-frame angular
    // velocity, which the carrier (world-frame angular velocity only) does not
    // hold, so the read falls back to zero — the test pins shape and TF-gating,
    // not the rate value.
    fn make_state() -> FrameAwareState {
        FrameAwareState::from_schema(Arc::new(kinematic_carrier_schema(agent())), 0.0)
    }

    #[test]
    fn schema_is_three_long_and_tags_the_sensor_frame_angular_velocity() {
        let schema = make_model().schema();
        // The schema is now the sole source of the measurement length.
        assert_eq!(schema.dim(), 3);
        // One block: angular velocity in the sensor frame, expressed FLU.
        assert_eq!(schema.blocks().len(), 1);
        let block = &schema.blocks()[0];
        assert_eq!(block.quantity(), &Quantity::AngularVelocity(sensor()));
        assert_eq!(block.conventions, vec![(sensor(), Convention::Flu)]);
    }

    #[test]
    fn predict_without_tf_returns_none() {
        let model = make_model();
        let state = make_state();
        assert_eq!(
            model.predict_measurement(&state, None, AT),
            Prediction::Unavailable(Unavailable::NoProvider)
        );
    }

    #[test]
    fn predict_with_tf_returns_some() {
        let model = make_model();
        let state = make_state();
        let tf = IdentityTf;
        assert!(matches!(
            model.predict_measurement(&state, Some(&tf), AT),
            Prediction::Ready(_)
        ));
    }

    #[test]
    fn predict_with_unresolved_edge_reports_missing_transform() {
        let model = make_model();
        let state = make_state();
        // The provider is present but the sensor→base_link edge does not resolve:
        // the loud case the reason-carrying return exists to surface.
        assert_eq!(
            model.predict_measurement(&state, Some(&NoEdgeTf), AT),
            Prediction::Unavailable(Unavailable::MissingTransform {
                from: sensor(),
                to: FrameId::base_link(agent()),
            })
        );
    }

    #[test]
    fn jacobian_has_correct_shape() {
        let model = make_model();
        let state = make_state();
        let tf = IdentityTf;
        let h = model.jacobian(&state, Some(&tf), AT);
        assert_eq!(h.nrows(), 3);
        // H is tangent-sized: a quaternion block spends one fewer column than it stores.
        assert_eq!(h.ncols(), state.tangent_dim());
    }

    // A state carrying a *body-frame* angular velocity block (FLU) seeded to
    // `omega`. The carrier schema only holds world-frame rate, which the model
    // does not read, so this composes the block the gyro actually resolves.
    fn make_body_rate_state(omega: [f64; 3]) -> FrameAwareState {
        let body = FrameId::base_link(agent());
        let noise = TangentNoise::from_variances(nalgebra::DVector::from_element(3, 0.1)).unwrap();
        let schema = StateSchema::compose(vec![StateSchemaBlock::new(
            Quantity::AngularVelocity(body.clone()),
            Convention::Flu,
            Some(noise),
            nalgebra::DVector::zeros(3),
            DMatrix::identity(3, 3),
        )]);
        let mut state = FrameAwareState::from_schema(Arc::new(schema), 0.0);
        for (component, value) in [Component::X, Component::Y, Component::Z]
            .into_iter()
            .zip(omega)
        {
            state.set_variable(
                &StateVariable::new(Quantity::AngularVelocity(body.clone()), component),
                value,
            );
        }
        state
    }

    #[test]
    fn rate_is_resolved_through_the_mount_rotation() {
        // Body angular velocity is +1 rad/s about body +X. The sensor is yawed
        // +90° about body +Z, so resolving the body-frame rate into sensor axes
        // maps body +X onto sensor −Y — the reading is −1 along sensor +Y. This
        // pins the *direction* of the mount lookup: querying base_link-in-sensor
        // instead of sensor-in-body would rotate the rate the other way, to
        // sensor +Y, and fail.
        let model = make_model();
        let state = make_body_rate_state([1.0, 0.0, 0.0]);
        let mount = MountTf(Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2),
        ));
        let Prediction::Ready(z) = model.predict_measurement(&state, Some(&mount), AT) else {
            panic!("a resolvable mount yields a ready prediction");
        };
        assert!(z[0].abs() < 1e-9);
        assert!((z[1] + 1.0).abs() < 1e-9);
        assert!(z[2].abs() < 1e-9);
    }
}
