use crate::spatial::tf::TfProvider;
use crate::prelude::AgentId;
use crate::prelude::MonotonicTime;
use crate::estimation::measurement::{MeasurementModel, Prediction, Unavailable};
use crate::estimation::schema::{MeasurementSchema, MeasurementSchemaBlock};
use crate::spatial::conventions::{Enu, Flu};
use crate::spatial::quantities::FreeVector;
use crate::spatial::transforms::{Convention, Rotation};
use crate::spatial::{FrameAwareState, FrameId};
use crate::spatial::state::Quantity;

use nalgebra::{DVector, Vector3};

/// What the filter believes an accelerometer reports: **specific force**, the
/// quantity an accelerometer physically senses.
///
/// "Specific" means *per unit mass* — as in specific heat or specific impulse —
/// so specific force is force divided by mass and carries units of m/s². It is
/// not measured in newtons, and the name does not imply that it is.
///
/// It is nonetheless **not** the kinematic acceleration `StateVariable::Ax`
/// carries. The two never agree while gravity acts:
///
/// - In free fall, an accelerometer reads **zero** while kinematic
///   acceleration is one g downward.
/// - At rest on a table, it reads **one g upward** while kinematic
///   acceleration is zero.
///
/// The model therefore predicts `a - g` rotated into the sensor frame, plus
/// the lever-arm terms a sensor mounted off the body origin also feels.
#[derive(Debug, Clone)]
pub struct SpecificForceModel {
    pub agent: AgentId,
    pub sensor: FrameId,
    pub gravity_world: Vector3<f64>,
}

impl MeasurementModel for SpecificForceModel {
    /// One block: specific force in the sensor frame (FLU). Specific force is
    /// its own quantity, distinct from the state's kinematic `Acceleration`, so
    /// the construction-time agreement check never conflates the two.
    fn schema(&self) -> MeasurementSchema {
        let frame = self.sensor.clone();
        let blocks = vec![MeasurementSchemaBlock::new(
            Quantity::SpecificForce(frame),
            Convention::Flu,
        )];

        MeasurementSchema::compose(blocks)
    }

    /// Predicts the proper acceleration measured by an accelerometer in its sensor frame.
    ///
    /// The body→sensor transform is required to project the predicted acceleration
    /// into the sensor frame, so this returns [`Prediction::Unavailable`]
    /// ([`Unavailable::NoProvider`] with no `tf`, [`Unavailable::MissingTransform`]
    /// when the sensor→base_link edge does not resolve).
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

        // Sensor-in-body: translation is the lever arm (sensor origin in body
        // axes); rotation maps sensor axes into body axes.
        let lever_arm_body = iso.translation.vector;
        let rot_body_from_sensor = iso.rotation;

        let linear_accel_body = filter_state
            .acceleration::<Flu>(body_frame.clone())
            .map(FreeVector::into_inner)
            .unwrap_or_default();
        let angular_vel_body = filter_state
            .angular_velocity::<Flu>(body_frame.clone())
            .map(FreeVector::into_inner)
            .unwrap_or_default();
        let angular_accel_body = filter_state
            .angular_acceleration::<Flu>(body_frame.clone())
            .map(FreeVector::into_inner)
            .unwrap_or_default();
        let orientation_body_to_world = filter_state
            .orientation::<Flu, Enu>(body_frame.clone(), FrameId::odom(self.agent.clone()))
            .map(Rotation::into_inner)
            .unwrap_or_default();

        let tangential_accel = angular_accel_body.cross(&lever_arm_body);
        let centripetal_accel = angular_vel_body.cross(&angular_vel_body.cross(&lever_arm_body));
        let total_kinematic_accel_at_sensor =
            linear_accel_body + tangential_accel + centripetal_accel;

        let q_body_from_world = orientation_body_to_world.inverse();

        let gravity_effect_in_body = q_body_from_world * self.gravity_world;

        let proper_accel_in_body_frame = total_kinematic_accel_at_sensor - gravity_effect_in_body;
        let predicted_accel = rot_body_from_sensor.inverse() * proper_accel_in_body_frame;

        let mut z_pred = DVector::zeros(3);
        z_pred.fixed_rows_mut::<3>(0).copy_from(&predicted_accel);
        Prediction::Ready(z_pred)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spatial::tf::TfProvider;
    use crate::prelude::AgentId;
    use crate::prelude::MonotonicTime;
    use crate::estimation::carrier::kinematic_carrier_schema;
    use crate::spatial::transforms::{Convention, ErasedTransform};
    use crate::spatial::{FrameAwareState, FrameId};
    use crate::spatial::state::Quantity;

    use nalgebra::{Isometry3, UnitQuaternion};
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
    /// swapped argument order projects gravity through the wrong rotation and
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

    fn make_model() -> SpecificForceModel {
        SpecificForceModel {
            agent: agent(),
            sensor: sensor(),
            gravity_world: Vector3::new(0.0, 0.0, -9.81),
        }
    }

    // A composed kinematic state carrying the orientation the model reads. Its
    // body-frame acceleration and angular-acceleration reads have no block here
    // and fall back to zero, as they do against a real INS estimate.
    fn make_state() -> FrameAwareState {
        FrameAwareState::from_schema(Arc::new(kinematic_carrier_schema(agent())), 0.0)
    }

    #[test]
    fn schema_is_three_long_and_tags_the_sensor_frame_specific_force() {
        let schema = make_model().schema();
        assert_eq!(schema.dim(), 3);
        assert_eq!(schema.blocks().len(), 1);
        let block = &schema.blocks()[0];
        // Specific force, not kinematic acceleration — its own quantity, so the
        // agreement check never mistakes it for the state's Acceleration block.
        assert_eq!(block.quantity(), &Quantity::SpecificForce(sensor()));
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

    #[test]
    fn gravity_is_projected_through_the_mount_rotation() {
        // Body level (identity orientation) and at rest: the only specific force
        // is the +1 g reaction, +9.81 along body +Z. The sensor is rolled +90°
        // about body +X, so expressing the body-frame vector in sensor axes maps
        // body +Z onto sensor +Y — the reading is +9.81 along sensor +Y. This
        // pins the *mount rotation direction*: querying base_link-in-sensor
        // instead of sensor-in-body would rotate gravity the opposite way, onto
        // sensor −Y, and fail.
        let model = make_model();
        let state = make_state();
        let mount = MountTf(Isometry3::from_parts(
            nalgebra::Translation3::identity(),
            UnitQuaternion::from_euler_angles(FRAC_PI_2, 0.0, 0.0),
        ));
        let Prediction::Ready(z) = model.predict_measurement(&state, Some(&mount), AT) else {
            panic!("a resolvable mount yields a ready prediction");
        };
        assert!(z[0].abs() < 1e-9);
        assert!((z[1] - 9.81).abs() < 1e-9);
        assert!(z[2].abs() < 1e-9);
    }
}
