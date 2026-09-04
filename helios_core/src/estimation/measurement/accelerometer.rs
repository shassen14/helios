use crate::data::ports::TfProvider;
use crate::data::primitives::FrameHandle;
use crate::data::MonotonicTime;
use crate::estimation::measurement::MeasurementModel;
use crate::estimation::schema::{MeasurementSchema, MeasurementSchemaBlock};
use crate::frames::conventions::{Enu, Flu};
use crate::frames::quantities::FreeVector;
use crate::frames::transforms::{Convention, Rotation};
use crate::frames::{FrameAwareState, FrameId};
use crate::state::Quantity;

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
    pub agent_handle: FrameHandle,
    pub sensor_handle: FrameHandle,
    pub gravity_world: Vector3<f64>,
}

impl MeasurementModel for SpecificForceModel {
    fn dim(&self) -> usize {
        3
    }

    /// One block: specific force in the sensor frame (FLU). Specific force is
    /// its own quantity, distinct from the state's kinematic `Acceleration`, so
    /// the construction-time agreement check never conflates the two.
    fn schema(&self) -> MeasurementSchema {
        let frame = FrameId::Sensor(self.sensor_handle);
        let blocks = vec![MeasurementSchemaBlock::new(
            Quantity::SpecificForce(frame),
            Convention::Flu,
        )];

        MeasurementSchema::compose(blocks)
    }

    /// Predicts the proper acceleration measured by an accelerometer in its sensor frame.
    ///
    /// Returns `None` when `tf` is unavailable — the body→sensor transform is
    /// required to project the predicted acceleration into the sensor frame.
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        tf: Option<&dyn TfProvider>,
        at: MonotonicTime,
    ) -> Option<DVector<f64>> {
        let tf = tf?;
        let body_frame = FrameId::Body(self.agent_handle);

        let erased = tf.get_transform(
            FrameId::Body(self.agent_handle),
            FrameId::Sensor(self.sensor_handle),
            at,
        )?;

        let Ok(tf_sensor_from_body) = erased.typed::<Flu, Flu>() else {
            return None;
        };

        let iso = tf_sensor_from_body.into_inner();

        let r_body_to_sensor = iso.translation.vector;
        let rot_sensor_from_body = iso.rotation;

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
            .orientation::<Flu, Enu>(body_frame.clone(), FrameId::Odom(self.agent_handle))
            .map(Rotation::into_inner)
            .unwrap_or_default();

        let tangential_accel = angular_accel_body.cross(&r_body_to_sensor);
        let centripetal_accel = angular_vel_body.cross(&angular_vel_body.cross(&r_body_to_sensor));
        let total_kinematic_accel_at_sensor =
            linear_accel_body + tangential_accel + centripetal_accel;

        let q_body_from_world = orientation_body_to_world.inverse();

        let gravity_effect_in_body = q_body_from_world * self.gravity_world;

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
    use crate::data::ports::TfProvider;
    use crate::data::primitives::FrameHandle;
    use crate::data::MonotonicTime;
    use crate::estimation::carrier::kinematic_carrier_schema;
    use crate::estimation::schema::BlockConvention;
    use crate::frames::transforms::{Convention, ErasedTransform};
    use crate::frames::{FrameAwareState, FrameId};
    use crate::state::Quantity;

    use nalgebra::Isometry3;
    use std::sync::Arc;

    const AGENT: FrameHandle = FrameHandle(1);
    const SENSOR: FrameHandle = FrameHandle(2);
    const AT: MonotonicTime = MonotonicTime(0.0);

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

    fn make_model() -> SpecificForceModel {
        SpecificForceModel {
            agent_handle: AGENT,
            sensor_handle: SENSOR,
            gravity_world: Vector3::new(0.0, 0.0, -9.81),
        }
    }

    // A composed kinematic state carrying the orientation the model reads. Its
    // body-frame acceleration and angular-acceleration reads have no block here
    // and fall back to zero, as they do against a real INS estimate.
    fn make_state() -> FrameAwareState {
        FrameAwareState::from_schema(Arc::new(kinematic_carrier_schema(AGENT)), 0.0)
    }

    #[test]
    fn dim_is_three() {
        assert_eq!(make_model().dim(), 3);
    }

    #[test]
    fn schema_matches_dim_and_tags_the_sensor_frame_specific_force() {
        let schema = make_model().schema();
        assert_eq!(schema.dim(), make_model().dim());
        assert_eq!(schema.blocks().len(), 1);
        let block = &schema.blocks()[0];
        // Specific force, not kinematic acceleration — its own quantity, so the
        // agreement check never mistakes it for the state's Acceleration block.
        assert_eq!(
            block.quantity(),
            &Quantity::SpecificForce(FrameId::Sensor(SENSOR))
        );
        assert_eq!(block.convention(), &BlockConvention::Single(Convention::Flu));
    }

    #[test]
    fn predict_without_tf_returns_none() {
        let model = make_model();
        let state = make_state();
        assert!(model.predict_measurement(&state, None, AT).is_none());
    }

    #[test]
    fn predict_with_tf_returns_some() {
        let model = make_model();
        let state = make_state();
        let tf = IdentityTf;
        assert!(model.predict_measurement(&state, Some(&tf), AT).is_some());
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
}
