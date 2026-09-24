use nalgebra::DVector;

use crate::spatial::tf::TfProvider;
use crate::prelude::AgentId;
use crate::prelude::MonotonicTime;
use crate::estimation::measurement::MeasurementModel;
use crate::estimation::measurement::Prediction;
use crate::estimation::measurement::Unavailable;
use crate::estimation::schema::{MeasurementSchema, MeasurementSchemaBlock};
use crate::spatial::conventions::{Enu, Flu};
use crate::spatial::quantities::Point;
use crate::spatial::transforms::{Convention, Rotation};
use crate::spatial::{FrameAwareState, FrameId};
use crate::spatial::state::Quantity;

/// A measurement model for a standard GPS sensor that provides 3D position.
///
/// Maps a 3D ENU position measurement to the filter's position states
/// `(Px, Py, Pz)`, accounting for the antenna's physical offset from the body
/// origin via the TF tree (same pattern as [`SpecificForceModel`]).
///
/// Note: `R` (measurement noise covariance) is **not** held here. It lives at
/// the call site and is passed per `update`. See `algorithm_family_traits.md` §2.1.
///
/// [`SpecificForceModel`]: crate::estimation::measurement::accelerometer::SpecificForceModel
#[derive(Debug, Clone)]
pub struct GpsPositionModel {
    pub agent: AgentId,
    /// The GPS antenna's own frame. Used to look up the antenna's offset from the
    /// body origin via the TF tree at prediction time.
    pub sensor: FrameId,
}

impl MeasurementModel for GpsPositionModel {
    /// One block: antenna position in the agent's odom frame (ENU), keyed by the
    /// agent's odom `FrameId` — the same one the state carries — not the sensor.
    fn schema(&self) -> MeasurementSchema {
        let frame = FrameId::odom(self.agent.clone());
        let blocks = vec![MeasurementSchemaBlock::new(
            Quantity::Position(frame),
            Convention::Enu,
        )];

        MeasurementSchema::compose(blocks)
    }
    /// Predicts antenna position in the ENU world frame.
    ///
    /// Requires `tf` to resolve the body→antenna translation. Returns
    /// [`Prediction::Unavailable`] when a precondition is missing —
    /// [`Unavailable::NoProvider`] with no `tf`, [`Unavailable::ColdStart`] before
    /// the position block initializes, or [`Unavailable::MissingTransform`] when
    /// the sensor→base_link edge does not resolve.
    ///
    /// `predicted = P_world + R(q_body→world) * antenna_offset_body`
    /// where `antenna_offset_body` comes from `tf.get_transform(sensor, base_link).translation`
    /// (the sensor's origin expressed in body axes — see [`TfProvider::get_transform`]).
    fn predict_measurement(
        &self,
        filter_state: &FrameAwareState,
        tf: Option<&dyn TfProvider>,
        at: MonotonicTime,
    ) -> Prediction {
        let Some(tf) = tf else {
            return Prediction::Unavailable(Unavailable::NoProvider);
        };
        let Some(body_position_world) = filter_state
            .position::<Enu>(FrameId::odom(self.agent.clone()))
            .map(Point::into_inner)
        else {
            return Prediction::Unavailable(Unavailable::ColdStart);
        };
        let body_orientation_world = filter_state
            .orientation::<Flu, Enu>(
                FrameId::base_link(self.agent.clone()),
                FrameId::odom(self.agent.clone()),
            )
            .map(Rotation::into_inner)
            .unwrap_or_default();

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

        let antenna_offset_body = iso.translation.vector;

        let antenna_offset_world = body_orientation_world * antenna_offset_body;
        let predicted_antenna_position_world = body_position_world + antenna_offset_world;

        Prediction::Ready(DVector::from_row_slice(
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
    use crate::spatial::tf::TfProvider;
    use crate::prelude::AgentId;
    use crate::prelude::MonotonicTime;
    use crate::estimation::carrier::kinematic_carrier_schema;
    use crate::spatial::transforms::{Convention, ErasedTransform};
    use crate::spatial::{FrameAwareState, FrameId, StateVariable};
    use crate::spatial::state::{Component, Quantity};
    use nalgebra::{Isometry3, Translation3, UnitQuaternion};
    use std::sync::Arc;

    fn agent() -> AgentId {
        AgentId::new("test_agent")
    }

    fn sensor() -> FrameId {
        FrameId::sensor(agent(), "gps_antenna")
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

    /// Holds the sensor's mount *in body axes* (sensor-in-body). Honours the
    /// canonical [`TfProvider::get_transform`] direction: the stored isometry is
    /// returned for `get_transform(sensor, base_link)` and its inverse for the
    /// reverse query. Being direction-aware (unlike a value-only mock) is what
    /// lets these tests catch a swapped argument order.
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

    fn make_model() -> GpsPositionModel {
        GpsPositionModel {
            agent: agent(),
            sensor: sensor(),
        }
    }

    // A composed kinematic state with the world position seeded. The GPS model
    // reads the position block (present) and the orientation block (present); the
    // world-frame antenna position it predicts is that position plus the rotated
    // lever arm.
    fn make_state(px: f64, py: f64, pz: f64) -> FrameAwareState {
        let mut state =
            FrameAwareState::from_schema(Arc::new(kinematic_carrier_schema(agent())), 0.0);
        state.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::odom(agent())), Component::X),
            px,
        );
        state.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::odom(agent())), Component::Y),
            py,
        );
        state.set_variable(
            &StateVariable::new(Quantity::Position(FrameId::odom(agent())), Component::Z),
            pz,
        );
        state
    }

    #[test]
    fn schema_is_three_long_and_tags_the_odom_frame_position() {
        let schema = make_model().schema();
        assert_eq!(schema.dim(), 3);
        assert_eq!(schema.blocks().len(), 1);
        let block = &schema.blocks()[0];
        // Position in the agent's odom frame (ENU), keyed by the agent's odom
        // FrameId — the same one the state carries, so the agreement check lines up.
        assert_eq!(
            block.quantity(),
            &Quantity::Position(FrameId::odom(agent()))
        );
        assert_eq!(
            block.conventions,
            vec![(FrameId::odom(agent()), Convention::Enu)]
        );
    }

    #[test]
    fn predict_without_tf_returns_none() {
        let model = make_model();
        let state = make_state(3.0, 4.0, 5.0);
        assert_eq!(
            model.predict_measurement(&state, None, AT),
            Prediction::Unavailable(Unavailable::NoProvider)
        );
    }

    #[test]
    fn predict_with_unresolved_edge_reports_missing_transform() {
        let model = make_model();
        let state = make_state(3.0, 4.0, 5.0);
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
    fn predict_identity_tf_returns_body_position() {
        let model = make_model();
        let state = make_state(3.0, 4.0, 5.0);
        let tf = MountTf(Isometry3::identity());
        let Prediction::Ready(z) = model.predict_measurement(&state, Some(&tf), AT) else {
            panic!("identity TF yields a ready prediction");
        };
        assert!((z[0] - 3.0).abs() < 1e-9);
        assert!((z[1] - 4.0).abs() < 1e-9);
        assert!((z[2] - 5.0).abs() < 1e-9);
    }

    #[test]
    fn predict_lever_arm_adds_rotated_offset() {
        let model = make_model();
        let state = make_state(1.0, 2.0, 3.0);
        // Antenna is 0.5 m forward, 0.1 m up from body origin. The mount also
        // carries a 90° yaw: the GPS model reads only the translation, so the
        // yaw does not change the expected value — but it makes the mount's
        // inverse have a *different* translation, so a swapped-argument lookup
        // (querying base_link-in-sensor instead of sensor-in-body) would land
        // the antenna somewhere else and fail this assertion.
        let offset = Isometry3::from_parts(
            Translation3::new(0.5, 0.0, 0.1),
            UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2),
        );
        let tf = MountTf(offset);
        let Prediction::Ready(z) = model.predict_measurement(&state, Some(&tf), AT) else {
            panic!("a resolvable mount yields a ready prediction");
        };
        assert!((z[0] - 1.5).abs() < 1e-9);
        assert!((z[1] - 2.0).abs() < 1e-9);
        assert!((z[2] - 3.1).abs() < 1e-9);
    }

    #[test]
    fn jacobian_position_columns_are_identity() {
        let model = make_model();
        let state = make_state(0.0, 0.0, 0.0);
        let tf = MountTf(Isometry3::identity());
        let h = model.jacobian(&state, Some(&tf), AT);
        assert_eq!(h.nrows(), 3);
        // H maps a tangent-space error to the measurement, so its column count is
        // the tangent dimension (a quaternion block spends one fewer than it stores).
        assert_eq!(h.ncols(), state.tangent_dim());
        assert!((h[(0, 0)] - 1.0).abs() < 1e-4);
        assert!((h[(1, 1)] - 1.0).abs() < 1e-4);
        assert!((h[(2, 2)] - 1.0).abs() < 1e-4);
    }
}
