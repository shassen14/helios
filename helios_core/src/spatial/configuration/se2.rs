//! [`Se2`]: the planar pose as a configuration space, a position with a
//! heading, and [`WeightedSe2Metric`], a distance that weighs a turn against a
//! meter.

use crate::kernel::angle;
use crate::spatial::configuration::{BodyFrame, Configuration, FromState, Metric};
use crate::spatial::conventions::Frame;
use crate::spatial::transforms::ConventionOf;
use crate::spatial::FrameAwareState;

use serde::{Deserialize, Serialize};
use std::fmt;
use std::marker::PhantomData;

/// A pose in the plane of frame `F`: a position and the heading the agent
/// faces, the configuration of a car or any vehicle whose facing a planner
/// must respect.
///
/// The plane is the frame's x–y plane and the heading is a turn about its z
/// axis, in radians, counterclockwise from x when z points at the viewer. The
/// heading is always held in (−π, π]: every way of making one, deserializing
/// included, wraps it.
//
// `#[serde(skip)]` on the phantom keeps `F` out of the wire form and out of
// the bounds serde's derive infers, as on `Point`. Reading goes through
// `Se2Wire` so that a stored heading is wrapped by `new` like any other.
#[derive(Debug, PartialEq, Serialize, Deserialize)]
#[serde(from = "Se2Wire")]
pub struct Se2<F: Frame> {
    x: f64,
    y: f64,
    heading: f64,
    #[serde(skip)]
    _frame: PhantomData<F>,
}

impl<F: Frame> Se2<F> {
    /// The pose at `x`, `y` in frame `F`, in meters, facing `heading`, in
    /// radians. Any heading is accepted and wrapped into (−π, π], so a heading
    /// of 3π/2 is stored as −π/2.
    pub fn new(x: f64, y: f64, heading: f64) -> Self {
        let heading = angle::wrap(heading);
        Self {
            x,
            y,
            heading,
            _frame: PhantomData,
        }
    }

    /// The x coordinate, in meters.
    pub fn x(&self) -> f64 {
        self.x
    }

    /// The y coordinate, in meters.
    pub fn y(&self) -> f64 {
        self.y
    }

    /// The heading, in radians, in (−π, π].
    pub fn heading(&self) -> f64 {
        self.heading
    }
}

// Hand-written so the only bound is `F: Frame`; a derive would also demand
// `F: Clone`, as on `Point`.
impl<F: Frame> Clone for Se2<F> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<F: Frame> Copy for Se2<F> {}

impl<F: Frame> Configuration for Se2<F> {
    fn dimension(&self) -> usize {
        3
    }

    /// The straight-line distance between the two positions. The heading is
    /// not a length, so it is left out: a turn in place has length zero.
    fn path_length(&self, other: &Self) -> f64 {
        f64::hypot(other.x - self.x, other.y - self.y)
    }

    /// The position along the straight line between the two, and the heading
    /// turned the same fraction of the short way round, so −179° to 179°
    /// passes through 180°, not 0°.
    ///
    /// The ends return early: adding the whole turn to `self`'s heading can
    /// miss `other`'s by a rounding error.
    fn interpolate(&self, other: &Self, fraction: f64) -> Self {
        let fraction = fraction.clamp(0.0, 1.0);
        if fraction == 0.0 {
            return *self;
        } else if fraction == 1.0 {
            return *other;
        }

        let x = self.x * (1.0 - fraction) + other.x * fraction;
        let y = self.y * (1.0 - fraction) + other.y * fraction;
        let heading = self.heading + angle::difference(self.heading, other.heading) * fraction;

        Self::new(x, y, heading)
    }
}

/// Reads the state's position in its reference frame, dropping height, and the
/// heading of the body named by the key. The key names the body's frame and
/// convention, since the state's orientation block is held per body.
impl<F: ConventionOf, B: ConventionOf> FromState<BodyFrame<B>> for Se2<F> {
    fn from_state(state: &FrameAwareState, key: &BodyFrame<B>) -> Option<Self> {
        let root = state.reference_frame()?;
        let position = state.position::<F>(root.clone())?;
        let orientation = state.orientation::<B, F>(key.frame().clone(), root)?;
        // The heading is the yaw of a roll–pitch–yaw decomposition: the turn
        // about `F`'s z axis, which is the direction the body's forward axis
        // points once projected onto the plane. A tilted body, such as a car on
        // a slope, still has its heading; only a body pitched straight up or
        // down has none, and the decomposition then picks one.
        let heading = orientation.into_inner().euler_angles().2;

        Some(Self::new(position.x(), position.y(), heading))
    }
}

/// A pose as stored, before its heading is wrapped.
#[derive(Deserialize)]
struct Se2Wire {
    x: f64,
    y: f64,
    heading: f64,
}

impl<F: Frame> From<Se2Wire> for Se2<F> {
    fn from(wire: Se2Wire) -> Self {
        Self::new(wire.x, wire.y, wire.heading)
    }
}

/// A distance between poses that counts both how far apart they are and how
/// far one must turn to face the other's way, in meters:
///
/// √((w_t · d)² + (w_h · Δθ)²)
///
/// `d` is the straight-line distance between the positions and `Δθ` the turn
/// between the headings the short way round. The heading weight `w_h`, in
/// meters per radian, says how many meters of travel a radian of turning is
/// worth; the translation weight `w_t` is a plain scale.
///
/// A ranking, not a drivable length: it does not know a car cannot turn in
/// place.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WeightedSe2Metric {
    translation_weight: f64,
    heading_weight: f64,
}

impl WeightedSe2Metric {
    /// A metric with the given translation weight and heading weight (meters
    /// per radian). A weight of zero is allowed and ignores that part; a
    /// negative, NaN or infinite weight is refused.
    pub fn new(translation_weight: f64, heading_weight: f64) -> Result<Self, InvalidSe2Weight> {
        if !translation_weight.is_finite() || translation_weight < 0.0 {
            return Err(InvalidSe2Weight::Translation(translation_weight));
        }
        if !heading_weight.is_finite() || heading_weight < 0.0 {
            return Err(InvalidSe2Weight::Heading(heading_weight));
        }

        Ok(Self {
            translation_weight,
            heading_weight,
        })
    }
}

impl<F: Frame> Metric<Se2<F>> for WeightedSe2Metric {
    fn distance(&self, a: &Se2<F>, b: &Se2<F>) -> f64 {
        let planar_distance = a.path_length(b);
        let heading_difference = angle::difference(a.heading, b.heading);

        ((self.translation_weight * planar_distance).powi(2)
            + (self.heading_weight * heading_difference).powi(2))
        .sqrt()
    }
}

/// A weight [`WeightedSe2Metric::new`] refused, naming which weight it was and
/// the value given.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InvalidSe2Weight {
    Translation(f64),
    Heading(f64),
}

impl fmt::Display for InvalidSe2Weight {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Translation(value) => write!(
                f,
                "SE(2) metric translation weight is {value}; it must be finite and \
                 not negative"
            ),
            Self::Heading(value) => write!(
                f,
                "SE(2) metric heading weight is {value} m/rad; it must be finite and \
                 not negative"
            ),
        }
    }
}

impl std::error::Error for InvalidSe2Weight {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimation::schema::{StateSchema, StateSchemaBlock};
    use crate::kernel::manifold::TangentNoise;
    use crate::prelude::AgentId;
    use crate::spatial::conventions::{Enu, Flu};
    use crate::spatial::primitives::MonotonicTime;
    use crate::spatial::quantities::Point;
    use crate::spatial::state::Quantity;
    use crate::spatial::transforms::{Convention, Rotation};
    use crate::spatial::FrameId;

    use nalgebra::{DMatrix, DVector, UnitQuaternion};
    use std::f64::consts::{FRAC_PI_2, PI};
    use std::sync::Arc;

    /// An arbitrary global frame the test state holds its pose in.
    const DATUM: &str = "test_datum";

    /// The agent whose body the test state orients.
    const AGENT: &str = "test_agent";

    /// Slack for values that went through float arithmetic.
    const TOLERANCE: f64 = 1e-12;

    fn degrees(value: f64) -> f64 {
        value.to_radians()
    }

    fn body() -> FrameId {
        FrameId::base_link(AgentId::new(AGENT))
    }

    fn position_block() -> StateSchemaBlock {
        StateSchemaBlock::new(
            Quantity::Position(FrameId::global(DATUM)),
            Convention::Enu,
            None,
            DVector::zeros(3),
            DMatrix::identity(3, 3),
        )
    }

    // The body (FLU) → datum (ENU) orientation, seeded at the identity.
    fn orientation_block() -> StateSchemaBlock {
        StateSchemaBlock::orientation(
            body(),
            FrameId::global(DATUM),
            Convention::Flu,
            Convention::Enu,
            Some(TangentNoise::from_variances(DVector::from_element(3, 0.1)).unwrap()),
            DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
            DMatrix::identity(3, 3),
        )
    }

    fn state_of(blocks: Vec<StateSchemaBlock>) -> FrameAwareState {
        FrameAwareState::from_schema(Arc::new(StateSchema::compose(blocks)), MonotonicTime(0.0))
    }

    fn metric(translation_weight: f64, heading_weight: f64) -> WeightedSe2Metric {
        WeightedSe2Metric::new(translation_weight, heading_weight).unwrap()
    }

    #[test]
    fn a_planar_pose_has_three_dimensions() {
        assert_eq!(Se2::<Enu>::new(1.0, 2.0, 0.5).dimension(), 3);
    }

    #[test]
    fn a_heading_out_of_range_is_wrapped() {
        let pose = Se2::<Enu>::new(0.0, 0.0, 3.0 * FRAC_PI_2);
        assert!((pose.heading() + FRAC_PI_2).abs() < TOLERANCE);
    }

    #[test]
    fn path_length_ignores_the_heading() {
        let a = Se2::<Enu>::new(1.0, 1.0, 0.0);
        let b = Se2::<Enu>::new(4.0, 5.0, 2.0);
        assert!((a.path_length(&b) - 5.0).abs() < TOLERANCE);
    }

    #[test]
    fn a_turn_in_place_has_no_length() {
        let a = Se2::<Enu>::new(1.0, 1.0, 0.0);
        let b = Se2::<Enu>::new(1.0, 1.0, FRAC_PI_2);
        assert_eq!(a.path_length(&b), 0.0);
    }

    #[test]
    fn the_midpoint_is_halfway_in_position_and_heading() {
        let a = Se2::<Enu>::new(0.0, 0.0, 0.0);
        let b = Se2::<Enu>::new(2.0, -4.0, FRAC_PI_2);
        let mid = a.interpolate(&b, 0.5);
        assert!((mid.x() - 1.0).abs() < TOLERANCE);
        assert!((mid.y() + 2.0).abs() < TOLERANCE);
        assert!((mid.heading() - FRAC_PI_2 / 2.0).abs() < TOLERANCE);
    }

    #[test]
    fn the_heading_interpolates_the_short_way_round() {
        let a = Se2::<Enu>::new(0.0, 0.0, degrees(-179.0));
        let b = Se2::<Enu>::new(0.0, 0.0, degrees(179.0));
        let mid = a.interpolate(&b, 0.5);
        // Halfway the short way is 180°, held as +π; the long way would be 0.
        assert!((mid.heading().abs() - PI).abs() < TOLERANCE);
    }

    #[test]
    fn the_endpoints_come_back_exactly() {
        let a = Se2::<Enu>::new(0.1, 0.7, 0.3);
        let b = Se2::<Enu>::new(-3.3, 2.9, -2.9);
        assert_eq!(a.interpolate(&b, 0.0), a);
        assert_eq!(a.interpolate(&b, 1.0), b);
    }

    #[test]
    fn a_fraction_past_either_end_clamps_to_it() {
        let a = Se2::<Enu>::new(0.1, 0.7, 0.3);
        let b = Se2::<Enu>::new(-3.3, 2.9, -2.9);
        assert_eq!(a.interpolate(&b, 1.5), b);
        assert_eq!(a.interpolate(&b, -0.5), a);
    }

    #[test]
    fn reading_from_a_state_takes_the_body_yaw_and_drops_height() {
        let mut state = state_of(vec![position_block(), orientation_block()]);
        state
            .set_position::<Enu>(FrameId::global(DATUM), Point::new(2.0, -3.0, 7.0))
            .unwrap();
        let facing_north = UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2);
        state
            .set_orientation::<Flu, Enu>(
                body(),
                FrameId::global(DATUM),
                Rotation::from_unit_quaternion(facing_north),
            )
            .unwrap();

        let read = Se2::<Enu>::from_state(&state, &BodyFrame::<Flu>::new(body())).unwrap();
        assert!((read.x() - 2.0).abs() < TOLERANCE);
        assert!((read.y() + 3.0).abs() < TOLERANCE);
        assert!((read.heading() - FRAC_PI_2).abs() < TOLERANCE);
    }

    #[test]
    fn a_tilted_body_keeps_its_heading() {
        let mut state = state_of(vec![position_block(), orientation_block()]);
        // Facing north, nose up and rolled, as on a banked slope.
        let tilted = UnitQuaternion::from_euler_angles(degrees(10.0), degrees(-15.0), FRAC_PI_2);
        state
            .set_orientation::<Flu, Enu>(
                body(),
                FrameId::global(DATUM),
                Rotation::from_unit_quaternion(tilted),
            )
            .unwrap();

        let read = Se2::<Enu>::from_state(&state, &BodyFrame::<Flu>::new(body())).unwrap();
        assert!((read.heading() - FRAC_PI_2).abs() < TOLERANCE);
    }

    #[test]
    fn a_state_with_no_orientation_reads_as_none() {
        let state = state_of(vec![position_block()]);
        assert_eq!(
            Se2::<Enu>::from_state(&state, &BodyFrame::<Flu>::new(body())),
            None
        );
    }

    #[test]
    fn a_state_with_no_position_reads_as_none() {
        let state = state_of(vec![orientation_block()]);
        assert_eq!(
            Se2::<Enu>::from_state(&state, &BodyFrame::<Flu>::new(body())),
            None
        );
    }

    #[test]
    fn the_metric_weighs_translation_and_turning() {
        let a = Se2::<Enu>::new(0.0, 0.0, 0.0);
        let b = Se2::<Enu>::new(3.0, 4.0, 0.0);
        let c = Se2::<Enu>::new(0.0, 0.0, FRAC_PI_2);
        let weighted = metric(2.0, 3.0);
        assert!((weighted.distance(&a, &b) - 10.0).abs() < TOLERANCE);
        assert!((weighted.distance(&a, &c) - 3.0 * FRAC_PI_2).abs() < TOLERANCE);
    }

    #[test]
    fn the_metric_turns_the_short_way_round() {
        let a = Se2::<Enu>::new(0.0, 0.0, degrees(-179.0));
        let b = Se2::<Enu>::new(0.0, 0.0, degrees(179.0));
        assert!((metric(1.0, 1.0).distance(&a, &b) - degrees(2.0)).abs() < TOLERANCE);
    }

    #[test]
    fn the_metric_combines_both_parts_as_a_root_sum_of_squares() {
        let a = Se2::<Enu>::new(0.0, 0.0, 0.0);
        let b = Se2::<Enu>::new(3.0, 0.0, 4.0);
        // 4 rad wraps to 4 − 2π, a turn of 2π − 4 the short way.
        let turn = 2.0 * PI - 4.0;
        let expected = f64::hypot(3.0, turn);
        assert!((metric(1.0, 1.0).distance(&a, &b) - expected).abs() < TOLERANCE);
    }

    #[test]
    fn a_zero_weight_ignores_its_part() {
        let a = Se2::<Enu>::new(0.0, 0.0, 0.0);
        let b = Se2::<Enu>::new(3.0, 4.0, FRAC_PI_2);
        assert!((metric(1.0, 0.0).distance(&a, &b) - 5.0).abs() < TOLERANCE);
        assert!((metric(0.0, 1.0).distance(&a, &b) - FRAC_PI_2).abs() < TOLERANCE);
    }

    #[test]
    fn a_negative_weight_is_refused_and_named() {
        assert_eq!(
            WeightedSe2Metric::new(-1.0, 1.0),
            Err(InvalidSe2Weight::Translation(-1.0))
        );
        assert_eq!(
            WeightedSe2Metric::new(1.0, -1.0),
            Err(InvalidSe2Weight::Heading(-1.0))
        );
    }

    #[test]
    fn a_weight_that_is_not_finite_is_refused() {
        assert!(matches!(
            WeightedSe2Metric::new(f64::NAN, 1.0),
            Err(InvalidSe2Weight::Translation(_))
        ));
        assert!(matches!(
            WeightedSe2Metric::new(1.0, f64::INFINITY),
            Err(InvalidSe2Weight::Heading(_))
        ));
    }

    #[test]
    fn the_wire_form_is_the_three_coordinates() {
        let pose = Se2::<Enu>::new(1.5, -2.5, 0.25);
        let json = serde_json::to_string(&pose).unwrap();
        assert_eq!(json, r#"{"x":1.5,"y":-2.5,"heading":0.25}"#);
        assert_eq!(serde_json::from_str::<Se2<Enu>>(&json).unwrap(), pose);
    }

    #[test]
    fn a_stored_heading_out_of_range_is_wrapped_on_reading() {
        let json = format!(r#"{{"x":0.0,"y":0.0,"heading":{}}}"#, 3.0 * FRAC_PI_2);
        let pose = serde_json::from_str::<Se2<Enu>>(&json).unwrap();
        assert!((pose.heading() + FRAC_PI_2).abs() < TOLERANCE);
    }
}
