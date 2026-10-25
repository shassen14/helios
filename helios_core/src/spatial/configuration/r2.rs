//! [`R2`]: the plane as a configuration space, a position with no heading,
//! and [`EuclideanR2Metric`], straight-line distance across it.

use crate::spatial::configuration::{Configuration, FromState, Metric};
use crate::spatial::conventions::Frame;
use crate::spatial::transforms::ConventionOf;
use crate::spatial::FrameAwareState;

use serde::{Deserialize, Serialize};
use std::marker::PhantomData;

/// A position in the plane of frame `F`: the configuration of an agent whose
/// heading a planner ignores, such as a point robot or a route over a map.
///
/// The plane is the frame's x–y plane; height is not part of it, so reading
/// one from a state drops z.
//
// `#[serde(skip)]` on the phantom keeps `F` out of the wire form and out of
// the bounds serde's derive infers, as on `Point`.
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct R2<F: Frame> {
    x: f64,
    y: f64,
    #[serde(skip)]
    _frame: PhantomData<F>,
}

impl<F: Frame> R2<F> {
    /// The position at `x`, `y` in frame `F`, in meters.
    pub fn new(x: f64, y: f64) -> Self {
        Self {
            x,
            y,
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
}

// Hand-written so the only bound is `F: Frame`; a derive would also demand
// `F: Clone`, as on `Point`.
impl<F: Frame> Clone for R2<F> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<F: Frame> Copy for R2<F> {}

impl<F: Frame> Configuration for R2<F> {
    fn dimension(&self) -> usize {
        2
    }

    /// The straight-line distance between the two positions.
    fn path_length(&self, other: &Self) -> f64 {
        f64::hypot(other.x - self.x, other.y - self.y)
    }

    /// The point along the straight line between the two positions. The blend
    /// weights each end by its share, so a fraction of exactly 0 or 1 returns
    /// that end unchanged.
    fn interpolate(&self, other: &Self, fraction: f64) -> Self {
        let fraction = fraction.clamp(0.0, 1.0);
        let x = self.x * (1.0 - fraction) + other.x * fraction;
        let y = self.y * (1.0 - fraction) + other.y * fraction;
        Self::new(x, y)
    }
}

/// Reads the state's position in its reference frame, dropping height. The
/// key is `()`: the position is the whole answer, so nothing else is named.
impl<F: ConventionOf> FromState<()> for R2<F> {
    fn from_state(state: &FrameAwareState, _key: &()) -> Option<Self> {
        let root = state.reference_frame()?;
        let position = state.position::<F>(root)?;
        Some(Self::new(position.x(), position.y()))
    }
}

/// Straight-line distance between two positions, in meters: the same number
/// as [`Configuration::path_length`], since in the plane the shortest path is
/// the straight one.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct EuclideanR2Metric;

impl<F: Frame> Metric<R2<F>> for EuclideanR2Metric {
    fn distance(&self, a: &R2<F>, b: &R2<F>) -> f64 {
        a.path_length(b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimation::schema::{StateSchema, StateSchemaBlock};
    use crate::kernel::manifold::TangentNoise;
    use crate::prelude::AgentId;
    use crate::spatial::conventions::Enu;
    use crate::spatial::primitives::MonotonicTime;
    use crate::spatial::quantities::Point;
    use crate::spatial::state::Quantity;
    use crate::spatial::transforms::Convention;
    use crate::spatial::FrameId;

    use nalgebra::{DMatrix, DVector};
    use std::sync::Arc;

    /// An arbitrary global frame the test state holds its position in.
    const DATUM: &str = "test_datum";

    /// Slack for values that went through float arithmetic.
    const TOLERANCE: f64 = 1e-12;

    fn position_block() -> StateSchemaBlock {
        StateSchemaBlock::new(
            Quantity::Position(FrameId::global(DATUM)),
            Convention::Enu,
            None,
            DVector::zeros(3),
            DMatrix::identity(3, 3),
        )
    }

    // An orientation block on its own: a state with an attitude but no
    // position, so no reference frame.
    fn orientation_block() -> StateSchemaBlock {
        StateSchemaBlock::orientation(
            FrameId::base_link(AgentId::new("test_agent")),
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

    #[test]
    fn the_plane_has_two_dimensions() {
        assert_eq!(R2::<Enu>::new(1.0, 2.0).dimension(), 2);
    }

    #[test]
    fn path_length_is_the_straight_line_between_the_points() {
        let a = R2::<Enu>::new(1.0, 1.0);
        let b = R2::<Enu>::new(4.0, 5.0);
        assert!((a.path_length(&b) - 5.0).abs() < TOLERANCE);
        assert!((b.path_length(&a) - 5.0).abs() < TOLERANCE);
    }

    #[test]
    fn points_the_same_distance_from_the_origin_are_still_apart() {
        let a = R2::<Enu>::new(3.0, 4.0);
        let b = R2::<Enu>::new(-3.0, -4.0);
        assert!((a.path_length(&b) - 10.0).abs() < TOLERANCE);
    }

    #[test]
    fn the_midpoint_is_halfway() {
        let a = R2::<Enu>::new(0.0, 0.0);
        let b = R2::<Enu>::new(2.0, -4.0);
        let mid = a.interpolate(&b, 0.5);
        assert!((mid.x() - 1.0).abs() < TOLERANCE);
        assert!((mid.y() + 2.0).abs() < TOLERANCE);
    }

    #[test]
    fn the_endpoints_come_back_exactly() {
        let a = R2::<Enu>::new(0.1, 0.7);
        let b = R2::<Enu>::new(-3.3, 2.9);
        assert_eq!(a.interpolate(&b, 0.0), a);
        assert_eq!(a.interpolate(&b, 1.0), b);
    }

    #[test]
    fn a_fraction_past_either_end_clamps_to_it() {
        let a = R2::<Enu>::new(0.1, 0.7);
        let b = R2::<Enu>::new(-3.3, 2.9);
        assert_eq!(a.interpolate(&b, 1.5), b);
        assert_eq!(a.interpolate(&b, -0.5), a);
    }

    #[test]
    fn the_euclidean_metric_equals_path_length() {
        let a = R2::<Enu>::new(1.0, 1.0);
        let b = R2::<Enu>::new(4.0, 5.0);
        assert_eq!(EuclideanR2Metric.distance(&a, &b), a.path_length(&b));
    }

    #[test]
    fn reading_from_a_state_drops_height() {
        let mut state = state_of(vec![position_block()]);
        state
            .set_position::<Enu>(FrameId::global(DATUM), Point::new(2.0, -3.0, 7.0))
            .unwrap();
        let read = R2::<Enu>::from_state(&state, &()).unwrap();
        assert_eq!(read, R2::new(2.0, -3.0));
    }

    #[test]
    fn a_state_with_no_position_reads_as_none() {
        let state = state_of(vec![orientation_block()]);
        assert_eq!(R2::<Enu>::from_state(&state, &()), None);
    }

    #[test]
    fn the_wire_form_is_the_two_coordinates() {
        let point = R2::<Enu>::new(1.5, -2.5);
        let json = serde_json::to_string(&point).unwrap();
        assert_eq!(json, r#"{"x":1.5,"y":-2.5}"#);
        assert_eq!(serde_json::from_str::<R2<Enu>>(&json).unwrap(), point);
    }
}
