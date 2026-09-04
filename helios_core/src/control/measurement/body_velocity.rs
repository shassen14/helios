//! The estimate's linear velocity, projected into the body frame.
//!
//! This is a controller *measurement model* — the dual of a sensor model. A
//! sensor model maps truth to an observation; this maps the estimator's belief to
//! the quantity a control law consumes. It is a frame projection, never a layout
//! read: the estimate's velocity lives in the estimator's kinematic frame while a
//! control reference lives in the body frame, so the error is only meaningful once
//! both are expressed in one frame.

use crate::frames::conventions::{Enu, Flu};
use crate::frames::quantities::FluVector;
use crate::frames::{FrameAwareState, FrameId};

/// The vehicle's linear velocity in its own body (FLU) frame, projected from an
/// estimate whose velocity may be stored in any frame.
///
/// The source frame *identity* is discovered from the schema, never assumed to be
/// global. Two cases, tried in order:
///
/// 1. **Already body.** If the schema stores velocity in `body` directly, it is
///    returned unrotated.
/// 2. **Kinematic frame, then rotate.** Otherwise velocity lives in the estimate's
///    kinematic frame ([`reference_frame`](FrameAwareState::reference_frame) —
///    odom or world) and is rotated into `body` using the estimate's own
///    `body → kinematic` orientation, inverted. No transform provider is needed:
///    the estimate carries both its velocity and the orientation relating it to
///    the body.
///
/// Returns `None` when the estimate carries no velocity reachable this way, or
/// (case 2) is missing the orientation block — a cold-start state the caller
/// answers with a zero command.
///
/// The frame *convention* (ENU for the kinematic frame, FLU for the body) is
/// asserted here, not discovered: the schema does not yet carry axis convention,
/// so the turbofish is the caller's promise — the same one every block extractor
/// makes. The convention-agnostic form, where the looked-up rotation itself
/// carries the convention change and no convention is named, waits on the schema
/// carrying convention.
pub fn body_velocity(state: &FrameAwareState, body: FrameId) -> Option<FluVector> {
    // Case 1: velocity already stored in the body frame — return it unrotated.
    if let Some(v_body) = state.velocity::<Flu>(body.clone()) {
        return Some(v_body);
    }

    // Case 2: velocity is in the kinematic frame; rotate it into body using the
    // estimate's own (body → kinematic) orientation, inverted to (kinematic → body).
    let kinematic = state.reference_frame()?;
    let v_kinematic = state.velocity::<Enu>(kinematic.clone())?;
    let kinematic_to_body = state.orientation::<Flu, Enu>(body, kinematic)?.inverse();

    Some(kinematic_to_body.act(v_kinematic))
}

/// The forward (body +X) component of [`body_velocity`], in m/s — the scalar the
/// longitudinal loop controls. `None` propagates from the projection.
pub fn body_forward_speed(state: &FrameAwareState, body: FrameId) -> Option<f64> {
    Some(body_velocity(state, body)?.x())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data::primitives::FrameHandle;
    use crate::estimation::schema::{SchemaBlock, StateSchema};
    use crate::frames::transforms::Convention;
    use crate::manifold::TangentNoise;
    use crate::state::Quantity;

    use nalgebra::{DMatrix, DVector, Vector3};
    use std::f64::consts::FRAC_PI_2;
    use std::sync::Arc;

    fn body() -> FrameId {
        FrameId::Body(FrameHandle(1))
    }

    fn odom() -> FrameId {
        FrameId::Odom(FrameHandle(1))
    }

    // Isotropic 3-DOF noise. Irrelevant to a projection (a read), but an
    // orientation block refuses `None`, so every block carries some.
    fn noise() -> Option<TangentNoise> {
        Some(TangentNoise::from_variances(DVector::from_element(3, 0.1)).unwrap())
    }

    // The `body → odom` quaternion for a yaw of `rad` about +Z, stored in the
    // block's `[Qx, Qy, Qz, Qw]` order.
    fn yaw_quaternion(rad: f64) -> DVector<f64> {
        DVector::from_vec(vec![0.0, 0.0, (rad / 2.0).sin(), (rad / 2.0).cos()])
    }

    // A composed estimate in odom: position + velocity `v` (odom frame) + a
    // `body → odom` orientation `q`. This is the case-2 (rotate) shape.
    fn odom_estimate(v: Vector3<f64>, q: DVector<f64>) -> FrameAwareState {
        let schema = StateSchema::compose(vec![
            SchemaBlock::new(
                Quantity::Position(odom()),
                Convention::Enu,
                noise(),
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
            SchemaBlock::new(
                Quantity::Velocity(odom()),
                Convention::Enu,
                noise(),
                DVector::from_vec(vec![v.x, v.y, v.z]),
                DMatrix::identity(3, 3),
            ),
            SchemaBlock::orientation(
                body(),
                odom(),
                Convention::Flu,
                Convention::Enu,
                noise(),
                q,
                DMatrix::identity(3, 3),
            ),
        ]);
        FrameAwareState::from_schema(Arc::new(schema), 0.0)
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn identity_orientation_passes_velocity_through() {
        // Body aligned with odom (zero yaw): odom velocity is already body
        // velocity, so forward speed is the odom +X component unchanged.
        let s = odom_estimate(Vector3::new(2.0, 0.0, 0.0), yaw_quaternion(0.0));
        let v = body_velocity(&s, body()).unwrap();
        assert!(close(v.x(), 2.0));
        assert!(close(v.y(), 0.0));
    }

    #[test]
    fn yawed_body_recovers_forward_speed() {
        // Body yawed +90° faces odom-north. An estimate moving due north (odom +Y)
        // is therefore moving straight forward: the projection must read forward
        // speed 1, not the raw odom +X (which is 0).
        let s = odom_estimate(Vector3::new(0.0, 1.0, 0.0), yaw_quaternion(FRAC_PI_2));
        let v = body_velocity(&s, body()).unwrap();
        assert!(close(v.x(), 1.0));
        assert!(close(v.y(), 0.0));
    }

    #[test]
    fn yawed_body_reads_sideways_motion_as_lateral() {
        // Same +90° yaw, but moving due east (odom +X): the vehicle faces north, so
        // this is pure rightward motion — forward speed 0, lateral −1 (right is −Y
        // in FLU). Proves the projection is a real rotation, not an axis read.
        let s = odom_estimate(Vector3::new(1.0, 0.0, 0.0), yaw_quaternion(FRAC_PI_2));
        let v = body_velocity(&s, body()).unwrap();
        assert!(close(v.x(), 0.0));
        assert!(close(v.y(), -1.0));
    }

    #[test]
    fn velocity_stored_in_body_is_returned_unrotated() {
        // Case 1: an estimate whose velocity block is already in the body frame is
        // returned directly, with no orientation block present at all.
        let schema = StateSchema::compose(vec![SchemaBlock::new(
            Quantity::Velocity(body()),
            Convention::Flu,
            noise(),
            DVector::from_vec(vec![3.0, 0.0, 0.0]),
            DMatrix::identity(3, 3),
        )]);
        let s = FrameAwareState::from_schema(Arc::new(schema), 0.0);
        assert!(close(body_velocity(&s, body()).unwrap().x(), 3.0));
    }

    #[test]
    fn cold_start_without_velocity_is_none() {
        // Only a position block: nothing to project.
        let schema = StateSchema::compose(vec![SchemaBlock::new(
            Quantity::Position(odom()),
            Convention::Enu,
            noise(),
            DVector::zeros(3),
            DMatrix::identity(3, 3),
        )]);
        let s = FrameAwareState::from_schema(Arc::new(schema), 0.0);
        assert!(body_velocity(&s, body()).is_none());
    }

    #[test]
    fn missing_orientation_is_none() {
        // Velocity is in odom but no orientation block relates it to the body, so
        // case 2 cannot rotate it.
        let schema = StateSchema::compose(vec![
            SchemaBlock::new(
                Quantity::Position(odom()),
                Convention::Enu,
                noise(),
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
            SchemaBlock::new(
                Quantity::Velocity(odom()),
                Convention::Enu,
                noise(),
                DVector::from_vec(vec![1.0, 0.0, 0.0]),
                DMatrix::identity(3, 3),
            ),
        ]);
        let s = FrameAwareState::from_schema(Arc::new(schema), 0.0);
        assert!(body_velocity(&s, body()).is_none());
    }

    #[test]
    fn forward_speed_is_the_x_component() {
        let s = odom_estimate(Vector3::new(0.0, 1.0, 0.0), yaw_quaternion(FRAC_PI_2));
        assert!(close(body_forward_speed(&s, body()).unwrap(), 1.0));
    }
}
