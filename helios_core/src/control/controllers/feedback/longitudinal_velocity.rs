use crate::control::commands::DriveForce;
use crate::control::kernels::siso_pid::SisoPid;
use crate::control::measurement::body_forward_speed;
use crate::control::{BodyTwistRef, ControlInputs, Controller};
use crate::frames::FrameId;

/// Longitudinal velocity feedback: drives body-forward speed to the reference's
/// forward speed and emits a scalar drive force.
///
/// This is the first closed loop — unlike the pass-through
/// [`DirectTwistController`](super::direct_twist::DirectTwistController), it reads
/// the estimate, forms an error, and acts on it. The controlled quantity is the
/// estimate's forward speed *projected into the body frame* (via
/// [`body_forward_speed`]), not a raw state slot: the reference is a body-frame
/// twist, so the error is only meaningful once the measurement is in the same
/// frame.
///
/// The output is a `DriveForce` — a scalar longitudinal effort, axis-less until an
/// allocator stamps it onto an actuator downstream. A sibling feedforward
/// controller emits the same type; the two fold by summation.
pub struct LongitudinalVelocityController {
    pid: SisoPid,
    /// The body frame this controller drives — the target of the velocity
    /// projection and the frame the reference speaks.
    body: FrameId,
}

impl LongitudinalVelocityController {
    pub fn new(pid: SisoPid, body: FrameId) -> Self {
        Self { pid, body }
    }
}

impl Controller for LongitudinalVelocityController {
    type Inputs = ControlInputs<BodyTwistRef>;
    type Out = DriveForce;

    fn compute(&mut self, dt: f64, inputs: &Self::Inputs) -> Self::Out {
        // Command zero — and leave the integrator frozen, by not calling `update`
        // — until both a reference and a usable velocity estimate are present.
        // Freezing is the bumpless-transfer mechanism: nothing accumulates while
        // idle, so re-engaging does not lurch from a stale integral.
        let (Some(reference), Some(measured)) = (
            inputs.reference.as_ref(),
            body_forward_speed(&inputs.state, self.body.clone()),
        ) else {
            return DriveForce::zero();
        };

        let error = reference.twist().linear().x() - measured;
        DriveForce::new(self.pid.update(error, dt))
    }

    fn reset(&mut self) {
        self.pid.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::commands::BodyTwist;
    use crate::data::primitives::FrameHandle;
    use crate::estimation::schema::{SchemaBlock, StateSchema};
    use crate::frames::FrameAwareState;
    use crate::manifold::TangentNoise;
    use crate::state::Quantity;

    use nalgebra::{DMatrix, DVector};
    use std::sync::Arc;

    fn body() -> FrameId {
        FrameId::Body(FrameHandle(1))
    }

    fn noise() -> Option<TangentNoise> {
        Some(TangentNoise::from_variances(DVector::from_element(3, 0.1)).unwrap())
    }

    // An estimate whose velocity is stored directly in the body frame, so the
    // projection returns `vx` unrotated — isolating the controller's error math
    // from the frame projection (which `body_velocity`'s own tests cover).
    fn state_with_forward_speed(vx: f64) -> FrameAwareState {
        let schema = StateSchema::compose(vec![SchemaBlock::new(
            Quantity::Velocity(body()),
            noise(),
            DVector::from_vec(vec![vx, 0.0, 0.0]),
            DMatrix::identity(3, 3),
        )]);
        FrameAwareState::from_schema(Arc::new(schema), 0.0)
    }

    fn inputs(reference_vx: Option<f64>, measured_vx: f64) -> ControlInputs<BodyTwistRef> {
        ControlInputs {
            state: state_with_forward_speed(measured_vx),
            reference: reference_vx.map(|vx| BodyTwistRef::new(BodyTwist::unicycle(vx, 0.0))),
        }
    }

    // A pure-proportional controller (kp only) makes the output a direct readout
    // of the error, so force sign and magnitude pin the error math.
    fn proportional(kp: f64) -> LongitudinalVelocityController {
        LongitudinalVelocityController::new(SisoPid::new(kp, 0.0, 0.0), body())
    }

    #[test]
    fn under_speed_commands_positive_force() {
        // Reference 5, measured 0 → error +5 → positive force (kp·error).
        let mut c = proportional(10.0);
        let out = c.compute(0.1, &inputs(Some(5.0), 0.0));
        assert_eq!(out, DriveForce::new(50.0));
    }

    #[test]
    fn over_speed_commands_negative_force() {
        // Reference 2, measured 5 → error −3 → braking force.
        let mut c = proportional(10.0);
        let out = c.compute(0.1, &inputs(Some(2.0), 5.0));
        assert_eq!(out, DriveForce::new(-30.0));
    }

    #[test]
    fn zero_error_commands_zero_force() {
        let mut c = proportional(10.0);
        let out = c.compute(0.1, &inputs(Some(3.0), 3.0));
        assert_eq!(out, DriveForce::zero());
    }

    #[test]
    fn no_reference_commands_zero() {
        let mut c = proportional(10.0);
        assert_eq!(c.compute(0.1, &inputs(None, 4.0)), DriveForce::zero());
    }

    #[test]
    fn missing_velocity_estimate_commands_zero() {
        // Reference present, but a cold-start state with no velocity block → the
        // projection is `None`, so the loop commands zero rather than acting on a
        // fabricated measurement.
        let schema = StateSchema::compose(vec![SchemaBlock::new(
            Quantity::Position(FrameId::Odom(FrameHandle(1))),
            noise(),
            DVector::zeros(3),
            DMatrix::identity(3, 3),
        )]);
        let cold = ControlInputs {
            state: FrameAwareState::from_schema(Arc::new(schema), 0.0),
            reference: Some(BodyTwistRef::new(BodyTwist::unicycle(5.0, 0.0))),
        };
        let mut c = proportional(10.0);
        assert_eq!(c.compute(0.1, &cold), DriveForce::zero());
    }

    #[test]
    fn absent_reference_freezes_the_integrator() {
        // A pure integrator (ki only, unit dt) accumulates error·dt each engaged
        // tick. A dropped reference in the middle must *freeze* — not reset — the
        // accumulator, so the third tick reflects two accumulations (2 + 2 = 4),
        // not a restart back to 2.
        let mut c = LongitudinalVelocityController::new(SisoPid::new(0.0, 1.0, 0.0), body());

        assert_eq!(
            c.compute(1.0, &inputs(Some(2.0), 0.0)),
            DriveForce::new(2.0)
        );
        assert_eq!(c.compute(1.0, &inputs(None, 0.0)), DriveForce::zero());
        assert_eq!(
            c.compute(1.0, &inputs(Some(2.0), 0.0)),
            DriveForce::new(4.0)
        );
    }

    #[test]
    fn reset_clears_the_integrator() {
        let mut c = LongitudinalVelocityController::new(SisoPid::new(0.0, 1.0, 0.0), body());

        assert_eq!(
            c.compute(1.0, &inputs(Some(2.0), 0.0)),
            DriveForce::new(2.0)
        );
        c.reset();
        // After reset the accumulator is empty, so the same tick reads as the first.
        assert_eq!(
            c.compute(1.0, &inputs(Some(2.0), 0.0)),
            DriveForce::new(2.0)
        );
    }
}
