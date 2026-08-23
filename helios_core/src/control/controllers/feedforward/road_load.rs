use crate::control::{commands::DriveForce, BodyTwistRef, ControlInputs, Controller};

/// Road-load feedforward: the drive force that *should* hold the reference
/// speed against a ground vehicle's known resistances, computed open-loop from
/// the reference alone.
///
/// "Road load" is the standard term for a ground vehicle's steady tractive
/// resistance — the rolling and aerodynamic terms below. That model is what makes
/// this feedforward morphology-specific: it inverts a *car's* plant, not a generic
/// longitudinal one, which is why it names the plant rather than the axis.
///
/// Where [`LongitudinalVelocityController`](crate::control::controllers::feedback::longitudinal_velocity::LongitudinalVelocityController)
/// reacts to error — it waits for the vehicle to be off-speed, then corrects —
/// this reacts to *intent*: given the target speed it precomputes the effort to
/// sustain it, before any error appears. It is the inverse plant map, the dual of
/// the feedback law, and the two fold by summation (both emit [`DriveForce`]).
///
/// The model is steady-state cruise: overcome
///
/// - **rolling resistance**, a roughly constant magnitude opposing motion:
///   `c_roll · sign(v_ref)`;
/// - **aerodynamic drag**, quadratic in speed: `c_drag · v_ref·|v_ref|` — the
///   `v·|v|` form (not `v²`) keeps the sign, so a reverse reference is met with a
///   reverse-facing drag term.
///
/// Two things it deliberately does *not* do. It never reads the estimate: the
/// reference is already a body-frame twist, so there is no frame projection — the
/// whole odom→body crossing lives on the feedback side. And it carries no state:
/// the map is a pure function of the reference, so [`reset`](Self::reset) is a
/// no-op and there is no integrator to freeze.
///
/// Acceleration feedforward (`m · a_ref`) is out of scope: [`BodyTwistRef`] carries
/// a velocity, not an acceleration, so there is no `a_ref` to invert. That waits on
/// a reference type that carries the derivative.
pub struct RoadLoadFeedforward {
    /// Rolling-resistance magnitude, in newtons — the speed-independent force
    /// opposing motion.
    c_roll: f64,
    /// Aerodynamic drag coefficient, in N·s²/m², lumping ½·ρ·Cd·A into one term
    /// against `v·|v|`.
    c_drag: f64,
}

impl Controller for RoadLoadFeedforward {
    type Inputs = ControlInputs<BodyTwistRef>;
    type Out = DriveForce;

    fn compute(&mut self, _dt: f64, inputs: &Self::Inputs) -> Self::Out {
        let Some(reference) = inputs.reference.as_ref() else {
            return DriveForce::zero();
        };

        let vel_x = reference.twist().linear().x();
        DriveForce::new(self.c_roll * vel_x.signum() + self.c_drag * vel_x * vel_x.abs())
    }

    fn reset(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::commands::BodyTwist;
    use crate::estimation::schema::StateSchema;
    use crate::frames::FrameAwareState;

    use std::sync::Arc;

    // Tests build the struct directly; a registry-facing constructor is the
    // controller factory's concern, not this leaf's.
    fn feedforward(c_roll: f64, c_drag: f64) -> RoadLoadFeedforward {
        RoadLoadFeedforward { c_roll, c_drag }
    }

    // Feedforward never reads the estimate, so an empty state suffices — the point
    // is precisely that the output depends on the reference alone.
    fn empty_state() -> FrameAwareState {
        FrameAwareState::from_schema(Arc::new(StateSchema::compose(vec![])), 0.0)
    }

    fn reference(vx: f64) -> ControlInputs<BodyTwistRef> {
        ControlInputs {
            state: empty_state(),
            reference: Some(BodyTwistRef::new(BodyTwist::unicycle(vx, 0.0))),
        }
    }

    #[test]
    fn holds_speed_against_drag_and_rolling() {
        // v_ref 4, c_roll 2, c_drag 3 → 2·sign(4) + 3·4·|4| = 2 + 48 = 50.
        let mut c = feedforward(2.0, 3.0);
        assert_eq!(c.compute(0.1, &reference(4.0)), DriveForce::new(50.0));
    }

    #[test]
    fn reverse_reference_flips_both_terms() {
        // v_ref −4 → 2·sign(−4) + 3·(−4)·|−4| = −2 − 48 = −50. Pins the sign
        // handling: rolling flips through `signum`, drag through `v·|v|` (a plain
        // `v²` would stay +48 and get the drag direction wrong in reverse).
        let mut c = feedforward(2.0, 3.0);
        assert_eq!(c.compute(0.1, &reference(-4.0)), DriveForce::new(-50.0));
    }

    #[test]
    fn drag_grows_quadratically() {
        // Pure drag (no rolling): tripling nothing, doubling speed quadruples force.
        let mut c = feedforward(0.0, 1.0);
        assert_eq!(c.compute(0.1, &reference(2.0)), DriveForce::new(4.0));
        assert_eq!(c.compute(0.1, &reference(4.0)), DriveForce::new(16.0));
    }

    #[test]
    fn rolling_is_speed_independent() {
        // Pure rolling (no drag): a constant magnitude regardless of speed.
        let mut c = feedforward(5.0, 0.0);
        assert_eq!(c.compute(0.1, &reference(1.0)), DriveForce::new(5.0));
        assert_eq!(c.compute(0.1, &reference(100.0)), DriveForce::new(5.0));
    }

    #[test]
    fn no_reference_commands_zero() {
        let mut c = feedforward(2.0, 3.0);
        let idle = ControlInputs {
            state: empty_state(),
            reference: None,
        };
        assert_eq!(c.compute(0.1, &idle), DriveForce::zero());
    }

    #[test]
    fn reset_is_a_noop() {
        // The map is stateless: the same reference yields the same force with a
        // `reset` in between, unlike the feedback controller's integrator.
        let mut c = feedforward(2.0, 3.0);
        let before = c.compute(0.1, &reference(4.0));
        c.reset();
        let after = c.compute(0.1, &reference(4.0));
        assert_eq!(before, after);
    }

    #[test]
    fn zero_reference_applies_a_phantom_rolling_force() {
        // Documents a sharp edge, not an endorsement: `0.0_f64.signum()` is `1.0`,
        // so a standstill reference (v_ref = 0) commands the full rolling term
        // `c_roll` — a creep force with no motion to resist. If stationary hold
        // should command zero, the law needs a guard at v_ref = 0; this test pins
        // the *current* behavior so that change is a deliberate, visible flip.
        let mut c = feedforward(2.0, 3.0);
        assert_eq!(c.compute(0.1, &reference(0.0)), DriveForce::new(2.0));
    }
}
