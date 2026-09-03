use crate::control::{commands::SteerAngle, BodyTwistRef, ControlInputs, Controller};

/// Bicycle-model steering feedforward: the front steer angle that realizes a
/// reference yaw rate open-loop, computed from the reference twist alone.
///
/// This is the lateral twin of
/// [`RoadLoadFeedforward`](crate::control::controllers::feedforward::road_load::RoadLoadFeedforward).
/// Both are reference-only feedforward laws that invert a *car's* plant — road
/// load inverts its longitudinal resistance, this inverts its steering geometry —
/// which is why each names the plant rather than the axis it acts on.
///
/// The map is the bicycle model's yaw kinematics solved for the steer angle:
/// `ωz = vx·tan(δ)/L` inverted to `δ = atan(L·ωz / vx)`, with `L` the wheelbase.
/// Given a reference forward speed and yaw rate it precomputes the steer angle
/// that would produce that yaw, before any tracking error appears.
///
/// # Feedforward, not passthrough, and not the tracking loop
///
/// It reads the reference twist and nothing else — no estimate, so no odom→body
/// projection. Cross-track and heading feedback do not live here: the path
/// follower closes those geometrically and folds the correction into the reference
/// `ωz` it emits. So this stage is a pure kinematic inverse, the open-loop dual of
/// a feedback steering law; were a lateral inner-loop feedback controller ever
/// added, its correction would sum with this term (both emit [`SteerAngle`]).
///
/// # Low-speed guard
///
/// The inverse is ill-conditioned as `vx → 0`: the ratio grows without bound and
/// `atan` saturates toward ±π/2 — a physically absurd 90° steer — while at a true
/// standstill `0/0` is `NaN`. Below `MIN_STEER_SPEED` the wheel is centered rather
/// than chasing that garbage angle. A stopped car cannot realize a curved path
/// kinematically anyway, so commanding zero there loses nothing.
///
/// Stateless: the map is a pure function of the reference, so [`reset`](Self::reset)
/// is a no-op and there is no integrator to freeze.
pub struct BicycleSteerFeedforward {
    /// Wheelbase `L` (m) — front-to-rear axle distance, the lever in
    /// `δ = atan(L·ωz/vx)`.
    wheelbase: f64,
}

impl BicycleSteerFeedforward {
    /// Forward-speed *magnitude* (m/s) below which the steer angle is centered
    /// rather than computed. The inverse `atan(L·ωz/vx)` is ill-conditioned as
    /// `vx → 0` (and `0/0 → NaN` at a standstill), so near zero the wheel holds
    /// straight instead of chasing a garbage angle.
    const MIN_STEER_SPEED: f64 = 0.1;

    /// Constructs the feedforward from the vehicle wheelbase `L` (m).
    pub fn new(wheelbase: f64) -> Self {
        Self { wheelbase }
    }
}

impl Controller for BicycleSteerFeedforward {
    type Inputs = ControlInputs<BodyTwistRef>;
    type Out = SteerAngle;

    /// Inverts the bicycle yaw kinematics to the steer angle that realizes the
    /// reference yaw rate, `δ = atan(L·ωz / vx)`, reading forward speed and yaw
    /// rate off the reference twist (FLU: `+X` forward, `+Z` up). Centers the wheel
    /// below `MIN_STEER_SPEED`; with no reference, commands zero.
    fn compute(&mut self, _dt: f64, inputs: &Self::Inputs) -> Self::Out {
        let Some(reference) = inputs.reference.as_ref() else {
            return SteerAngle::zero();
        };

        let vel_x = reference.twist().linear().x();
        let ang_vel_z = reference.twist().angular().z();

        // δ = atan(L·ωz / vx), guarded near vx = 0 where the inverse blows up.
        let steer_angle = if vel_x.abs() < Self::MIN_STEER_SPEED {
            0.0
        } else {
            (self.wheelbase * ang_vel_z / vel_x).atan()
        };

        SteerAngle::new(steer_angle)
    }

    fn reset(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::commands::BodyTwist;
    use crate::estimation::schema::StateSchema;
    use crate::frames::FrameAwareState;

    use std::f64::consts::FRAC_PI_4;
    use std::sync::Arc;

    // Tests build the struct directly; a registry-facing constructor is the
    // controller factory's concern, not this leaf's.
    fn feedforward(wheelbase: f64) -> BicycleSteerFeedforward {
        BicycleSteerFeedforward { wheelbase }
    }

    // Feedforward never reads the estimate, so an empty state suffices — the point
    // is precisely that the output depends on the reference alone.
    fn empty_state() -> FrameAwareState {
        FrameAwareState::from_schema(Arc::new(StateSchema::compose(vec![])), 0.0)
    }

    // A reference twist carrying forward speed `vx` and yaw rate `wz` (FLU).
    fn reference(vx: f64, wz: f64) -> ControlInputs<BodyTwistRef> {
        ControlInputs {
            state: empty_state(),
            reference: Some(BodyTwistRef::new(BodyTwist::unicycle(vx, wz))),
        }
    }

    #[test]
    fn straight_reference_centers_the_wheel() {
        // ωz = 0 → atan(0) = 0: a straight-line reference commands zero steer.
        let mut c = feedforward(2.0);
        assert_eq!(c.compute(0.1, &reference(5.0, 0.0)), SteerAngle::zero());
    }

    #[test]
    fn steers_into_a_turn() {
        // L·ωz/vx = 2·1/2 = 1 → δ = atan(1) = π/4.
        let mut c = feedforward(2.0);
        let delta = c.compute(0.1, &reference(2.0, 1.0)).radians();
        assert!((delta - FRAC_PI_4).abs() < 1e-12, "got {delta}");
    }

    #[test]
    fn yaw_sign_sets_steer_sign() {
        // A right-hand turn (ωz < 0) mirrors to a right (negative) steer of equal
        // magnitude — the inverse carries the sign straight through atan.
        let mut c = feedforward(2.0);
        let left = c.compute(0.1, &reference(2.0, 1.0)).radians();
        let right = c.compute(0.1, &reference(2.0, -1.0)).radians();
        assert_eq!(left, -right);
    }

    #[test]
    fn longer_wheelbase_needs_more_steer() {
        // δ = atan(L·ωz/vx) grows with L for a fixed reference: a longer car must
        // crank the wheel further to hold the same yaw rate.
        let short = feedforward(1.0)
            .compute(0.1, &reference(2.0, 1.0))
            .radians();
        let long = feedforward(3.0)
            .compute(0.1, &reference(2.0, 1.0))
            .radians();
        assert!(long > short, "long {long} should exceed short {short}");
    }

    #[test]
    fn low_speed_centers_the_wheel() {
        // Below MIN_STEER_SPEED the inverse is ill-conditioned, so the wheel holds
        // straight even with a nonzero commanded yaw rate.
        let mut c = feedforward(2.0);
        let creep = BicycleSteerFeedforward::MIN_STEER_SPEED / 2.0;
        assert_eq!(c.compute(0.1, &reference(creep, 1.0)), SteerAngle::zero());
    }

    #[test]
    fn standstill_does_not_produce_nan() {
        // vx = 0 with a commanded yaw would be 0/0 → NaN without the guard; the
        // guard centers the wheel instead. Pins the NaN-avoidance explicitly.
        let mut c = feedforward(2.0);
        let out = c.compute(0.1, &reference(0.0, 1.0));
        assert_eq!(out, SteerAngle::zero());
        assert!(out.radians().is_finite());
    }

    #[test]
    fn no_reference_centers_the_wheel() {
        // With nothing to track the wheel holds straight, mirroring the drive
        // feedforward's zero-command-on-no-reference.
        let mut c = feedforward(2.0);
        let idle = ControlInputs {
            state: empty_state(),
            reference: None,
        };
        assert_eq!(c.compute(0.1, &idle), SteerAngle::zero());
    }

    #[test]
    fn reset_is_a_noop() {
        // The map is stateless: the same reference yields the same angle with a
        // `reset` in between, unlike a feedback controller's integrator.
        let mut c = feedforward(2.0);
        let before = c.compute(0.1, &reference(2.0, 1.0));
        c.reset();
        let after = c.compute(0.1, &reference(2.0, 1.0));
        assert_eq!(before, after);
    }
}
