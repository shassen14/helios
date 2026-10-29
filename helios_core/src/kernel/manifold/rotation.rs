//! `SO(3)` rotation maths shared by everything that stores an orientation as
//! numbers: the estimator's [`QuaternionBlock`](super::quaternion::QuaternionBlock)
//! and any other stored attitude. One copy, so every caller agrees on the
//! storage order, the side the increment is applied on, and the log map.
//!
//! A unit quaternion stores 4 numbers but a rotation has only 3 degrees of
//! freedom, so a stored rotation is [`STORAGE_DIM`] wide and a tangent step
//! (an error, a velocity, a covariance axis) is [`TANGENT_DIM`] wide:
//!
//! - `oplus(q, δ) = q ⊗ exp(δ)`: rotate `q` by the small rotation `exp(δ)`.
//! - `ominus(y, x) = log(x⁻¹ ⊗ y)`: the tangent step that carries `x` to `y`.
//!
//! `exp` / `log` are `nalgebra`'s `UnitQuaternion::from_scaled_axis` /
//! `scaled_axis` (a tangent vector is an axis scaled by its angle). The step is
//! applied on the **right**, a body-frame step, matching the INS kinematics
//! `q̇ = ½ q ⊗ ω_body`; that side is fixed by the dynamics, not free. The log
//! takes the short way: a quaternion and its negation name one rotation, and
//! `ominus` returns a step of at most π.
//!
//! **Two orderings collide, so only this module reorders.** Storage is
//! `[x, y, z, w]` (scalar last), while `Quaternion::new` takes `(w, i, j, k)`
//! (scalar first). [`from_storage`] and [`to_storage`] are the only crossings
//! between the two; the group operations work on typed `nalgebra` values and
//! never see the order.

use nalgebra::{Quaternion, UnitQuaternion, Vector3};

/// Numbers a stored rotation occupies: a unit quaternion `[x, y, z, w]`.
pub const STORAGE_DIM: usize = 4;
/// Degrees of freedom of a rotation: the size of its tangent step and its
/// covariance.
pub const TANGENT_DIM: usize = 3;

/// The rotation a stored `[x, y, z, w]` names.
///
/// Takes a slice, not an array, because callers hold views into larger
/// buffers; each caller guarantees the length where it lays out its storage,
/// so it is only debug-asserted here. The quaternion is normalized on the way
/// in: a stored value that drifted off unit length is read as the nearest
/// rotation, not refused. Refusing one belongs to whoever writes the storage.
pub fn from_storage(stored: &[f64]) -> UnitQuaternion<f64> {
    debug_assert!(stored.len() == STORAGE_DIM);
    UnitQuaternion::from_quaternion(Quaternion::new(stored[3], stored[0], stored[1], stored[2]))
}

/// `rotation` as stored numbers, `[x, y, z, w]`.
pub fn to_storage(rotation: &UnitQuaternion<f64>) -> [f64; STORAGE_DIM] {
    let quat = rotation.quaternion();

    [quat.i, quat.j, quat.k, quat.w]
}

/// `rotation ⊞ delta`: `rotation` turned by the body-frame step `delta`
/// (axis × angle, radians).
pub fn oplus(rotation: &UnitQuaternion<f64>, delta: &Vector3<f64>) -> UnitQuaternion<f64> {
    // exp: the tangent rotation vector (axis·angle) becomes a unit quaternion.
    let exp_delta = UnitQuaternion::from_scaled_axis(*delta);

    // Right-multiply — the error is applied in the body frame, matching the
    // INS kinematics q̇ = ½ q ⊗ ω_body.
    rotation * exp_delta
}

/// `y ⊟ x`: the body-frame step from `x` to `y`, so that
/// `oplus(x, ominus(y, x))` is `y`. Its angle is at most π.
pub fn ominus(y: &UnitQuaternion<f64>, x: &UnitQuaternion<f64>) -> Vector3<f64> {
    // log of the relative rotation x⁻¹ ⊗ y: the tangent vector that, applied
    // to x via oplus, lands on y. `scaled_axis` is the SO(3) log map.
    (x.inverse() * y).scaled_axis()
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI};

    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    /// Slack for results that went through float arithmetic.
    const TOLERANCE: f64 = 1e-12;
    /// Slack for round trips through exp and log, which lose a little more.
    const ROUND_TRIP_TOLERANCE: f64 = 1e-9;
    /// Random cases per property test.
    const SAMPLES: usize = 1000;
    /// Bound on each random axis·angle component; ±3 rad lets a rotation's
    /// angle pass π, where the double cover shows.
    const COMPONENT_BOUND: f64 = 3.0;
    /// Bound on each random step component; |step| ≤ √3 · 0.5 < π, inside
    /// the range where the log returns the step exp consumed.
    const SMALL_STEP_BOUND: f64 = 0.5;

    fn random_rotation(rng: &mut StdRng) -> UnitQuaternion<f64> {
        UnitQuaternion::from_scaled_axis(Vector3::new(
            rng.gen_range(-COMPONENT_BOUND..COMPONENT_BOUND),
            rng.gen_range(-COMPONENT_BOUND..COMPONENT_BOUND),
            rng.gen_range(-COMPONENT_BOUND..COMPONENT_BOUND),
        ))
    }

    #[test]
    fn storage_puts_the_scalar_last() {
        // A quarter turn of yaw is (w, z) = (cos π/4, sin π/4) with x = y = 0.
        // Stored scalar-last, the z part is index 2 and w is index 3.
        let yaw = UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.0, FRAC_PI_2));

        let stored = to_storage(&yaw);

        assert!(stored[0].abs() < TOLERANCE);
        assert!(stored[1].abs() < TOLERANCE);
        assert!((stored[2] - FRAC_1_SQRT_2).abs() < TOLERANCE);
        assert!((stored[3] - FRAC_1_SQRT_2).abs() < TOLERANCE);
    }

    #[test]
    fn identity_is_stored_as_w_one() {
        assert_eq!(
            to_storage(&UnitQuaternion::identity()),
            [0.0, 0.0, 0.0, 1.0]
        );
        assert_eq!(
            from_storage(&[0.0, 0.0, 0.0, 1.0]),
            UnitQuaternion::identity()
        );
    }

    #[test]
    fn storage_round_trip_reads_back_the_same_rotation() {
        let mut rng = StdRng::seed_from_u64(0x0A0B_0C0D);

        for _ in 0..SAMPLES {
            let rotation = random_rotation(&mut rng);

            let recovered = from_storage(&to_storage(&rotation));

            assert!(recovered.angle_to(&rotation) < TOLERANCE);
        }
    }

    #[test]
    fn a_stored_value_off_unit_length_reads_as_its_rotation() {
        // Twice the identity: from_storage normalizes, it does not refuse.
        let recovered = from_storage(&[0.0, 0.0, 0.0, 2.0]);

        assert!(recovered.angle_to(&UnitQuaternion::identity()) < TOLERANCE);
    }

    #[test]
    fn oplus_turns_on_the_body_side() {
        // Rotations do not commute, so a yaw step after a quarter turn of
        // pitch lands somewhere different depending on the side it is applied
        // on. oplus must match the right-hand (body-frame) product, and the
        // left-hand (world-frame) one must differ, or the test proves nothing.
        let pitch = UnitQuaternion::from_scaled_axis(Vector3::new(0.0, FRAC_PI_2, 0.0));
        let yaw_step = Vector3::new(0.0, 0.0, FRAC_PI_2);

        let turned = oplus(&pitch, &yaw_step);

        let body_side = pitch * UnitQuaternion::from_scaled_axis(yaw_step);
        let world_side = UnitQuaternion::from_scaled_axis(yaw_step) * pitch;
        assert!(turned.angle_to(&body_side) < TOLERANCE);
        assert!(turned.angle_to(&world_side) > ROUND_TRIP_TOLERANCE);
    }

    #[test]
    fn ominus_of_a_rotation_and_its_negation_is_zero() {
        // q and −q name one rotation (the double cover), so the step between
        // them is nothing, not a full turn.
        let mut rng = StdRng::seed_from_u64(0x1112_1314);

        for _ in 0..SAMPLES {
            let q = random_rotation(&mut rng);
            let stored = to_storage(&q);
            let negated = from_storage(&stored.map(|component| -component));

            let step = ominus(&negated, &q);

            assert!(
                step.norm() < ROUND_TRIP_TOLERANCE,
                "step was {}",
                step.norm()
            );
        }
    }

    #[test]
    fn ominus_takes_the_short_way() {
        let mut rng = StdRng::seed_from_u64(0x2122_2324);

        for _ in 0..SAMPLES {
            let x = random_rotation(&mut rng);
            let y = random_rotation(&mut rng);

            let step = ominus(&y, &x);

            assert!(step.norm() <= PI + TOLERANCE, "step was {}", step.norm());
        }
    }

    #[test]
    fn oplus_undoes_ominus_past_a_half_turn() {
        let mut rng = StdRng::seed_from_u64(0x3132_3334);

        for _ in 0..SAMPLES {
            let x = random_rotation(&mut rng);
            let y = random_rotation(&mut rng);

            let recovered = oplus(&x, &ominus(&y, &x));

            assert!(recovered.angle_to(&y) < ROUND_TRIP_TOLERANCE);
        }
    }

    #[test]
    fn ominus_undoes_oplus_for_steps_under_a_half_turn() {
        // Below π the log returns exactly the step exp consumed: no double
        // cover ambiguity in that range.
        let mut rng = StdRng::seed_from_u64(0x4142_4344);

        for _ in 0..SAMPLES {
            let x = random_rotation(&mut rng);
            let step = Vector3::new(
                rng.gen_range(-SMALL_STEP_BOUND..SMALL_STEP_BOUND),
                rng.gen_range(-SMALL_STEP_BOUND..SMALL_STEP_BOUND),
                rng.gen_range(-SMALL_STEP_BOUND..SMALL_STEP_BOUND),
            );

            let recovered = ominus(&oplus(&x, &step), &x);

            assert!((recovered - step).amax() < ROUND_TRIP_TOLERANCE);
        }
    }
}
