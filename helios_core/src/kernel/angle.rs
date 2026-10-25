//! Arithmetic on angles that wrap around a full circle: yaw, roll, a steering
//! or bearing angle.
//!
//! An angle and the same angle plus a whole turn name one direction, so these
//! helpers keep a stored angle in one canonical range and compare two angles by
//! the short way round. Euler pitch is not such an angle: its range is a half
//! turn that does not wrap, and these helpers do not apply to it.

use std::f64::consts::{PI, TAU};

/// The same direction as `angle`, expressed in (−π, π].
///
/// The range is closed at +π and open at −π, so a half turn has one stored
/// value: −π and 3π both wrap to π.
pub fn wrap(angle: f64) -> f64 {
    let mut wrapped = PI - (PI - angle).rem_euclid(TAU);
    // `rem_euclid` can round up to τ itself for a remainder a hair below zero,
    // which lands the result on −π, the end the range excludes.
    if wrapped <= -PI {
        wrapped += TAU;
    }

    wrapped
}

/// The signed turn from `angle_a` to `angle_b` the short way round, in
/// (−π, π]. Positive is counterclockwise: `angle_a` plus the result points the
/// same way as `angle_b`.
///
/// Two angles exactly a half turn apart are π apart either way; the result is
/// then +π, never −π, since that is the end of the range [`wrap`] keeps.
pub fn difference(angle_a: f64, angle_b: f64) -> f64 {
    wrap(angle_b - angle_a)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Slack for comparing angles that went through float arithmetic.
    const TOLERANCE: f64 = 1e-12;

    fn degrees(value: f64) -> f64 {
        value.to_radians()
    }

    #[test]
    fn an_angle_already_in_range_is_unchanged() {
        assert_eq!(wrap(0.0), 0.0);
        assert_eq!(wrap(1.0), 1.0);
        assert_eq!(wrap(-1.0), -1.0);
    }

    #[test]
    fn a_half_turn_wraps_to_plus_pi_from_either_side() {
        assert_eq!(wrap(PI), PI);
        assert_eq!(wrap(-PI), PI);
        assert!((wrap(3.0 * PI) - PI).abs() < TOLERANCE);
        assert!((wrap(-3.0 * PI) - PI).abs() < TOLERANCE);
    }

    #[test]
    fn whole_turns_are_removed() {
        assert!((wrap(1.0 + 2.0 * TAU) - 1.0).abs() < TOLERANCE);
        assert!((wrap(1.0 - 2.0 * TAU) - 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn an_angle_a_rounding_error_past_pi_stays_in_range() {
        // The next float above π, whose remainder rounds up to a full turn.
        let just_past_pi = f64::from_bits(PI.to_bits() + 1);
        let wrapped = wrap(just_past_pi);
        assert!(
            wrapped > -PI && wrapped <= PI,
            "{wrapped} is outside (-π, π]"
        );
    }

    #[test]
    fn the_difference_takes_the_short_way_round() {
        assert!((difference(degrees(359.0), degrees(1.0)) - degrees(2.0)).abs() < TOLERANCE);
        assert!((difference(degrees(1.0), degrees(359.0)) - degrees(-2.0)).abs() < TOLERANCE);
    }

    #[test]
    fn the_difference_is_counterclockwise_positive() {
        assert!((difference(0.0, 1.0) - 1.0).abs() < TOLERANCE);
        assert!((difference(1.0, 0.0) + 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn a_half_turn_difference_is_plus_pi_either_way() {
        assert_eq!(difference(0.0, PI), PI);
        assert_eq!(difference(PI, 0.0), PI);
    }
}
