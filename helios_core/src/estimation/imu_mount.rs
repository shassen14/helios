//! An IMU's readings and the body's motion, related through the IMU's mount.
//!
//! A gyro on a rigid body reads the body's angular velocity in its own axes.
//! An accelerometer reads the specific force at its own mount point, which
//! differs from the body origin's by the lever-arm terms: the centripetal
//! `ω×(ω×r)` and the tangential `α×r`. Gravity is the same at both points, so
//! it cancels from the difference.
//!
//! Both directions live here, so every estimation-side use shares one
//! formula:
//! - **body → sensor** ([`sensor_angular_velocity`],
//!   [`sensor_specific_force`]): what an IMU on the body reads; a
//!   measurement model's prediction.
//! - **sensor → body** ([`body_angular_velocity`], [`body_specific_force`]):
//!   the body-origin quantities behind a reading; the input of dynamics
//!   driven by the IMU.
//!
//! The mount is given as `body_from_sensor` (`R`), rotating sensor axes into
//! body axes, and `offset` (`r`), the sensor's origin in body axes. `ω` and
//! `α` are the body's, in body axes.
//!
//! The simulator's forward model in `sensors/` computes the same physics on
//! its own and must keep doing so: sharing this with it would make the
//! filter's model of the IMU exact by construction.

use nalgebra::{UnitQuaternion, Vector3};

/// The body's angular velocity `ω`, resolved in the sensor's axes: what a gyro
/// on the body reads, noise and bias aside.
///
/// Exact for any mount: every point of a rigid body turns at the same rate,
/// so the gyro's offset does not enter.
pub fn sensor_angular_velocity(
    body_from_sensor: &UnitQuaternion<f64>,
    angular_velocity: &Vector3<f64>,
) -> Vector3<f64> {
    body_from_sensor.inverse() * angular_velocity
}

/// The body's angular velocity, in body axes, from a gyro's reading in its
/// own axes. The inverse of [`sensor_angular_velocity`].
pub fn body_angular_velocity(
    body_from_sensor: &UnitQuaternion<f64>,
    sensor_rate: &Vector3<f64>,
) -> Vector3<f64> {
    body_from_sensor * sensor_rate
}

/// What an accelerometer at `offset` reads, in its own axes, while the body
/// origin feels `body_specific_force`:
/// `f_s = Rᵀ·(f_b + ω×(ω×r) + α×r)`.
///
/// `body_specific_force` is the origin's kinematic acceleration minus gravity,
/// in body axes. Without `α` the tangential term is left out.
pub fn sensor_specific_force(
    body_from_sensor: &UnitQuaternion<f64>,
    offset: &Vector3<f64>,
    body_specific_force: &Vector3<f64>,
    angular_velocity: &Vector3<f64>,
    angular_acceleration: Option<&Vector3<f64>>,
) -> Vector3<f64> {
    let at_sensor = body_specific_force
        + lever_arm_acceleration(offset, angular_velocity, angular_acceleration);
    body_from_sensor.inverse() * at_sensor
}

/// The specific force at the body origin, in body axes, from an
/// accelerometer's reading in its own axes:
/// `f_b = R·f_s − ω×(ω×r) − α×r`. The inverse of [`sensor_specific_force`].
///
/// Without `α` the tangential term is left in, an error of `|α||r|` while the
/// body's rotation speeds up or slows down. Dynamics that take the IMU as
/// their input pass none: a state holding `α` holds `ω` too, and there the
/// IMU is a measurement predicted with [`sensor_specific_force`].
///
/// The centripetal term is only as good as `ω`. Fed the raw gyro, it carries
/// the gyro's bias, an error of about `2|ω||b_g||r|`, and its noise, which
/// no process noise models; both are small beside the accelerometer's own.
pub fn body_specific_force(
    body_from_sensor: &UnitQuaternion<f64>,
    offset: &Vector3<f64>,
    sensor_specific_force: &Vector3<f64>,
    angular_velocity: &Vector3<f64>,
    angular_acceleration: Option<&Vector3<f64>>,
) -> Vector3<f64> {
    body_from_sensor * sensor_specific_force
        - lever_arm_acceleration(offset, angular_velocity, angular_acceleration)
}

/// How much more a point at `offset` accelerates than the body origin:
/// `ω×(ω×r) + α×r`, the tangential term only when `α` is given.
fn lever_arm_acceleration(
    offset: &Vector3<f64>,
    angular_velocity: &Vector3<f64>,
    angular_acceleration: Option<&Vector3<f64>>,
) -> Vector3<f64> {
    let centripetal = angular_velocity.cross(&angular_velocity.cross(offset));
    let tangential = angular_acceleration
        .map(|alpha| alpha.cross(offset))
        .unwrap_or_default();
    centripetal + tangential
}

#[cfg(test)]
mod tests {
    use super::*;

    use approx::assert_relative_eq;
    use std::f64::consts::FRAC_PI_2;

    const G: f64 = 9.81;
    /// Tolerance on vectors built from a rotation and a few cross products.
    const EPS: f64 = 1e-12;
    /// A steady yaw rate (rad/s).
    const YAW_RATE: f64 = 2.0;
    /// A yaw acceleration (rad/s²).
    const YAW_ACCEL: f64 = 3.0;
    /// The sensor's distance ahead of the body origin (m).
    const LEVER: f64 = 0.5;

    /// A sensor turned a quarter turn left about up.
    fn quarter_yaw() -> UnitQuaternion<f64> {
        UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2)
    }

    /// A sensor upside down, yawed and tilted: no axis left in place.
    fn skewed() -> UnitQuaternion<f64> {
        UnitQuaternion::from_euler_angles(3.0, 0.4, 1.2)
    }

    fn ahead() -> Vector3<f64> {
        Vector3::new(LEVER, 0.0, 0.0)
    }

    fn yawing() -> Vector3<f64> {
        Vector3::new(0.0, 0.0, YAW_RATE)
    }

    /// What a body standing level reads at its origin: one g up.
    fn level() -> Vector3<f64> {
        Vector3::new(0.0, 0.0, G)
    }

    /// At identity on the origin, the readings are the body's, both ways.
    #[test]
    fn an_identity_mount_changes_nothing() {
        let force = Vector3::new(0.3, -0.2, G);
        let rate = Vector3::new(0.1, 0.2, 0.3);
        let identity = UnitQuaternion::identity();
        let origin = Vector3::zeros();

        assert_eq!(body_angular_velocity(&identity, &rate), rate);
        assert_eq!(sensor_angular_velocity(&identity, &rate), rate);
        assert_eq!(
            body_specific_force(&identity, &origin, &force, &rate, None),
            force
        );
        assert_eq!(
            sensor_specific_force(&identity, &origin, &force, &rate, None),
            force
        );
    }

    /// A rotated mount's readings come back in body axes, both quantities.
    #[test]
    fn a_rotated_mount_is_rotated_into_body_axes() {
        let sensor_forward = Vector3::new(1.0, 0.0, 0.0);
        let body_left = Vector3::new(0.0, 1.0, 0.0);

        assert_relative_eq!(
            body_angular_velocity(&quarter_yaw(), &sensor_forward),
            body_left,
            epsilon = EPS
        );
        assert_relative_eq!(
            body_specific_force(
                &quarter_yaw(),
                &Vector3::zeros(),
                &sensor_forward,
                &Vector3::zeros(),
                None
            ),
            body_left,
            epsilon = EPS
        );
    }

    /// A body turning in place at a steady rate: a sensor ahead of the origin
    /// feels `ω²r` toward it on top of gravity's reaction.
    #[test]
    fn steady_turning_adds_the_centripetal_term_at_the_sensor() {
        let reading = sensor_specific_force(&quarter_yaw(), &ahead(), &level(), &yawing(), None);
        let expected = Vector3::new(-YAW_RATE.powi(2) * LEVER, 0.0, G);

        assert_relative_eq!(quarter_yaw() * reading, expected, epsilon = EPS);
    }

    /// `α` enters only when given: the tangential term `α×r`, and nothing
    /// else.
    #[test]
    fn angular_acceleration_enters_only_when_given() {
        let alpha = Vector3::new(0.0, 0.0, YAW_ACCEL);
        let identity = UnitQuaternion::identity();

        let with = sensor_specific_force(&identity, &ahead(), &level(), &yawing(), Some(&alpha));
        let without = sensor_specific_force(&identity, &ahead(), &level(), &yawing(), None);

        assert_relative_eq!(with - without, alpha.cross(&ahead()), epsilon = EPS);
    }

    /// Each inverse undoes its forward for any mount, with and without `α`,
    /// so a model predicting readings and a builder reading them agree.
    #[test]
    fn each_direction_inverts_the_other() {
        let offset = Vector3::new(0.8, -0.3, 0.4);
        let force = Vector3::new(0.4, -1.1, G);
        let omega = Vector3::new(0.2, -0.1, YAW_RATE);
        let alpha = Vector3::new(-0.5, 0.3, YAW_ACCEL);

        let rate = sensor_angular_velocity(&skewed(), &omega);
        assert_relative_eq!(
            body_angular_velocity(&skewed(), &rate),
            omega,
            epsilon = EPS
        );

        for alpha in [None, Some(&alpha)] {
            let reading = sensor_specific_force(&skewed(), &offset, &force, &omega, alpha);
            assert_relative_eq!(
                body_specific_force(&skewed(), &offset, &reading, &omega, alpha),
                force,
                epsilon = EPS
            );
        }
    }
}
