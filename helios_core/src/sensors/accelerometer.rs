//! Forward model for an accelerometer.

use crate::data::sensor::LinearAcceleration3D;
use crate::sensors::noise::TriaxialGaussian;

use nalgebra::{UnitQuaternion, Vector3};
use rand::RngCore;

/// Forward model for an accelerometer: reports specific force — coordinate
/// acceleration minus gravity, plus the lever-arm acceleration of a mount
/// offset from the body origin — expressed in the sensor frame, with bias and
/// per-axis noise. A sensor at rest reads the reaction to gravity, not zero.
#[derive(Debug, Clone)]
pub struct AccelerometerModel {
    noise: TriaxialGaussian,
}

impl AccelerometerModel {
    /// Builds the model. Returns `None` if any component of `stddev` is not
    /// strictly positive.
    pub fn new(bias: Vector3<f64>, stddev: Vector3<f64>) -> Option<Self> {
        Some(Self {
            noise: TriaxialGaussian::new(bias, stddev)?,
        })
    }

    /// Noise-free, bias-free specific force in the sensor frame.
    ///
    /// A sensor mounted at `lever_arm_world` (the offset from the body origin
    /// to the sensor, in world axes) on a rotating body feels the acceleration
    /// of its own mount point, not the origin's: the tangential term `α × r`
    /// and the centripetal term `ω × (ω × r)` add to the body acceleration.
    /// Gravity is then subtracted — `gravity_world` points down, so a sensor at
    /// rest reads its reaction — and the result is rotated into the sensor frame.
    pub fn ideal(
        &self,
        accel_world: Vector3<f64>,
        gravity_world: Vector3<f64>,
        angular_vel_world: Vector3<f64>,
        angular_accel_world: Vector3<f64>,
        lever_arm_world: Vector3<f64>,
        q_sensor_from_world: UnitQuaternion<f64>,
    ) -> Vector3<f64> {
        let accel_sensor_point = accel_world
            + angular_accel_world.cross(&lever_arm_world)
            + angular_vel_world.cross(&angular_vel_world.cross(&lever_arm_world));
        let specific_force = accel_sensor_point - gravity_world;
        q_sensor_from_world * specific_force
    }

    /// Full measurement: the ideal specific force perturbed by bias and noise.
    pub fn sample(
        &self,
        accel_world: Vector3<f64>,
        gravity_world: Vector3<f64>,
        angular_vel_world: Vector3<f64>,
        angular_accel_world: Vector3<f64>,
        lever_arm_world: Vector3<f64>,
        q_sensor_from_world: UnitQuaternion<f64>,
        rng: &mut dyn RngCore,
    ) -> LinearAcceleration3D {
        let ideal = self.ideal(
            accel_world,
            gravity_world,
            angular_vel_world,
            angular_accel_world,
            lever_arm_world,
            q_sensor_from_world,
        );

        LinearAcceleration3D {
            value: self.noise.apply(ideal, rng),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use rand::rngs::StdRng;
    use rand::SeedableRng;

    fn model() -> AccelerometerModel {
        AccelerometerModel::new(Vector3::zeros(), Vector3::new(0.01, 0.01, 0.01)).unwrap()
    }

    #[test]
    fn ideal_at_rest_reads_the_reaction_to_gravity() {
        // Stationary, level sensor in ENU: gravity points down (-Z), so the
        // specific force is +Z at one g.
        let accel_world = Vector3::zeros();
        let gravity_world = Vector3::new(0.0, 0.0, -9.81);
        let got = model().ideal(
            accel_world,
            gravity_world,
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
            UnitQuaternion::identity(),
        );
        assert!((got - Vector3::new(0.0, 0.0, 9.81)).norm() < 1e-9);
    }

    #[test]
    fn ideal_subtracts_gravity_from_coordinate_acceleration() {
        let accel_world = Vector3::new(2.0, 0.0, 0.0);
        let gravity_world = Vector3::new(0.0, 0.0, -9.81);
        let got = model().ideal(
            accel_world,
            gravity_world,
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
            UnitQuaternion::identity(),
        );
        assert!((got - Vector3::new(2.0, 0.0, 9.81)).norm() < 1e-9);
    }

    #[test]
    fn ideal_adds_the_centripetal_term_for_a_spinning_offset_mount() {
        // Spinning at 1 rad/s about world Up with the sensor 2 m forward of the
        // body origin: the centripetal term ω × (ω × r) has magnitude ω²·r = 2
        // and points inward (−forward). No gravity, no linear accel, so that is
        // the whole reading.
        let angular_vel = Vector3::new(0.0, 0.0, 1.0);
        let lever_arm = Vector3::new(2.0, 0.0, 0.0);
        let got = model().ideal(
            Vector3::zeros(),
            Vector3::zeros(),
            angular_vel,
            Vector3::zeros(),
            lever_arm,
            UnitQuaternion::identity(),
        );
        assert!((got - Vector3::new(-2.0, 0.0, 0.0)).norm() < 1e-9);
    }

    #[test]
    fn ideal_adds_the_tangential_term_for_an_angularly_accelerating_offset_mount() {
        // Angular acceleration 1 rad/s² about world Up with the sensor 2 m
        // forward: the tangential term α × r has magnitude α·r = 2 and points
        // +left. No spin, no gravity, no linear accel.
        let angular_accel = Vector3::new(0.0, 0.0, 1.0);
        let lever_arm = Vector3::new(2.0, 0.0, 0.0);
        let got = model().ideal(
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
            angular_accel,
            lever_arm,
            UnitQuaternion::identity(),
        );
        assert!((got - Vector3::new(0.0, 2.0, 0.0)).norm() < 1e-9);
    }

    #[test]
    fn new_rejects_nonpositive_stddev() {
        assert!(AccelerometerModel::new(Vector3::zeros(), Vector3::new(1.0, 1.0, 0.0)).is_none());
    }

    #[test]
    fn sample_is_reproducible_under_equal_seeds() {
        let model = model();
        let accel_world = Vector3::new(1.0, 0.0, 0.0);
        let gravity_world = Vector3::new(0.0, 0.0, -9.81);
        let q = UnitQuaternion::identity();

        let mut rng_a = StdRng::seed_from_u64(5);
        let mut rng_b = StdRng::seed_from_u64(5);

        let sample = |rng: &mut StdRng| {
            model
                .sample(
                    accel_world,
                    gravity_world,
                    Vector3::zeros(),
                    Vector3::zeros(),
                    Vector3::zeros(),
                    q,
                    rng,
                )
                .value
        };
        assert_eq!(sample(&mut rng_a), sample(&mut rng_b));
    }
}
