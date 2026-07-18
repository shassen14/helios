//! Forward model for an accelerometer.

use crate::data::sensor::LinearAcceleration3D;
use crate::sensors::noise::TriaxialGaussian;

use nalgebra::{UnitQuaternion, Vector3};
use rand::RngCore;

/// Forward model for an accelerometer: reports specific force — the difference
/// between coordinate acceleration and gravity — expressed in the sensor frame,
/// with bias and per-axis noise. A sensor at rest reads the reaction to gravity,
/// not zero.
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

    /// Noise-free, bias-free measurement: the specific force
    /// `accel_world - gravity_world`, rotated into the sensor frame.
    /// `gravity_world` is the gravitational acceleration vector (points down),
    /// so subtracting it yields the proper acceleration an accelerometer feels.
    pub fn ideal(
        &self,
        accel_world: Vector3<f64>,
        gravity_world: Vector3<f64>,
        q_sensor_from_world: UnitQuaternion<f64>,
    ) -> Vector3<f64> {
        q_sensor_from_world * (accel_world - gravity_world)
    }

    /// Full measurement: the ideal specific force perturbed by bias and noise.
    pub fn sample(
        &self,
        accel_world: Vector3<f64>,
        gravity_world: Vector3<f64>,
        q_sensor_from_world: UnitQuaternion<f64>,
        rng: &mut dyn RngCore,
    ) -> LinearAcceleration3D {
        let ideal = self.ideal(accel_world, gravity_world, q_sensor_from_world);

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
        let got = model().ideal(accel_world, gravity_world, UnitQuaternion::identity());
        assert!((got - Vector3::new(0.0, 0.0, 9.81)).norm() < 1e-9);
    }

    #[test]
    fn ideal_subtracts_gravity_from_coordinate_acceleration() {
        let accel_world = Vector3::new(2.0, 0.0, 0.0);
        let gravity_world = Vector3::new(0.0, 0.0, -9.81);
        let got = model().ideal(accel_world, gravity_world, UnitQuaternion::identity());
        assert!((got - Vector3::new(2.0, 0.0, 9.81)).norm() < 1e-9);
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

        assert_eq!(
            model.sample(accel_world, gravity_world, q, &mut rng_a).value,
            model.sample(accel_world, gravity_world, q, &mut rng_b).value
        );
    }
}
