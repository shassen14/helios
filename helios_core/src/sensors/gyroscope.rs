//! Forward model for a rate gyroscope.

use crate::data::sensor::AngularVelocity3D;
use crate::sensors::noise::TriaxialGaussian;

use nalgebra::{UnitQuaternion, Vector3};
use rand::RngCore;

/// Forward model for a rate gyroscope: reports the body's angular velocity
/// expressed in the sensor frame, with bias and per-axis noise.
#[derive(Debug, Clone)]
pub struct GyroscopeModel {
    noise: TriaxialGaussian,
}

impl GyroscopeModel {
    /// Builds the model. Returns `None` if any component of `stddev` is not
    /// strictly positive.
    pub fn new(bias: Vector3<f64>, stddev: Vector3<f64>) -> Option<Self> {
        Some(Self {
            noise: TriaxialGaussian::new(bias, stddev)?,
        })
    }

    /// Noise-free, bias-free measurement: the true world-frame angular velocity
    /// rotated into the sensor frame. `q_sensor_from_world` maps world vectors
    /// into the sensor's own axes.
    pub fn ideal(
        &self,
        angular_velocity_world: Vector3<f64>,
        q_sensor_from_world: UnitQuaternion<f64>,
    ) -> Vector3<f64> {
        q_sensor_from_world * angular_velocity_world
    }

    /// Full measurement: the ideal rate perturbed by bias and noise.
    pub fn sample(
        &self,
        angular_velocity_world: Vector3<f64>,
        q_sensor_from_world: UnitQuaternion<f64>,
        rng: &mut dyn RngCore,
    ) -> AngularVelocity3D {
        let ideal = self.ideal(angular_velocity_world, q_sensor_from_world);
        AngularVelocity3D {
            value: self.noise.apply(ideal, rng),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::f64::consts::FRAC_PI_2;

    use rand::rngs::StdRng;
    use rand::SeedableRng;

    fn model() -> GyroscopeModel {
        GyroscopeModel::new(Vector3::zeros(), Vector3::new(0.01, 0.01, 0.01)).unwrap()
    }

    #[test]
    fn ideal_under_identity_passes_the_rate_through() {
        let rate = Vector3::new(1.0, 2.0, 3.0);
        assert_eq!(model().ideal(rate, UnitQuaternion::identity()), rate);
    }

    #[test]
    fn ideal_rotates_the_rate_into_the_sensor_frame() {
        // A sensor yawed +90° about world Z sees a world +X rate as +Y.
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        let got = model().ideal(Vector3::new(1.0, 0.0, 0.0), q);
        assert!((got - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-9);
    }

    #[test]
    fn new_rejects_nonpositive_stddev() {
        assert!(GyroscopeModel::new(Vector3::zeros(), Vector3::new(1.0, 0.0, 1.0)).is_none());
    }

    #[test]
    fn sample_is_reproducible_under_equal_seeds() {
        let model = model();
        let rate = Vector3::new(0.5, 0.5, 0.5);
        let q = UnitQuaternion::identity();

        let mut rng_a = StdRng::seed_from_u64(3);
        let mut rng_b = StdRng::seed_from_u64(3);

        assert_eq!(
            model.sample(rate, q, &mut rng_a).value,
            model.sample(rate, q, &mut rng_b).value
        );
    }
}
