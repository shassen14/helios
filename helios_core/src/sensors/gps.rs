//! Forward model for a GPS position sensor.

use crate::data::sensor::GpsPosition;
use crate::sensors::noise::TriaxialGaussian;

use nalgebra::Vector3;
use rand::RngCore;

/// Forward model for a GPS receiver: reports the sensor's world position with
/// bias and per-axis noise.
#[derive(Debug, Clone)]
pub struct GpsModel {
    noise: TriaxialGaussian,
}

impl GpsModel {
    /// Builds the model. Returns `None` if any component of `stddev` is not
    /// strictly positive.
    pub fn new(bias: Vector3<f64>, stddev: Vector3<f64>) -> Option<Self> {
        Some(Self {
            noise: TriaxialGaussian::new(bias, stddev)?,
        })
    }

    /// Noise-free, bias-free measurement. A GPS observes world position
    /// directly, so the ideal reading is the position itself.
    pub fn ideal(&self, position: Vector3<f64>) -> Vector3<f64> {
        position
    }

    /// Full measurement: the ideal position perturbed by bias and noise.
    pub fn sample(&self, position: Vector3<f64>, rng: &mut dyn RngCore) -> GpsPosition {
        let ideal = self.ideal(position);
        GpsPosition {
            position: self.noise.apply(ideal, rng),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use rand::rngs::StdRng;
    use rand::SeedableRng;

    #[test]
    fn ideal_is_the_position_itself() {
        let model = GpsModel::new(Vector3::zeros(), Vector3::new(1.0, 1.0, 1.0)).unwrap();
        let position = Vector3::new(3.0, -4.0, 5.0);
        assert_eq!(model.ideal(position), position);
    }

    #[test]
    fn new_rejects_nonpositive_stddev() {
        assert!(GpsModel::new(Vector3::zeros(), Vector3::new(0.0, 1.0, 1.0)).is_none());
    }

    #[test]
    fn sample_is_reproducible_under_equal_seeds() {
        let model = GpsModel::new(Vector3::zeros(), Vector3::new(0.5, 0.5, 0.5)).unwrap();
        let position = Vector3::new(1.0, 1.0, 1.0);

        let mut rng_a = StdRng::seed_from_u64(1);
        let mut rng_b = StdRng::seed_from_u64(1);

        assert_eq!(
            model.sample(position, &mut rng_a).position,
            model.sample(position, &mut rng_b).position
        );
    }
}
