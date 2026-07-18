//! Additive noise shared by the state-sensor forward models.

use nalgebra::Vector3;
use rand::RngCore;
use rand_distr::{Distribution, Normal};

/// A constant per-axis bias plus zero-mean Gaussian noise on each of the three
/// axes. This is the error term every state-sensor forward model adds on top of
/// its noise-free measurement.
///
/// The three axes are sampled in `x, y, z` order on every call, so a given RNG
/// stream yields the same draw sequence on every run — the determinism the
/// seeded-PRNG rule depends on.
#[derive(Debug, Clone)]
pub struct TriaxialGaussian {
    bias: Vector3<f64>,
    dist: [Normal<f64>; 3],
}

impl TriaxialGaussian {
    /// Builds the noise model. Returns `None` if any component of `stddev` is
    /// not strictly positive, since a non-positive standard deviation has no
    /// valid Gaussian.
    pub fn new(bias: Vector3<f64>, stddev: Vector3<f64>) -> Option<Self> {
        if stddev.iter().any(|s| *s <= 0.0) {
            return None;
        }

        Some(Self {
            bias,
            dist: [
                Normal::new(0.0, stddev.x).ok()?,
                Normal::new(0.0, stddev.y).ok()?,
                Normal::new(0.0, stddev.z).ok()?,
            ],
        })
    }

    /// Returns `ideal` perturbed by the constant bias and one Gaussian draw per
    /// axis. The caller owns the RNG, so the perturbation is reproducible for a
    /// given seed.
    pub fn apply(&self, ideal: Vector3<f64>, rng: &mut dyn RngCore) -> Vector3<f64> {
        ideal
            + self.bias
            + Vector3::new(
                self.dist[0].sample(rng),
                self.dist[1].sample(rng),
                self.dist[2].sample(rng),
            )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use rand::rngs::StdRng;
    use rand::SeedableRng;

    #[test]
    fn new_rejects_nonpositive_stddev() {
        let bias = Vector3::zeros();
        assert!(TriaxialGaussian::new(bias, Vector3::new(0.0, 1.0, 1.0)).is_none());
        assert!(TriaxialGaussian::new(bias, Vector3::new(1.0, -1.0, 1.0)).is_none());
        assert!(TriaxialGaussian::new(bias, Vector3::new(1.0, 1.0, 1.0)).is_some());
    }

    #[test]
    fn apply_is_reproducible_under_equal_seeds() {
        let noise = TriaxialGaussian::new(Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)).unwrap();
        let ideal = Vector3::new(1.0, 2.0, 3.0);

        let mut rng_a = StdRng::seed_from_u64(42);
        let mut rng_b = StdRng::seed_from_u64(42);

        assert_eq!(noise.apply(ideal, &mut rng_a), noise.apply(ideal, &mut rng_b));
    }

    #[test]
    fn apply_mean_approaches_ideal_plus_bias() {
        let bias = Vector3::new(0.5, -0.25, 1.0);
        let noise = TriaxialGaussian::new(bias, Vector3::new(0.2, 0.2, 0.2)).unwrap();
        let ideal = Vector3::new(1.0, 2.0, 3.0);

        let mut rng = StdRng::seed_from_u64(7);
        let count = 20_000;
        let mut sum = Vector3::zeros();
        for _ in 0..count {
            sum += noise.apply(ideal, &mut rng);
        }
        let mean = sum / count as f64;

        // The zero-mean draws average out, leaving `ideal + bias` within the
        // standard error of the mean (~0.0014/axis at this stddev and count).
        assert!((mean - (ideal + bias)).norm() < 0.02);
    }
}
