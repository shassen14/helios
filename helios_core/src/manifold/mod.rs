//! State-manifold blocks: the building units of an estimator's state vector.
//!
//! Each [`StateBlock`] owns one contiguous slice of the state and answers every
//! question about that slice independently of where it sits in the full vector.
//! A block is defined by two sizes that coincide only for flat (Euclidean)
//! quantities:
//!
//! - `storage_dim` — how many numbers the block occupies in the *stored* state
//!   (a unit quaternion stores 4).
//! - `tangent_dim` — its true degrees of freedom, the size of its covariance
//!   and error slice (a rotation has only 3).
//!
//! Motion on the block goes through the retraction pair `oplus` (⊞) and
//! `ominus` (⊟). They default to ordinary `+` / `-` — exactly right for a
//! Euclidean block, overridden by curved ones — and must round-trip:
//! `x ⊞ (y ⊟ x) == y`. Uncertainty lives in [`TangentNoise`], a tangent-space
//! Gaussian holding one covariance with two faces: the matrix a Kalman update
//! reads, and the sampler a particle filter or simulator draws from.

pub mod euclidean;
pub mod quaternion;

use std::fmt::Debug;

use nalgebra::{DMatrix, DVector, DVectorView};
use rand::Rng;
use rand_distr::StandardNormal;

pub trait StateBlock: Debug + Send + Sync {
    fn storage_dim(&self) -> usize;

    fn tangent_dim(&self) -> usize;

    fn oplus(&self, x: DVectorView<f64>, delta: DVectorView<f64>) -> DVector<f64> {
        x.into_owned() + delta
    }

    fn ominus(&self, y: DVectorView<f64>, x: DVectorView<f64>) -> DVector<f64> {
        y.into_owned() - x
    }

    fn process_noise(&self) -> Option<TangentNoise>;

    fn initial_value(&self) -> DVector<f64>;

    fn initial_covariance(&self) -> DMatrix<f64>;
}

#[derive(Clone, Debug)]
pub struct TangentNoise {
    covariance: DMatrix<f64>,
    cholesky_l: DMatrix<f64>,
}

impl TangentNoise {
    pub fn from_covariance(covariance: DMatrix<f64>) -> Option<Self> {
        let l = covariance.clone().cholesky()?.l();

        Some(Self {
            covariance,
            cholesky_l: l,
        })
    }

    pub fn from_variances(variances: DVector<f64>) -> Option<Self> {
        Self::from_covariance(DMatrix::from_diagonal(&variances))
    }

    pub fn from_std_devs(std_devs: DVector<f64>) -> Option<Self> {
        Self::from_variances(std_devs.map(|s| s * s))
    }

    pub fn covariance(&self) -> &DMatrix<f64> {
        &self.covariance
    }

    pub fn sample(&self, rng: &mut impl Rng) -> DVector<f64> {
        let z = DVector::from_fn(self.covariance.nrows(), |_, _| rng.sample(StandardNormal));

        &self.cholesky_l * z
    }
}

#[cfg(test)]
mod tests {
    use super::TangentNoise;

    use rand::rngs::StdRng;
    use rand::SeedableRng;

    use nalgebra::{DMatrix, DVector};

    #[test]
    fn from_std_devs_squares_into_diagonal_variances() {
        let noise = TangentNoise::from_std_devs(DVector::from_vec(vec![2.0, 3.0])).unwrap();
        let cov = noise.covariance();

        assert_eq!(cov[(0, 0)], 4.0);
        assert_eq!(cov[(1, 1)], 9.0);
        assert_eq!(cov[(0, 1)], 0.0);
        assert_eq!(cov[(1, 0)], 0.0);
    }

    #[test]
    fn cholesky_factor_reconstructs_covariance() {
        let cov = DMatrix::from_row_slice(2, 2, &[2.0, 0.5, 0.5, 1.0]);
        let noise = TangentNoise::from_covariance(cov.clone()).unwrap();

        let reconstructed = &noise.cholesky_l * noise.cholesky_l.transpose();
        assert!((reconstructed - cov).amax() < 1e-12);
    }

    #[test]
    fn non_positive_definite_covariance_is_rejected() {
        // Eigenvalues 3 and -1 → indefinite, so there is no Cholesky factor.
        let indefinite = DMatrix::from_row_slice(2, 2, &[1.0, 2.0, 2.0, 1.0]);
        assert!(TangentNoise::from_covariance(indefinite).is_none());
    }

    #[test]
    fn sampler_and_covariance_views_agree() {
        // The two faces of TangentNoise must describe the same Gaussian: the
        // empirical covariance of many samples must match the stored matrix.
        let cov = DMatrix::from_row_slice(2, 2, &[2.0, 0.5, 0.5, 1.0]);
        let noise = TangentNoise::from_covariance(cov.clone()).unwrap();

        let mut rng = StdRng::seed_from_u64(0xC0FFEE);
        let n = 200_000;
        let mut acc = DMatrix::<f64>::zeros(2, 2);
        for _ in 0..n {
            let s = noise.sample(&mut rng);
            acc += &s * s.transpose();
        }
        acc /= n as f64;

        assert!((acc - cov).amax() < 0.05);
    }
}
