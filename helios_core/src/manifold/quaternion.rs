//! [`QuaternionBlock`]: orientation carried as a *placeholder* 4/4 Euclidean
//! block.
//!
//! This is deliberately not the correct manifold form. A unit quaternion has 4
//! stored numbers but only 3 real degrees of freedom, and its retraction is
//! `q ⊗ exp(δ)`, not componentwise addition. Until the error-state form is
//! implemented, orientation rides as a degenerate 4/4 Euclidean block so the
//! stored state layout reproduces the legacy fixed-layout estimator exactly.
//! Do not build error-state covariance math on top of this placeholder — its
//! 4-dimensional tangent is a stand-in, not the truth.

use super::{StateBlock, TangentNoise};

use nalgebra::{DMatrix, DVector};

pub struct QuaternionBlock {
    process_noise: TangentNoise,
    initial_value: DVector<f64>,
    initial_covariance: DMatrix<f64>,
}

impl QuaternionBlock {
    pub fn new(
        process_noise: TangentNoise,
        initial_value: DVector<f64>,
        initial_covariance: DMatrix<f64>,
    ) -> Self {
        assert_eq!(initial_value.len(), 4, "quaternion storage must be 4");
        assert_eq!(
            process_noise.covariance().nrows(),
            4,
            "noise must be 4×4 (placeholder)"
        );
        assert_eq!(
            initial_covariance.nrows(),
            4,
            "init-cov must be 4×4 (placeholder)"
        );
        assert_eq!(initial_covariance.ncols(), 4, "init-cov not square");

        Self {
            process_noise,
            initial_value,
            initial_covariance,
        }
    }
}

impl StateBlock for QuaternionBlock {
    fn storage_dim(&self) -> usize {
        4
    }

    fn tangent_dim(&self) -> usize {
        // Placeholder: a rotation's true tangent dimension is 3. Carrying it as
        // 4 keeps this block flat until the error-state form replaces it.
        4
    }

    fn process_noise(&self) -> TangentNoise {
        self.process_noise.clone()
    }
    fn initial_value(&self) -> DVector<f64> {
        self.initial_value.clone()
    }
    fn initial_covariance(&self) -> DMatrix<f64> {
        self.initial_covariance.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::QuaternionBlock;
    use crate::manifold::{StateBlock, TangentNoise};

    use nalgebra::{DMatrix, DVector};

    fn block() -> QuaternionBlock {
        let noise = TangentNoise::from_variances(DVector::from_element(4, 0.01)).unwrap();
        let init = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]); // identity quaternion (x, y, z, w)
        let init_cov = DMatrix::identity(4, 4);
        QuaternionBlock::new(noise, init, init_cov)
    }

    #[test]
    fn placeholder_carries_four_storage_and_four_tangent() {
        // Documents the deliberate placeholder invariant. When the real
        // error-state form lands, tangent_dim becomes 3 and this test changes.
        let b = block();
        assert_eq!(b.storage_dim(), 4);
        assert_eq!(b.tangent_dim(), 4);
    }

    #[test]
    fn euclidean_round_trip_holds_for_placeholder() {
        let b = block();
        let x = DVector::from_vec(vec![0.1, 0.2, 0.3, 0.9]);
        let y = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]);

        let delta = b.ominus(y.rows(0, 4), x.rows(0, 4));
        let recovered = b.oplus(x.rows(0, 4), delta.rows(0, 4));
        assert!((recovered - &y).amax() < 1e-12);
    }

    #[test]
    #[should_panic(expected = "quaternion storage must be 4")]
    fn non_four_storage_panics() {
        let noise = TangentNoise::from_variances(DVector::from_element(3, 0.01)).unwrap();
        let init = DVector::from_element(3, 0.0);
        let init_cov = DMatrix::identity(3, 3);
        QuaternionBlock::new(noise, init, init_cov);
    }
}
