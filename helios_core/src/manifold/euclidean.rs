//! [`EuclideanBlock`]: a flat ℝⁿ state block (position, velocity, biases,
//! clock). Storage and tangent dimensions are equal, so it inherits the
//! trait's default `+` / `-` retraction unchanged — the degenerate case the
//! whole block abstraction generalizes away from.

use super::{StateBlock, TangentNoise};

use nalgebra::{DMatrix, DVector};

pub struct EuclideanBlock {
    dim: usize,
    process_noise: TangentNoise,
    initial_value: DVector<f64>,
    initial_covariance: DMatrix<f64>,
}

impl EuclideanBlock {
    pub fn new(
        process_noise: TangentNoise,
        initial_value: DVector<f64>,
        initial_covariance: DMatrix<f64>,
    ) -> Self {
        let dim = initial_value.len();
        assert_eq!(
            process_noise.covariance().nrows(),
            dim,
            "noise dim ≠ state dim"
        );
        assert_eq!(initial_covariance.nrows(), dim, "init-cov dim ≠ state dim");
        assert_eq!(initial_covariance.ncols(), dim, "init-cov not square");

        Self {
            dim,
            process_noise,
            initial_value,
            initial_covariance,
        }
    }
}

impl StateBlock for EuclideanBlock {
    fn storage_dim(&self) -> usize {
        self.dim
    }

    fn tangent_dim(&self) -> usize {
        self.dim
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
    use super::EuclideanBlock;
    use crate::manifold::{StateBlock, TangentNoise};

    use nalgebra::{DMatrix, DVector};

    fn block(dim: usize) -> EuclideanBlock {
        let noise = TangentNoise::from_variances(DVector::from_element(dim, 0.1)).unwrap();
        let init = DVector::from_element(dim, 0.0);
        let init_cov = DMatrix::identity(dim, dim);
        EuclideanBlock::new(noise, init, init_cov)
    }

    #[test]
    fn storage_and_tangent_dims_are_equal() {
        let b = block(3);
        assert_eq!(b.storage_dim(), 3);
        assert_eq!(b.tangent_dim(), 3);
    }

    #[test]
    fn oplus_reduces_to_addition() {
        let b = block(3);
        let x = DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let d = DVector::from_vec(vec![0.5, -0.5, 1.0]);

        let out = b.oplus(x.rows(0, 3), d.rows(0, 3));
        assert_eq!(out, &x + &d);
    }

    #[test]
    fn ominus_reduces_to_subtraction() {
        let b = block(3);
        let y = DVector::from_vec(vec![4.0, 4.0, 4.0]);
        let x = DVector::from_vec(vec![1.0, 2.0, 3.0]);

        let out = b.ominus(y.rows(0, 3), x.rows(0, 3));
        assert_eq!(out, &y - &x);
    }

    #[test]
    fn oplus_ominus_round_trip() {
        // The trait law: x ⊞ (y ⊟ x) == y.
        let b = block(3);
        let x = DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let y = DVector::from_vec(vec![-1.0, 5.0, 0.25]);

        let delta = b.ominus(y.rows(0, 3), x.rows(0, 3));
        let recovered = b.oplus(x.rows(0, 3), delta.rows(0, 3));
        assert!((recovered - &y).amax() < 1e-12);
    }

    #[test]
    fn init_value_and_covariance_pass_through() {
        let noise = TangentNoise::from_variances(DVector::from_element(2, 1.0)).unwrap();
        let init = DVector::from_vec(vec![7.0, 8.0]);
        let init_cov = DMatrix::from_row_slice(2, 2, &[2.0, 0.0, 0.0, 3.0]);
        let b = EuclideanBlock::new(noise, init.clone(), init_cov.clone());

        assert_eq!(b.initial_value(), init);
        assert_eq!(b.initial_covariance(), init_cov);
    }

    #[test]
    #[should_panic(expected = "init-cov dim")]
    fn mismatched_init_cov_dim_panics() {
        let noise = TangentNoise::from_variances(DVector::from_element(3, 1.0)).unwrap();
        let init = DVector::from_element(3, 0.0);
        let wrong = DMatrix::identity(2, 2);
        EuclideanBlock::new(noise, init, wrong);
    }
}
