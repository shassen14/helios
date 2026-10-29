//! [`QuaternionBlock`]: orientation as a proper `SO(3)` error-state block —
//! **4 stored components, 3 tangent degrees of freedom**.
//!
//! A unit quaternion stores 4 numbers but a rotation has only 3 real degrees of
//! freedom, so the covariance, process noise, and error slice all live in a
//! 3-dimensional tangent space while the mean keeps all 4 stored components. The
//! retraction is the group operation, not componentwise addition:
//!
//! - `oplus(q, δ) = q ⊗ exp(δ)` — rotate `q` by the small rotation `exp(δ)`.
//! - `ominus(y, x) = log(x⁻¹ ⊗ y)` — the tangent rotation that carries `x` to `y`.
//!
//! The maths, the body-side convention and the `[x, y, z, w]` storage order
//! live in [`rotation`], shared with every other stored orientation, so the
//! estimator's attitude error and any other attitude error mean the same
//! thing. This block only moves its state slices in and out of it.

use super::{rotation, StateBlock, TangentNoise};

use nalgebra::{DMatrix, DVector, DVectorView, Vector3};

#[derive(Debug)]
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
        assert_eq!(
            initial_value.len(),
            rotation::STORAGE_DIM,
            "quaternion storage must be 4"
        );
        assert_eq!(
            process_noise.covariance().nrows(),
            rotation::TANGENT_DIM,
            "noise must be 3×3"
        );
        assert_eq!(
            initial_covariance.nrows(),
            rotation::TANGENT_DIM,
            "init-cov must be 3×3"
        );
        assert_eq!(
            initial_covariance.ncols(),
            rotation::TANGENT_DIM,
            "init-cov not square"
        );

        Self {
            process_noise,
            initial_value,
            initial_covariance,
        }
    }
}

impl StateBlock for QuaternionBlock {
    fn storage_dim(&self) -> usize {
        rotation::STORAGE_DIM
    }

    fn tangent_dim(&self) -> usize {
        rotation::TANGENT_DIM
    }

    fn is_euclidean(&self) -> bool {
        false
    }

    fn oplus(&self, x: DVectorView<f64>, delta: DVectorView<f64>) -> DVector<f64> {
        let q = rotation::from_storage(x.as_slice());
        let delta_vec = Vector3::new(delta[0], delta[1], delta[2]);

        let retracted = rotation::oplus(&q, &delta_vec);

        DVector::from_column_slice(&rotation::to_storage(&retracted))
    }

    fn ominus(&self, y: DVectorView<f64>, x: DVectorView<f64>) -> DVector<f64> {
        let q_y = rotation::from_storage(y.as_slice());
        let q_x = rotation::from_storage(x.as_slice());

        let delta = rotation::ominus(&q_y, &q_x);
        DVector::from_column_slice(delta.as_slice())
    }

    fn process_noise(&self) -> Option<TangentNoise> {
        Some(self.process_noise.clone())
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
    use crate::kernel::manifold::{StateBlock, TangentNoise};

    use nalgebra::{DMatrix, DVector, Quaternion, UnitQuaternion, Vector3};
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    /// A well-formed 4/3 block. Noise and P₀ are 3×3 (tangent), the mean is the
    /// 4-component identity quaternion.
    fn block() -> QuaternionBlock {
        let noise = TangentNoise::from_variances(DVector::from_element(3, 0.01)).unwrap();
        let init = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]); // identity quaternion (x, y, z, w)
        let init_cov = DMatrix::identity(3, 3);
        QuaternionBlock::new(noise, init, init_cov)
    }

    /// A random rotation. The axis·angle components span roughly ±3 rad, so the
    /// samples cross π and exercise the quaternion double-cover of `SO(3)`.
    fn rand_rotation(rng: &mut StdRng) -> UnitQuaternion<f64> {
        UnitQuaternion::from_scaled_axis(Vector3::new(
            rng.gen_range(-3.0..3.0),
            rng.gen_range(-3.0..3.0),
            rng.gen_range(-3.0..3.0),
        ))
    }

    /// A rotation packed into the block's `[x, y, z, w]` storage order.
    fn storage(q: &UnitQuaternion<f64>) -> DVector<f64> {
        DVector::from_column_slice(q.coords.as_slice())
    }

    /// The rotation a storage vector encodes (scalar-last → scalar-first).
    fn rotation(v: &DVector<f64>) -> UnitQuaternion<f64> {
        UnitQuaternion::from_quaternion(Quaternion::new(v[3], v[0], v[1], v[2]))
    }

    #[test]
    fn storage_is_four_and_tangent_is_three() {
        let b = block();
        assert_eq!(b.storage_dim(), 4);
        assert_eq!(b.tangent_dim(), 3);
    }

    #[test]
    fn oplus_by_zero_delta_returns_the_point_unchanged() {
        let b = block();
        let q = storage(&rand_rotation(&mut StdRng::seed_from_u64(7)));

        let out = b.oplus(q.rows(0, 4), DVector::zeros(3).rows(0, 3));

        // exp(0) is the identity rotation, so q ⊞ 0 = q exactly (q is already unit).
        assert!((out - &q).amax() < 1e-12);
    }

    #[test]
    fn ominus_of_a_point_with_itself_is_zero() {
        let b = block();
        let q = storage(&rand_rotation(&mut StdRng::seed_from_u64(11)));

        let delta = b.ominus(q.rows(0, 4), q.rows(0, 4));

        assert_eq!(delta.len(), 3, "ominus is tangent-sized");
        assert!(delta.amax() < 1e-12);
    }

    #[test]
    fn oplus_applies_a_known_body_frame_rotation() {
        // A π/2 yaw about +Z applied to identity is [x, y, z, w] =
        // [0, 0, sin(π/4), cos(π/4)] — NOT the componentwise sum [0, 0, π/2, 1] a
        // Euclidean block would give. Pins the convention and proves the
        // retraction is genuinely non-linear (the distinguishing check).
        use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2};
        let b = block();
        let identity = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]);
        let yaw = DVector::from_vec(vec![0.0, 0.0, FRAC_PI_2]);

        let out = b.oplus(identity.rows(0, 4), yaw.rows(0, 3));

        assert!(out[0].abs() < 1e-12);
        assert!(out[1].abs() < 1e-12);
        assert!((out[2] - FRAC_1_SQRT_2).abs() < 1e-12);
        assert!((out[3] - FRAC_1_SQRT_2).abs() < 1e-12);
    }

    #[test]
    fn oplus_ominus_round_trip_over_random_rotations() {
        // The trait law x ⊞ (y ⊟ x) == y, lifted to SO(3). Compared at the
        // rotation level (angle_to) rather than on raw components: a unit
        // quaternion double-covers SO(3), so the reconstruction may carry the
        // equivalent −y sign while naming the identical rotation.
        let b = block();
        let mut rng = StdRng::seed_from_u64(0x0405_0607);

        for _ in 0..1000 {
            let x = storage(&rand_rotation(&mut rng));
            let y = storage(&rand_rotation(&mut rng));

            let delta = b.ominus(y.rows(0, 4), x.rows(0, 4));
            let recovered = b.oplus(x.rows(0, 4), delta.rows(0, 3));

            let residual = rotation(&recovered).angle_to(&rotation(&y));
            assert!(residual < 1e-9, "round trip lost the rotation: {residual}");
        }
    }

    #[test]
    fn ominus_then_oplus_recovers_the_tangent_delta() {
        // exp then log is the identity on the tangent: for |δ| < π,
        // ominus(oplus(q, δ), q) = δ exactly, with no double-cover ambiguity in
        // this range. The sharp check that separates a real SO(3) retraction from
        // the old Euclidean stand-in, which could never recover a curved δ.
        let b = block();
        let mut rng = StdRng::seed_from_u64(0x0102_0304);

        for _ in 0..1000 {
            let q = storage(&rand_rotation(&mut rng));
            // |δ| ≤ √(3·0.5²) ≈ 0.87 < π, so log returns the vector exp consumed.
            let delta = DVector::from_vec(vec![
                rng.gen_range(-0.5..0.5),
                rng.gen_range(-0.5..0.5),
                rng.gen_range(-0.5..0.5),
            ]);

            let moved = b.oplus(q.rows(0, 4), delta.rows(0, 3));
            let recovered = b.ominus(moved.rows(0, 4), q.rows(0, 4));

            assert!((recovered - &delta).amax() < 1e-9);
        }
    }

    #[test]
    #[should_panic(expected = "quaternion storage must be 4")]
    fn non_four_storage_panics() {
        let noise = TangentNoise::from_variances(DVector::from_element(3, 0.01)).unwrap();
        let init = DVector::from_element(3, 0.0);
        let init_cov = DMatrix::identity(3, 3);
        QuaternionBlock::new(noise, init, init_cov);
    }

    #[test]
    #[should_panic(expected = "noise must be 3×3")]
    fn non_three_tangent_noise_panics() {
        // A 4-wide noise is the old placeholder shape; the tangent is 3 now.
        let noise = TangentNoise::from_variances(DVector::from_element(4, 0.01)).unwrap();
        let init = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]);
        let init_cov = DMatrix::identity(3, 3);
        QuaternionBlock::new(noise, init, init_cov);
    }
}
