//! Estimator consistency against truth: the error of an estimate's blocks
//! from their true values, and its normalized squared size (NEES) under the
//! estimate's own covariance.
//!
//! The error of each block is `truth ⊟ estimate` by the block's own retraction,
//! so it lies in the tangent the covariance is kept in, never re-derived. NEES
//! is `δᵀ P⁻¹ δ` with `P` the marginal covariance of the blocks given (their
//! rows and columns of the state's covariance; a Gaussian's marginal is exact).
//! For a consistent filter it is chi-squared with `dof` degrees of freedom, so
//! it averages near `dof` over many samples.

use crate::spatial::{state::Quantity, FrameAwareState};

use nalgebra::{Cholesky, DMatrix, DVector, Dyn, UnitQuaternion, Vector3};
use std::error::Error;
use std::fmt::{self, Display};

/// The error of each block given a truth, and the NEES of every one of them
/// jointly.
#[derive(Debug, Clone)]
pub struct Consistency {
    /// In the order the truths were given.
    blocks: Vec<BlockConsistency>,
    joint: Result<Nees, NotPositiveDefinite>,
}

impl Consistency {
    /// Each block given a truth, in the order given.
    pub fn blocks(&self) -> &[BlockConsistency] {
        &self.blocks
    }

    /// The NEES over every block given, under their joint marginal, which
    /// counts their correlations; or why it has none.
    pub fn joint(&self) -> Result<Nees, NotPositiveDefinite> {
        self.joint
    }
}

/// One block's error from its truth and the NEES of that error under the
/// block's own marginal covariance.
#[derive(Debug, Clone)]
pub struct BlockConsistency {
    quantity: Quantity,
    error: DVector<f64>,
    nees: Result<Nees, NotPositiveDefinite>,
}

impl BlockConsistency {
    pub fn quantity(&self) -> &Quantity {
        &self.quantity
    }

    /// `truth ⊟ estimate` in the block's tangent: the correction that carries
    /// the estimate to the truth.
    pub fn error(&self) -> &DVector<f64> {
        &self.error
    }

    /// The error's NEES under the block's marginal, or why it has none.
    pub fn nees(&self) -> Result<Nees, NotPositiveDefinite> {
        self.nees
    }
}

/// A normalized estimation error squared and its degrees of freedom: the
/// tangent dimension of the blocks it covers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Nees {
    pub nees: f64,
    pub dof: usize,
}

/// A marginal covariance with no Cholesky factor: not finite, or not
/// positive definite (a block held with zero covariance, say), so the error
/// has no normalized size.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NotPositiveDefinite;

impl Display for NotPositiveDefinite {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("the marginal covariance is not finite and positive definite")
    }
}

impl Error for NotPositiveDefinite {}

/// The true value of one state block, in the block's storage space: three
/// numbers for a flat quantity, a unit quaternion's four for an orientation.
#[derive(Debug, Clone)]
pub struct BlockTruth {
    quantity: Quantity,
    value: DVector<f64>,
}

impl BlockTruth {
    /// `quantity`'s true value, `value` in its block's storage space.
    pub fn new(quantity: Quantity, value: DVector<f64>) -> Self {
        Self { quantity, value }
    }

    /// The true value of a flat (three-component) quantity.
    pub fn vector3(quantity: Quantity, value: Vector3<f64>) -> Self {
        Self::new(quantity, DVector::from_column_slice(value.as_slice()))
    }

    /// The true `quantity` orientation, stored as the orientation block stores
    /// it: `[x, y, z, w]`, scalar last.
    pub fn orientation(quantity: Quantity, rotation: UnitQuaternion<f64>) -> Self {
        // `.coords` is [i, j, k, w], already in storage order.
        Self::new(
            quantity,
            DVector::from_column_slice(rotation.coords.as_slice()),
        )
    }

    pub fn quantity(&self) -> &Quantity {
        &self.quantity
    }
}

/// Why truths can't be compared with an estimate at all. Each names the block.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum ConsistencyError {
    /// The estimate's schema holds no block for this quantity.
    Absent(Quantity),
    /// A truth whose value is not its block's storage length.
    StorageLength {
        quantity: Quantity,
        expected: usize,
        given: usize,
    },
    /// Two truths for one block, whose rows the marginal would hold twice.
    Repeated(Quantity),
}

impl Display for ConsistencyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConsistencyError::Absent(quantity) => {
                write!(f, "the estimate holds no block for {quantity}")
            }
            ConsistencyError::StorageLength {
                quantity,
                expected,
                given,
            } => write!(
                f,
                "the truth of {quantity} has {given} numbers; its block stores {expected}"
            ),
            ConsistencyError::Repeated(quantity) => {
                write!(f, "{quantity} is given two truths")
            }
        }
    }
}

impl Error for ConsistencyError {}

/// How far `estimate` is from `truths`: each block's tangent error and NEES,
/// and the NEES of every block given jointly. Only the given blocks' marginal
/// is factored, so a large state costs no more than the blocks graded. A
/// marginal that can't be factored refuses that NEES alone; the errors are
/// still given.
pub fn consistency(
    estimate: &FrameAwareState,
    truths: &[BlockTruth],
) -> Result<Consistency, ConsistencyError> {
    let schema = estimate.schema();
    let mut blocks = Vec::with_capacity(truths.len());
    let mut indices = Vec::new();
    for (i, truth) in truths.iter().enumerate() {
        let quantity = &truth.quantity;
        if truths[..i]
            .iter()
            .any(|earlier| &earlier.quantity == quantity)
        {
            return Err(ConsistencyError::Repeated(quantity.clone()));
        }
        let (Some(block), Some(storage), Some(tangent)) = (
            schema.block_of(quantity),
            schema.storage_offset_of_block(quantity),
            schema.tangent_offset_of_block(quantity),
        ) else {
            return Err(ConsistencyError::Absent(quantity.clone()));
        };
        if truth.value.len() != block.storage_dim() {
            return Err(ConsistencyError::StorageLength {
                quantity: quantity.clone(),
                expected: block.storage_dim(),
                given: truth.value.len(),
            });
        }
        let error = block.ominus(
            truth.value.as_view(),
            estimate.mean.rows(storage, block.storage_dim()),
        );
        let rows: Vec<usize> = (tangent..tangent + block.tangent_dim()).collect();
        let nees = normalized(&marginal(&estimate.covariance, &rows), &error);
        indices.extend(rows);
        blocks.push(BlockConsistency {
            quantity: quantity.clone(),
            error,
            nees,
        });
    }
    let joint_error = DVector::from_iterator(
        indices.len(),
        blocks.iter().flat_map(|block| block.error.iter().copied()),
    );
    let joint = normalized(&marginal(&estimate.covariance, &indices), &joint_error);
    Ok(Consistency { blocks, joint })
}

/// The rows and columns `indices` of `covariance`, in that order.
fn marginal(covariance: &DMatrix<f64>, indices: &[usize]) -> DMatrix<f64> {
    DMatrix::from_fn(indices.len(), indices.len(), |r, c| {
        covariance[(indices[r], indices[c])]
    })
}

/// `error`'s NEES under `marginal`, or why it has none. Finiteness is checked
/// first: a Cholesky factor can exist for a non-finite matrix, a Gaussian's
/// covariance can't.
fn normalized(marginal: &DMatrix<f64>, error: &DVector<f64>) -> Result<Nees, NotPositiveDefinite> {
    if !marginal.iter().all(|entry| entry.is_finite()) {
        return Err(NotPositiveDefinite);
    }
    let factor = marginal.clone().cholesky().ok_or(NotPositiveDefinite)?;
    Ok(Nees {
        nees: normalized_squared(&factor, error),
        dof: error.nrows(),
    })
}

/// `yᵀ S⁻¹ y` through `S`'s Cholesky factor, never forming `S⁻¹`. The one
/// solve behind both NEES and a filter's NIS, so the two read one number for
/// one `y` and `S`.
pub(crate) fn normalized_squared(factor: &Cholesky<f64, Dyn>, y: &DVector<f64>) -> f64 {
    y.dot(&factor.solve(y))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::estimation::schema::{StateSchema, StateSchemaBlock};
    use crate::kernel::manifold::TangentNoise;
    use crate::prelude::{AgentId, MonotonicTime};
    use crate::spatial::{transforms::Convention, FrameId};

    use rand::rngs::StdRng;
    use rand::SeedableRng;
    use std::sync::Arc;

    const TOL: f64 = 1e-12;

    fn agent() -> AgentId {
        AgentId::new("car")
    }

    fn position() -> Quantity {
        Quantity::Position(FrameId::odom(agent()))
    }

    fn velocity() -> Quantity {
        Quantity::Velocity(FrameId::odom(agent()))
    }

    fn orientation() -> Quantity {
        Quantity::Orientation {
            from: FrameId::base_link(agent()),
            to: FrameId::odom(agent()),
        }
    }

    fn noise() -> Option<TangentNoise> {
        TangentNoise::from_variances(DVector::from_element(3, 0.01))
    }

    fn flat(quantity: Quantity) -> StateSchemaBlock {
        StateSchemaBlock::new(
            quantity,
            Convention::Enu,
            noise(),
            DVector::zeros(3),
            DMatrix::identity(3, 3),
        )
    }

    fn attitude() -> StateSchemaBlock {
        StateSchemaBlock::orientation(
            FrameId::base_link(agent()),
            FrameId::odom(agent()),
            Convention::Flu,
            Convention::Enu,
            noise(),
            DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
            DMatrix::identity(3, 3),
        )
    }

    /// A state of `blocks` with `mean` and `covariance`.
    fn state(
        blocks: Vec<StateSchemaBlock>,
        mean: DVector<f64>,
        covariance: DMatrix<f64>,
    ) -> FrameAwareState {
        let mut state = FrameAwareState::from_schema(
            Arc::new(StateSchema::compose(blocks)),
            MonotonicTime(0.0),
        );
        state.mean = mean;
        state.covariance = covariance;
        state
    }

    fn vector(values: &[f64]) -> DVector<f64> {
        DVector::from_column_slice(values)
    }

    #[test]
    fn a_flat_blocks_nees_is_its_error_against_its_covariance_by_hand() {
        let covariance =
            DMatrix::from_row_slice(3, 3, &[4.0, 1.0, 0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 1.0]);
        let estimate = state(vec![flat(position())], vector(&[1.0, 2.0, 3.0]), covariance);
        let truth = BlockTruth::new(position(), vector(&[2.0, 2.0, 5.0]));

        let result = consistency(&estimate, &[truth]).expect("comparable");

        // δ = (1, 0, 2). The xy corner [[4, 1], [1, 2]] inverts to
        // [[2, -1], [-1, 4]] / 7, giving 2/7; z gives 2² / 1 = 4.
        let block = &result.blocks()[0];
        assert!((block.error() - vector(&[1.0, 0.0, 2.0])).amax() < TOL);
        let nees = block.nees().expect("positive definite");
        assert!((nees.nees - 30.0 / 7.0).abs() < TOL, "{nees:?}");
        assert_eq!(nees.dof, 3);
        assert_eq!(result.joint(), block.nees());
    }

    #[test]
    fn a_truth_retracted_from_the_estimate_gives_back_the_tangent_step() {
        let mean = vector(&[0.1, -0.2, 0.3, 0.927_361_849_549_570_4]);
        let mean = &mean / mean.norm();
        let estimate = state(vec![attitude()], mean.clone(), DMatrix::identity(3, 3));
        let step = vector(&[0.1, -0.2, 0.05]);
        let block = estimate
            .schema()
            .block_of(&orientation())
            .expect("held")
            .clone();
        let truth = BlockTruth::new(orientation(), block.oplus(mean.as_view(), step.as_view()));

        let result = consistency(&estimate, &[truth]).expect("comparable");

        assert!((result.blocks()[0].error() - &step).amax() < 1e-9);
    }

    #[test]
    fn correlated_blocks_joint_nees_differs_from_the_sum_of_their_own() {
        let mut covariance = DMatrix::identity(6, 6);
        for axis in 0..3 {
            covariance[(axis, axis + 3)] = 0.8;
            covariance[(axis + 3, axis)] = 0.8;
        }
        let estimate = state(
            vec![flat(position()), flat(velocity())],
            DVector::zeros(6),
            covariance,
        );
        let truths = [
            BlockTruth::new(position(), vector(&[1.0, 0.0, 0.0])),
            BlockTruth::new(velocity(), vector(&[-1.0, 0.0, 0.0])),
        ];

        let result = consistency(&estimate, &truths).expect("comparable");

        let own: f64 = result
            .blocks()
            .iter()
            .map(|block| block.nees().expect("positive definite").nees)
            .sum();
        let joint = result.joint().expect("positive definite");
        // Each alone: 1. Jointly, errors against their correlation:
        // (1, -1) against [[1, 0.8], [0.8, 1]] gives 2 / 0.2 = 10.
        assert!((own - 2.0).abs() < TOL);
        assert!((joint.nees - 10.0).abs() < 1e-9, "{joint:?}");
        assert_eq!(joint.dof, 6);
    }

    #[test]
    fn a_zero_covariance_block_has_no_nees_but_keeps_its_error() {
        let mut covariance = DMatrix::identity(6, 6);
        covariance.view_mut((0, 0), (3, 3)).fill(0.0);
        let estimate = state(
            vec![flat(position()), attitude()],
            vector(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            covariance,
        );
        let truths = [
            BlockTruth::new(position(), vector(&[1.0, 0.0, 0.0])),
            BlockTruth::orientation(orientation(), UnitQuaternion::identity()),
        ];

        let result = consistency(&estimate, &truths).expect("comparable");

        assert_eq!(result.blocks()[0].nees(), Err(NotPositiveDefinite));
        assert!((result.blocks()[0].error() - vector(&[1.0, 0.0, 0.0])).amax() < TOL);
        assert!(result.blocks()[1].nees().is_ok());
        assert_eq!(result.joint(), Err(NotPositiveDefinite));
    }

    #[test]
    fn truths_drawn_from_the_estimates_covariance_average_near_the_dof() {
        let mut covariance = DMatrix::from_diagonal(&vector(&[0.4, 0.2, 0.1, 0.03, 0.02, 0.05]));
        covariance[(0, 3)] = 0.05;
        covariance[(3, 0)] = 0.05;
        let mean = vector(&[
            1.0,
            -2.0,
            0.5,
            0.0,
            0.0,
            0.382_683_432_365_089_8,
            0.923_879_532_511_286_7,
        ]);
        let estimate = state(
            vec![flat(position()), attitude()],
            mean.clone(),
            covariance.clone(),
        );
        let draw = TangentNoise::from_covariance(covariance).expect("positive definite");
        let mut rng = StdRng::seed_from_u64(0x5EED);
        let samples = 20_000;

        let mut total = 0.0;
        for _ in 0..samples {
            let truth = estimate
                .schema()
                .oplus(mean.as_view(), draw.sample(&mut rng).as_view());
            let truths = [
                BlockTruth::new(position(), truth.rows(0, 3).into_owned()),
                BlockTruth::new(orientation(), truth.rows(3, 4).into_owned()),
            ];
            let result = consistency(&estimate, &truths).expect("comparable");
            total += result.joint().expect("positive definite").nees;
        }

        // χ²(6) has mean 6 and variance 12, so the mean of 20 000 has a
        // standard deviation of about 0.025.
        let mean_nees = total / samples as f64;
        assert!((mean_nees - 6.0).abs() < 0.1, "{mean_nees}");
    }

    #[test]
    fn an_absent_block_a_misshapen_truth_and_a_repeated_one_are_refused_naming_it() {
        let estimate = state(
            vec![flat(position())],
            DVector::zeros(3),
            DMatrix::identity(3, 3),
        );

        assert_eq!(
            consistency(&estimate, &[BlockTruth::new(velocity(), DVector::zeros(3))]).err(),
            Some(ConsistencyError::Absent(velocity()))
        );
        assert_eq!(
            consistency(&estimate, &[BlockTruth::new(position(), DVector::zeros(4))]).err(),
            Some(ConsistencyError::StorageLength {
                quantity: position(),
                expected: 3,
                given: 4,
            })
        );
        let twice = BlockTruth::new(position(), DVector::zeros(3));
        assert_eq!(
            consistency(&estimate, &[twice.clone(), twice]).err(),
            Some(ConsistencyError::Repeated(position()))
        );
    }

    #[test]
    fn an_orientation_truth_is_stored_scalar_last() {
        let rotation = UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);

        let truth = BlockTruth::orientation(orientation(), rotation);

        let q = rotation.quaternion();
        assert_eq!(truth.value, vector(&[q.i, q.j, q.k, q.w]));
    }
}
