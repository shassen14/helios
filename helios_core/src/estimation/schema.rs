//! [`StateSchema`]: the composed *shape* of an estimator's state — which
//! variables exist, each one's manifold block, and the offset / `Q` / `P`
//! tables baked from them.
//!
//! A schema is assembled once from an ordered list of [`SchemaBlock`]s (via
//! [`StateSchema::compose`]) and is immutable thereafter. It is the shape of the
//! state, never its values: a [`crate::frames::FrameAwareState`] pairs an
//! `Arc<StateSchema>` with the mutable `(mean, covariance)` it describes, so one
//! schema is shared cheaply across every snapshot of a run.
//!
//! Two parallel coordinate spaces run through every method:
//! - **storage** — the stored state vector, one slot per component a block
//!   keeps (a quaternion stores 4). `layout`, `initial_value`, and
//!   `storage_offsets` live here.
//! - **tangent** — the covariance / error space, sized by a block's true
//!   degrees of freedom (a rotation has 3). `initial_covariance`,
//!   `process_noise`, and `tangent_offsets` live here.
//!
//! The two coincide exactly while every block is Euclidean, and diverge the
//! moment a curved block enters.

use crate::{
    frames::{FrameId, StateVariable},
    manifold::{euclidean::EuclideanBlock, StateBlock},
};

use nalgebra::{DMatrix, DVector, DVectorView};
use std::sync::Arc;

/// One block in a composed schema: the manifold [`StateBlock`], the ordered
/// [`StateVariable`]s naming its stored slots, and the sensor frame it is tied
/// to (if any). `variables.len()` must equal `block.storage_dim()`.
#[derive(Debug)]
pub struct SchemaBlock {
    pub block: Arc<dyn StateBlock>,

    pub variables: Vec<StateVariable>,

    pub sensor: Option<FrameId>,
}

/// The immutable shape of a state estimate: its ordered blocks plus the offset,
/// value, and covariance tables baked from them at [`compose`](Self::compose)
/// time. Cheap to share behind an `Arc`; never mutated after construction.
#[derive(Debug)]
pub struct StateSchema {
    /// Live blocks, walked by [`oplus`](Self::oplus) / [`ominus`](Self::ominus).
    blocks: Vec<SchemaBlock>,
    /// Ordered storage-space names; `layout.len() == storage_dim`.
    layout: Vec<StateVariable>,
    storage_dim: usize,
    tangent_dim: usize,
    /// Start index of each block in storage space (parallel to `blocks`).
    storage_offsets: Vec<usize>,
    /// Start index of each block in tangent space (parallel to `blocks`).
    tangent_offsets: Vec<usize>,
    initial_value: DVector<f64>,
    initial_covariance: DMatrix<f64>,
    process_noise: DMatrix<f64>,
}

impl StateSchema {
    /// Composes an ordered list of blocks into one schema, baking the dual
    /// offset tables and the block-diagonal `initial_value`, `initial_covariance`,
    /// and `process_noise`. A block contributing `None` process noise leaves its
    /// diagonal block zero.
    ///
    /// # Panics
    /// If any block's `variables.len()` disagrees with its `storage_dim()` — a
    /// construction-time programming error that would otherwise silently desync
    /// `layout` from the stored vector and corrupt every later name lookup.
    pub fn compose(blocks: Vec<SchemaBlock>) -> Self {
        let storage_dim = blocks
            .iter()
            .map(|schema_block| schema_block.block.storage_dim())
            .sum();
        let tangent_dim = blocks
            .iter()
            .map(|schema_block| schema_block.block.tangent_dim())
            .sum();

        let mut layout = Vec::with_capacity(storage_dim);
        let mut storage_offsets = Vec::with_capacity(blocks.len());
        let mut tangent_offsets = Vec::with_capacity(blocks.len());
        let mut initial_value = DVector::zeros(storage_dim);
        let mut initial_covariance = DMatrix::zeros(tangent_dim, tangent_dim);
        let mut process_noise = DMatrix::zeros(tangent_dim, tangent_dim);

        // Assign each block's start index in both spaces as we walk left to right.
        let (mut s_off, mut t_off) = (0, 0);

        for entry in &blocks {
            let (sd, td) = (entry.block.storage_dim(), entry.block.tangent_dim());
            assert_eq!(
                entry.variables.len(),
                sd,
                "schema block variable count ({}) ≠ storage_dim ({})",
                entry.variables.len(),
                sd
            );

            storage_offsets.push(s_off);
            tangent_offsets.push(t_off);

            // Storage-space contributions: names and the initial mean slice.
            layout.extend(entry.variables.iter().cloned());
            initial_value
                .rows_mut(s_off, sd)
                .copy_from(&entry.block.initial_value());

            // Tangent-space contributions: the block's diagonal P and Q blocks.
            initial_covariance
                .view_mut((t_off, t_off), (td, td))
                .copy_from(&entry.block.initial_covariance());

            if let Some(noise) = entry.block.process_noise() {
                process_noise
                    .view_mut((t_off, t_off), (td, td))
                    .copy_from(noise.covariance());
            }

            s_off += sd;
            t_off += td;
        }

        Self {
            blocks,
            layout,
            storage_dim,
            tangent_dim,
            storage_offsets,
            tangent_offsets,
            initial_value,
            initial_covariance,
            process_noise,
        }
    }

    /// Builds a flat, all-Euclidean schema over `layout` — the staging path for
    /// states that carry no dynamics model (simulation ground truth, mocks,
    /// tests). Every variable is one axis of a single Euclidean block, so
    /// retraction reduces to `x + δ` and there is no process noise.
    ///
    /// Any `Qw` slot is seeded to `1.0` so a freshly built state holds a valid
    /// identity quaternion rather than an all-zero one, which is not a rotation
    /// and propagates `NaN`.
    ///
    /// This is a temporary bridge: it lets call sites that predate schema
    /// ownership keep constructing state from a bare layout. It is retired once
    /// every producer of state carries a real, dynamics-authored schema.
    pub fn degenerate(layout: &[StateVariable]) -> Self {
        let dim = layout.len();

        let mut initial_value = DVector::zeros(dim);
        for (i, var) in layout.iter().enumerate() {
            if matches!(var, StateVariable::Qw(_, _)) {
                initial_value[i] = 1.0;
            }
        }

        let block = EuclideanBlock::without_noise(initial_value, DMatrix::zeros(dim, dim));
        Self::compose(vec![SchemaBlock {
            block: Arc::new(block),
            variables: layout.to_vec(),
            sensor: None,
        }])
    }

    /// Retracts `x` (storage space) by `delta` (tangent space), walking each
    /// block at its own offsets: `out = x ⊞ delta`. Output is storage-sized.
    pub fn oplus(&self, x: DVectorView<f64>, delta: DVectorView<f64>) -> DVector<f64> {
        let mut out = DVector::zeros(self.storage_dim);
        for (i, entry) in self.blocks.iter().enumerate() {
            let (so, to) = (self.storage_offsets[i], self.tangent_offsets[i]);
            let (sd, td) = (entry.block.storage_dim(), entry.block.tangent_dim());
            let moved = entry.block.oplus(x.rows(so, sd), delta.rows(to, td));
            out.rows_mut(so, sd).copy_from(&moved);
        }
        out
    }

    /// Inverse of [`oplus`](Self::oplus): the tangent-space difference `y ⊟ x`
    /// of two storage-space points. Output is tangent-sized.
    pub fn ominus(&self, y: DVectorView<f64>, x: DVectorView<f64>) -> DVector<f64> {
        let mut out = DVector::zeros(self.tangent_dim);
        for (i, entry) in self.blocks.iter().enumerate() {
            let (so, to) = (self.storage_offsets[i], self.tangent_offsets[i]);
            let (sd, td) = (entry.block.storage_dim(), entry.block.tangent_dim());
            let d = entry.block.ominus(y.rows(so, sd), x.rows(so, sd));
            out.rows_mut(to, td).copy_from(&d);
        }
        out
    }

    /// Ordered storage-space layout; `len() == storage_dim()`.
    pub fn layout(&self) -> &[StateVariable] {
        &self.layout
    }

    /// Number of stored components (the `mean` vector's length).
    pub fn storage_dim(&self) -> usize {
        self.storage_dim
    }

    /// Number of degrees of freedom (the covariance's side length).
    pub fn tangent_dim(&self) -> usize {
        self.tangent_dim
    }

    /// Block-diagonal tangent-space process noise `Q`.
    pub fn process_noise(&self) -> &DMatrix<f64> {
        &self.process_noise
    }

    /// Concatenated storage-space initial mean.
    pub fn initial_value(&self) -> &DVector<f64> {
        &self.initial_value
    }

    /// Block-diagonal tangent-space initial covariance `P₀`.
    pub fn initial_covariance(&self) -> &DMatrix<f64> {
        &self.initial_covariance
    }

    /// Storage-space index of `var`, or `None` if absent. Named lookup — never
    /// index the stored vector by a hardcoded position.
    pub fn storage_offset_of(&self, var: &StateVariable) -> Option<usize> {
        self.layout.iter().position(|v| v == var)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::manifold::TangentNoise;

    fn euclid(dim: usize, noise_var: f64, init: f64) -> Arc<dyn StateBlock> {
        Arc::new(EuclideanBlock::new(
            TangentNoise::from_variances(DVector::from_element(dim, noise_var)).unwrap(),
            DVector::from_element(dim, init),
            DMatrix::identity(dim, dim),
        ))
    }

    fn pos_vel_schema() -> StateSchema {
        StateSchema::compose(vec![
            SchemaBlock {
                block: euclid(3, 0.1, 0.0),
                variables: vec![
                    StateVariable::Px(FrameId::World),
                    StateVariable::Py(FrameId::World),
                    StateVariable::Pz(FrameId::World),
                ],
                sensor: None,
            },
            SchemaBlock {
                block: euclid(2, 0.5, 0.0),
                variables: vec![
                    StateVariable::Vx(FrameId::World),
                    StateVariable::Vy(FrameId::World),
                ],
                sensor: None,
            },
        ])
    }

    #[test]
    fn dims_and_layout_agree() {
        let s = pos_vel_schema();
        assert_eq!(s.storage_dim(), 5);
        assert_eq!(s.tangent_dim(), 5);
        assert_eq!(s.layout().len(), s.storage_dim());
    }

    #[test]
    fn offsets_are_monotonic_and_equal_while_euclidean() {
        let s = pos_vel_schema();
        assert_eq!(s.storage_offsets, vec![0, 3]);
        assert_eq!(s.tangent_offsets, s.storage_offsets);
    }

    #[test]
    fn process_noise_is_block_diagonal() {
        let s = pos_vel_schema();
        let q = s.process_noise();
        // Position block variance 0.1, velocity block 0.5, no cross terms.
        assert_eq!(q[(0, 0)], 0.1);
        assert_eq!(q[(3, 3)], 0.5);
        assert_eq!(q[(0, 3)], 0.0);
        assert_eq!(q[(3, 0)], 0.0);
    }

    #[test]
    fn oplus_ominus_round_trip() {
        // The trait law lifted to the whole schema: x ⊞ (y ⊟ x) == y.
        let s = pos_vel_schema();
        let x = DVector::from_vec(vec![1.0, 2.0, 3.0, 4.0, 5.0]);
        let y = DVector::from_vec(vec![-1.0, 0.5, 9.0, 2.0, -2.0]);

        let delta = s.ominus(y.as_view(), x.as_view());
        let recovered = s.oplus(x.as_view(), delta.as_view());
        assert!((recovered - &y).amax() < 1e-12);
    }

    #[test]
    fn storage_offset_of_finds_names() {
        let s = pos_vel_schema();
        assert_eq!(
            s.storage_offset_of(&StateVariable::Px(FrameId::World)),
            Some(0)
        );
        assert_eq!(
            s.storage_offset_of(&StateVariable::Vx(FrameId::World)),
            Some(3)
        );
        assert_eq!(
            s.storage_offset_of(&StateVariable::Az(FrameId::World)),
            None
        );
    }

    #[test]
    fn degenerate_seeds_identity_quaternion_and_no_noise() {
        let layout = vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Qx(FrameId::World, FrameId::World),
            StateVariable::Qy(FrameId::World, FrameId::World),
            StateVariable::Qz(FrameId::World, FrameId::World),
            StateVariable::Qw(FrameId::World, FrameId::World),
        ];
        let s = StateSchema::degenerate(&layout);

        assert_eq!(s.storage_dim(), 5);
        // Qw is the last slot, seeded to 1.0; everything else 0.0.
        assert_eq!(s.initial_value()[4], 1.0);
        assert_eq!(s.initial_value()[0], 0.0);
        // No block contributed process noise.
        assert_eq!(s.process_noise(), &DMatrix::zeros(5, 5));
    }

    #[test]
    fn degenerate_oplus_is_addition() {
        let layout = vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Py(FrameId::World),
        ];
        let s = StateSchema::degenerate(&layout);
        let x = DVector::from_vec(vec![1.0, 2.0]);
        let d = DVector::from_vec(vec![0.5, -0.5]);

        assert_eq!(
            s.oplus(x.as_view(), d.as_view()),
            DVector::from_vec(vec![1.5, 1.5])
        );
    }

    #[test]
    #[should_panic(expected = "storage_dim")]
    fn compose_rejects_variable_count_mismatch() {
        // Block stores three components but only two variables are named.
        StateSchema::compose(vec![SchemaBlock {
            block: euclid(3, 0.1, 0.0),
            variables: vec![
                StateVariable::Px(FrameId::World),
                StateVariable::Py(FrameId::World),
            ],
            sensor: None,
        }]);
    }
}
