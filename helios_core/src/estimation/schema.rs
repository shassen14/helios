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
#[derive(Debug, Clone)]
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

    /// Returns a new schema with `extra` blocks appended after this schema's own,
    /// re-baking every offset / `P₀` / `Q` table. The base schema is left
    /// unchanged; block clones are shallow (each is an `Arc` bump).
    ///
    /// The augmentation path: a base state (pose, velocity) gains per-device
    /// nuisance blocks (sensor biases) without the base's dynamics knowing they
    /// exist. Placement is inherited from [`compose`](Self::compose) — the new
    /// blocks land in a fresh diagonal corner past the base, and the base's rows
    /// and columns come out byte-identical — so this method never re-derives the
    /// baking math, it re-runs it over the longer block list.
    pub fn extended(&self, extra: Vec<SchemaBlock>) -> StateSchema {
        let mut combined = self.blocks.clone();
        combined.extend(extra);
        StateSchema::compose(combined)
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

    /// The start index of `var` in **tangent** (error / covariance) space.
    ///
    /// Distinct from [`storage_offset_of`](Self::storage_offset_of): a block may
    /// store more components than it has tangent dimensions (a quaternion stores
    /// 4, moves on a 3-D tangent), so the two offsets diverge for every variable
    /// past the first such block. Index a covariance or a tangent-sized error
    /// vector through here — never through the storage offset.
    ///
    /// `None` if `var` is absent, or if it names a stored component with no
    /// tangent coordinate of its own — the quaternion's scalar `Qw`, which is
    /// carried by the mean but not parameterised in the error state.
    pub fn tangent_offset_of(&self, var: &StateVariable) -> Option<usize> {
        self.blocks.iter().enumerate().find_map(|(i, block)| {
            // `?` bails out of *this* block when `var` isn't one of its names,
            // letting `find_map` move on; a name is unique across the schema, so
            // at most one block ever matches.
            let j = block.variables.iter().position(|v| v == var)?;
            // Within a block, storage index `j` is the tangent index too — except
            // where storage outruns the tangent (Qw sits at j = 3 of a 3-D
            // tangent). There, refuse rather than return a wrong-but-plausible
            // slot: a silent off-by-one in a covariance index is the exact bug
            // the storage/tangent split exists to prevent.
            (j < block.block.tangent_dim()).then_some(self.tangent_offsets[i] + j)
        })
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

    // Augmentation prior std-dev and random-walk for the extended-schema tests.
    // Kept distinct from the base blocks' 0.1 / 0.5 so the new diagonal corner is
    // unmistakably the augmentation's and not a base value bleeding across.
    const AUG_INIT_UNCERTAINTY: f64 = 0.25;
    const AUG_RANDOM_WALK: f64 = 0.02;

    fn mag_sensor() -> FrameId {
        FrameId::Sensor(crate::data::primitives::FrameHandle(3))
    }

    fn mag_bias_block() -> SchemaBlock {
        crate::estimation::augmentation::augmentation_block(
            crate::estimation::augmentation::MAGNETOMETER_BIAS,
            mag_sensor(),
            AUG_INIT_UNCERTAINTY,
            AUG_RANDOM_WALK,
        )
        .expect("well-formed magnetometer-bias block builds")
    }

    #[test]
    fn extended_appends_block_and_grows_both_dims_by_its_size() {
        let base = pos_vel_schema();
        let base_storage = base.storage_dim();
        let base_tangent = base.tangent_dim();

        let extended = base.extended(vec![mag_bias_block()]);

        // A 3-DOF Euclidean block grows storage and tangent equally.
        assert_eq!(extended.storage_dim(), base_storage + 3);
        assert_eq!(extended.tangent_dim(), base_tangent + 3);
    }

    #[test]
    fn extended_layout_ends_with_the_augmentation_variables() {
        let base = pos_vel_schema();
        let base_storage = base.storage_dim();

        let extended = base.extended(vec![mag_bias_block()]);

        // The base names are untouched at the front …
        assert_eq!(&extended.layout()[..base_storage], base.layout());
        // … and the three bias vars, tagged with the sensor, are appended.
        assert_eq!(
            &extended.layout()[base_storage..],
            &[
                StateVariable::MagBiasX(mag_sensor()),
                StateVariable::MagBiasY(mag_sensor()),
                StateVariable::MagBiasZ(mag_sensor()),
            ]
        );
    }

    #[test]
    fn extended_first_bias_slot_sits_at_the_base_boundary() {
        let base = pos_vel_schema();
        let base_storage = base.storage_dim();

        let extended = base.extended(vec![mag_bias_block()]);

        assert_eq!(
            extended.storage_offset_of(&StateVariable::MagBiasX(mag_sensor())),
            Some(base_storage)
        );
    }

    #[test]
    fn extended_puts_augmentation_on_a_disjoint_diagonal_corner() {
        let base = pos_vel_schema();
        // Euclidean throughout, so storage and tangent indices coincide.
        let boundary = base.tangent_dim();

        let extended = base.extended(vec![mag_bias_block()]);
        let p = extended.initial_covariance();
        let q = extended.process_noise();

        // The base corner is byte-identical to the un-extended schema's tables.
        let (bp, bq) = (base.initial_covariance(), base.process_noise());
        for i in 0..boundary {
            for j in 0..boundary {
                assert_eq!(p[(i, j)], bp[(i, j)], "P base corner changed at ({i},{j})");
                assert_eq!(q[(i, j)], bq[(i, j)], "Q base corner changed at ({i},{j})");
            }
        }

        // The augmentation corner carries init_uncertainty² on P and
        // random_walk² on Q, isotropic on the diagonal.
        for k in 0..3 {
            let d = boundary + k;
            assert_eq!(p[(d, d)], AUG_INIT_UNCERTAINTY * AUG_INIT_UNCERTAINTY);
            assert_eq!(q[(d, d)], AUG_RANDOM_WALK * AUG_RANDOM_WALK);
        }

        // No correlation is introduced between the base and the augmentation:
        // every off-diagonal cross term stays zero in both tables.
        for i in 0..boundary {
            for k in 0..3 {
                let d = boundary + k;
                assert_eq!(p[(i, d)], 0.0);
                assert_eq!(p[(d, i)], 0.0);
                assert_eq!(q[(i, d)], 0.0);
                assert_eq!(q[(d, i)], 0.0);
            }
        }
    }

    #[test]
    fn extended_leaves_the_base_schema_untouched() {
        let base = pos_vel_schema();
        let base_storage = base.storage_dim();

        let _extended = base.extended(vec![mag_bias_block()]);

        // `extended` takes `&self` and clones; the original is unchanged and
        // still usable afterward.
        assert_eq!(base.storage_dim(), base_storage);
        assert_eq!(base.layout().len(), base_storage);
    }
}
