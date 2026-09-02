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
    manifold::{euclidean::EuclideanBlock, quaternion::QuaternionBlock, StateBlock, TangentNoise},
    state::Quantity,
};

use nalgebra::{DMatrix, DVector, DVectorView};
use std::sync::Arc;

/// Builds this quantity's manifold block: a [`QuaternionBlock`] for
/// [`Orientation`](Quantity::Orientation), a [`EuclideanBlock`] for every
/// other (flat) kind. `noise` is the tangent-space process noise, already
/// validated by the caller; `None` means the block carries no process noise
/// (a fixed prior — e.g. a position seeded from GPS with no random walk).
///
/// # Panics
/// An orientation block requires process noise: there is no positive-definite
/// zero-noise covariance, so a quaternion retraction cannot be built noiseless.
/// Passing `None` for an [`Orientation`](Quantity::Orientation) is therefore a
/// construction-time programming error, not a recoverable one.
fn block_for(
    quantity: &Quantity,
    noise: Option<TangentNoise>,
    x0: DVector<f64>,
    p0: DMatrix<f64>,
) -> Arc<dyn StateBlock> {
    match quantity {
        Quantity::Orientation { .. } => {
            let noise = noise.expect("an orientation block requires process noise");
            Arc::new(QuaternionBlock::new(noise, x0, p0))
        }
        _ => match noise {
            Some(noise) => Arc::new(EuclideanBlock::new(noise, x0, p0)),
            None => Arc::new(EuclideanBlock::without_noise(x0, p0)),
        },
    }
}

/// One block in a composed schema: the manifold [`StateBlock`] that retracts it,
/// paired with the [`Quantity`] that gives it meaning. The quantity is the single
/// source of the block's identity — its frame(s) and, via [`Quantity::variables`],
/// the ordered [`StateVariable`] names of its stored slots. Those names must number
/// exactly `block.storage_dim()`, a match [`StateSchema::compose`] asserts.
#[derive(Debug, Clone)]
pub struct SchemaBlock {
    pub block: Arc<dyn StateBlock>,
    pub quantity: Quantity,
}

impl SchemaBlock {
    pub fn new(
        quantity: Quantity,
        noise: Option<TangentNoise>,
        x0: DVector<f64>,
        p0: DMatrix<f64>,
    ) -> Self {
        let block = block_for(&quantity, noise, x0, p0);
        Self { block, quantity }
    }

    pub fn variables(&self) -> Vec<StateVariable> {
        self.quantity.variables()
    }
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
            let variables = entry.variables();
            assert_eq!(
                variables.len(),
                sd,
                "schema block variable count ({}) ≠ storage_dim ({})",
                variables.len(),
                sd
            );

            storage_offsets.push(s_off);
            tangent_offsets.push(t_off);

            // Storage-space contributions: names and the initial mean slice.
            layout.extend(variables);
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
            let j = block.variables().iter().position(|v| v == var)?;
            // Within a block, storage index `j` is the tangent index too — except
            // where storage outruns the tangent (Qw sits at j = 3 of a 3-D
            // tangent). There, refuse rather than return a wrong-but-plausible
            // slot: a silent off-by-one in a covariance index is the exact bug
            // the storage/tangent split exists to prevent.
            (j < block.block.tangent_dim()).then_some(self.tangent_offsets[i] + j)
        })
    }

    pub fn storage_offset_of_block(&self, quantity: &Quantity) -> Option<usize> {
        self.blocks
            .iter()
            .position(|b| &b.quantity == quantity)
            .map(|i| self.storage_offsets[i])
    }

    pub fn block_of(&self, quantity: &Quantity) -> Option<&Arc<dyn StateBlock>> {
        self.blocks
            .iter()
            .find(|b| &b.quantity == quantity)
            .map(|b| &b.block)
    }

    /// The frame this estimate's kinematics are expressed in, taken from the
    /// position block. A consumer that has no agent handle of its own (a
    /// planner reading "the robot's position") asks the estimate what frame it
    /// speaks, rather than hardcoding one — so the same reader works whether the
    /// estimate is an odom-frame filter output or a world-frame reference.
    pub fn reference_frame(&self) -> Option<FrameId> {
        self.blocks.iter().find_map(|b| match &b.quantity {
            Quantity::Position(frame) => Some(frame.clone()),
            _ => None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::manifold::TangentNoise;
    use crate::state::Component;

    fn noise(var: f64) -> Option<TangentNoise> {
        Some(TangentNoise::from_variances(DVector::from_element(3, var)).unwrap())
    }

    // ── Quantity: names and manifold selection ───────────────────────────────

    #[test]
    fn variables_spells_each_kind_in_order() {
        // `Quantity::variables` is the single source of a block's stored names, so
        // each kind's exact ordered spelling is pinned here rather than left to
        // transitive coverage. A reordering or a wrong axis tag would silently
        // desync every name lookup built on top of it.
        let f = FrameId::World;
        let b = FrameId::Body(crate::data::primitives::FrameHandle(2));

        // A block's stored names are `StateVariable { quantity, component }` for
        // each of the quantity's components. Every flat kind spells x, y, z;
        // orientation adds w last.
        let flat = |q: &Quantity| {
            vec![
                StateVariable::new(q.clone(), Component::X),
                StateVariable::new(q.clone(), Component::Y),
                StateVariable::new(q.clone(), Component::Z),
            ]
        };

        for kind in [
            Quantity::Position(f.clone()),
            Quantity::Velocity(f.clone()),
            Quantity::Acceleration(f.clone()),
            Quantity::AngularVelocity(f.clone()),
            Quantity::AngularAcceleration(f.clone()),
            Quantity::Mag(f.clone()),
            Quantity::MagBias(f.clone()),
            Quantity::AccelBias(f.clone()),
            Quantity::GyroBias(f.clone()),
        ] {
            assert_eq!(kind.variables(), flat(&kind));
        }

        // Orientation stores four scalars in x, y, z, w order — w last, matching
        // the identity-quaternion seed `[0, 0, 0, 1]`.
        let orientation = Quantity::Orientation {
            from: b.clone(),
            to: f.clone(),
        };
        assert_eq!(
            orientation.variables(),
            vec![
                StateVariable::new(orientation.clone(), Component::X),
                StateVariable::new(orientation.clone(), Component::Y),
                StateVariable::new(orientation.clone(), Component::Z),
                StateVariable::new(orientation.clone(), Component::W),
            ]
        );
    }

    #[test]
    fn manifold_orientation_is_a_four_three_quaternion_block() {
        // The one curved kind: four stored components against a three-DOF tangent.
        let quantity = Quantity::Orientation {
            from: FrameId::Body(crate::data::primitives::FrameHandle(2)),
            to: FrameId::World,
        };

        let block = block_for(
            &quantity,
            noise(0.1),
            DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
            DMatrix::identity(3, 3),
        );
        assert_eq!(block.storage_dim(), 4);
        assert_eq!(block.tangent_dim(), 3);
    }

    #[test]
    fn manifold_flat_kind_is_an_equal_dim_euclidean_block() {
        // Every non-orientation kind is Euclidean: storage and tangent coincide.
        let quantity = Quantity::Velocity(FrameId::World);

        let block = block_for(
            &quantity,
            noise(0.5),
            DVector::zeros(3),
            DMatrix::identity(3, 3),
        );
        assert_eq!(block.storage_dim(), 3);
        assert_eq!(block.tangent_dim(), 3);
    }

    #[test]
    fn manifold_flat_kind_without_noise_carries_no_process_noise() {
        // `None` on a flat kind routes to `EuclideanBlock::without_noise`: a fixed
        // prior with no random walk (a position seeded from GPS, say).
        let quantity = Quantity::Position(FrameId::World);
        let block = block_for(&quantity, None, DVector::zeros(3), DMatrix::identity(3, 3));
        assert!(block.process_noise().is_none());
    }

    #[test]
    #[should_panic(expected = "orientation block requires process noise")]
    fn manifold_orientation_without_noise_panics() {
        // A quaternion retraction has no positive-definite zero-noise covariance, so
        // a noiseless orientation is impossible, not a recoverable default. This is
        // the branch deliberately chosen over a silent `unwrap_or_else` fallback:
        // passing `None` here is a construction-time programming error and must
        // panic rather than fabricate zero process noise.
        let quantity = Quantity::Orientation {
            from: FrameId::Body(crate::data::primitives::FrameHandle(2)),
            to: FrameId::World,
        };

        block_for(
            &quantity,
            None,
            DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
            DMatrix::identity(3, 3),
        );
    }

    fn pos_vel_schema() -> StateSchema {
        StateSchema::compose(vec![
            SchemaBlock::new(
                Quantity::Position(FrameId::World),
                noise(0.1),
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
            SchemaBlock::new(
                Quantity::Velocity(FrameId::World),
                noise(0.5),
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
        ])
    }

    #[test]
    fn dims_and_layout_agree() {
        let s = pos_vel_schema();
        assert_eq!(s.storage_dim(), 6);
        assert_eq!(s.tangent_dim(), 6);
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
        let x = DVector::from_vec(vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let y = DVector::from_vec(vec![-1.0, 0.5, 9.0, 2.0, -2.0, 7.0]);

        let delta = s.ominus(y.as_view(), x.as_view());
        let recovered = s.oplus(x.as_view(), delta.as_view());
        assert!((recovered - &y).amax() < 1e-12);
    }

    #[test]
    fn storage_offset_of_finds_names() {
        let s = pos_vel_schema();
        assert_eq!(
            s.storage_offset_of(&StateVariable::new(
                Quantity::Position(FrameId::World),
                Component::X
            )),
            Some(0)
        );
        assert_eq!(
            s.storage_offset_of(&StateVariable::new(
                Quantity::Velocity(FrameId::World),
                Component::X
            )),
            Some(3)
        );
        assert_eq!(
            s.storage_offset_of(&StateVariable::new(
                Quantity::Acceleration(FrameId::World),
                Component::Z
            )),
            None
        );
    }

    #[test]
    #[should_panic(expected = "storage_dim")]
    fn compose_rejects_variable_count_mismatch() {
        // `Position` names three slots, but a 2-D initial value builds a 2-D
        // block — the wrong-sized value desyncs storage_dim from the derived
        // variable count, which `compose` must reject.
        StateSchema::compose(vec![SchemaBlock::new(
            Quantity::Position(FrameId::World),
            None,
            DVector::zeros(2),
            DMatrix::zeros(2, 2),
        )]);
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
                StateVariable::new(Quantity::MagBias(mag_sensor()), Component::X),
                StateVariable::new(Quantity::MagBias(mag_sensor()), Component::Y),
                StateVariable::new(Quantity::MagBias(mag_sensor()), Component::Z),
            ]
        );
    }

    #[test]
    fn extended_first_bias_slot_sits_at_the_base_boundary() {
        let base = pos_vel_schema();
        let base_storage = base.storage_dim();

        let extended = base.extended(vec![mag_bias_block()]);

        assert_eq!(
            extended.storage_offset_of(&StateVariable::new(
                Quantity::MagBias(mag_sensor()),
                Component::X
            )),
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
