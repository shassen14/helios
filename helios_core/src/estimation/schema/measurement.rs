use crate::{
    estimation::schema::BlockConvention,
    frames::{transforms::Convention, FrameId},
    state::{Quantity, StateVariable},
};

/// One block of a [`MeasurementSchema`]: the [`Quantity`] a measurement predicts
/// and the [`BlockConvention`] its components are expressed in. The measurement
/// mirror of [`StateSchemaBlock`](super::StateSchemaBlock), minus the manifold
/// retraction — a measurement contributes a flat innovation `z − h(x)`, never a
/// point on a curved space, so there is no `StateBlock` to store. Its identity is
/// exactly `quantity + convention`; the innovation length comes from
/// [`dim`](Self::dim).
#[derive(Debug, Clone)]
pub struct MeasurementSchemaBlock {
    pub(crate) quantity: Quantity,
    pub(crate) convention: BlockConvention,
}

impl MeasurementSchemaBlock {
    /// Builds a flat block for a measurement of `quantity`, tagging it with the
    /// one axis `convention` its components are expressed in, stored as
    /// [`BlockConvention::Single`].
    ///
    /// # Panics
    /// Rejects [`Orientation`](Quantity::Orientation): a rotation is a map between
    /// two conventions, so it must carry one per endpoint. Build it through
    /// [`orientation`](Self::orientation). A construction-time programming error,
    /// caught at startup.
    pub fn new(quantity: Quantity, convention: Convention) -> Self {
        assert!(
            !matches!(quantity, Quantity::Orientation { .. }),
            "MeasurementSchemaBlock::new builds Euclidean blocks; use MeasurementSchemaBlock::orientation for an orientation block",
        );

        Self {
            quantity,
            convention: BlockConvention::Single(convention),
        }
    }

    /// Builds an orientation block for the `from → to` rotation, recording the
    /// axis convention of each endpoint as a [`BlockConvention::Pair`]. Reserved
    /// for an attitude/heading measurement (an AHRS): no measurement model
    /// produces one today, so a `Pair` block has a defined innovation length
    /// ([`dim`](Self::dim) = 3, the rotation tangent) but no defined component
    /// names yet — see [`MeasurementSchema::compose`].
    pub fn orientation(
        from: FrameId,
        to: FrameId,
        from_conv: Convention,
        to_conv: Convention,
    ) -> Self {
        let quantity = Quantity::Orientation { from, to };

        let convention = BlockConvention::Pair {
            from: from_conv,
            to: to_conv,
        };

        Self {
            quantity,
            convention,
        }
    }

    pub fn quantity(&self) -> &Quantity {
        &self.quantity
    }

    pub fn convention(&self) -> &BlockConvention {
        &self.convention
    }

    /// The block's contribution to the innovation vector — its *tangent* length,
    /// not its stored-component count. Flat blocks coincide (a 3-vector names
    /// three and innovates in three); an orientation stores four quaternion
    /// numbers but innovates on the three-DOF rotation tangent, so it
    /// contributes `3`.
    pub(crate) fn dim(&self) -> usize {
        match self.convention {
            BlockConvention::Single(_) => self.quantity.variables().len(),
            BlockConvention::Pair { .. } => 3,
        }
    }
}

/// The composed shape of a filter's measurement: its ordered blocks, the flat
/// innovation `layout`, and the total innovation `dim`. The measurement mirror of
/// [`StateSchema`](super::StateSchema), but with a single coordinate space — an
/// innovation is flat, so there is no storage-vs-tangent split, no `P₀` / `Q`,
/// and no `oplus` / `ominus`. Produced by a measurement model's `schema()` and
/// compared against the state schema at estimator construction.
#[derive(Debug)]
pub struct MeasurementSchema {
    blocks: Vec<MeasurementSchemaBlock>,
    layout: Vec<StateVariable>,
    dim: usize,
}

impl MeasurementSchema {
    /// Composes an ordered list of blocks into one measurement schema, summing
    /// the innovation `dim` and concatenating each block's component names into
    /// `layout`.
    ///
    /// Simpler than [`StateSchema::compose`](super::StateSchema::compose): a
    /// measurement has one coordinate space, so there are no dual offset tables
    /// and no `variables().len()`-vs-`storage_dim` check — the length is
    /// definitional here, taken straight from [`MeasurementSchemaBlock::dim`].
    ///
    /// # Panics
    /// On a [`Pair`](BlockConvention::Pair) block. Its innovation length is known
    /// (3), but the *names* of an orientation measurement's three tangent
    /// components are not yet defined — no model produces one. When an AHRS
    /// measurement lands, the resolution is three rotation-vector components
    /// (so(3) axes, reusing `Component::{X, Y, Z}`), never the four quaternion
    /// storage names. Until then the block is refused rather than mislabeled.
    pub fn compose(blocks: Vec<MeasurementSchemaBlock>) -> Self {
        let mut layout = Vec::new();
        let mut dim = 0;

        for block in &blocks {
            dim += block.dim();
            match block.convention {
                BlockConvention::Single(_) => layout.extend(block.quantity.variables()),
                BlockConvention::Pair { .. } => panic!(
                    "orientation-measurement layout is not yet defined; no measurement model produces a Pair block"
                ),
            }
        }

        Self {
            blocks,
            layout,
            dim,
        }
    }

    /// The ordered measurement blocks — walked by the estimator's
    /// construction-time agreement check against the state schema.
    pub fn blocks(&self) -> &[MeasurementSchemaBlock] {
        &self.blocks
    }

    /// Ordered innovation-component names; `layout.len() == dim`.
    pub fn layout(&self) -> &[StateVariable] {
        &self.layout
    }

    /// Total innovation length — the size of the `z` / `R` side of the update.
    pub fn dim(&self) -> usize {
        self.dim
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data::primitives::FrameHandle;
    use crate::state::Component;

    // ── MeasurementSchemaBlock: convention tagging and the constructor split ──

    #[test]
    fn new_tags_a_flat_block_with_a_single_convention() {
        let block =
            MeasurementSchemaBlock::new(Quantity::Position(FrameId::World), Convention::Enu);
        assert_eq!(
            block.convention(),
            &BlockConvention::Single(Convention::Enu)
        );
        // A flat block innovates in as many dimensions as it names.
        assert_eq!(block.dim(), 3);
    }

    #[test]
    fn orientation_tags_each_endpoint_as_a_pair() {
        let handle = FrameHandle(2);
        let block = MeasurementSchemaBlock::orientation(
            FrameId::Body(handle),
            FrameId::Odom(handle),
            Convention::Flu,
            Convention::Enu,
        );
        assert_eq!(
            block.convention(),
            &BlockConvention::Pair {
                from: Convention::Flu,
                to: Convention::Enu,
            }
        );
        // An orientation's innovation is the 3-DOF rotation tangent, never the
        // four stored quaternion components.
        assert_eq!(block.dim(), 3);
    }

    #[test]
    #[should_panic(expected = "use MeasurementSchemaBlock::orientation")]
    fn new_rejects_an_orientation_quantity() {
        // `new` stores a single convention, so an orientation would pair a 3-DOF
        // rotation with one tag — the mismatch `BlockConvention` exists to stop.
        let handle = FrameHandle(2);
        MeasurementSchemaBlock::new(
            Quantity::Orientation {
                from: FrameId::Body(handle),
                to: FrameId::Odom(handle),
            },
            Convention::Enu,
        );
    }

    // ── MeasurementSchema::compose ──

    fn pos_vel_measurement() -> MeasurementSchema {
        MeasurementSchema::compose(vec![
            MeasurementSchemaBlock::new(Quantity::Position(FrameId::World), Convention::Enu),
            MeasurementSchemaBlock::new(Quantity::Velocity(FrameId::World), Convention::Enu),
        ])
    }

    #[test]
    fn compose_sums_the_block_innovation_dims() {
        let schema = pos_vel_measurement();
        assert_eq!(schema.dim(), 6);
        assert_eq!(schema.blocks().len(), 2);
    }

    #[test]
    fn compose_names_each_flat_innovation_component_in_order() {
        let schema = pos_vel_measurement();
        // One name per innovation component, blocks concatenated left to right.
        assert_eq!(schema.layout().len(), schema.dim());
        assert_eq!(
            schema.layout()[0],
            StateVariable::new(Quantity::Position(FrameId::World), Component::X)
        );
        assert_eq!(
            schema.layout()[3],
            StateVariable::new(Quantity::Velocity(FrameId::World), Component::X)
        );
    }

    #[test]
    #[should_panic(expected = "orientation-measurement layout")]
    fn compose_refuses_a_pair_block_until_an_ahrs_measurement_exists() {
        // The length is known (3), but the component names are not — see the
        // `compose` docs. Refused rather than mislabeled.
        let handle = FrameHandle(2);
        MeasurementSchema::compose(vec![MeasurementSchemaBlock::orientation(
            FrameId::Body(handle),
            FrameId::Odom(handle),
            Convention::Flu,
            Convention::Enu,
        )]);
    }
}
