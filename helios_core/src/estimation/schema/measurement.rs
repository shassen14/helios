use crate::{
    estimation::schema::flat::{FlatSchema, FlatSchemaBlock},
    spatial::state::StateVariable,
};

/// One block of a [`MeasurementSchema`]: the quantity a measurement predicts and
/// the axis convention of each frame it references. A measurement contributes a
/// flat innovation `z − h(x)`, so its block is the shared flat block.
pub type MeasurementSchemaBlock = FlatSchemaBlock;

/// The composed shape of a filter's measurement: its ordered blocks, the flat
/// innovation `layout`, and the total innovation `dim`. The measurement mirror of
/// [`StateSchema`](super::StateSchema), but with a single coordinate space — an
/// innovation is flat, so there is no storage-vs-tangent split, no `P₀` / `Q`,
/// and no `oplus` / `ominus`. Produced by a measurement model's `schema()` and
/// compared against the state schema at estimator construction.
#[derive(Debug)]
pub struct MeasurementSchema(FlatSchema);

impl MeasurementSchema {
    /// Composes an ordered list of blocks into one measurement schema.
    ///
    /// # Panics
    /// On an orientation block: an attitude measurement's innovation names are
    /// not yet defined. See [`FlatSchemaBlock::orientation`].
    pub fn compose(blocks: Vec<MeasurementSchemaBlock>) -> Self {
        Self(FlatSchema::compose(blocks, "measurement"))
    }

    /// The ordered measurement blocks — walked by the estimator's
    /// construction-time agreement check against the state schema.
    pub fn blocks(&self) -> &[MeasurementSchemaBlock] {
        self.0.blocks()
    }

    /// Ordered innovation-component names; `layout.len() == dim`.
    pub fn layout(&self) -> &[StateVariable] {
        self.0.layout()
    }

    /// Total innovation length — the size of the `z` / `R` side of the update.
    pub fn dim(&self) -> usize {
        self.0.dim()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prelude::AgentId;
    use crate::spatial::state::Quantity;
    use crate::spatial::transforms::Convention;
    use crate::spatial::FrameId;

    #[test]
    fn compose_carries_the_blocks_layout_and_dim_through() {
        let schema = MeasurementSchema::compose(vec![
            MeasurementSchemaBlock::new(Quantity::Position(FrameId::world()), Convention::Enu),
            MeasurementSchemaBlock::new(Quantity::Velocity(FrameId::world()), Convention::Enu),
        ]);
        assert_eq!(schema.dim(), 6);
        assert_eq!(schema.blocks().len(), 2);
        assert_eq!(schema.layout().len(), schema.dim());
    }

    #[test]
    #[should_panic(expected = "orientation measurement layout")]
    fn compose_refuses_an_orientation_block_until_an_ahrs_measurement_exists() {
        // The length is known (3), but the component names are not. Refused
        // rather than mislabeled.
        let agent = AgentId::new("test_agent");
        MeasurementSchema::compose(vec![MeasurementSchemaBlock::orientation(
            FrameId::base_link(agent.clone()),
            FrameId::odom(agent.clone()),
            Convention::Flu,
            Convention::Enu,
        )]);
    }
}
