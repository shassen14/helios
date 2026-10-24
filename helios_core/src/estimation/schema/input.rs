use crate::{
    estimation::schema::flat::{FlatSchema, FlatSchemaBlock},
    spatial::state::StateVariable,
};

/// One block of an [`InputSchema`]: a quantity the dynamics consume and the axis
/// convention of each frame it references. An input is a flat vector, so its
/// block is the shared flat block.
pub type InputSchemaBlock = FlatSchemaBlock;

/// The named layout of a dynamics model's input vector `u`: its ordered blocks,
/// one name per component, and the total length. The input dual of
/// [`MeasurementSchema`](super::MeasurementSchema). The dynamics declare it, the
/// input builder declares what it assembles, and
/// [`check_input_agreement`](super::check_input_agreement) compares the two at
/// build, so a swapped or reframed input fails loudly instead of being
/// integrated as the wrong quantity. Length alone would pass an accel/gyro swap.
#[derive(Debug)]
pub struct InputSchema(FlatSchema);

impl InputSchema {
    /// Composes an ordered list of blocks into one input schema. An empty list is
    /// a model that takes no input.
    ///
    /// # Panics
    /// On an orientation block: a rotation-increment input's component names are
    /// not yet defined. See [`FlatSchemaBlock::orientation`].
    pub fn compose(blocks: Vec<InputSchemaBlock>) -> Self {
        Self(FlatSchema::compose(blocks, "input"))
    }

    /// The ordered input blocks — walked by the agreement check.
    pub fn blocks(&self) -> &[InputSchemaBlock] {
        self.0.blocks()
    }

    /// Ordered input-component names; `layout.len() == dim`.
    pub fn layout(&self) -> &[StateVariable] {
        self.0.layout()
    }

    /// Total input length — the number of rows `u` must have.
    pub fn dim(&self) -> usize {
        self.0.dim()
    }

    /// The row of `u` holding `variable`, or `None` if no block names it. A model
    /// reads its inputs through this rather than hardcoding rows, so its schema
    /// is the one place the order is written.
    pub fn offset_of(&self, variable: &StateVariable) -> Option<usize> {
        self.0.offset_of(variable)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prelude::AgentId;
    use crate::spatial::state::{Component, Quantity};
    use crate::spatial::transforms::Convention;
    use crate::spatial::FrameId;

    fn body() -> FrameId {
        FrameId::base_link(AgentId::new("test_agent"))
    }

    #[test]
    fn compose_lays_out_blocks_in_declared_order() {
        let schema = InputSchema::compose(vec![
            InputSchemaBlock::new(Quantity::SpecificForce(body()), Convention::Flu),
            InputSchemaBlock::new(Quantity::AngularVelocity(body()), Convention::Flu),
        ]);

        assert_eq!(schema.dim(), 6);
        assert_eq!(
            schema.offset_of(&StateVariable::new(
                Quantity::SpecificForce(body()),
                Component::X
            )),
            Some(0)
        );
        assert_eq!(
            schema.offset_of(&StateVariable::new(
                Quantity::AngularVelocity(body()),
                Component::X
            )),
            Some(3)
        );
    }

    #[test]
    fn an_empty_input_schema_is_a_model_with_no_input() {
        assert_eq!(InputSchema::compose(vec![]).dim(), 0);
    }

    #[test]
    #[should_panic(expected = "orientation input layout")]
    fn compose_refuses_an_orientation_block_until_a_rotation_increment_input_exists() {
        InputSchema::compose(vec![InputSchemaBlock::orientation(
            body(),
            FrameId::odom(AgentId::new("test_agent")),
            Convention::Flu,
            Convention::Enu,
        )]);
    }
}
