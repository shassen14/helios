//! The flat-schema core shared by the measurement and input schemas.
//!
//! A measurement's innovation and a dynamics model's input are both flat
//! vectors of named quantities, each expressed in one axis convention per frame
//! it references. Neither has a storage-vs-tangent split, a `P₀` / `Q`, or an
//! `oplus` / `ominus`; that machinery belongs to the state. This module owns the
//! composition the two share (blocks, layout, length); `MeasurementSchema` and
//! `InputSchema` are role-named types over it, so one cannot be passed where the
//! other is expected.

use crate::{
    spatial::state::{Quantity, StateVariable},
    spatial::{transforms::Convention, FrameId},
};

/// One block of a flat schema: a [`Quantity`] and the axis convention of each
/// frame it references. The flat mirror of
/// [`StateSchemaBlock`](super::StateSchemaBlock), minus the manifold retraction:
/// a flat vector is never a point on a curved space, so there is no
/// `StateBlock` to store. Its identity is exactly `quantity + conventions`; its
/// length comes from [`dim`](Self::dim).
///
/// Named by role where it is used: [`MeasurementSchemaBlock`](super::MeasurementSchemaBlock)
/// for a measurement, [`InputSchemaBlock`](super::InputSchemaBlock) for a
/// dynamics input.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FlatSchemaBlock {
    pub(crate) quantity: Quantity,
    /// The axis convention of each frame this block references — one entry for a
    /// flat quantity, one per endpoint for an orientation.
    pub(crate) conventions: Vec<(FrameId, Convention)>,
}

impl FlatSchemaBlock {
    /// Builds a block for a flat `quantity`, recording the one axis `convention`
    /// its components are expressed in — every flat quantity lives in exactly
    /// one — as this block's single frame → convention entry.
    ///
    /// # Panics
    /// Rejects [`Orientation`](Quantity::Orientation): a rotation is a map between
    /// two conventions, so it must carry one per endpoint. Build it through
    /// [`orientation`](Self::orientation). A construction-time programming error,
    /// caught at startup.
    pub fn new(quantity: Quantity, convention: Convention) -> Self {
        assert!(
            !matches!(quantity, Quantity::Orientation { .. }),
            "`new` builds flat blocks; build an orientation block with `orientation`",
        );

        let frame = quantity
            .frame()
            .expect("new rejects orientation, so a flat quantity has a frame");

        let conventions = vec![(frame.clone(), convention)];

        Self {
            quantity,
            conventions,
        }
    }

    /// Builds an orientation block for the `from → to` rotation, recording the
    /// axis convention of each endpoint — `from_conv` for the source, `to_conv`
    /// for the target. Reserved for an attitude measurement (an AHRS) or a
    /// rotation-increment input (discrete odometry): neither exists yet, so an
    /// orientation block has a defined length ([`dim`](Self::dim) = 3, the
    /// rotation tangent) but no defined component names — see
    /// [`FlatSchema::compose`].
    pub fn orientation(
        from: FrameId,
        to: FrameId,
        from_conv: Convention,
        to_conv: Convention,
    ) -> Self {
        let quantity = Quantity::Orientation {
            from: from.clone(),
            to: to.clone(),
        };

        let conventions = vec![(from, from_conv), (to, to_conv)];

        Self {
            quantity,
            conventions,
        }
    }

    pub fn quantity(&self) -> &Quantity {
        &self.quantity
    }

    /// The axis convention of each frame this block references — one entry for a
    /// flat quantity, one per endpoint for an orientation.
    pub fn conventions(&self) -> &[(FrameId, Convention)] {
        &self.conventions
    }

    /// The block's contribution to the flat vector — its *tangent* length, not
    /// its stored-component count. Flat quantities coincide (a 3-vector names
    /// three and occupies three); an orientation stores four quaternion numbers
    /// but occupies the three-DOF rotation tangent, so it contributes `3`.
    pub(crate) fn dim(&self) -> usize {
        match self.quantity {
            Quantity::Orientation { .. } => 3,
            _ => self.quantity.variables().len(),
        }
    }
}

/// The composed shape of a flat vector: its ordered blocks, the per-component
/// `layout`, and the total `dim`. Simpler than
/// [`StateSchema`](super::StateSchema): one coordinate space, so no dual offset
/// tables and no storage-vs-tangent check.
#[derive(Debug)]
pub(super) struct FlatSchema {
    blocks: Vec<FlatSchemaBlock>,
    layout: Vec<StateVariable>,
    dim: usize,
}

impl FlatSchema {
    /// Composes an ordered list of blocks, summing `dim` and concatenating each
    /// block's component names into `layout`. `role` names the schema in the
    /// panic message ("measurement", "input").
    ///
    /// # Panics
    /// On an orientation block. Its length is known (3), but the *names* of its
    /// three tangent components are not yet defined — nothing produces one. When
    /// the first does, the resolution is three rotation-vector components (so(3)
    /// axes, reusing `Component::{X, Y, Z}`), never the four quaternion storage
    /// names. Until then the block is refused rather than mislabeled.
    pub(super) fn compose(blocks: Vec<FlatSchemaBlock>, role: &str) -> Self {
        let mut layout = Vec::new();
        let mut dim = 0;

        for block in &blocks {
            dim += block.dim();
            match block.quantity {
                Quantity::Orientation { .. } => panic!(
                    "orientation {role} layout is not yet defined; no {role} model produces one"
                ),
                _ => layout.extend(block.quantity.variables()),
            }
        }

        Self {
            blocks,
            layout,
            dim,
        }
    }

    pub(super) fn blocks(&self) -> &[FlatSchemaBlock] {
        &self.blocks
    }

    pub(super) fn layout(&self) -> &[StateVariable] {
        &self.layout
    }

    pub(super) fn dim(&self) -> usize {
        self.dim
    }

    /// The index of `variable` in the flat vector, or `None` if no block names it.
    pub(super) fn offset_of(&self, variable: &StateVariable) -> Option<usize> {
        self.layout.iter().position(|v| v == variable)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prelude::AgentId;
    use crate::spatial::state::Component;

    // ── FlatSchemaBlock: convention tagging and the constructor split ──

    #[test]
    fn new_records_a_flat_block_as_one_frame_convention_entry() {
        // A flat kind touches one frame, so `new` records exactly one
        // (frame, convention) entry: the quantity's frame in the given convention.
        let block = FlatSchemaBlock::new(Quantity::Position(FrameId::world()), Convention::Enu);
        assert_eq!(block.conventions, vec![(FrameId::world(), Convention::Enu)]);
        // A flat block occupies as many components as it names.
        assert_eq!(block.dim(), 3);
    }

    #[test]
    fn orientation_records_one_entry_per_endpoint() {
        // A rotation touches two frames, so `orientation` records two entries: the
        // source in `from_conv`, the target in `to_conv`, each on its own frame.
        let agent = AgentId::new("test_agent");
        let block = FlatSchemaBlock::orientation(
            FrameId::base_link(agent.clone()),
            FrameId::odom(agent.clone()),
            Convention::Flu,
            Convention::Enu,
        );
        assert_eq!(
            block.conventions,
            vec![
                (FrameId::base_link(agent.clone()), Convention::Flu),
                (FrameId::odom(agent.clone()), Convention::Enu),
            ]
        );
        // An orientation occupies the 3-DOF rotation tangent, never the four
        // stored quaternion components.
        assert_eq!(block.dim(), 3);
    }

    #[test]
    #[should_panic(expected = "build an orientation block with `orientation`")]
    fn new_rejects_an_orientation_quantity() {
        // `new` records a single frame → convention entry, but an orientation
        // touches two frames; it must go through `orientation`, which records both.
        let agent = AgentId::new("test_agent");
        FlatSchemaBlock::new(
            Quantity::Orientation {
                from: FrameId::base_link(agent.clone()),
                to: FrameId::odom(agent.clone()),
            },
            Convention::Enu,
        );
    }

    // ── FlatSchema::compose ──

    fn pos_vel() -> FlatSchema {
        FlatSchema::compose(
            vec![
                FlatSchemaBlock::new(Quantity::Position(FrameId::world()), Convention::Enu),
                FlatSchemaBlock::new(Quantity::Velocity(FrameId::world()), Convention::Enu),
            ],
            "test",
        )
    }

    #[test]
    fn compose_sums_the_block_dims() {
        let schema = pos_vel();
        assert_eq!(schema.dim(), 6);
        assert_eq!(schema.blocks().len(), 2);
    }

    #[test]
    fn compose_names_each_component_in_order() {
        let schema = pos_vel();
        // One name per component, blocks concatenated left to right.
        assert_eq!(schema.layout().len(), schema.dim());
        assert_eq!(
            schema.layout()[0],
            StateVariable::new(Quantity::Position(FrameId::world()), Component::X)
        );
        assert_eq!(
            schema.layout()[3],
            StateVariable::new(Quantity::Velocity(FrameId::world()), Component::X)
        );
    }

    #[test]
    fn offset_of_finds_a_named_component_and_misses_an_absent_one() {
        let schema = pos_vel();
        assert_eq!(
            schema.offset_of(&StateVariable::new(
                Quantity::Velocity(FrameId::world()),
                Component::Y
            )),
            Some(4)
        );
        assert_eq!(
            schema.offset_of(&StateVariable::new(
                Quantity::Acceleration(FrameId::world()),
                Component::X
            )),
            None
        );
    }

    #[test]
    fn an_empty_schema_has_no_components() {
        let schema = FlatSchema::compose(vec![], "test");
        assert_eq!(schema.dim(), 0);
        assert!(schema.layout().is_empty());
    }

    #[test]
    #[should_panic(expected = "orientation test layout is not yet defined")]
    fn compose_refuses_an_orientation_block_until_its_names_are_defined() {
        // The length is known (3), but the component names are not — see the
        // `compose` docs. Refused rather than mislabeled.
        let agent = AgentId::new("test_agent");
        FlatSchema::compose(
            vec![FlatSchemaBlock::orientation(
                FrameId::base_link(agent.clone()),
                FrameId::odom(agent.clone()),
                Convention::Flu,
                Convention::Enu,
            )],
            "test",
        );
    }
}
