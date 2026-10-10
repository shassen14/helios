//! What the running pipeline keeps for each node, and the leaves it adds to
//! every node's catalog.

use crate::{
    observe::buffer::NodeObservations,
    pipeline::rate_gate::RateTimer,
    port::{Determinism, Observable},
    prelude::PipelineNode,
    NodeId,
};

/// The group the pipeline's own leaves sit under. The pipeline adds these to
/// every node, so a node may not declare this leaf or any leaf under it; the
/// build refuses one, so a leaf the pipeline adds later never breaks a node.
pub const PIPELINE_LEAF_GROUP: &str = "tick";

/// The leaf under which the pipeline reports how long one run of a node took,
/// in seconds. Every node has it; nodes never declare it themselves. Wall-clock
/// time, so it differs between runs.
pub const TICK_DURATION_LEAF: &str = "tick.duration";

/// A node as the running pipeline holds it: the node plus the state the
/// pipeline keeps for it.
///
/// A field belongs here only when the pipeline gives it to every node, such
/// as rate gating or an observation buffer. State specific to one kind of
/// node stays inside that node.
pub(in crate::pipeline) struct ScheduledNode {
    /// Assigned at build in level-major order; stamps the node's bus writes.
    pub(in crate::pipeline) node_id: NodeId,
    pub(in crate::pipeline) node: Box<dyn PipelineNode>,
    /// Decides each tick whether the node is due. Fires every tick when the
    /// node declares no rate.
    pub(in crate::pipeline) rate_timer: RateTimer,
    /// What the node emitted for watchers since the last drain. Nothing in
    /// the pipeline reads it.
    pub(in crate::pipeline) observations: NodeObservations,
}

impl ScheduledNode {
    /// Wraps `node` with a timer at its declared rate and an empty buffer
    /// under its name, holding every leaf in its
    /// [`observable_catalog`], none of them watched.
    pub(in crate::pipeline) fn new(node_id: NodeId, node: Box<dyn PipelineNode>) -> Self {
        let rate_timer = RateTimer::new(node.port_descriptor().rate());
        let observations = NodeObservations::new(
            node.name(),
            observable_catalog(node.as_ref())
                .into_iter()
                .map(|o| o.leaf_name().clone()),
        );

        Self {
            node_id,
            node,
            rate_timer,
            observations,
        }
    }
}

/// Everything `node` can emit for watchers: the leaves it declares on its
/// descriptor, in declaration order, then the leaves the pipeline adds to every
/// node ([`TICK_DURATION_LEAF`]).
///
/// The pipeline adds a leaf only when it can measure that value the same way
/// for every node, from outside the node. Anything about a node's insides is
/// the node's own declaration. Every reader of a node's catalog (its buffer,
/// the build checks, the startup log) goes through here, so none of them
/// needs to know which leaves the pipeline adds.
pub(crate) fn observable_catalog(node: &dyn PipelineNode) -> Vec<Observable> {
    let mut observables = node.port_descriptor().observables().to_vec();
    observables.push(Observable::new(TICK_DURATION_LEAF, Determinism::WallClock));
    observables
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        port::{MockNodePortDescriptor, PortBus, PortDescriptor},
        prelude::TickContext,
    };

    use helios_core::prelude::TfProvider;

    const NIS: &str = "aiding.gps.nis";
    const DROPPED: &str = "aiding.gps.dropped";

    /// A node that only declares; it never runs in these tests.
    struct Declaring {
        descriptor: PortDescriptor,
    }

    impl PipelineNode for Declaring {
        fn name(&self) -> &str {
            "declaring"
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {}
    }

    fn declaring(leaves: &[&str]) -> Box<dyn PipelineNode> {
        let descriptor = leaves
            .iter()
            .fold(MockNodePortDescriptor::new(), |builder, leaf| {
                builder.observable(*leaf, Determinism::Reproducible)
            })
            .build();
        Box::new(Declaring { descriptor })
    }

    #[test]
    fn catalog_lists_declared_leaves_then_tick_duration() {
        let node = declaring(&[NIS, DROPPED]);

        assert_eq!(
            observable_catalog(node.as_ref()),
            vec![
                Observable::new(NIS, Determinism::Reproducible),
                Observable::new(DROPPED, Determinism::Reproducible),
                Observable::new(TICK_DURATION_LEAF, Determinism::WallClock),
            ]
        );
    }

    #[test]
    fn catalog_of_a_node_declaring_nothing_is_tick_duration() {
        let node = declaring(&[]);

        assert_eq!(
            observable_catalog(node.as_ref()),
            vec![Observable::new(TICK_DURATION_LEAF, Determinism::WallClock)]
        );
    }

    #[test]
    fn every_leaf_the_pipeline_adds_is_in_its_group() {
        let node = declaring(&[]);
        let group = format!("{PIPELINE_LEAF_GROUP}.");

        for observable in observable_catalog(node.as_ref()) {
            assert!(
                observable.leaf_name().starts_with(&group),
                "{} is outside the pipeline's group",
                observable.leaf_name()
            );
        }
    }

    #[test]
    fn scheduled_node_buffer_holds_the_whole_catalog() {
        let mut scheduled = ScheduledNode::new(0, declaring(&[NIS]));

        assert!(scheduled.observations.set_watched(NIS, true));
        assert!(scheduled.observations.set_watched(TICK_DURATION_LEAF, true));
        assert!(!scheduled.observations.set_watched(DROPPED, true));
    }
}
