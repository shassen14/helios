//! What the running pipeline keeps for each node, and the leaves it adds to
//! every node's catalog.

use super::tap::{tapped_instances, Tap};

use crate::{
    observe::{buffer::NodeObservations, observation::ObservedValue},
    pipeline::rate_gate::RateTimer,
    port::{Determinism, Observable, PortBus, SlotVersion},
    prelude::PipelineNode,
    NodeId,
};

use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::AgentId;

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
    /// What was recorded during the node's runs since the last drain: its
    /// emissions and its watched taps. Nothing in the pipeline reads it.
    pub(in crate::pipeline) observations: NodeObservations,
    /// One per channel instance the node writes, in output order: what a
    /// watched channel records once per run that wrote it.
    pub(in crate::pipeline) taps: Vec<Tap>,
}

impl ScheduledNode {
    /// Wraps `node`, a node of `agent`'s pipeline, with a timer at its
    /// declared rate, an empty buffer under its name holding every leaf in
    /// its [`observable_catalog`], and a tap per channel instance it writes,
    /// none of them watched.
    pub(in crate::pipeline) fn new(
        node_id: NodeId,
        node: Box<dyn PipelineNode>,
        agent: &AgentId,
    ) -> Self {
        let rate_timer = RateTimer::new(node.port_descriptor().rate());
        let observations = NodeObservations::new(
            agent,
            node.name(),
            observable_catalog(node.as_ref())
                .into_iter()
                .map(|o| o.name().clone()),
        );
        let taps = output_taps(node.as_ref(), agent).collect();

        Self {
            node_id,
            node,
            rate_timer,
            observations,
            taps,
        }
    }

    /// Each watched tap with its channel's version before the node runs, for
    /// [`record_tap_writes`](Self::record_tap_writes) to compare after. Empty,
    /// and without allocating, when no tap is watched.
    pub(in crate::pipeline) fn watched_tap_versions(&self, bus: &PortBus) -> Vec<PendingTap<'_>> {
        self.taps
            .iter()
            .filter(|tap| tap.watched)
            .map(|tap| PendingTap {
                tap,
                before: tap.read(bus).map(|(_, version)| version),
            })
            .collect()
    }

    /// Records the value of each pending tap whose channel the node wrote
    /// during its run, stamped with the value's own time. A channel whose
    /// version is unchanged was not written, so it records nothing.
    pub(in crate::pipeline) fn record_tap_writes(
        &self,
        bus: &PortBus,
        pending: Vec<PendingTap<'_>>,
    ) {
        for pending in pending {
            let Some((value, version)) = pending.tap.read(bus) else {
                continue;
            };
            if Some(version) == pending.before {
                continue;
            }
            self.observations.record(
                pending.tap.key.clone(),
                value.timestamp(),
                ObservedValue::Typed(value),
            );
        }
    }
}

/// An unwatched tap for each channel instance `node` writes, in output order:
/// one per output, or on a keyed output one per instance it declares, in
/// declaration order. The build refuses a keyed output with no instances.
fn output_taps<'a>(
    node: &'a dyn PipelineNode,
    agent: &'a AgentId,
) -> impl Iterator<Item = Tap> + 'a {
    node.port_descriptor()
        .output_ports()
        .iter()
        .flat_map(move |port| {
            let channel = port.channel();
            tapped_instances(channel, port.instances())
                .into_iter()
                // Every node today writes outputs that depend only on its
                // inputs, the tick's time and its seed. A node whose outputs
                // depend on wall time (an anytime planner) would make its taps
                // nondeterministic, and its descriptor can't say so yet.
                .map(move |instance| Tap::new(agent, channel, instance, Determinism::Deterministic))
        })
}

/// A watched tap during one run of its node: the channel's version before the
/// run, `None` while the channel has never been written.
///
/// Borrows the node's [`Tap`], so it lives only as long as the run it belongs
/// to.
pub(in crate::pipeline) struct PendingTap<'a> {
    tap: &'a Tap,
    before: Option<SlotVersion>,
}

/// Everything `node` can emit about itself for watchers: the leaves it
/// declares on its descriptor, in declaration order, then the leaves the
/// pipeline adds to every node ([`TICK_DURATION_LEAF`]).
///
/// The pipeline adds a leaf only when it can see that value the same way for
/// every node, from outside the node, such as how long it ran. Anything about
/// a node's insides is the node's own declaration. What a node writes is not
/// here: it is recorded under its channel's key, not the node's. Every reader
/// of a node's emissions (its buffer, the watch set, the startup log) goes
/// through here, so none of them needs to know which leaves the pipeline
/// adds.
pub(crate) fn observable_catalog(node: &dyn PipelineNode) -> Vec<Observable> {
    let mut observables = node.port_descriptor().observables().to_vec();
    observables.push(Observable::scalar(
        TICK_DURATION_LEAF,
        Determinism::Nondeterministic,
        SiUnit::Seconds,
    ));
    observables
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        port::{
            ChannelKey, ChannelKind, InstanceKey, Keyed, PortBus, PortDescriptor,
            PortDescriptorBuilder,
        },
        prelude::TickContext,
    };

    use helios_core::prelude::TfProvider;

    const AGENT: &str = "car";
    const NIS: &str = "aiding.gps.nis";
    const DROPPED: &str = "aiding.gps.dropped";
    const COVARIANCE: &str = "est.cov";
    const MEAN: &str = "est.mean";
    const EDGES: &str = "edges";

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

    struct Edge(&'static str);

    impl Keyed for Edge {
        type Instance = &'static str;

        fn instance(&self) -> &&'static str {
            &self.0
        }
    }

    fn edges() -> ChannelKey {
        ChannelKey::keyed::<Edge>(ChannelKind::Internal, EDGES)
    }

    fn agent() -> AgentId {
        AgentId::new(AGENT)
    }

    fn declaring_leaves(leaves: &[&str]) -> PortDescriptorBuilder {
        leaves
            .iter()
            .fold(PortDescriptorBuilder::algorithm(), |builder, leaf| {
                builder.observable(*leaf, Determinism::Deterministic, SiUnit::Dimensionless)
            })
    }

    /// A node declaring `leaves` that writes no channels.
    fn declaring(leaves: &[&str]) -> Box<dyn PipelineNode> {
        declaring_with_outputs(leaves, &[])
    }

    fn declaring_with_outputs(leaves: &[&str], outputs: &[ChannelKey]) -> Box<dyn PipelineNode> {
        let descriptor = outputs
            .iter()
            .fold(declaring_leaves(leaves), |builder, output| {
                builder.output(output.clone())
            })
            .build();
        Box::new(Declaring { descriptor })
    }

    fn two_outputs() -> [ChannelKey; 2] {
        [
            ChannelKey::internal::<f64>(COVARIANCE),
            ChannelKey::internal::<f64>(MEAN),
        ]
    }

    fn tap(channel: ChannelKey, instance: Option<InstanceKey>) -> Tap {
        Tap::new(&agent(), &channel, instance, Determinism::Deterministic)
    }

    #[test]
    fn catalog_lists_declared_leaves_then_tick_duration_and_no_outputs() {
        let node = declaring_with_outputs(&[NIS, DROPPED], &two_outputs());

        assert_eq!(
            observable_catalog(node.as_ref()),
            vec![
                Observable::scalar(NIS, Determinism::Deterministic, SiUnit::Dimensionless),
                Observable::scalar(DROPPED, Determinism::Deterministic, SiUnit::Dimensionless),
                Observable::scalar(
                    TICK_DURATION_LEAF,
                    Determinism::Nondeterministic,
                    SiUnit::Seconds
                ),
            ]
        );
    }

    #[test]
    fn catalog_of_a_node_declaring_nothing_is_tick_duration() {
        let node = declaring(&[]);

        assert_eq!(
            observable_catalog(node.as_ref()),
            vec![Observable::scalar(
                TICK_DURATION_LEAF,
                Determinism::Nondeterministic,
                SiUnit::Seconds
            )]
        );
    }

    #[test]
    fn every_leaf_the_pipeline_measures_is_in_its_group() {
        let node = declaring(&[]);
        let group = format!("{PIPELINE_LEAF_GROUP}.");

        for observable in observable_catalog(node.as_ref()) {
            assert!(
                observable.name().starts_with(&group),
                "{} is outside the pipeline's group",
                observable.name()
            );
        }
    }

    #[test]
    fn scheduled_node_has_an_unwatched_tap_per_output_keyed_by_the_channel() {
        let outputs = two_outputs();
        let scheduled = ScheduledNode::new(0, declaring_with_outputs(&[NIS], &outputs), &agent());

        assert_eq!(
            scheduled.taps,
            vec![tap(outputs[0].clone(), None), tap(outputs[1].clone(), None)]
        );
    }

    #[test]
    fn a_keyed_output_has_a_tap_per_declared_instance() {
        let descriptor = PortDescriptorBuilder::algorithm()
            .output_keyed::<Edge>(edges(), ["wheel", "base_link"])
            .build();
        let scheduled = ScheduledNode::new(0, Box::new(Declaring { descriptor }), &agent());

        assert_eq!(
            scheduled.taps,
            vec![
                tap(edges(), Some(InstanceKey::of::<Edge>(&"wheel"))),
                tap(edges(), Some(InstanceKey::of::<Edge>(&"base_link"))),
            ]
        );
    }

    #[test]
    fn scheduled_node_buffer_holds_the_emissions_only() {
        let mut scheduled =
            ScheduledNode::new(0, declaring_with_outputs(&[NIS], &two_outputs()), &agent());

        assert!(scheduled.observations.set_watched(NIS, true));
        assert!(scheduled.observations.set_watched(TICK_DURATION_LEAF, true));
        assert!(!scheduled.observations.set_watched(COVARIANCE, true));
        assert!(!scheduled.observations.set_watched(DROPPED, true));
    }
}
