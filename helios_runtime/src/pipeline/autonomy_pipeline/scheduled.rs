//! What the running pipeline keeps for each node, and the leaves it adds to
//! every node's catalog.

use crate::{
    observe::{buffer::NodeObservations, observation::ObservedValue},
    pipeline::rate_gate::RateTimer,
    port::{ChannelKey, Determinism, Observable, PortBus, SlotVersion},
    prelude::PipelineNode,
    NodeId,
};

use std::sync::Arc;

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
    /// One per output channel, in output order: which channel each tap leaf
    /// in the catalog reads.
    pub(in crate::pipeline) taps: Vec<Tap>,
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
        let taps = output_taps(node.as_ref()).collect();

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
            .filter_map(|tap| {
                let leaf = self.observations.watched_leaf(&tap.leaf)?;
                Some(PendingTap {
                    leaf,
                    channel: &tap.channel,
                    before: bus
                        .read_erased_versioned(&tap.channel)
                        .map(|(_, version)| version),
                })
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
        for tap in pending {
            let Some((value, version)) = bus.read_erased_versioned(tap.channel) else {
                continue;
            };
            if Some(version) == tap.before {
                continue;
            }
            self.observations
                .record(tap.leaf, value.timestamp(), ObservedValue::Typed(value));
        }
    }
}

/// A leaf that reports what a node writes to one of its output channels: the
/// value itself, shared with the bus, once per run that wrote it.
#[derive(Debug, Clone, PartialEq)]
pub(in crate::pipeline) struct Tap {
    /// The channel's name.
    pub(in crate::pipeline) leaf: Arc<str>,
    pub(in crate::pipeline) channel: ChannelKey,
}

/// A tap for each of `node`'s outputs, in output order. The catalog and the
/// scheduled node both build their taps here, so a tap leaf a watcher can ask
/// for always has a channel behind it.
fn output_taps(node: &dyn PipelineNode) -> impl Iterator<Item = Tap> + '_ {
    node.port_descriptor().outputs().iter().map(|channel| Tap {
        leaf: channel.instance().clone(),
        channel: channel.clone(),
    })
}

/// A watched tap during one run of its node: the channel's version before the
/// run, `None` while the channel has never been written.
///
/// Borrows the channel from the node's [`Tap`], so it lives only as long as
/// the run it belongs to.
pub(in crate::pipeline) struct PendingTap<'a> {
    /// The buffer's own name for the leaf, from
    /// [`watched_leaf`](NodeObservations::watched_leaf).
    leaf: Arc<str>,
    channel: &'a ChannelKey,
    before: Option<SlotVersion>,
}

/// Everything `node` can emit for watchers: the leaves it declares on its
/// descriptor, in declaration order, then the leaves the pipeline adds to every
/// node: a tap per output channel, in output order, then the pipeline's own
/// measurements ([`TICK_DURATION_LEAF`]).
///
/// The pipeline adds a leaf only when it can see that value the same way for
/// every node, from outside the node: what the node wrote, or how long it
/// ran. Anything about a node's insides is the node's own declaration. Every
/// reader of a node's catalog (its buffer, the watch set, the startup log)
/// goes through here, so none of them needs to know which leaves the
/// pipeline adds.
///
/// Every tap is marked deterministic, because every node today writes outputs
/// that depend only on its inputs, the tick's time and its seed. A node whose
/// outputs depend on wall time (an anytime planner) would make its taps
/// nondeterministic, and the catalog can't tell yet. A tap arrives typed, as
/// its channel's payload.
pub(crate) fn observable_catalog(node: &dyn PipelineNode) -> Vec<Observable> {
    let mut observables = node.port_descriptor().observables().to_vec();
    observables.extend(
        output_taps(node)
            .map(|tap| Observable::typed(tap.leaf, Determinism::Deterministic, &tap.channel)),
    );
    observables.push(Observable::scalar(
        TICK_DURATION_LEAF,
        Determinism::Nondeterministic,
    ));
    observables
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        port::{InternalChannel, MockNodePortDescriptor, PortBus, PortDescriptor},
        prelude::TickContext,
    };

    use helios_core::prelude::TfProvider;

    const NIS: &str = "aiding.gps.nis";
    const DROPPED: &str = "aiding.gps.dropped";
    /// Output channel names, which are also their tap leaves.
    const COVARIANCE: &str = "est.cov";
    const MEAN: &str = "est.mean";

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

    /// A node declaring `leaves` that writes no channels.
    fn declaring(leaves: &[&str]) -> Box<dyn PipelineNode> {
        declaring_with_outputs(leaves, &[])
    }

    fn declaring_with_outputs(
        leaves: &[&str],
        outputs: &[InternalChannel],
    ) -> Box<dyn PipelineNode> {
        let builder = leaves
            .iter()
            .fold(MockNodePortDescriptor::new(), |builder, leaf| {
                builder.observable(*leaf, Determinism::Deterministic)
            });
        let descriptor = outputs
            .iter()
            .fold(builder, |builder, output| {
                builder.output_internal(output.clone())
            })
            .build();
        Box::new(Declaring { descriptor })
    }

    fn two_outputs() -> [InternalChannel; 2] {
        [
            InternalChannel::named::<f64>(COVARIANCE),
            InternalChannel::named::<f64>(MEAN),
        ]
    }

    #[test]
    fn catalog_lists_declared_leaves_then_taps_then_tick_duration() {
        let node = declaring_with_outputs(&[NIS, DROPPED], &two_outputs());
        let [covariance, mean] = two_outputs().map(ChannelKey::from);

        assert_eq!(
            observable_catalog(node.as_ref()),
            vec![
                Observable::scalar(NIS, Determinism::Deterministic),
                Observable::scalar(DROPPED, Determinism::Deterministic),
                Observable::typed(COVARIANCE, Determinism::Deterministic, &covariance),
                Observable::typed(MEAN, Determinism::Deterministic, &mean),
                Observable::scalar(TICK_DURATION_LEAF, Determinism::Nondeterministic),
            ]
        );
    }

    #[test]
    fn catalog_of_a_node_declaring_and_writing_nothing_is_tick_duration() {
        let node = declaring(&[]);

        assert_eq!(
            observable_catalog(node.as_ref()),
            vec![Observable::scalar(
                TICK_DURATION_LEAF,
                Determinism::Nondeterministic
            )]
        );
    }

    #[test]
    fn every_leaf_the_pipeline_measures_is_in_its_group() {
        let node = declaring_with_outputs(&[], &two_outputs());
        let taps: Vec<Tap> = output_taps(node.as_ref()).collect();
        let group = format!("{PIPELINE_LEAF_GROUP}.");

        let measured = observable_catalog(node.as_ref())
            .into_iter()
            .filter(|observable| !taps.iter().any(|tap| &tap.leaf == observable.leaf_name()));
        for observable in measured {
            assert!(
                observable.leaf_name().starts_with(&group),
                "{} is outside the pipeline's group",
                observable.leaf_name()
            );
        }
    }

    #[test]
    fn scheduled_node_taps_pair_each_catalog_tap_with_its_output() {
        let outputs = two_outputs();
        let scheduled = ScheduledNode::new(0, declaring_with_outputs(&[NIS], &outputs));

        assert_eq!(
            scheduled.taps,
            vec![
                Tap {
                    leaf: COVARIANCE.into(),
                    channel: outputs[0].clone().into(),
                },
                Tap {
                    leaf: MEAN.into(),
                    channel: outputs[1].clone().into(),
                },
            ]
        );
    }

    #[test]
    fn scheduled_node_buffer_holds_the_whole_catalog() {
        let mut scheduled = ScheduledNode::new(0, declaring_with_outputs(&[NIS], &two_outputs()));

        assert!(scheduled.observations.set_watched(NIS, true));
        assert!(scheduled.observations.set_watched(COVARIANCE, true));
        assert!(scheduled.observations.set_watched(MEAN, true));
        assert!(scheduled.observations.set_watched(TICK_DURATION_LEAF, true));
        assert!(!scheduled.observations.set_watched(DROPPED, true));
    }
}
