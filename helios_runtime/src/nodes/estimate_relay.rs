//! [`EstimateRelay`]: forwards the authoritative estimator's state as the
//! agent's estimate, and publishes the `odom → base_link` edge it implies.
//!
//! The estimate seam adds one per stack. Estimators write on channels named
//! after themselves and publish no edge, so the stack decides which one is
//! authoritative and the rest run in shadow. The relay does no math: it
//! republishes the source's `Stamped` unchanged, keeping its timestamp,
//! producer and health, so a reader's freshness check and a degraded
//! estimator's reason both reach the readers and the TF tree as the estimator
//! wrote them.
//!
//! It is not a [`Selector`](crate::nodes::combinators::Selector): a selector is
//! generic over its payload, and the edge is specific to the estimate. A
//! fallback policy that switches source on divergence can still put a
//! selector in front of the relay.

use crate::channels::estimate;
use crate::channels::tf::{publish_edge, tf_edge};
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{
    AlgorithmNodePortDescriptor, ChannelError, ChannelKey, InternalChannel, PortBus, PortDescriptor,
};
use crate::stamped::Stamped;

use helios_core::prelude::AgentId;
use helios_core::spatial::conventions::{Enu, Flu};
use helios_core::spatial::tf::TfProvider;
use helios_core::spatial::transforms::tf::stamped::{FrameEdge, StampedTransform};
use helios_core::spatial::transforms::ErasedTransform;
use helios_core::spatial::{FrameAwareState, FrameId};

use tracing::warn;

/// Forwards one estimator's state onto [`estimate::estimate`] and publishes the
/// agent's `odom → base_link` edge from it.
pub(crate) struct EstimateRelay {
    name: String,
    source: ChannelKey,
    output: ChannelKey,
    edge: FrameEdge,
    descriptor: PortDescriptor,
}

impl EstimateRelay {
    /// A relay named `name` forwarding the state on `source`, for `agent`'s
    /// `odom → base_link` edge.
    pub(crate) fn new(name: impl Into<String>, source: InternalChannel, agent: &AgentId) -> Self {
        let edge = FrameEdge {
            child: FrameId::base_link(agent.clone()),
            parent: FrameId::odom(agent.clone()),
        };
        let output = estimate::estimate();
        let descriptor = AlgorithmNodePortDescriptor::new()
            .input_internal(source.clone())
            .output_internal(output.clone())
            .output_internal(tf_edge(&edge))
            .build();

        Self {
            name: name.into(),
            source: source.into(),
            output: output.into(),
            edge,
            descriptor,
        }
    }
}

impl PipelineNode for EstimateRelay {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    /// Republishes the source's latest state and the edge it implies. Nothing
    /// is published before the source's first write (cold start).
    ///
    /// The edge is a pure read of the state's orientation and reference-frame
    /// position, stamped when the state holds. If either block is absent (a
    /// schema not yet seeded with a pose), the edge is skipped rather than
    /// feeding the TF buffer a bogus identity; the state still goes out.
    fn execute(&self, bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {
        let Some(source) = bus.read::<FrameAwareState>(self.source.clone()) else {
            return;
        };
        let stamped = (*source).clone();

        if let Some(pose) = stamped
            .value
            .pose::<Flu, Enu>(self.edge.child.clone(), self.edge.parent.clone())
        {
            let transform = StampedTransform {
                parent: self.edge.parent.clone(),
                child: self.edge.child.clone(),
                // When the pose held. It coincides with the envelope timestamp
                // but means a different thing (pose-held vs published-at), so
                // they are kept as two fields, not merged.
                stamp: stamped.timestamp,
                transform: ErasedTransform::erase::<Flu, Enu>(pose),
            };
            publish_edge(
                bus,
                Stamped {
                    value: transform,
                    timestamp: stamped.timestamp,
                    health: stamped.health.clone(),
                    producer: stamped.producer,
                },
            );
        }

        if let Err(ChannelError::UnknownChannel) = bus.write(self.output.clone(), stamped) {
            warn!(channel = %self.output, "estimate output channel is not wired into the DAG");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::channels::estimate::estimator_output;
    use crate::stamped::Health;

    use helios_core::estimation::carrier::kinematic_carrier_schema;
    use helios_core::estimation::schema::{StateSchema, StateSchemaBlock};
    use helios_core::spatial::primitives::MonotonicTime;
    use helios_core::spatial::state::Quantity;
    use helios_core::spatial::transforms::Convention;
    use nalgebra::{DMatrix, DVector, Isometry3};
    use std::borrow::Cow;
    use std::sync::Arc;

    struct NoTf;

    impl TfProvider for NoTf {
        fn get_transform(
            &self,
            _: FrameId,
            _: FrameId,
            _: MonotonicTime,
        ) -> Option<ErasedTransform> {
            None
        }
    }

    fn agent() -> AgentId {
        AgentId::new("test_agent")
    }

    fn source() -> InternalChannel {
        estimator_output("primary")
    }

    fn edge() -> FrameEdge {
        FrameEdge {
            child: FrameId::base_link(agent()),
            parent: FrameId::odom(agent()),
        }
    }

    /// A bus with slots for the source (as its estimator would declare it)
    /// and every output of `relay`.
    fn bus_for(relay: &EstimateRelay) -> PortBus {
        let estimator = PortDescriptor::new(vec![], vec![], vec![source().into()], None);
        PortBus::new([relay.port_descriptor(), &estimator])
    }

    fn tick() -> TickContext<'static> {
        TickContext::detached(MonotonicTime(2.0), 0.1, 9)
    }

    /// A state whose schema anchors position in odom and orientation
    /// base_link → odom, so its pose resolves (to the identity).
    fn posed_state() -> FrameAwareState {
        FrameAwareState::from_schema(
            Arc::new(kinematic_carrier_schema(agent())),
            MonotonicTime(1.5),
        )
    }

    /// A state with neither orientation nor an odom position: the shape before
    /// a filter is seeded with a pose.
    fn poseless_state() -> FrameAwareState {
        let schema = StateSchema::compose(vec![StateSchemaBlock::new(
            Quantity::Velocity(FrameId::odom(agent())),
            Convention::Enu,
            None,
            DVector::zeros(3),
            DMatrix::zeros(3, 3),
        )]);
        FrameAwareState::from_schema(Arc::new(schema), MonotonicTime(1.5))
    }

    fn write_source(bus: &PortBus, state: FrameAwareState, health: Health) {
        bus.write(
            source().into(),
            Stamped {
                value: state,
                timestamp: MonotonicTime(1.5),
                health,
                producer: 3,
            },
        )
        .expect("the source slot exists");
    }

    #[test]
    fn descriptor_reads_the_source_and_writes_the_estimate_and_the_edge() {
        let relay = EstimateRelay::new("estimate_relay", source(), &agent());
        let descriptor = relay.port_descriptor();

        let required: Vec<&ChannelKey> = descriptor.required_inputs().collect();
        assert_eq!(required, vec![&ChannelKey::from(source())]);
        let outputs = descriptor.outputs();
        assert_eq!(outputs.len(), 2);
        assert!(outputs.contains(&estimate::estimate().into()));
        assert!(outputs.contains(&tf_edge(&edge()).into()));
    }

    #[test]
    fn nothing_is_published_before_the_source_writes() {
        let relay = EstimateRelay::new("estimate_relay", source(), &agent());
        let bus = bus_for(&relay);

        relay.execute(&bus, &NoTf, tick());

        assert!(bus
            .read::<FrameAwareState>(estimate::estimate().into())
            .is_none());
        assert!(bus
            .read::<StampedTransform>(tf_edge(&edge()).into())
            .is_none());
    }

    #[test]
    fn the_source_is_forwarded_unchanged_with_its_edge() {
        let relay = EstimateRelay::new("estimate_relay", source(), &agent());
        let bus = bus_for(&relay);
        write_source(&bus, posed_state(), Health::Ok);

        relay.execute(&bus, &NoTf, tick());

        let estimate = bus
            .read::<FrameAwareState>(estimate::estimate().into())
            .expect("the estimate is forwarded");
        // The source's envelope, not the relay's tick.
        assert_eq!(estimate.timestamp, MonotonicTime(1.5));
        assert_eq!(estimate.producer, 3);
        assert_eq!(estimate.value.timestamp, MonotonicTime(1.5));

        let published = bus
            .read::<StampedTransform>(tf_edge(&edge()).into())
            .expect("the edge is published");
        assert_eq!(published.value.parent, edge().parent);
        assert_eq!(published.value.child, edge().child);
        assert_eq!(published.value.stamp, MonotonicTime(1.5));
        assert_eq!(published.timestamp, MonotonicTime(1.5));
        assert_eq!(published.producer, 3);
        let typed = published
            .value
            .transform
            .typed::<Flu, Enu>()
            .expect("the edge carries a Flu → Enu transform");
        assert_eq!(typed.into_inner(), Isometry3::identity());
    }

    #[test]
    fn a_degraded_source_degrades_the_estimate_and_the_edge() {
        let relay = EstimateRelay::new("estimate_relay", source(), &agent());
        let bus = bus_for(&relay);
        let reason = "predict skipped";
        write_source(
            &bus,
            posed_state(),
            Health::Degraded {
                reason: Cow::Borrowed(reason),
            },
        );

        relay.execute(&bus, &NoTf, tick());

        for health in [
            bus.read::<FrameAwareState>(estimate::estimate().into())
                .map(|s| s.health.clone()),
            bus.read::<StampedTransform>(tf_edge(&edge()).into())
                .map(|s| s.health.clone()),
        ] {
            assert!(
                matches!(&health, Some(Health::Degraded { reason: r }) if r == reason),
                "{health:?}"
            );
        }
    }

    #[test]
    fn a_poseless_state_is_forwarded_without_an_edge() {
        let relay = EstimateRelay::new("estimate_relay", source(), &agent());
        let bus = bus_for(&relay);
        write_source(&bus, poseless_state(), Health::Ok);

        relay.execute(&bus, &NoTf, tick());

        assert!(bus
            .read::<FrameAwareState>(estimate::estimate().into())
            .is_some());
        assert!(bus
            .read::<StampedTransform>(tf_edge(&edge()).into())
            .is_none());
    }
}
