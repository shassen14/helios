//! [`EstimateRelay`]: forwards the authoritative estimator's state as the
//! agent's estimate, and publishes the edge it implies, from the estimator's
//! root (the agent's `odom`) to `base_link`.
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
use crate::channels::tf::{publish_edge, tf};
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{ChannelKey, PortBus, PortDescriptor, PortDescriptorBuilder};
use crate::stamped::Stamped;

use helios_core::estimation::frames::EstimationFrames;
use helios_core::spatial::conventions::{Enu, Flu};
use helios_core::spatial::tf::TfProvider;
use helios_core::spatial::transforms::tf::stamped::{FrameEdge, StampedTransform};
use helios_core::spatial::transforms::{Convention, ConventionOf, ErasedTransform};
use helios_core::spatial::{FrameAwareState, FrameId};

use std::sync::atomic::AtomicBool;

use tracing::warn;

/// The axes the relay reads the estimate's `odom` frame in.
type OdomAxes = Enu;

/// The axis convention of the agent's `odom` frame in the estimated tree, as
/// the relay publishes its `odom → base_link` edge. A host that tracks its
/// own `odom` frame names it in these axes.
pub const ODOM_CONVENTION: Convention = OdomAxes::CONVENTION;

/// Forwards one estimator's state onto [`estimate::estimate`] and publishes the
/// edge from its root to `base_link`.
pub(crate) struct EstimateRelay {
    name: String,
    source: ChannelKey,
    output: ChannelKey,
    edge: FrameEdge,
    descriptor: PortDescriptor,
    /// Whether a dropped edge has been warned of.
    edge_warned: AtomicBool,
}

impl EstimateRelay {
    /// A relay named `name` forwarding the state on `source`, held in
    /// `frames`, for the `frames.root → frames.body` edge.
    pub(crate) fn new(
        name: impl Into<String>,
        source: ChannelKey,
        frames: EstimationFrames,
    ) -> Self {
        let edge = FrameEdge {
            child: frames.body,
            parent: frames.root,
        };
        let output = estimate::estimate();
        let descriptor = PortDescriptorBuilder::algorithm()
            .input(source.clone())
            .output(output.clone())
            .output_keyed::<StampedTransform>(tf(), [edge.clone()])
            .build();

        Self {
            name: name.into(),
            source,
            output,
            edge,
            descriptor,
            edge_warned: AtomicBool::new(false),
        }
    }

    /// The frame the relayed estimate holds the body's pose in: the parent of
    /// the edge it publishes.
    pub(crate) fn root(&self) -> &FrameId {
        &self.edge.parent
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
            .pose::<Flu, OdomAxes>(self.edge.child.clone(), self.edge.parent.clone())
        {
            let transform = StampedTransform {
                parent: self.edge.parent.clone(),
                child: self.edge.child.clone(),
                // When the pose held. It coincides with the envelope timestamp
                // but means a different thing (pose-held vs published-at), so
                // they are kept as two fields, not merged.
                stamp: stamped.timestamp,
                transform: ErasedTransform::erase::<Flu, OdomAxes>(pose),
            };
            publish_edge(
                bus,
                Stamped {
                    value: transform,
                    timestamp: stamped.timestamp,
                    health: stamped.health.clone(),
                    producer: stamped.producer,
                },
                &self.edge_warned,
            );
        }

        if let Err(error) = bus.write(self.output.clone(), stamped) {
            warn!(channel = %self.output, error = %error, "estimate output channel write failed");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::channels::estimate::estimator_output;
    use crate::port::InstanceKey;
    use crate::stamped::Health;

    use helios_core::estimation::carrier::kinematic_carrier_schema;

    use helios_core::estimation::frames::EstimationFrames;
    use helios_core::estimation::schema::{StateSchema, StateSchemaBlock};
    use helios_core::prelude::AgentId;
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

    fn source() -> ChannelKey {
        estimator_output("primary")
    }

    fn edge() -> FrameEdge {
        FrameEdge {
            child: FrameId::base_link(agent()),
            parent: FrameId::odom(agent()),
        }
    }

    /// The edge `bus` holds for `child`, if any.
    fn published(bus: &PortBus, child: &FrameId) -> Option<Arc<Stamped<StampedTransform>>> {
        bus.read_instance::<StampedTransform>(tf(), &InstanceKey::of::<StampedTransform>(child))
    }

    /// A bus with slots for the source (as its estimator would declare it)
    /// and every output of `relay`.
    fn bus_for(relay: &EstimateRelay) -> PortBus {
        let estimator = PortDescriptor::new(vec![], vec![], vec![source()], None);
        PortBus::of_descriptors([relay.port_descriptor(), &estimator])
    }

    fn tick() -> TickContext<'static> {
        TickContext::detached(MonotonicTime(2.0), 0.1, 9)
    }

    /// A state whose schema anchors position in odom and orientation
    /// base_link → odom, so its pose resolves (to the identity).
    fn posed_state() -> FrameAwareState {
        FrameAwareState::from_schema(
            Arc::new(kinematic_carrier_schema(&EstimationFrames::odom(agent()))),
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
            source(),
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
        let relay = EstimateRelay::new("estimate_relay", source(), EstimationFrames::odom(agent()));
        let descriptor = relay.port_descriptor();

        let required: Vec<&ChannelKey> = descriptor.required_inputs().collect();
        assert_eq!(required, vec![&source()]);
        let outputs: Vec<ChannelKey> = descriptor.outputs().cloned().collect();
        assert_eq!(outputs.len(), 2);
        assert!(outputs.contains(&estimate::estimate()));
        assert!(outputs.contains(&tf()));
        let instances: Vec<&str> = descriptor
            .output_ports()
            .iter()
            .flat_map(|port| port.instances())
            .map(InstanceKey::as_str)
            .collect();
        assert_eq!(instances, ["agent.test_agent.base_link"]);
    }

    #[test]
    fn nothing_is_published_before_the_source_writes() {
        let relay = EstimateRelay::new("estimate_relay", source(), EstimationFrames::odom(agent()));
        let bus = bus_for(&relay);

        relay.execute(&bus, &NoTf, tick());

        assert!(bus.read::<FrameAwareState>(estimate::estimate()).is_none());
        assert!(published(&bus, &edge().child).is_none());
    }

    #[test]
    fn the_source_is_forwarded_unchanged_with_its_edge() {
        let relay = EstimateRelay::new("estimate_relay", source(), EstimationFrames::odom(agent()));
        let bus = bus_for(&relay);
        write_source(&bus, posed_state(), Health::Ok);

        relay.execute(&bus, &NoTf, tick());

        let estimate = bus
            .read::<FrameAwareState>(estimate::estimate())
            .expect("the estimate is forwarded");
        // The source's envelope, not the relay's tick.
        assert_eq!(estimate.timestamp, MonotonicTime(1.5));
        assert_eq!(estimate.producer, 3);
        assert_eq!(estimate.value.timestamp, MonotonicTime(1.5));

        let published = published(&bus, &edge().child).expect("the edge is published");
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
    fn the_edge_runs_from_the_root_the_estimate_is_held_in() {
        let frames = EstimationFrames {
            body: FrameId::base_link(agent()),
            root: FrameId::agent(agent(), "primary.odom"),
        };
        let relay = EstimateRelay::new("estimate_relay", source(), frames.clone());
        let bus = bus_for(&relay);
        let state = FrameAwareState::from_schema(
            Arc::new(kinematic_carrier_schema(&frames)),
            MonotonicTime(1.5),
        );
        write_source(&bus, state, Health::Ok);

        relay.execute(&bus, &NoTf, tick());

        let published =
            published(&bus, &frames.body).expect("the edge from the estimate's root is published");
        assert_eq!(published.value.parent, frames.root);
        assert_eq!(relay.root(), &frames.root);
    }

    #[test]
    fn a_degraded_source_degrades_the_estimate_and_the_edge() {
        let relay = EstimateRelay::new("estimate_relay", source(), EstimationFrames::odom(agent()));
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
            bus.read::<FrameAwareState>(estimate::estimate())
                .map(|s| s.health.clone()),
            published(&bus, &edge().child).map(|s| s.health.clone()),
        ] {
            assert!(
                matches!(&health, Some(Health::Degraded { reason: r }) if r == reason),
                "{health:?}"
            );
        }
    }

    #[test]
    fn a_poseless_state_is_forwarded_without_an_edge() {
        let relay = EstimateRelay::new("estimate_relay", source(), EstimationFrames::odom(agent()));
        let bus = bus_for(&relay);
        write_source(&bus, poseless_state(), Health::Ok);

        relay.execute(&bus, &NoTf, tick());

        assert!(bus.read::<FrameAwareState>(estimate::estimate()).is_some());
        assert!(published(&bus, &edge().child).is_none());
    }
}
