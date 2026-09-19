// Estimated-tf → aiding path, end to end (the silent-aiding-drop guard).
//
// `tf_service.rs`'s own unit tests exercise the `TfService` in isolation: a
// hand-built bus, a manual `fold`, a direct query. What they do not cover is the
// *composed* system this bug class actually lived in: the real
// `AutonomyPipeline.tick`, the host's fold-then-tick ordering, several ticks in a
// row, and a node performing its tf lookup inside `execute`. That is where a
// broken lookup turned into `None`, got swallowed by a `?`, and left the filter
// running unaided with nothing to show it had happened.
//
// Each test drives the true host loop (`drive`) and asserts on a consumer that
// stands in for a measurement model placing a reading: it counts every tick it
// was *aided* (transform resolved) versus left *unaided* (transform absent). The
// central property under guard is that the estimated tree reports a frame it was
// never fed as `None` — never a fabricated identity, and never (as the old sim
// runtime did) the physics *truth* transform — and that the `None` reaches the
// consumer where it can be seen.

#![allow(dead_code)]

use std::sync::{
    atomic::{AtomicU32, Ordering},
    Arc,
};

use helios_runtime::{
    channels::tf::publish_edge,
    pipeline::PipelineBuilder,
    port::{PortBus, PortDescriptor},
    prelude::{Health, PipelineNode, Stamped, TickContext},
    tf_service::TfService,
    AutonomyPipeline,
};

use helios_core::data::{AgentId, MonotonicDuration, MonotonicTime, TfProvider};
use helios_core::frames::id::FrameId;
use helios_core::frames::transforms::tf::buffer::TfWindow;
use helios_core::frames::transforms::tf::stamped::{FrameEdge, StampedTransform};
use helios_core::frames::transforms::{Convention, ErasedTransform};

use nalgebra::Isometry3;

// =========================================================================
// == Frames (one agent's spine) ==
// =========================================================================
// base_link is FLU; the odom root above it is ENU — the convention seam every
// edge crossing them carries, and that the composition test walks through.

fn agent() -> AgentId {
    AgentId::new("bot")
}
fn base_link() -> FrameId {
    FrameId::base_link(agent())
}
fn odom() -> FrameId {
    FrameId::odom(agent())
}
fn imu() -> FrameId {
    FrameId::sensor(agent(), "imu")
}

/// The `imu → base_link` sensor mount: time-invariant calibration, seeded into
/// the service as a static edge. A static edge answers every query with no fold,
/// so it stands in for the extrinsic whose silent absence caused the real drift.
fn imu_mount() -> StampedTransform {
    StampedTransform {
        parent: base_link(),
        child: imu(),
        // `from_parts` takes the child's convention first (imu, FLU), the
        // parent's second (base_link, FLU).
        transform: ErasedTransform::from_parts(
            Isometry3::translation(0.1, 0.0, 0.0),
            Convention::Flu,
            Convention::Flu,
        ),
        stamp: MonotonicTime(0.0),
    }
}

/// A permissive window: neither bound bites in these tests.
fn window() -> TfWindow {
    TfWindow {
        horizon: MonotonicDuration(100.0),
        max_samples: 100,
    }
}

// =========================================================================
// == Nodes ==
// =========================================================================

/// Stands in for an aiding handler / measurement model: on every tick it needs
/// `from` expressed in `to` to place its reading. A resolved transform aids the
/// filter (counts `applied`); an unresolved one leaves it unaided but *visibly*
/// so (counts `unaided`). It queries the tf snapshot, not the bus, so intra-tick
/// node order cannot change what it sees.
struct AidingConsumerNode {
    name: String,
    descriptor: PortDescriptor,
    from: FrameId,
    to: FrameId,
    applied: Arc<AtomicU32>,
    unaided: Arc<AtomicU32>,
}

impl AidingConsumerNode {
    fn new(
        name: &str,
        from: FrameId,
        to: FrameId,
        applied: Arc<AtomicU32>,
        unaided: Arc<AtomicU32>,
    ) -> Self {
        Self {
            name: name.to_string(),
            // Reads tf, not the bus: no ports to declare.
            descriptor: PortDescriptor {
                required_inputs: vec![],
                optional_inputs: vec![],
                outputs: vec![],
                rate: None,
            },
            from,
            to,
            applied,
            unaided,
        }
    }
}

impl PipelineNode for AidingConsumerNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, _bus: &PortBus, tf: &dyn TfProvider, tick: TickContext) {
        if tf
            .get_transform(self.from.clone(), self.to.clone(), tick.now)
            .is_some()
        {
            self.applied.fetch_add(1, Ordering::Relaxed);
        } else {
            self.unaided.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Stands in for an estimator/localizer that owns a transform edge: each tick it
/// dual-publishes a bare `StampedTransform` onto the edge's channel via the real
/// `publish_edge` helper, exactly as a production producer does. The inner
/// (pose-held) stamp is the tick's `now`.
struct EdgeProducerNode {
    name: String,
    descriptor: PortDescriptor,
    edge: FrameEdge,
    from_conv: Convention,
    to_conv: Convention,
}

impl EdgeProducerNode {
    fn new(name: &str, edge: FrameEdge, from_conv: Convention, to_conv: Convention) -> Self {
        Self {
            name: name.to_string(),
            // Declares the edge as an output so the pipeline allocates its slot
            // and `tf_edge_channels()` surfaces it into the drain list.
            descriptor: PortDescriptor {
                required_inputs: vec![],
                optional_inputs: vec![],
                outputs: vec![helios_runtime::channels::tf::tf_edge(&edge).into()],
                rate: None,
            },
            edge,
            from_conv,
            to_conv,
        }
    }
}

impl PipelineNode for EdgeProducerNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, bus: &PortBus, _tf: &dyn TfProvider, tick: TickContext) {
        let msg = StampedTransform {
            parent: self.edge.parent.clone(),
            child: self.edge.child.clone(),
            stamp: tick.now,
            transform: ErasedTransform::from_parts(
                Isometry3::identity(),
                self.from_conv,
                self.to_conv,
            ),
        };
        publish_edge(
            bus,
            Stamped {
                value: msg,
                timestamp: tick.now,
                health: Health::Ok,
                producer: tick.node_id,
            },
        );
    }
}

// =========================================================================
// == Host loop ==
// =========================================================================

/// One iteration of the real host loop (`brain_bridge::tick`): fold the tick's
/// fresh edge samples into the tree first, then hand every node a shared, frozen
/// view of it. The `&mut` fold followed by the `&` provider is the snapshot — the
/// borrow checker forbids a mid-tick mutation, so no node reads a half-folded
/// tree and node order cannot reach a lookup.
fn drive(pipeline: &AutonomyPipeline, service: &mut TfService, now: f64, dt: f64) {
    service.fold(pipeline.bus());
    pipeline.tick(MonotonicTime(now), dt, service.as_provider());
}

// =========================================================================
// == Tests ==
// =========================================================================

#[test]
fn present_static_extrinsic_aids_the_consumer() {
    // The happy path: a seeded static mount answers immediately, no fold and no
    // dynamic edge needed, so the consumer is aided on the very first tick.
    let applied = Arc::new(AtomicU32::new(0));
    let unaided = Arc::new(AtomicU32::new(0));
    let pipeline = PipelineBuilder::new()
        .add_node(Box::new(AidingConsumerNode::new(
            "imu_aiding",
            imu(),
            base_link(),
            applied.clone(),
            unaided.clone(),
        )))
        .build()
        .expect("single consumer builds");
    let mut service = TfService::new(window(), pipeline.tf_edge_channels(), vec![imu_mount()]);

    drive(&pipeline, &mut service, 1.0, 0.1);

    assert_eq!(applied.load(Ordering::Relaxed), 1);
    assert_eq!(unaided.load(Ordering::Relaxed), 0);
}

#[test]
fn missing_extrinsic_yields_none_never_a_fabricated_transform() {
    // The regression guard for the silent-aiding-drop class. Nothing ever feeds
    // imu → base_link (no static seed, no producer). The estimated tree must
    // report that honestly as `None` every tick — not paper over it with an
    // identity, and not fall back to the physics truth transform as the old sim
    // runtime did — so the drop stays visible at the seam.
    let applied = Arc::new(AtomicU32::new(0));
    let unaided = Arc::new(AtomicU32::new(0));
    let pipeline = PipelineBuilder::new()
        .add_node(Box::new(AidingConsumerNode::new(
            "imu_aiding",
            imu(),
            base_link(),
            applied.clone(),
            unaided.clone(),
        )))
        .build()
        .expect("single consumer builds");
    let mut service = TfService::new(window(), pipeline.tf_edge_channels(), vec![]);

    for t in 1..=3 {
        drive(&pipeline, &mut service, t as f64, 0.1);
    }

    // Zero aided passes proves the lookup never fabricated a transform; three
    // unaided passes proves each drop reached the consumer to be counted.
    assert_eq!(applied.load(Ordering::Relaxed), 0);
    assert_eq!(unaided.load(Ordering::Relaxed), 3);
}

#[test]
fn dual_published_edge_reaches_the_consumer_after_the_fold() {
    // A dynamic edge, dual-published by a producer and drained by the service,
    // reaches a consumer one tick later — the deliberate snapshot latency (fold
    // runs before the nodes, so an edge written this tick folds next tick).
    let applied = Arc::new(AtomicU32::new(0));
    let unaided = Arc::new(AtomicU32::new(0));
    let edge = FrameEdge {
        child: base_link(),
        parent: odom(),
    };
    let pipeline = PipelineBuilder::new()
        .add_node(Box::new(EdgeProducerNode::new(
            "estimator",
            edge.clone(),
            Convention::Flu,
            Convention::Enu,
        )))
        .add_node(Box::new(AidingConsumerNode::new(
            "odom_aiding",
            base_link(),
            odom(),
            applied.clone(),
            unaided.clone(),
        )))
        .build()
        .expect("producer + consumer build");
    // The drain list is derived from the graph's own tf-edge outputs, so the
    // service folds exactly what the estimator dual-publishes — no hand-authored
    // second list to drift from the graph.
    let mut service = TfService::new(window(), pipeline.tf_edge_channels(), vec![]);

    // `now` is held constant so a single dynamic sample (which answers only at
    // its exact stamp) is queryable across both ticks; the latency here is driven
    // by the fold, not by time advancing.
    //
    // Tick 1: fold runs before any node, so the edge the producer writes *this*
    // tick is not in the tree yet — the consumer sees None.
    drive(&pipeline, &mut service, 1.0, 0.1);
    assert_eq!(
        applied.load(Ordering::Relaxed),
        0,
        "edge is not folded until the next tick"
    );
    assert_eq!(unaided.load(Ordering::Relaxed), 1);

    // Tick 2: fold ingests tick 1's sample, then the consumer resolves it.
    drive(&pipeline, &mut service, 1.0, 0.1);
    assert_eq!(
        applied.load(Ordering::Relaxed),
        1,
        "edge is available the tick after it was published"
    );
    assert_eq!(unaided.load(Ordering::Relaxed), 1);
}

#[test]
fn aiding_chain_composes_static_mount_with_folded_dynamic_edge() {
    // The real measurement-placement query: a reading in the sensor frame must be
    // expressed in the estimator's world (odom) frame, which needs BOTH the
    // static mount (imu → base_link) and the filter's own dynamic edge
    // (base_link → odom). This walks the live LCA composition through the service.
    let applied = Arc::new(AtomicU32::new(0));
    let unaided = Arc::new(AtomicU32::new(0));
    let edge = FrameEdge {
        child: base_link(),
        parent: odom(),
    };
    let pipeline = PipelineBuilder::new()
        .add_node(Box::new(EdgeProducerNode::new(
            "estimator",
            edge.clone(),
            Convention::Flu,
            Convention::Enu,
        )))
        .add_node(Box::new(AidingConsumerNode::new(
            "imu_in_odom",
            imu(),
            odom(),
            applied.clone(),
            unaided.clone(),
        )))
        .build()
        .expect("producer + consumer build");
    let mut service = TfService::new(window(), pipeline.tf_edge_channels(), vec![imu_mount()]);

    // Tick 1: the static mount resolves, but base_link → odom has not folded yet,
    // so the chain to odom is broken — the static leg alone cannot reach it.
    drive(&pipeline, &mut service, 1.0, 0.1);
    assert_eq!(applied.load(Ordering::Relaxed), 0);
    assert_eq!(unaided.load(Ordering::Relaxed), 1);

    // Tick 2: the dynamic edge is folded, so the LCA walk composes
    // imu → base_link (static) with base_link → odom (dynamic) across the
    // FLU → ENU seam and the full chain resolves.
    drive(&pipeline, &mut service, 1.0, 0.1);
    assert_eq!(applied.load(Ordering::Relaxed), 1);
    assert_eq!(unaided.load(Ordering::Relaxed), 1);
}
