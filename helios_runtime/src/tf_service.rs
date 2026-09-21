//! The runtime glue that turns transform-edge bus traffic into a queryable tree.
//!
//! [`TfBuffer`] lives in `helios_core` and is deliberately clock-free and
//! bus-free — it knows how to compose and interpolate edges, nothing about where
//! samples come from. [`TfService`] is the missing half: each tick it drains the
//! per-edge channels (the vocabulary in [`crate::channels::tf`]) into that buffer,
//! then hands the buffer out as a read-only [`TfProvider`] for consumers to query.
//! The drain is driven by an *explicit* list of edge keys, not a "read every
//! transform" scan — the bus has no such primitive, and each edge is its own
//! last-known-good slot.

use crate::{port::PortBus, ChannelKey};

use helios_core::{
    data::{MonotonicTime, TfProvider},
    frames::transforms::tf::{
        buffer::{TfBuffer, TfWindow},
        stamped::StampedTransform,
    },
};

/// Owns the transform buffer and the set of edges it drains into it.
///
/// Lifecycle over a tick is two-phase and borrow-enforced: `fold(&mut self)`
/// ingests the tick's fresh samples, then `as_provider(&self)` yields a shared
/// borrow for every query that follows. The `&mut` → `&` split makes it a
/// compile error to fold mid-query, so a consumer never reads a half-updated tree.
pub struct TfService {
    /// The composed tree. The service is its sole writer; callers only ever see
    /// it through [`TfService::as_provider`], never the concrete type.
    buffer: TfBuffer,
    /// The edges to drain, one per bus slot, each carrying its own dedup cursor.
    edges: Vec<DrainedEdge>,
}

impl TfService {
    pub fn new(
        window: TfWindow,
        drain_keys: Vec<ChannelKey>,
        static_seeds: Vec<StampedTransform>,
    ) -> Self {
        let mut buffer = TfBuffer::new(window);
        for tf in static_seeds {
            // A static seed that fails to ingest is a config/wiring error caught
            // at assembly, not a runtime fault — so a startup panic is correct
            // here. The error's Debug names the offending frame(s) and the
            // invariant they broke (cycle, multiple parents, convention conflict).
            buffer
                .insert_static(tf)
                .unwrap_or_else(|e| panic!("tf static seed rejected at assembly: {e:?}"));
        }
        let edges = drain_keys
            .into_iter()
            .map(|channel| DrainedEdge {
                channel,
                last_seen: MonotonicTime(0.0),
            })
            .collect();

        Self { buffer, edges }
    }

    /// The tree as a read-only [`TfProvider`], for consumers to query this tick.
    ///
    /// Erased to the trait on purpose: callers depend on the query contract, not
    /// on the buffer being a `TfBuffer`. This is the shared-borrow half of the
    /// tick — see the type-level note on the fold/query ordering.
    pub fn as_provider(&self) -> &dyn TfProvider {
        &self.buffer
    }

    /// The concrete buffer, for the viz to enumerate the tree's topology.
    ///
    /// A read-only convenience off the pipeline path, and the one accessor that
    /// widens the surface past the `TfProvider` firewall. The autonomy pipeline
    /// still queries exclusively through [`as_provider`](Self::as_provider), so
    /// it can never satisfy an estimated lookup from anything but the erased
    /// query contract. This wider borrow exists only so an out-of-band reader —
    /// the tf overlay — can walk the tree's shape via [`TfBuffer::edges`];
    /// nothing on the pipeline path calls it.
    pub fn buffer(&self) -> &TfBuffer {
        &self.buffer
    }

    /// Drain every edge's freshest sample into the buffer, once.
    pub fn fold(&mut self, bus: &PortBus) {
        // Split the mutable borrow of `self` into its two fields up front:
        // the loop needs `edges` mutably (to advance each cursor) *and* `buffer`
        // mutably (to ingest) at the same time. Going through `self.edges` and
        // `self.buffer` inside the loop would be two overlapping borrows of one
        // `&mut self`; destructuring hands out an independent borrow of each.
        let Self { buffer, edges } = self;

        for edge in edges.iter_mut() {
            let Some(stamped) = bus.read::<StampedTransform>(edge.channel.clone()) else {
                continue;
            };

            // Two clocks live on one sample and must not be collapsed. The
            // envelope `timestamp` (published-at) is the dedup key: it advances
            // once per producer write, so it tells us whether this is a sample we
            // have already folded. The inner `value.stamp` (pose-held time) is
            // the buffer's business, not ours. Gate on the envelope alone.
            if stamped.timestamp <= edge.last_seen {
                continue;
            }
            edge.last_seen = stamped.timestamp;

            // A dynamic sample rejected mid-run is a runtime fault, not an
            // assembly one, so warn and keep draining the rest — never panic and
            // never abort the loop (contrast the startup panic in `new`). The
            // channel names the culprit; the Debug carries the broken invariant.
            if let Err(e) = buffer.insert_dynamic(stamped.value.clone()) {
                tracing::warn!(channel = %edge.channel, "tf edge sample rejected on ingest: {e:?}");
            }
        }
    }
}

/// One drained edge: the bus slot to read, plus the envelope time last folded
/// from it, so a slot that has not been rewritten since is skipped.
pub struct DrainedEdge {
    channel: ChannelKey,
    last_seen: MonotonicTime,
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::channels::tf::tf_edge;
    use crate::port::PortDescriptor;
    use crate::stamped::{Health, Stamped};

    use helios_core::data::{AgentId, MonotonicDuration};
    use helios_core::frames::id::FrameId;
    use helios_core::frames::transforms::tf::stamped::{EdgeKindTag, FrameEdge};
    use helios_core::frames::transforms::{Convention, ErasedTransform};

    use nalgebra::Isometry3;

    // --- frames ---
    // base_link is FLU; the odom/map roots above it are ENU — the convention
    // split every edge that crosses them must carry.

    fn agent() -> AgentId {
        AgentId::new("bot")
    }
    fn base_link() -> FrameId {
        FrameId::base_link(agent())
    }
    fn odom() -> FrameId {
        FrameId::odom(agent())
    }
    fn map() -> FrameId {
        FrameId::map(agent())
    }
    fn sensor(name: &str) -> FrameId {
        FrameId::sensor(agent(), name)
    }

    // A permissive window: neither bound bites unless a test asks it to.
    fn window() -> TfWindow {
        TfWindow {
            horizon: MonotonicDuration(100.0),
            max_samples: 100,
        }
    }

    fn edge_key(child: FrameId, parent: FrameId) -> ChannelKey {
        tf_edge(&FrameEdge { child, parent }).into()
    }

    // One edge message: `child` expressed in `parent` at inner time `stamp`, a
    // pure +x translation so samples are told apart by one number.
    fn message(
        child: FrameId,
        parent: FrameId,
        from: Convention,
        to: Convention,
        stamp: f64,
        x: f64,
    ) -> StampedTransform {
        StampedTransform {
            parent,
            child,
            stamp: MonotonicTime(stamp),
            transform: ErasedTransform::from_parts(Isometry3::translation(x, 0.0, 0.0), from, to),
        }
    }

    fn bl_odom(stamp: f64, x: f64) -> StampedTransform {
        message(
            base_link(),
            odom(),
            Convention::Flu,
            Convention::Enu,
            stamp,
            x,
        )
    }
    fn odom_map(stamp: f64, x: f64) -> StampedTransform {
        message(odom(), map(), Convention::Enu, Convention::Enu, stamp, x)
    }

    // A bus with a slot for each edge channel — the minimum a producer needs to
    // publish onto, standing in for the pipeline's descriptor-driven allocation.
    fn edge_bus(keys: Vec<ChannelKey>) -> PortBus {
        let descriptor = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: keys,
            rate: None,
        };
        PortBus::new(&[descriptor])
    }

    // Dual-publish one edge message onto its channel, stamping the *envelope*
    // (published-at) clock separately from the message's own inner `stamp`.
    fn publish(bus: &PortBus, tf: StampedTransform, envelope: f64) {
        let key = edge_key(tf.child.clone(), tf.parent.clone());
        bus.write(
            key,
            Stamped {
                value: tf,
                timestamp: MonotonicTime(envelope),
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("edge slot exists");
    }

    #[test]
    fn folded_edge_is_queryable() {
        let key = edge_key(base_link(), odom());
        let mut service = TfService::new(window(), vec![key.clone()], vec![]);
        let bus = edge_bus(vec![key]);
        publish(&bus, bl_odom(1.0, 2.0), 1.0);

        service.fold(&bus);

        // The drained edge answers a query at its own stamp.
        let tf = service
            .as_provider()
            .get_transform(base_link(), odom(), MonotonicTime(1.0));
        assert!(tf.is_some());
    }

    #[test]
    fn fold_skips_when_envelope_has_not_advanced() {
        let key = edge_key(base_link(), odom());
        let mut service = TfService::new(window(), vec![key.clone()], vec![]);
        let bus = edge_bus(vec![key]);

        // A sample stamped (inner) at t=1, envelope 1.
        publish(&bus, bl_odom(1.0, 2.0), 1.0);
        service.fold(&bus);

        // The producer emits a newer sample (inner t=2) but its envelope clock
        // did not advance. The dedup gate keys off the envelope, so this second
        // sample must be ignored — proving the two clocks are read distinctly.
        publish(&bus, bl_odom(2.0, 9.0), 1.0);
        service.fold(&bus);

        // The t=2 sample never folded: nothing answers a query at t=2 ...
        assert!(service
            .as_provider()
            .get_transform(base_link(), odom(), MonotonicTime(2.0))
            .is_none());
        // ... while the originally folded t=1 sample is still present.
        assert!(service
            .as_provider()
            .get_transform(base_link(), odom(), MonotonicTime(1.0))
            .is_some());
    }

    #[test]
    fn fold_survives_a_rejected_edge_and_keeps_going() {
        // A self-edge (child == parent) is a cycle the buffer rejects on ingest.
        let bad = edge_key(base_link(), base_link());
        let good = edge_key(base_link(), odom());
        // `bad` is drained first, so the error precedes the good edge — the loop
        // must warn-and-continue, not panic or abort.
        let mut service = TfService::new(window(), vec![bad.clone(), good.clone()], vec![]);
        let bus = edge_bus(vec![bad, good]);

        publish(
            &bus,
            message(
                base_link(),
                base_link(),
                Convention::Flu,
                Convention::Flu,
                1.0,
                0.0,
            ),
            1.0,
        );
        publish(&bus, bl_odom(1.0, 2.0), 1.0);

        service.fold(&bus);

        // The good edge folded despite the rejected one ahead of it.
        assert!(service
            .as_provider()
            .get_transform(base_link(), odom(), MonotonicTime(1.0))
            .is_some());
    }

    #[test]
    fn static_seed_is_queryable_without_folding() {
        // A sensor mount is time-invariant calibration seeded at construction.
        let mount = message(
            sensor("lidar"),
            base_link(),
            Convention::Flu,
            Convention::Flu,
            0.0,
            0.1,
        );
        let service = TfService::new(window(), vec![], vec![mount]);

        // No fold: a static edge answers every time straight from the seed.
        let tf =
            service
                .as_provider()
                .get_transform(sensor("lidar"), base_link(), MonotonicTime(5.0));
        assert!(tf.is_some());
    }

    #[test]
    fn fold_composes_a_two_edge_chain() {
        let e1 = edge_key(base_link(), odom());
        let e2 = edge_key(odom(), map());
        let mut service = TfService::new(window(), vec![e1.clone(), e2.clone()], vec![]);
        let bus = edge_bus(vec![e1, e2]);
        publish(&bus, bl_odom(1.0, 2.0), 1.0);
        publish(&bus, odom_map(1.0, 3.0), 1.0);

        service.fold(&bus);

        // base_link → map walks both drained edges and composes across the
        // FLU→ENU seam between them.
        let tf = service
            .as_provider()
            .get_transform(base_link(), map(), MonotonicTime(1.0));
        assert!(tf.is_some());
    }

    #[test]
    fn buffer_enumerates_seeded_and_folded_edges() {
        // The viz reaches topology through `buffer().edges()`. A static mount
        // seeded at construction and a dynamic edge folded from the bus must both
        // appear, each tagged by kind — the estimated tree's shape as the overlay
        // sees it, off the `as_provider` lookup path.
        let mount = message(
            sensor("lidar"),
            base_link(),
            Convention::Flu,
            Convention::Flu,
            0.0,
            0.1,
        );
        let key = edge_key(base_link(), odom());
        let mut service = TfService::new(window(), vec![key.clone()], vec![mount]);
        let bus = edge_bus(vec![key]);
        publish(&bus, bl_odom(1.0, 2.0), 1.0);

        service.fold(&bus);

        let edges = service.buffer().edges();
        assert_eq!(edges.len(), 2, "the seeded mount and the folded edge");

        // Order is unspecified (HashMap walk), so assert by membership.
        let has = |child: FrameId, parent: FrameId, tag: EdgeKindTag| {
            edges
                .iter()
                .any(|(e, k)| e.child == child && e.parent == parent && *k == tag)
        };
        assert!(has(sensor("lidar"), base_link(), EdgeKindTag::Static));
        assert!(has(base_link(), odom(), EdgeKindTag::Dynamic));
    }
}
