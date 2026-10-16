//! [`TfService`]: drains the tick's edge samples into the buffer and serves
//! it to consumers.

use crate::{pipeline::ReadMount, port::PortBus, ChannelKey};

use helios_core::{
    prelude::{MonotonicTime, TfProvider},
    spatial::{
        transforms::tf::{
            buffer::{TfBuffer, TfIngestError, TfWindow},
            stamped::{FrameEdge, StampedTransform},
        },
        transforms::Convention,
        FrameId,
    },
};

use std::fmt::{self, Display, Formatter};

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
    /// A service draining `drain_keys`, its tree seeded with the static edges.
    ///
    /// Fails with every seed the tree rejects (a cycle, a second parent, a
    /// convention conflict), each judged against the seeds folded before it:
    /// a wiring error in the host's static feed, which the host reports with
    /// the pipeline's build errors rather than panic on.
    pub fn new(
        window: TfWindow,
        drain_keys: Vec<ChannelKey>,
        static_seeds: Vec<StampedTransform>,
    ) -> Result<Self, Vec<SeedRejected>> {
        let mut buffer = TfBuffer::new(window);
        let mut rejected = Vec::new();
        for seed in static_seeds {
            let edge = FrameEdge {
                child: seed.child.clone(),
                parent: seed.parent.clone(),
            };
            if let Err(error) = buffer.insert_static(seed) {
                rejected.push(SeedRejected { edge, error });
            }
        }
        if !rejected.is_empty() {
            return Err(rejected);
        }
        let edges = drain_keys
            .into_iter()
            .map(|channel| DrainedEdge {
                channel,
                last_seen: MonotonicTime(0.0),
            })
            .collect();

        Ok(Self { buffer, edges })
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

    /// Checks that every frame in `mounts` sits below `base_link`, through the
    /// tree or the `declared` edges, the ones whose samples have not arrived
    /// yet, and that the tree holds it in every convention a node reads it in.
    /// Below, not merely in the same tree: a mount that meets `base_link` only
    /// through `odom` would move with the estimate. A mount reached only
    /// through a declared edge has no convention in the tree before its first
    /// sample, so only its reach is checked.
    ///
    /// Run once the host's static feed is complete, before the first tick: a
    /// mount that does not join, or is read in other axes than the tree holds,
    /// would otherwise surface only as a skip on every tick of every node
    /// reading it. Wiring only: whether a frame is in the tree and how its
    /// axes are named, never whether its edge's value is right.
    pub fn check_read_mounts(
        &self,
        mounts: &[ReadMount],
        base_link: &FrameId,
        declared: &[FrameEdge],
    ) -> Result<(), RefusedMounts> {
        let mut unreached = Vec::new();
        let mut misread = Vec::new();
        for mount in mounts {
            if !self.buffer.reaches(&mount.frame, base_link, declared) {
                unreached.push(mount.frame.clone());
                continue;
            }
            let Some(held_in) = self.buffer.convention_of(&mount.frame) else {
                continue;
            };
            for &read_in in &mount.read_in {
                if read_in != held_in {
                    misread.push(MisreadMount {
                        frame: mount.frame.clone(),
                        read_in,
                        held_in,
                    });
                }
            }
        }
        if unreached.is_empty() && misread.is_empty() {
            Ok(())
        } else {
            Err(RefusedMounts {
                base_link: base_link.clone(),
                unreached,
                misread,
            })
        }
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

/// A static seed the tree refused at [`TfService::new`].
#[derive(Debug)]
pub struct SeedRejected {
    pub edge: FrameEdge,
    pub error: TfIngestError,
}

impl Display for SeedRejected {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "tf static seed {} rejected: {:?}", self.edge, self.error)
    }
}

/// Mounts a pipeline reads that the tree refuses, found by
/// [`TfService::check_read_mounts`]: those not below `base_link`, and those a
/// node reads in other axes than the tree holds.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RefusedMounts {
    pub base_link: FrameId,
    pub unreached: Vec<FrameId>,
    pub misread: Vec<MisreadMount>,
}

impl Display for RefusedMounts {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let mut parts = Vec::new();
        if !self.unreached.is_empty() {
            let mounts: Vec<String> = self.unreached.iter().map(ToString::to_string).collect();
            parts.push(format!(
                "mounts not below {} in the estimated tf tree: {}",
                self.base_link,
                mounts.join(", ")
            ));
        }
        if !self.misread.is_empty() {
            let mounts: Vec<String> = self.misread.iter().map(ToString::to_string).collect();
            parts.push(format!(
                "mounts read in other axes than the estimated tf tree holds: {}",
                mounts.join(", ")
            ));
        }
        f.write_str(&parts.join("; "))
    }
}

/// A mount some node reads in `read_in` axes while the tree holds it in
/// `held_in`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MisreadMount {
    pub frame: FrameId,
    pub read_in: Convention,
    pub held_in: Convention,
}

impl Display for MisreadMount {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} (read as {}, held as {})",
            self.frame, self.read_in, self.held_in
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::channels::tf::tf_edge;
    use crate::port::PortDescriptor;
    use crate::stamped::{Health, Stamped};

    use helios_core::prelude::{AgentId, MonotonicDuration};
    use helios_core::spatial::id::FrameId;
    use helios_core::spatial::transforms::tf::stamped::{EdgeKindTag, FrameEdge};
    use helios_core::spatial::transforms::{Convention, ErasedTransform};

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
        FrameId::agent(agent(), name)
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
        let descriptor = PortDescriptor::new(vec![], vec![], keys, None);
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
        let mut service = TfService::new(window(), vec![key.clone()], vec![])
            .expect("the seeds form a valid tree");
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
        let mut service = TfService::new(window(), vec![key.clone()], vec![])
            .expect("the seeds form a valid tree");
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
        let mut service = TfService::new(window(), vec![bad.clone(), good.clone()], vec![])
            .expect("the seeds form a valid tree");
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
        let service =
            TfService::new(window(), vec![], vec![mount]).expect("the seeds form a valid tree");

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
        let mut service = TfService::new(window(), vec![e1.clone(), e2.clone()], vec![])
            .expect("the seeds form a valid tree");
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
        let mut service = TfService::new(window(), vec![key.clone()], vec![mount])
            .expect("the seeds form a valid tree");
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

    // --- bind ---

    // A static mount of `name` on `parent`, both FLU.
    fn seed(name: &str, parent: FrameId) -> StampedTransform {
        message(
            sensor(name),
            parent,
            Convention::Flu,
            Convention::Flu,
            0.0,
            0.1,
        )
    }

    // The mounts `names`, read by nodes that never resolve them.
    fn read(names: &[&str]) -> Vec<ReadMount> {
        names
            .iter()
            .map(|name| ReadMount {
                frame: sensor(name),
                read_in: vec![],
            })
            .collect()
    }

    // The mount `name`, read in `read_in`.
    fn read_as(name: &str, read_in: Convention) -> ReadMount {
        ReadMount {
            frame: sensor(name),
            read_in: vec![read_in],
        }
    }

    fn seeded(seeds: Vec<StampedTransform>) -> TfService {
        TfService::new(window(), vec![], seeds).expect("the seeds form a valid tree")
    }

    #[test]
    fn seeded_mounts_reach_base_link() {
        let service = seeded(vec![seed("gps", base_link()), seed("imu", base_link())]);

        assert_eq!(
            service.check_read_mounts(&read(&["gps", "imu"]), &base_link(), &[]),
            Ok(())
        );
    }

    #[test]
    fn every_unseeded_mount_is_named() {
        let service = seeded(vec![seed("gps", base_link())]);

        let refused = service.check_read_mounts(&read(&["gps", "imu", "mag"]), &base_link(), &[]);

        assert_eq!(
            refused,
            Err(RefusedMounts {
                base_link: base_link(),
                unreached: vec![sensor("imu"), sensor("mag")],
                misread: vec![],
            })
        );
    }

    #[test]
    fn a_mount_seeded_in_another_tree_does_not_reach() {
        // Seeded under the wrong parent: in the tree, but not below
        // base_link.
        let service = seeded(vec![seed("gps", sensor("rig"))]);

        assert!(service
            .check_read_mounts(&read(&["gps"]), &base_link(), &[])
            .is_err());
    }

    #[test]
    fn a_mount_reaches_through_a_declared_edge() {
        // A lidar on a pan unit whose motor edge is declared but not yet
        // sampled.
        let service = seeded(vec![seed("lidar", sensor("pan"))]);
        let motor = FrameEdge {
            child: sensor("pan"),
            parent: base_link(),
        };

        assert!(service
            .check_read_mounts(&read(&["lidar"]), &base_link(), &[])
            .is_err());
        assert_eq!(
            service.check_read_mounts(&read(&["lidar"]), &base_link(), &[motor]),
            Ok(())
        );
    }

    #[test]
    fn a_mount_read_in_the_axes_the_tree_holds_passes() {
        let service = seeded(vec![seed("imu", base_link())]);

        assert_eq!(
            service.check_read_mounts(&[read_as("imu", Convention::Flu)], &base_link(), &[]),
            Ok(())
        );
    }

    #[test]
    fn a_mount_read_in_other_axes_than_the_tree_holds_is_named() {
        // Two mounts, each reached; the one read as ENU is held as FLU.
        let service = seeded(vec![seed("gps", base_link()), seed("imu", base_link())]);
        let mounts = [
            read_as("gps", Convention::Flu),
            read_as("imu", Convention::Enu),
        ];

        let refused = service.check_read_mounts(&mounts, &base_link(), &[]);

        assert_eq!(
            refused,
            Err(RefusedMounts {
                base_link: base_link(),
                unreached: vec![],
                misread: vec![MisreadMount {
                    frame: sensor("imu"),
                    read_in: Convention::Enu,
                    held_in: Convention::Flu,
                }],
            })
        );
    }

    #[test]
    fn a_mount_reached_only_through_a_declared_edge_is_checked_for_reach_alone() {
        // A camera on a gimbal whose joint edge is declared but unsampled:
        // the tree holds no convention for the gimbal frame until then.
        let service = seeded(vec![seed("gps", base_link())]);
        let joint = FrameEdge {
            child: sensor("gimbal"),
            parent: base_link(),
        };

        assert_eq!(
            service.check_read_mounts(
                &[read_as("gimbal", Convention::Enu)],
                &base_link(),
                &[joint]
            ),
            Ok(())
        );
    }

    #[test]
    fn every_rejected_seed_is_named() {
        // Two mounts each seeded under a second parent; the valid seeds
        // around them do not hide either.
        let result = TfService::new(
            window(),
            vec![],
            vec![
                seed("gps", base_link()),
                seed("gps", sensor("rig")),
                seed("imu", base_link()),
                seed("imu", sensor("rig")),
            ],
        );

        let Err(rejected) = result else {
            panic!("a second parent must reject the seed");
        };
        let edges: Vec<FrameEdge> = rejected.into_iter().map(|r| r.edge).collect();
        assert_eq!(
            edges,
            [
                FrameEdge {
                    child: sensor("gps"),
                    parent: sensor("rig"),
                },
                FrameEdge {
                    child: sensor("imu"),
                    parent: sensor("rig"),
                },
            ]
        );
    }

    #[test]
    fn a_mount_under_odom_does_not_reach_with_the_estimate_edge_declared() {
        // In one tree with base_link once odom -> base_link is declared, but
        // not mounted on the body: its lever arm would move with the estimate.
        let misplaced = message(
            sensor("gps"),
            odom(),
            Convention::Flu,
            Convention::Enu,
            0.0,
            0.1,
        );
        let service = seeded(vec![misplaced]);
        let estimate = FrameEdge {
            child: base_link(),
            parent: odom(),
        };

        assert!(service
            .check_read_mounts(&read(&["gps"]), &base_link(), &[estimate])
            .is_err());
    }
}
