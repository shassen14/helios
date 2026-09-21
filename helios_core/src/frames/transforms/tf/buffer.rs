//! The buffer itself: a clock-free fold of transform messages into one tree, and
//! the pure lookup that reads a composed transform back out at a queried time.
//!
//! [`TfBuffer`] holds the entire tree for the whole system at once — every frame,
//! every edge — not one edge's history. A producer folds edges in through
//! [`insert_static`](TfBuffer::insert_static) /
//! [`insert_dynamic`](TfBuffer::insert_dynamic); a consumer reads a composed
//! transform out through [`lookup`](TfBuffer::lookup). The buffer holds no clock
//! and no interior mutability, so a lookup is a pure function of the folded state
//! plus the query time.
//!
//! [`TfWindow`] is the bounded-history policy, [`TfQuery`] selects how a lookup
//! reads time, and [`TfIngestError`] / [`TfLookupError`] are the disjoint failure
//! sets of the two directions. The message and per-edge vocabulary it stores live
//! in `stamped.rs`.

use crate::{
    data::{MonotonicDuration, MonotonicTime, TfProvider},
    frames::{
        transforms::{
            tf::stamped::{EdgeKind, EdgeKindTag, FrameEdge, StampedTransform, TimeSpan},
            Convention, ErasedTransform,
        },
        FrameId,
    },
};

use nalgebra::{Isometry3, Translation3};
use std::collections::{HashMap, HashSet, VecDeque};

/// The transform tree for the whole system, folded from timestamped edge messages.
///
/// Not a buffer of one edge's samples — the per-edge history lives *inside* each
/// entry (an [`EdgeKind::Dynamic`]'s sample deque). This is the tree itself, plus
/// the ledger of each frame's convention and the eviction policy every edge shares.
pub struct TfBuffer {
    /// The tree: each child maps to its single parent and that edge's pose(s).
    /// Keyed by child because the single-parent rule makes the child a unique key,
    /// which is what turns the multiple-parents check into an O(1) lookup and lets
    /// the walk climb toward the root one parent at a time.
    child_to_parent: HashMap<FrameId, (FrameId, EdgeKind)>,
    /// Each frame's axis convention, established by its first sighting and required
    /// to agree on every later edge that touches it. A convention belongs to a
    /// frame, not an edge, so it is keyed by frame — a frame that is a parent on
    /// one edge and a child on another carries the same convention in both.
    conventions: HashMap<FrameId, Convention>,
    /// The bounded-history policy, applied uniformly to every dynamic edge and
    /// consulted on each insert to evict samples that have aged out.
    window: TfWindow,
}

impl TfProvider for TfBuffer {
    fn get_transform(
        &self,
        from: FrameId,
        to: FrameId,
        at: MonotonicTime,
    ) -> Option<ErasedTransform> {
        self.lookup(&from, &to, TfQuery::At(at)).ok()
    }
}

impl TfBuffer {
    /// An empty tree that remembers its eviction policy. Edges arrive later through
    /// the two `insert_*` methods; the window is retained so eviction can consult
    /// it on every dynamic insert.
    pub fn new(window: TfWindow) -> Self {
        Self {
            child_to_parent: HashMap::default(),
            conventions: HashMap::default(),
            window,
        }
    }

    /// Folds in a time-invariant edge (a mount, an extrinsic) — one isometry that
    /// answers every query time. Idempotent when the same value is re-inserted;
    /// rejects a *different* value on the same edge as a [`ConflictingSample`],
    /// since runtime recalibration is not a supported operation.
    pub fn insert_static(&mut self, tf: StampedTransform) -> Result<(), TfIngestError> {
        let iso = tf.transform.isometry();
        let incoming = EdgeKind::Static(iso);
        self.validate(&tf, EdgeKindTag::Static)?;

        if let Some((_, EdgeKind::Static(existing))) = self.child_to_parent.get(&tf.child) {
            if *existing != iso {
                return Err(TfIngestError::ConflictingSample {
                    edge: FrameEdge {
                        child: tf.child.clone(),
                        parent: tf.parent.clone(),
                    },
                    stamp: tf.stamp,
                });
            }
            return Ok(()); // same value → idempotent, nothing to do
        }

        self.conventions
            .entry(tf.child.clone())
            .or_insert(tf.transform.from_convention());

        self.conventions
            .entry(tf.parent.clone())
            .or_insert(tf.transform.to_convention());

        self.child_to_parent.insert(tf.child, (tf.parent, incoming));

        Ok(())
    }

    /// Folds in one timestamped sample of a time-varying edge (odometry, a
    /// localizer correction). A new edge starts a fresh history; an existing edge
    /// accumulates the sample into its sorted deque and then evicts anything the
    /// [`window`](TfBuffer::window) no longer keeps. A duplicate stamp with a
    /// different value is a [`ConflictingSample`]; a sample already older than the
    /// retained window is dropped as a no-op.
    pub fn insert_dynamic(&mut self, tf: StampedTransform) -> Result<(), TfIngestError> {
        let iso = tf.transform.isometry();
        self.validate(&tf, EdgeKindTag::Dynamic)?;

        self.conventions
            .entry(tf.child.clone())
            .or_insert(tf.transform.from_convention());

        self.conventions
            .entry(tf.parent.clone())
            .or_insert(tf.transform.to_convention());

        // Copied out before the mutable borrow of the edge below (TfWindow is Copy).
        let horizon = self.window.horizon;
        let max_samples = self.window.max_samples;

        match self.child_to_parent.get_mut(&tf.child) {
            Some((_, EdgeKind::Dynamic(samples))) => {
                // Eviction is relative to the newest sample, never a wall clock —
                // the incoming sample may itself be the new newest.
                let current_newest = samples.back().map(|(s, _)| *s).unwrap_or(tf.stamp);
                let newest = if tf.stamp.0 > current_newest.0 {
                    tf.stamp
                } else {
                    current_newest
                };
                let cutoff = newest - horizon;

                // A sample that arrives already older than the window keeps is a
                // no-op — inserting it would only evict it again.
                if tf.stamp.0 < cutoff.0 {
                    return Ok(());
                }

                // Locate the stamp in the ascending deque. Found → the stamp already
                // has a sample: identical value is idempotent, a different value is a
                // contradiction. Absent → insert at the sorted position (a push_back
                // for in-order arrival, a rare O(n) middle insert for a late one).
                match samples.binary_search_by(|(s, _)| s.0.total_cmp(&tf.stamp.0)) {
                    Ok(i) => {
                        if samples[i].1 != iso {
                            return Err(TfIngestError::ConflictingSample {
                                edge: FrameEdge {
                                    child: tf.child.clone(),
                                    parent: tf.parent.clone(),
                                },
                                stamp: tf.stamp,
                            });
                        }
                    }
                    Err(i) => {
                        samples.insert(i, (tf.stamp, iso));
                    }
                }

                // Trim by time first (drop everything past the horizon), then the
                // memory backstop. The newest sample always survives both.
                while samples.front().is_some_and(|(s, _)| s.0 < cutoff.0) {
                    samples.pop_front();
                }
                while samples.len() > max_samples {
                    samples.pop_front();
                }
            }
            // Unreachable: validate's kind check already rejected a static edge
            // here. Asserted in debug, a silent no-op in release — never a panic.
            Some(_) => debug_assert!(false, "validate guarantees an existing edge is Dynamic"),
            None => {
                self.child_to_parent.insert(
                    tf.child,
                    (
                        tf.parent,
                        EdgeKind::Dynamic(VecDeque::from([(tf.stamp, iso)])),
                    ),
                );
            }
        }

        Ok(())
    }

    /// Every edge in the tree paired with its kind, for topology display.
    ///
    /// A viz-only convenience off the lookup path: it hands back the tree's
    /// *shape* — which frames connect and whether each edge is static or
    /// dynamic — and nothing else. Poses are deliberately not returned; a caller
    /// fetches those through [`lookup`](Self::lookup) with [`TfQuery::Latest`],
    /// keeping "enumeration is topology, lookup is pose" a clean split and never
    /// leaking the stored isometries or sample history. The kind is collapsed to
    /// its [`EdgeKindTag`] for that same reason — the caller only needs to know
    /// static-vs-dynamic to route the pose it fetches separately.
    ///
    /// Order is unspecified: it walks a `HashMap`, so iteration order is not
    /// stable across runs. A caller that needs a deterministic layout sorts the
    /// result itself.
    pub fn edges(&self) -> Vec<(FrameEdge, EdgeKindTag)> {
        self.child_to_parent
            .iter()
            .map(|(child, (parent, kind))| {
                (
                    FrameEdge {
                        child: child.clone(),
                        parent: parent.clone(),
                    },
                    kind.tag(),
                )
            })
            .collect()
    }

    /// The transform from `from` to `to` at the queried time, composed across the
    /// tree the folded edges describe.
    ///
    /// The strategy is lowest common ancestor. In a single-parent tree each frame
    /// has one path to its root, so two endpoints' paths meet at exactly one frame
    /// — the LCA — and the answer is `from → LCA` then the reverse of `to → LCA`.
    ///
    /// Finding the LCA and *evaluating* the paths are two separate phases, and the
    /// separation is load-bearing. [`lowest_common_ancestor`](TfBuffer::lowest_common_ancestor)
    /// walks `child_to_parent` **keys only** — it samples no edge — so an edge
    /// that cannot answer the query (an `At` past a dynamic edge's history) never
    /// enters the *search* for the meeting point. Only then does
    /// [`accumulate`](TfBuffer::accumulate) sample the edges on each side, and it
    /// stops at the LCA: an edge *above* the meeting point is never touched. This
    /// is what lets a purely static lookup (a sensor mount, `base_link → cam`)
    /// resolve even while the dynamic `base_link → odom` edge above it is momentarily
    /// unqueryable — the mount's answer does not cross that edge, so its time
    /// coverage is irrelevant. (An earlier version climbed every chain to the root
    /// before locating the LCA, so any out-of-range edge anywhere above either
    /// endpoint poisoned the whole lookup.)
    ///
    /// The failure cases are disjoint by cause. An endpoint the tree never saw is
    /// [`UnknownFrame`](TfLookupError::UnknownFrame); two endpoints whose chains
    /// never meet live in separate trees and are
    /// [`Disconnected`](TfLookupError::Disconnected); an `At` query outside the
    /// retained history of an edge *the answer actually crosses* surfaces from
    /// `accumulate` as [`OutOfTimeRange`](TfLookupError::OutOfTimeRange),
    /// propagated unchanged. The `from == to` case is identity, answered before
    /// any walk.
    pub fn lookup(
        &self,
        from: &FrameId,
        to: &FrameId,
        query: TfQuery,
    ) -> Result<ErasedTransform, TfLookupError> {
        // Both endpoints must be frames the tree has seen; the convention lookup
        // doubles as that existence check, and pins each end's convention for the
        // identity case below.
        let Some(from_conv) = self.conventions.get(from) else {
            return Err(TfLookupError::UnknownFrame(from.clone()));
        };

        let Some(to_conv) = self.conventions.get(to) else {
            return Err(TfLookupError::UnknownFrame(to.clone()));
        };

        // A frame to itself is identity, tagged with its own convention on both
        // ends. Answered directly so it holds even for a lone frame with no edges.
        if from == to {
            return Ok(ErasedTransform::from_parts(
                Isometry3::identity(),
                *from_conv,
                *to_conv,
            ));
        }

        // Phase 1 — find the meeting point structurally, sampling nothing. Chains
        // that never share a frame belong to separate trees: `Disconnected`.
        let lca =
            self.lowest_common_ancestor(from, to)
                .ok_or_else(|| TfLookupError::Disconnected {
                    from: from.clone(),
                    to: to.clone(),
                })?;

        // Phase 2 — sample only the edges each side crosses to reach the LCA, and
        // no further. `OutOfTimeRange` can surface here, but only for an edge on
        // the `from → LCA` / `to → LCA` path — never one above it.
        let from_to_lca = self.accumulate(from, &lca, query)?;
        let to_to_lca = self.accumulate(to, &lca, query)?;

        // `from → LCA` then the reverse of `to → LCA` (= `LCA → to`) gives
        // `from → to`. The shared LCA convention is the seam `then` checks.
        Ok(from_to_lca.then(to_to_lca.inverse()))
    }

    /// The shared structural validation both inserts run before storing anything —
    /// every invariant the tree relies on is checked here, once, so the lookup walk
    /// can assume a single-parent, acyclic, convention-consistent tree.
    ///
    /// The checks run in a fixed order, each mapping to exactly one error:
    /// 1. a self-edge (`child == parent`) is a degenerate cycle;
    /// 2. each endpoint's convention tag must match what that frame's first sighting
    ///    established (an unseen frame is *established* on commit, not here);
    /// 3. a child already bound to a *different* parent violates single-parent —
    ///    the *same* parent is just a new sample of an existing edge, and passes;
    /// 4. an edge whose parent is already a descendant of the child would close a
    ///    deeper loop, caught by walking parent-ward to the root;
    /// 5. re-sampling an existing edge with the other kind (static vs dynamic)
    ///    contradicts the kind its first insert declared.
    ///
    /// Purely inspective: it mutates nothing, so a rejected insert leaves no
    /// half-established frame behind. Establishing conventions and storing the edge
    /// happen in the caller, only after this returns `Ok`.
    fn validate(
        &self,
        tf: &StampedTransform,
        incoming_kind: EdgeKindTag,
    ) -> Result<(), TfIngestError> {
        if tf.child == tf.parent {
            return Err(TfIngestError::WouldCycle {
                child: tf.child.clone(),
                parent: tf.parent.clone(),
            });
        }

        if let Some(&established) = self.conventions.get(&tf.child) {
            if established != tf.transform.from_convention() {
                return Err(TfIngestError::ConventionConflict {
                    frame: tf.child.clone(),
                    established,
                    incoming: tf.transform.from_convention(),
                });
            }
        }

        if let Some(&established) = self.conventions.get(&tf.parent) {
            if established != tf.transform.to_convention() {
                return Err(TfIngestError::ConventionConflict {
                    frame: tf.parent.clone(),
                    established,
                    incoming: tf.transform.to_convention(),
                });
            }
        }

        if let Some((existing_parent, _)) = self.child_to_parent.get(&tf.child) {
            if existing_parent != &tf.parent {
                return Err(TfIngestError::MultipleParents {
                    child: tf.child.clone(),
                    existing: existing_parent.clone(),
                    incoming: tf.parent.clone(),
                });
            }
        }

        // Walk from `parent` toward the root; if we reach `child`, this edge would
        // close a loop (`parent` is already a descendant of `child`). Terminates
        // because the existing tree is acyclic, so parents always run out at a root.
        let mut cursor = &tf.parent;
        while let Some((next_parent, _)) = self.child_to_parent.get(cursor) {
            if next_parent == &tf.child {
                return Err(TfIngestError::WouldCycle {
                    child: tf.child.clone(),
                    parent: tf.parent.clone(),
                });
            }
            cursor = next_parent;
        }

        if let Some((_, existing_kind)) = self.child_to_parent.get(&tf.child) {
            // Compare only the kind, never the payload — a re-sample of a dynamic
            // edge legitimately carries different samples; only static-vs-dynamic
            // is the conflict.
            if existing_kind.tag() != incoming_kind {
                return Err(TfIngestError::EdgeKindConflict {
                    edge: FrameEdge {
                        child: tf.child.clone(),
                        parent: tf.parent.clone(),
                    },
                    existing: existing_kind.tag(),
                    incoming: incoming_kind,
                });
            }
        }

        Ok(())
    }

    /// Reads one edge's pose at the query time, before any tags are attached —
    /// the numeric core of a lookup, evaluated per edge as the walk climbs the
    /// tree. `child`/`parent` name the edge only so a failure can report *which*
    /// edge it fell off of; the success path never touches them.
    fn sample_edge(
        edge: &EdgeKind,
        query: TfQuery,
        child: &FrameId,
        parent: &FrameId,
    ) -> Result<Isometry3<f64>, TfLookupError> {
        match edge {
            // A static edge answers every query time with its one pose.
            EdgeKind::Static(iso) => Ok(*iso),

            EdgeKind::Dynamic(samples) => match query {
                // Snap to the newest sample. The deque is non-empty by
                // construction — an insert seeds one sample and eviction never
                // drops the last — so `back()` is always `Some`; the `None` arm
                // is unreachable, asserted in debug and a diagnosable error in
                // release rather than a panic (the crate forbids runtime panics).
                TfQuery::Latest => match samples.back() {
                    Some((_, iso)) => Ok(*iso),
                    None => {
                        debug_assert!(false, "a dynamic edge is never empty");
                        Err(TfLookupError::Disconnected {
                            from: child.clone(),
                            to: parent.clone(),
                        })
                    }
                },

                // A specific instant. Locate it in the ascending deque: an exact
                // stamp is answered untouched, an interior gap is interpolated, and
                // a time outside the retained history is an error — never an
                // extrapolation past either end.
                TfQuery::At(t) => match samples.binary_search_by(|(s, _)| s.0.total_cmp(&t.0)) {
                    // Exact hit: the stamp is stored, so no blending is needed.
                    Ok(i) => Ok(samples[i].1),
                    Err(i) => {
                        // `i` is where `t` would insert. At either end (`0` or `len`)
                        // the query falls before the oldest or after the newest
                        // sample; report the span actually held so the caller sees
                        // how far it missed.
                        if i == 0 || i == samples.len() {
                            let span = TimeSpan {
                                oldest: samples[0].0,
                                newest: samples[samples.len() - 1].0,
                            };
                            let frame_edge = FrameEdge {
                                child: child.clone(),
                                parent: parent.clone(),
                            };
                            return Err(TfLookupError::OutOfTimeRange {
                                edge: frame_edge,
                                query: t,
                                available: span,
                            });
                        }

                        // Interior gap: `t` sits strictly between samples[i-1] and
                        // samples[i]. `alpha` is the fraction of the bracket it has
                        // crossed; the clamp is a defensive belt (a strict bracket
                        // already yields a value in (0, 1)).
                        let t0 = samples[i - 1].0;
                        let t1 = samples[i].0;
                        let alpha = (t - t0).0 / (t1 - t0).0;
                        let alpha = alpha.clamp(0.0, 1.0);

                        // Translation is Euclidean: a straight-line lerp between the
                        // two positions.
                        let p0 = samples[i - 1].1.translation.vector;
                        let p1 = samples[i].1.translation.vector;
                        let lerped_translation = p0.lerp(&p1, alpha);

                        // Rotation is not: a componentwise average of two quaternions
                        // is not a rotation, so walk the shortest arc with slerp.
                        let q0 = samples[i - 1].1.rotation;
                        let q1 = samples[i].1.rotation;
                        // Below this tolerance on the sine of the angle between the two
                        // rotations, the shortest slerp arc is ill-defined (the samples
                        // are ~180° apart); `try_slerp` returns `None` and the blend
                        // snaps to the nearer endpoint instead.
                        const SLERP_ANTIPODAL_EPS: f64 = 1e-6;
                        let slerped_rotation = q0.try_slerp(&q1, alpha, SLERP_ANTIPODAL_EPS);
                        let rotation =
                            slerped_rotation.unwrap_or(if alpha < 0.5 { q0 } else { q1 });

                        Ok(Isometry3::from_parts(
                            Translation3::from(lerped_translation),
                            rotation,
                        ))
                    }
                },
            },
        }
    }

    /// The lowest frame on both `a`'s and `b`'s root-ward chains, or `None` when
    /// the two chains never meet (separate trees).
    ///
    /// Walks `child_to_parent` **keys only** — it reads no edge pose and calls no
    /// [`sample_edge`](TfBuffer::sample_edge). That is the whole point: the meeting
    /// point of two frames is a property of the tree's *structure*, independent of
    /// whether any edge can answer the query time, so an out-of-range dynamic edge
    /// anywhere in either chain must not influence — or fail — the search for it.
    /// [`accumulate`](TfBuffer::accumulate) then samples only the edges strictly
    /// below the returned LCA.
    ///
    /// Each chain includes its own start, so when one endpoint is an ancestor of
    /// the other (e.g. `base_link` and `odom` on one root-ward line) the LCA is
    /// that endpoint itself. The walk terminates at a root because
    /// [`validate`](TfBuffer::validate) keeps the tree acyclic, so parents always
    /// run out.
    fn lowest_common_ancestor(&self, a: &FrameId, b: &FrameId) -> Option<FrameId> {
        // `a`'s root-ward chain, itself included. Only the frame identities matter
        // here, so nothing is sampled.
        let mut a_chain: HashSet<FrameId> = HashSet::new();
        let mut cursor = Some(a.clone());
        while let Some(frame) = cursor {
            cursor = self
                .child_to_parent
                .get(&frame)
                .map(|(parent, _)| parent.clone());
            a_chain.insert(frame);
        }

        // Climb `b`'s chain (itself included) and take the first frame that is also
        // an ancestor of `a` — the lowest, since the walk ascends.
        let mut cursor = Some(b.clone());
        while let Some(frame) = cursor {
            if a_chain.contains(&frame) {
                return Some(frame);
            }
            cursor = self
                .child_to_parent
                .get(&frame)
                .map(|(parent, _)| parent.clone());
        }
        None
    }

    /// Composes `start → target`, climbing parent-ward from `start` and stopping
    /// the instant the cursor reaches `target`. `start == target` is identity, with
    /// no hops taken.
    ///
    /// Only the edges strictly between `start` and `target` are sampled — nothing
    /// above `target`. Callers pass an LCA that is a genuine ancestor of `start`,
    /// so this both bounds the work to the path the answer crosses and is what
    /// keeps an unqueryable edge higher in the tree from failing a lookup that does
    /// not reach it.
    ///
    /// Each hop reads its edge with [`sample_edge`](TfBuffer::sample_edge), tags the
    /// pose with the two frames' conventions, and composes it onto the running
    /// `start → cursor` transform. An edge whose history does not cover an `At`
    /// query surfaces as the `sample_edge` error, propagated unchanged. Reaching a
    /// root without meeting `target` means the two are not on one chain; that is a
    /// defensive [`Disconnected`](TfLookupError::Disconnected) never hit on a valid
    /// LCA, kept to honor the no-panic rule.
    fn accumulate(
        &self,
        start: &FrameId,
        target: &FrameId,
        query: TfQuery,
    ) -> Result<ErasedTransform, TfLookupError> {
        let Some(start_conv) = self.conventions.get(start) else {
            return Err(TfLookupError::UnknownFrame(start.clone()));
        };

        // `start → start` identity seeds the accumulator; if `start == target` it
        // is the answer, and no edge is sampled.
        let mut running =
            ErasedTransform::from_parts(Isometry3::identity(), *start_conv, *start_conv);
        let mut cursor = start;

        while cursor != target {
            let Some((parent, edge)) = self.child_to_parent.get(cursor) else {
                return Err(TfLookupError::Disconnected {
                    from: start.clone(),
                    to: target.clone(),
                });
            };

            // The edge's own pose at the query time, before conventions are attached.
            let sample = Self::sample_edge(edge, query, cursor, parent)?;

            // Conventions live per frame; every frame the tree stores has one, so a
            // miss here means the frame is genuinely unknown, not an internal gap.
            let Some(from) = self.conventions.get(cursor) else {
                return Err(TfLookupError::UnknownFrame(cursor.clone()));
            };
            let Some(to) = self.conventions.get(parent) else {
                return Err(TfLookupError::UnknownFrame(parent.clone()));
            };

            // This hop alone: `cursor → parent`, tagged with each end's convention.
            // Extend the accumulator: `start → cursor` then `cursor → parent` gives
            // `start → parent`. The shared `cursor` convention is the seam `then`
            // checks, so a flipped composition would trip its debug assert.
            running = running.then(ErasedTransform::from_parts(sample, *from, *to));
            cursor = parent;
        }

        Ok(running)
    }
}

/// How far back the history reaches, applied uniformly to every edge.
///
/// The primary bound is a **duration**, not a sample count: edge rates differ by
/// orders of magnitude, so a fixed count would mean a different time-depth per
/// edge, whereas a duration is rate-independent and names the thing that matters —
/// how far back a lookup may reach. `max_samples` is only a memory backstop against
/// a producer faster than the window assumed, so one edge cannot evict another's.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TfWindow {
    /// The primary bound: the oldest sample a lookup can reach, relative to the
    /// edge's newest.
    pub horizon: MonotonicDuration,
    /// A per-edge memory backstop; hitting it means the edge is faster than the
    /// window assumed.
    pub max_samples: usize,
}

/// How a lookup reads an edge's history in time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TfQuery {
    /// Snap each edge to its own newest sample — real-time, always answerable.
    Latest,
    /// Interpolate the samples bracketing a specific time; a query outside an
    /// edge's retained span is an error, never an extrapolation.
    At(MonotonicTime),
}

/// Why a fold rejected an edge. Every variant names the offending frame(s) so the
/// failure can be traced to the producer that emitted the bad message. Structural,
/// disjoint from lookup failures, and never conflated with them.
#[derive(Debug)]
pub enum TfIngestError {
    /// A child already bound to a different parent — the single-parent rule.
    MultipleParents {
        child: FrameId,
        existing: FrameId,
        incoming: FrameId,
    },
    /// The edge would close a loop: either a self-edge, or a parent already a
    /// descendant of the child.
    WouldCycle { child: FrameId, parent: FrameId },
    /// An endpoint's convention tag disagrees with the one its first sighting
    /// established for that frame.
    ConventionConflict {
        frame: FrameId,
        established: Convention,
        incoming: Convention,
    },
    /// An edge re-inserted with the other kind than it was first declared — static
    /// where dynamic was established, or vice versa.
    EdgeKindConflict {
        edge: FrameEdge,
        existing: EdgeKindTag,
        incoming: EdgeKindTag,
    },
    /// A stamp that already exists on this edge, carrying a different value —
    /// static recalibration or a contradicting dynamic sample.
    ConflictingSample {
        edge: FrameEdge,
        stamp: MonotonicTime,
    },
}

/// Why a lookup could not produce a transform. Disjoint from ingest failures —
/// these arise at read time, from the pipeline, service, or a debug reader — so a
/// diagnosable error reaches every richer caller rather than the silent `None`
/// that is this subsystem's drift trap.
#[derive(Debug)]
pub enum TfLookupError {
    /// A queried frame that no edge has ever mentioned — the catcher for a typo'd
    /// or not-yet-received frame, rather than a silently wrong answer.
    UnknownFrame(FrameId),
    /// Both frames are known but sit in separate components — no path connects them.
    Disconnected { from: FrameId, to: FrameId },
    /// An `At` query fell outside an edge's retained history; `available` reports
    /// the span it does hold, so the caller sees how far the query missed.
    OutOfTimeRange {
        edge: FrameEdge,
        query: MonotonicTime,
        available: TimeSpan,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::data::AgentId;

    use nalgebra::{UnitQuaternion, Vector3};
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

    // --- builders ---

    fn agent() -> AgentId {
        AgentId::new("bot")
    }

    // The spine frames used across the ingest tests. `base_link` is FLU, the
    // `odom`/`map` roots above it are ENU — the convention split every edge that
    // touches them must respect.
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

    // A pure translation along +X, identity rotation — enough to tell samples
    // apart by a single number.
    fn iso_x(x: f64) -> Isometry3<f64> {
        Isometry3::translation(x, 0.0, 0.0)
    }

    // Translation along +X plus a yaw about +Z, so an interpolation test can watch
    // the translation and the rotation blend independently.
    fn iso_x_yaw(x: f64, yaw: f64) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(x, 0.0, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw),
        )
    }

    fn stamped(
        child: FrameId,
        parent: FrameId,
        from: Convention,
        to: Convention,
        stamp: f64,
        iso: Isometry3<f64>,
    ) -> StampedTransform {
        StampedTransform {
            parent,
            child,
            stamp: MonotonicTime(stamp),
            transform: ErasedTransform::from_parts(iso, from, to),
        }
    }

    // The common case: a `base_link (FLU) -> odom (ENU)` dynamic sample.
    fn bl_odom(stamp: f64, x: f64) -> StampedTransform {
        stamped(
            base_link(),
            odom(),
            Convention::Flu,
            Convention::Enu,
            stamp,
            iso_x(x),
        )
    }

    // A permissive window: neither bound bites unless a test asks it to.
    fn window() -> TfWindow {
        TfWindow {
            horizon: MonotonicDuration(100.0),
            max_samples: 100,
        }
    }

    fn dynamic(pairs: &[(f64, Isometry3<f64>)]) -> EdgeKind {
        EdgeKind::Dynamic(pairs.iter().map(|(t, i)| (MonotonicTime(*t), *i)).collect())
    }

    fn edge_kind<'a>(buf: &'a TfBuffer, child: &FrameId) -> &'a EdgeKind {
        &buf.child_to_parent.get(child).expect("edge present").1
    }

    fn stamps_of(kind: &EdgeKind) -> Vec<f64> {
        match kind {
            EdgeKind::Dynamic(samples) => samples.iter().map(|(t, _)| t.0).collect(),
            EdgeKind::Static(_) => panic!("expected a dynamic edge"),
        }
    }

    fn assert_stamps(kind: &EdgeKind, expected: &[f64]) {
        let got = stamps_of(kind);
        assert_eq!(got.len(), expected.len(), "sample count");
        for (g, e) in got.iter().zip(expected) {
            assert!((g - e).abs() < 1e-12, "stamp {g} != expected {e}");
        }
    }

    // --- sample_edge: static ---

    #[test]
    fn static_edge_answers_every_query() {
        // A static edge is time-invariant, so `Latest` and any `At` return the one
        // pose it holds.
        let edge = EdgeKind::Static(iso_x(3.0));
        for query in [
            TfQuery::Latest,
            TfQuery::At(MonotonicTime(0.0)),
            TfQuery::At(MonotonicTime(9.0)),
        ] {
            let iso = TfBuffer::sample_edge(&edge, query, &base_link(), &odom())
                .ok()
                .expect("a static edge never fails");
            assert!((iso.translation.vector.x - 3.0).abs() < 1e-12);
        }
    }

    // --- sample_edge: dynamic + Latest ---

    #[test]
    fn dynamic_latest_returns_the_newest_sample() {
        let edge = dynamic(&[(1.0, iso_x(1.0)), (2.0, iso_x(2.0)), (3.0, iso_x(3.0))]);
        let iso = TfBuffer::sample_edge(&edge, TfQuery::Latest, &base_link(), &odom())
            .ok()
            .expect("latest is always answerable");
        assert!((iso.translation.vector.x - 3.0).abs() < 1e-12);
    }

    // --- sample_edge: dynamic + At ---

    #[test]
    fn dynamic_at_exact_stamp_returns_that_sample() {
        let edge = dynamic(&[(1.0, iso_x(1.0)), (2.0, iso_x(2.0))]);
        let iso = TfBuffer::sample_edge(
            &edge,
            TfQuery::At(MonotonicTime(2.0)),
            &base_link(),
            &odom(),
        )
        .ok()
        .expect("an exact stamp is in range");
        assert!((iso.translation.vector.x - 2.0).abs() < 1e-12);
    }

    #[test]
    fn dynamic_at_midpoint_interpolates_both_channels() {
        // t=0 at the origin, t=2 at x=10 and yawed 90°. Halfway (t=1) must land at
        // x=5 and 45° — translation lerps, rotation slerps.
        let edge = dynamic(&[
            (0.0, iso_x_yaw(0.0, 0.0)),
            (2.0, iso_x_yaw(10.0, FRAC_PI_2)),
        ]);
        let iso = TfBuffer::sample_edge(
            &edge,
            TfQuery::At(MonotonicTime(1.0)),
            &base_link(),
            &odom(),
        )
        .ok()
        .expect("an interior time is in range");
        assert!((iso.translation.vector.x - 5.0).abs() < 1e-9);
        assert!((iso.rotation.angle() - FRAC_PI_4).abs() < 1e-9);
    }

    #[test]
    fn dynamic_at_before_oldest_is_out_of_range() {
        let edge = dynamic(&[(1.0, iso_x(1.0)), (2.0, iso_x(2.0))]);
        let result = TfBuffer::sample_edge(
            &edge,
            TfQuery::At(MonotonicTime(0.5)),
            &base_link(),
            &odom(),
        );
        // The reported span is the whole retained history, not the missed end.
        match result {
            Err(TfLookupError::OutOfTimeRange {
                query, available, ..
            }) => {
                assert!((query.0 - 0.5).abs() < 1e-12);
                assert!((available.oldest.0 - 1.0).abs() < 1e-12);
                assert!((available.newest.0 - 2.0).abs() < 1e-12);
            }
            _ => panic!("a query before the oldest sample must be out of range"),
        }
    }

    #[test]
    fn dynamic_at_after_newest_is_out_of_range() {
        let edge = dynamic(&[(1.0, iso_x(1.0)), (2.0, iso_x(2.0))]);
        let result = TfBuffer::sample_edge(
            &edge,
            TfQuery::At(MonotonicTime(3.0)),
            &base_link(),
            &odom(),
        );
        assert!(matches!(result, Err(TfLookupError::OutOfTimeRange { .. })));
    }

    #[test]
    fn single_sample_answers_only_its_exact_stamp() {
        // A lone sample interpolates with nothing: it answers its own stamp and
        // rejects every other time rather than extrapolating.
        let edge = dynamic(&[(1.0, iso_x(1.0))]);
        assert!(TfBuffer::sample_edge(
            &edge,
            TfQuery::At(MonotonicTime(1.0)),
            &base_link(),
            &odom()
        )
        .is_ok());
        assert!(matches!(
            TfBuffer::sample_edge(
                &edge,
                TfQuery::At(MonotonicTime(0.5)),
                &base_link(),
                &odom()
            ),
            Err(TfLookupError::OutOfTimeRange { .. })
        ));
        assert!(matches!(
            TfBuffer::sample_edge(
                &edge,
                TfQuery::At(MonotonicTime(2.0)),
                &base_link(),
                &odom()
            ),
            Err(TfLookupError::OutOfTimeRange { .. })
        ));
    }

    // A sensor mounted on base_link at the given +X offset — the static extrinsic
    // shape every state-sensor lookup crosses.
    fn mount(buf: &mut TfBuffer, name: &str, x: f64) -> FrameId {
        mount_on(buf, name, base_link(), Convention::Flu, x)
    }

    // A static sensor mount on an arbitrary parent, whose convention names the
    // parent's end of the edge (the sensor itself is FLU).
    fn mount_on(
        buf: &mut TfBuffer,
        name: &str,
        parent: FrameId,
        parent_convention: Convention,
        x: f64,
    ) -> FrameId {
        let frame = sensor(name);
        assert!(buf
            .insert_static(stamped(
                frame.clone(),
                parent,
                Convention::Flu,
                parent_convention,
                0.0,
                iso_x(x),
            ))
            .is_ok());
        frame
    }

    // --- lowest_common_ancestor ---

    #[test]
    fn lca_of_two_siblings_is_their_shared_parent() {
        // cam and lidar both mount on base_link; neither is the other's ancestor,
        // so the meeting point is base_link.
        let mut buf = TfBuffer::new(window());
        let cam = mount(&mut buf, "cam", 1.0);
        let lidar = mount(&mut buf, "lidar", 2.0);

        assert_eq!(buf.lowest_common_ancestor(&cam, &lidar), Some(base_link()));
    }

    #[test]
    fn lca_of_a_frame_and_its_ancestor_is_the_ancestor() {
        // base_link and odom lie on one root-ward chain, so their only *common*
        // ancestor is the upper of the two, odom — reached from either order. A
        // straight-line lookup then accumulates base_link → odom on one side and an
        // empty odom → odom on the other.
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());

        assert_eq!(
            buf.lowest_common_ancestor(&base_link(), &odom()),
            Some(odom())
        );
        assert_eq!(
            buf.lowest_common_ancestor(&odom(), &base_link()),
            Some(odom())
        );
    }

    #[test]
    fn lca_of_separate_trees_is_none() {
        // base_link -> odom and a floating sensor -> map share no ancestor.
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());
        let floating = mount_on(&mut buf, "floating", map(), Convention::Enu, 1.0);

        assert_eq!(buf.lowest_common_ancestor(&base_link(), &floating), None);
    }

    #[test]
    fn lca_samples_no_edge_so_an_out_of_range_ancestor_is_irrelevant() {
        // gps mounts on base_link, whose only parent is a dynamic odom edge with
        // history at t=1,2. The LCA of base_link and gps is base_link regardless of
        // the query time — the search reads keys, never the odom edge's samples.
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());
        assert!(buf.insert_dynamic(bl_odom(2.0, 2.0)).is_ok());
        let gps = mount(&mut buf, "gps", 0.5);

        // No query is even passed: LCA is purely structural.
        assert_eq!(
            buf.lowest_common_ancestor(&base_link(), &gps),
            Some(base_link())
        );
        assert_eq!(
            buf.lowest_common_ancestor(&gps, &base_link()),
            Some(base_link())
        );
    }

    // --- accumulate ---

    #[test]
    fn accumulate_start_equals_target_is_identity_and_samples_nothing() {
        // With start == target the walk takes no hops, so even a dynamic parent
        // edge that is out of range at the query time is never touched.
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());
        assert!(buf.insert_dynamic(bl_odom(2.0, 2.0)).is_ok());

        let tf = buf
            .accumulate(&base_link(), &base_link(), TfQuery::At(MonotonicTime(5.0)))
            .ok()
            .expect("identity needs no edge, so the out-of-range parent is irrelevant");
        assert!(tf.isometry().translation.vector.norm() < 1e-12);
    }

    #[test]
    fn accumulate_composes_only_the_edges_up_to_target() {
        // gps -(x=1)-> base_link -(x=2)-> odom. Accumulating gps → base_link crosses
        // just the mount (x=1) and stops; the base_link → odom edge above the target
        // is never composed in.
        let mut buf = TfBuffer::new(window());
        let gps = mount(&mut buf, "gps", 1.0);
        assert!(buf
            .insert_static(stamped(
                base_link(),
                odom(),
                Convention::Flu,
                Convention::Enu,
                0.0,
                iso_x(2.0),
            ))
            .is_ok());

        let tf = buf
            .accumulate(&gps, &base_link(), TfQuery::Latest)
            .ok()
            .expect("gps reaches base_link across the mount");
        assert!((tf.isometry().translation.vector.x - 1.0).abs() < 1e-12);
    }

    #[test]
    fn accumulate_propagates_an_out_of_range_edge_on_the_path() {
        // The out-of-range edge is *on* the base_link → odom path this time, so the
        // error is legitimate and must surface unchanged.
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());
        assert!(buf.insert_dynamic(bl_odom(2.0, 2.0)).is_ok());

        let result = buf.accumulate(&base_link(), &odom(), TfQuery::At(MonotonicTime(5.0)));
        assert!(matches!(result, Err(TfLookupError::OutOfTimeRange { .. })));
    }

    #[test]
    fn accumulate_unknown_start_is_unknown_frame() {
        // A frame no edge ever mentioned has no convention, so the walk cannot even
        // seed its identity — it is reported unknown rather than silently empty.
        let buf = TfBuffer::new(window());
        let result = buf.accumulate(&sensor("ghost"), &base_link(), TfQuery::Latest);
        assert!(matches!(result, Err(TfLookupError::UnknownFrame(_))));
    }

    // --- lookup ---

    #[test]
    fn lookup_identity_for_the_same_frame() {
        // A frame to itself is identity, answered before any tree walk.
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());

        let tf = buf
            .lookup(&base_link(), &base_link(), TfQuery::Latest)
            .ok()
            .expect("a frame to itself is always answerable");
        assert!(tf.isometry().translation.vector.norm() < 1e-12);
        assert!(tf.isometry().rotation.angle().abs() < 1e-12);
    }

    #[test]
    fn lookup_direct_parent_hop_and_its_reverse() {
        // base_link -(x=1)-> odom. The forward lookup is that edge; the reverse is
        // its inverse, so the translation flips sign.
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());

        let forward = buf
            .lookup(&base_link(), &odom(), TfQuery::Latest)
            .ok()
            .expect("base_link and odom share the edge");
        assert!((forward.isometry().translation.vector.x - 1.0).abs() < 1e-12);

        let reverse = buf
            .lookup(&odom(), &base_link(), TfQuery::Latest)
            .ok()
            .expect("the reverse is just as answerable");
        assert!((reverse.isometry().translation.vector.x + 1.0).abs() < 1e-12);
    }

    #[test]
    fn lookup_between_siblings_crosses_their_common_ancestor() {
        // cam and lidar both mount on base_link, at x=1 and x=2. Neither is the
        // other's ancestor; the lookup meets at base_link and descends the far
        // side, placing cam 1m behind lidar (x=-1).
        let mut buf = TfBuffer::new(window());
        let cam = sensor("cam");
        let lidar = sensor("lidar");
        assert!(buf
            .insert_static(stamped(
                cam.clone(),
                base_link(),
                Convention::Flu,
                Convention::Flu,
                0.0,
                iso_x(1.0),
            ))
            .is_ok());
        assert!(buf
            .insert_static(stamped(
                lidar.clone(),
                base_link(),
                Convention::Flu,
                Convention::Flu,
                0.0,
                iso_x(2.0),
            ))
            .is_ok());

        let tf = buf
            .lookup(&cam, &lidar, TfQuery::Latest)
            .ok()
            .expect("cam and lidar meet at base_link");
        assert!((tf.isometry().translation.vector.x + 1.0).abs() < 1e-12);
    }

    #[test]
    fn lookup_across_separate_trees_is_disconnected() {
        // Two disjoint trees: base_link -> odom, and a floating sensor -> map. The
        // endpoints share no ancestor, so no transform relates them.
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());
        let floating = sensor("floating");
        assert!(buf
            .insert_static(stamped(
                floating.clone(),
                map(),
                Convention::Flu,
                Convention::Enu,
                0.0,
                iso_x(1.0),
            ))
            .is_ok());

        let result = buf.lookup(&base_link(), &floating, TfQuery::Latest);
        assert!(matches!(result, Err(TfLookupError::Disconnected { .. })));
    }

    #[test]
    fn lookup_unknown_source_frame_is_unknown() {
        // The source was never seen: reported unknown, and the error names that
        // endpoint rather than the known target.
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());

        let ghost = sensor("ghost");
        match buf.lookup(&ghost, &odom(), TfQuery::Latest) {
            Err(TfLookupError::UnknownFrame(f)) => assert!(f == ghost),
            _ => panic!("an unseen source frame must be UnknownFrame"),
        }
    }

    #[test]
    fn lookup_unknown_target_frame_is_unknown() {
        // The mirror of the source guard: an unseen target is reported unknown and
        // named, not mistaken for a disconnection.
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());

        let ghost = sensor("ghost");
        match buf.lookup(&base_link(), &ghost, TfQuery::Latest) {
            Err(TfLookupError::UnknownFrame(f)) => assert!(f == ghost),
            _ => panic!("an unseen target frame must be UnknownFrame"),
        }
    }

    #[test]
    fn lookup_at_a_time_reads_the_interpolated_edge() {
        // A dynamic base_link -> odom edge at t=0 (x=0) and t=2 (x=10). An At(1.0)
        // lookup interpolates the edge to x=5 on the way through.
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(0.0, 0.0)).is_ok());
        assert!(buf.insert_dynamic(bl_odom(2.0, 10.0)).is_ok());

        let tf = buf
            .lookup(&base_link(), &odom(), TfQuery::At(MonotonicTime(1.0)))
            .ok()
            .expect("t=1 is within the edge history");
        assert!((tf.isometry().translation.vector.x - 5.0).abs() < 1e-9);
    }

    #[test]
    fn lookup_of_a_static_mount_survives_an_out_of_range_dynamic_ancestor() {
        // The exact shape of the GPS-aiding regression: a static base_link → gps
        // mount, under a base_link whose only parent is a dynamic odom edge with
        // history at t=1,2. A consumer queries the mount at t=5 — past the odom
        // edge's history — but the mount's answer does not cross that edge, so it
        // must still resolve. (Before the LCA fix this failed with OutOfTimeRange,
        // silently dropping every GPS/mag update and drifting the estimate.)
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());
        assert!(buf.insert_dynamic(bl_odom(2.0, 2.0)).is_ok());
        let gps = mount(&mut buf, "gps", 0.5);

        // Both directions resolve at a time the odom edge cannot answer. The edge
        // is stored child=gps → parent=base_link (x=+0.5), so — as with any
        // parent-ward hop and its reverse — lookup(base_link, gps) is the inverse
        // (x=-0.5) and lookup(gps, base_link) is the stored value (x=+0.5).
        let mount_tf = buf
            .lookup(&base_link(), &gps, TfQuery::At(MonotonicTime(5.0)))
            .ok()
            .expect("a static mount does not cross the odom edge");
        assert!((mount_tf.isometry().translation.vector.x + 0.5).abs() < 1e-12);

        let reverse = buf
            .lookup(&gps, &base_link(), TfQuery::At(MonotonicTime(5.0)))
            .ok()
            .expect("the reverse mount is just as answerable");
        assert!((reverse.isometry().translation.vector.x - 0.5).abs() < 1e-12);

        // The guard is scoped, not blanket: a lookup that *does* cross the
        // out-of-range odom edge still fails, so real staleness is not masked.
        let crosses = buf.lookup(&base_link(), &odom(), TfQuery::At(MonotonicTime(5.0)));
        assert!(matches!(crosses, Err(TfLookupError::OutOfTimeRange { .. })));
    }

    // --- insert_static ---

    #[test]
    fn insert_static_stores_the_edge() {
        let mut buf = TfBuffer::new(window());
        let edge = stamped(
            base_link(),
            odom(),
            Convention::Flu,
            Convention::Enu,
            0.0,
            iso_x(1.0),
        );
        assert!(buf.insert_static(edge).is_ok());
        assert!(matches!(edge_kind(&buf, &base_link()), EdgeKind::Static(_)));
    }

    #[test]
    fn insert_static_is_idempotent_for_the_same_value() {
        let mut buf = TfBuffer::new(window());
        let make = || {
            stamped(
                base_link(),
                odom(),
                Convention::Flu,
                Convention::Enu,
                0.0,
                iso_x(1.0),
            )
        };
        assert!(buf.insert_static(make()).is_ok());
        assert!(buf.insert_static(make()).is_ok());
    }

    #[test]
    fn insert_static_rejects_a_different_value() {
        // A static edge is a calibration datum, not a stream; re-mounting it at a
        // different pose is a contradiction, not an update.
        let mut buf = TfBuffer::new(window());
        let first = stamped(
            base_link(),
            odom(),
            Convention::Flu,
            Convention::Enu,
            0.0,
            iso_x(1.0),
        );
        let second = stamped(
            base_link(),
            odom(),
            Convention::Flu,
            Convention::Enu,
            0.0,
            iso_x(2.0),
        );
        assert!(buf.insert_static(first).is_ok());
        assert!(matches!(
            buf.insert_static(second),
            Err(TfIngestError::ConflictingSample { .. })
        ));
    }

    // --- insert_dynamic ---

    #[test]
    fn insert_dynamic_new_edge_seeds_a_one_sample_history() {
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());
        assert_stamps(edge_kind(&buf, &base_link()), &[1.0]);
    }

    #[test]
    fn insert_dynamic_keeps_samples_sorted_regardless_of_arrival_order() {
        let mut buf = TfBuffer::new(window());
        // In-order, then a late arrival that must slot into the middle.
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());
        assert!(buf.insert_dynamic(bl_odom(3.0, 3.0)).is_ok());
        assert!(buf.insert_dynamic(bl_odom(2.0, 2.0)).is_ok());
        assert_stamps(edge_kind(&buf, &base_link()), &[1.0, 2.0, 3.0]);
    }

    #[test]
    fn insert_dynamic_rejects_a_contradicting_sample_at_a_known_stamp() {
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());
        assert!(matches!(
            buf.insert_dynamic(bl_odom(1.0, 2.0)),
            Err(TfIngestError::ConflictingSample { .. })
        ));
    }

    #[test]
    fn insert_dynamic_evicts_samples_older_than_the_horizon() {
        // Horizon of 1s: after a t=5 sample the cutoff is t=4, dropping t=0 and t=1.
        let mut buf = TfBuffer::new(TfWindow {
            horizon: MonotonicDuration(1.0),
            max_samples: 100,
        });
        assert!(buf.insert_dynamic(bl_odom(0.0, 0.0)).is_ok());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());
        assert!(buf.insert_dynamic(bl_odom(5.0, 5.0)).is_ok());
        assert_stamps(edge_kind(&buf, &base_link()), &[5.0]);
    }

    #[test]
    fn insert_dynamic_evicts_by_the_max_samples_backstop() {
        // Horizon can't bite here; the count backstop caps the history at 2.
        let mut buf = TfBuffer::new(TfWindow {
            horizon: MonotonicDuration(100.0),
            max_samples: 2,
        });
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());
        assert!(buf.insert_dynamic(bl_odom(2.0, 2.0)).is_ok());
        assert!(buf.insert_dynamic(bl_odom(3.0, 3.0)).is_ok());
        assert_stamps(edge_kind(&buf, &base_link()), &[2.0, 3.0]);
    }

    #[test]
    fn insert_dynamic_drops_a_stale_arrival_without_changing_history() {
        // A sample already older than the cutoff would only be evicted again, so it
        // is a no-op — accepted, but the history is untouched.
        let mut buf = TfBuffer::new(TfWindow {
            horizon: MonotonicDuration(1.0),
            max_samples: 100,
        });
        assert!(buf.insert_dynamic(bl_odom(5.0, 5.0)).is_ok());
        assert!(buf.insert_dynamic(bl_odom(3.0, 3.0)).is_ok());
        assert_stamps(edge_kind(&buf, &base_link()), &[5.0]);
    }

    // --- edges (topology enumeration) ---

    #[test]
    fn edges_reports_every_edge_tagged_by_kind() {
        // A tree with one static mount (sensor -> base_link) and one dynamic
        // edge (base_link -> odom). `edges` returns both, each carrying only its
        // kind tag, and the count is one entry per stored edge.
        let mut buf = TfBuffer::new(window());
        let mount = stamped(
            sensor("imu"),
            base_link(),
            Convention::Flu,
            Convention::Flu,
            0.0,
            iso_x(1.0),
        );
        assert!(buf.insert_static(mount).is_ok());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());

        let edges = buf.edges();
        assert_eq!(edges.len(), 2, "one entry per stored edge");

        // Order is unspecified (HashMap walk), so assert by membership.
        let has = |child: FrameId, parent: FrameId, tag: EdgeKindTag| {
            edges
                .iter()
                .any(|(e, k)| e.child == child && e.parent == parent && *k == tag)
        };
        assert!(has(sensor("imu"), base_link(), EdgeKindTag::Static));
        assert!(has(base_link(), odom(), EdgeKindTag::Dynamic));
    }

    #[test]
    fn edges_collapses_the_kind_to_its_tag_only() {
        // The full `EdgeKind` payload (the isometry, the sample history) must not
        // leak through enumeration; a dynamic edge with many samples still yields
        // exactly one entry, tagged Dynamic, with no history attached.
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());
        assert!(buf.insert_dynamic(bl_odom(2.0, 2.0)).is_ok());
        assert!(buf.insert_dynamic(bl_odom(3.0, 3.0)).is_ok());

        let edges = buf.edges();
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].1, EdgeKindTag::Dynamic);
    }

    #[test]
    fn edges_is_empty_on_a_fresh_buffer() {
        // No edges folded yet: an empty topology, not an error.
        assert!(TfBuffer::new(window()).edges().is_empty());
    }

    // --- validate (shared front) ---

    #[test]
    fn validate_rejects_a_self_edge() {
        let mut buf = TfBuffer::new(window());
        let edge = stamped(
            base_link(),
            base_link(),
            Convention::Flu,
            Convention::Flu,
            0.0,
            iso_x(0.0),
        );
        assert!(matches!(
            buf.insert_dynamic(edge),
            Err(TfIngestError::WouldCycle { .. })
        ));
    }

    #[test]
    fn validate_rejects_a_second_parent_for_a_child() {
        // base_link is bound to odom; re-parenting it under map violates single-parent.
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());
        let reparent = stamped(
            base_link(),
            map(),
            Convention::Flu,
            Convention::Enu,
            1.0,
            iso_x(1.0),
        );
        assert!(matches!(
            buf.insert_dynamic(reparent),
            Err(TfIngestError::MultipleParents { .. })
        ));
    }

    #[test]
    fn validate_rejects_a_convention_disagreement() {
        // base_link is established FLU; a later edge tagging it ENU contradicts the
        // frame's intrinsic.
        let mut buf = TfBuffer::new(window());
        assert!(buf.insert_dynamic(bl_odom(1.0, 1.0)).is_ok());
        let wrong = stamped(
            base_link(),
            odom(),
            Convention::Enu,
            Convention::Enu,
            2.0,
            iso_x(2.0),
        );
        assert!(matches!(
            buf.insert_dynamic(wrong),
            Err(TfIngestError::ConventionConflict { .. })
        ));
    }

    #[test]
    fn validate_rejects_switching_an_edges_kind() {
        // Established static, re-sampled dynamic on the same edge: the kind an edge
        // declares on its first insert is fixed.
        let mut buf = TfBuffer::new(window());
        let stat = stamped(
            base_link(),
            odom(),
            Convention::Flu,
            Convention::Enu,
            0.0,
            iso_x(1.0),
        );
        assert!(buf.insert_static(stat).is_ok());
        assert!(matches!(
            buf.insert_dynamic(bl_odom(1.0, 1.0)),
            Err(TfIngestError::EdgeKindConflict { .. })
        ));
    }

    #[test]
    fn validate_rejects_an_edge_that_would_close_a_loop() {
        // a -> b -> c is a chain; adding c -> a would make it a cycle, caught by the
        // parent-ward walk before anything is stored.
        let mut buf = TfBuffer::new(window());
        let a_b = stamped(
            sensor("a"),
            sensor("b"),
            Convention::Flu,
            Convention::Flu,
            1.0,
            iso_x(1.0),
        );
        let b_c = stamped(
            sensor("b"),
            sensor("c"),
            Convention::Flu,
            Convention::Flu,
            1.0,
            iso_x(1.0),
        );
        let c_a = stamped(
            sensor("c"),
            sensor("a"),
            Convention::Flu,
            Convention::Flu,
            1.0,
            iso_x(1.0),
        );
        assert!(buf.insert_dynamic(a_b).is_ok());
        assert!(buf.insert_dynamic(b_c).is_ok());
        assert!(matches!(
            buf.insert_dynamic(c_a),
            Err(TfIngestError::WouldCycle { .. })
        ));
    }
}
