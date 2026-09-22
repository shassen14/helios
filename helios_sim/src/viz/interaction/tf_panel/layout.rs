//! Pure tree geometry for the 2D tf panel: topology in, abstract grid cells out.
//!
//! This is the panel's one non-trivial computation, deliberately lifted out of
//! any `bevy_ui` system into a plain function over `helios_core` types so it is
//! Tier-1 testable (no `App`). It answers only *where each frame sits* — a
//! `(depth, slot)` cell per node. Turning a cell into pixels, drawing connector
//! lines, and hanging staleness badges off edges are all downstream renderer
//! concerns that never enter here.
//!
//! It takes the edge set alone (`TfBuffer::edges()` gives exactly this). Sample
//! stamps and rates are a separate overlay fed by `dynamic_edge_stats()`; they
//! do not affect geometry.

use helios_core::frames::{transforms::tf::stamped::FrameEdge, FrameId};

use std::collections::{HashMap, HashSet};

/// A node's abstract position in the panel grid, before any pixel mapping.
///
/// `depth` is the number of parent-ward hops from a root (a root is `depth 0`).
/// `slot` is the node's ordinal position among *every* node sharing its depth,
/// contiguous from `0`. The renderer maps `depth`→column (or row) and
/// `slot`→the cross-axis offset; this type carries no units and no styling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LayoutCell {
    pub depth: usize,
    pub slot: usize,
}

/// Places every frame reachable from a root onto the panel grid.
///
/// A **root** is a frame that appears as a `parent` but never as a `child` — the
/// top of a per-agent estimated tree. There may be more than one: the
/// multi-agent panel lays out N unjoined per-agent graphs at once, so a forest
/// is expected input, not an error. Roots share `depth 0` and take consecutive
/// slots.
///
/// The result is the placement of each reachable node, and it must be
/// **deterministic**: the same edge set in any order yields the same layout.
/// `edges()` and the buffer's backing `HashMap` are both unordered, so the
/// implementation has to impose an order on siblings itself — `FrameId` is
/// `Eq + Hash` but **not `Ord`**, so sort them by an explicit key (e.g. their
/// `Display`), never by hash-map iteration order. Slots within a depth are
/// assigned from that stable order.
///
/// Malformed input is tolerated, never fatal: nodes not reachable from any root
/// (an orphan, or a cycle with no root) are simply left unplaced, so a cyclic
/// buffer can neither hang the walk nor appear in the panel.
pub fn tree_layout(edges: &[FrameEdge]) -> Vec<(FrameId, LayoutCell)> {
    let mut children: HashSet<FrameId> = HashSet::new();
    let mut parents: HashSet<FrameId> = HashSet::new();
    for edge in edges {
        children.insert(edge.child.clone());
        parents.insert(edge.parent.clone());
    }

    let mut roots: Vec<FrameId> = parents.difference(&children).cloned().collect();

    roots.sort_by_key(|id| id.to_string());

    let mut children_of: HashMap<FrameId, Vec<FrameId>> = HashMap::new();

    for edge in edges {
        children_of
            .entry(edge.parent.clone())
            .or_default()
            .push(edge.child.clone());
    }

    let mut layout: Vec<(FrameId, LayoutCell)> = Vec::new();
    let mut visited: HashSet<FrameId> = HashSet::new();

    let mut current: Vec<FrameId> = roots;
    let mut depth = 0;

    while !current.is_empty() {
        let mut next: Vec<FrameId> = Vec::new();

        for (slot, frame) in current.iter().enumerate() {
            if !visited.insert(frame.clone()) {
                continue;
            }

            layout.push((frame.clone(), LayoutCell { depth, slot }));

            if let Some(kids) = children_of.get(frame) {
                let mut kids = kids.clone();
                kids.sort_by_key(|id| id.to_string());
                next.extend(kids);
            }
        }
        current = next;
        depth += 1;
    }

    layout
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::data::AgentId;

    /// A dynamic-style edge `child -> parent`. Poses are irrelevant to layout, so
    /// the tests speak only in identities.
    fn edge(child: FrameId, parent: FrameId) -> FrameEdge {
        FrameEdge { child, parent }
    }

    /// The cell for one frame, or `None` if the layout left it unplaced.
    fn cell_of(layout: &[(FrameId, LayoutCell)], frame: &FrameId) -> Option<LayoutCell> {
        layout
            .iter()
            .find_map(|(id, cell)| (id == frame).then_some(*cell))
    }

    /// A straight chain `base_link -> odom -> map` (map is the root) must fall on
    /// increasing depth: root at 0, each hop one deeper. This is the base case the
    /// whole panel geometry rests on.
    #[test]
    fn single_chain_assigns_increasing_depth() {
        let agent = AgentId::new("robot_1");
        let base_link = FrameId::base_link(agent.clone());
        let odom = FrameId::odom(agent.clone());
        let map = FrameId::map(agent.clone());

        let layout = tree_layout(&[
            edge(base_link.clone(), odom.clone()),
            edge(odom.clone(), map.clone()),
        ]);

        assert_eq!(cell_of(&layout, &map).map(|c| c.depth), Some(0));
        assert_eq!(cell_of(&layout, &odom).map(|c| c.depth), Some(1));
        assert_eq!(cell_of(&layout, &base_link).map(|c| c.depth), Some(2));
    }

    /// Two children of one parent share their parent's depth-plus-one and take
    /// distinct slots. The test pins the *contract* (same depth, contiguous unique
    /// slots) without pinning *which* sibling wins slot 0 — that tie-break is the
    /// implementation's to choose, as long as it is stable.
    #[test]
    fn siblings_share_depth_distinct_slots() {
        let agent = AgentId::new("robot_1");
        let odom = FrameId::odom(agent.clone());
        let base_link = FrameId::base_link(agent.clone());
        let imu = FrameId::sensor(agent.clone(), "imu");

        let layout = tree_layout(&[
            edge(base_link.clone(), odom.clone()),
            edge(imu.clone(), odom.clone()),
        ]);

        let a = cell_of(&layout, &base_link).expect("base_link is placed");
        let b = cell_of(&layout, &imu).expect("imu is placed");

        assert_eq!(a.depth, 1);
        assert_eq!(b.depth, 1);
        assert_ne!(a.slot, b.slot, "siblings may not share a slot");
        assert_eq!(
            [a.slot.min(b.slot), a.slot.max(b.slot)],
            [0, 1],
            "sibling slots at a depth are contiguous from 0"
        );
    }

    /// The load-bearing invariant: layout is a pure function of the edge *set*,
    /// not the edge *order*. Feed the same edges reversed and the placement must
    /// be byte-for-byte identical — otherwise the panel jitters as `edges()`
    /// reorders frame to frame.
    #[test]
    fn layout_is_order_independent() {
        let agent = AgentId::new("robot_1");
        let base_link = FrameId::base_link(agent.clone());
        let odom = FrameId::odom(agent.clone());
        let map = FrameId::map(agent.clone());
        let imu = FrameId::sensor(agent.clone(), "imu");

        let forward = [
            edge(base_link.clone(), odom.clone()),
            edge(odom.clone(), map.clone()),
            edge(imu.clone(), base_link.clone()),
        ];
        let mut reversed = forward.clone();
        reversed.reverse();

        assert_eq!(tree_layout(&forward), tree_layout(&reversed));
    }

    /// Two disjoint per-agent chains: the multi-agent case. Both roots land at
    /// depth 0 and every node is placed — the layout is a forest, never rejected
    /// for lacking a single shared root (helios has none across agents).
    #[test]
    fn forest_places_multiple_roots() {
        let a = AgentId::new("robot_a");
        let b = AgentId::new("robot_b");
        let (bl_a, od_a) = (FrameId::base_link(a.clone()), FrameId::odom(a.clone()));
        let (bl_b, od_b) = (FrameId::base_link(b.clone()), FrameId::odom(b.clone()));

        let layout = tree_layout(&[
            edge(bl_a.clone(), od_a.clone()),
            edge(bl_b.clone(), od_b.clone()),
        ]);

        assert_eq!(cell_of(&layout, &od_a).map(|c| c.depth), Some(0));
        assert_eq!(cell_of(&layout, &od_b).map(|c| c.depth), Some(0));
        assert_eq!(cell_of(&layout, &bl_a).map(|c| c.depth), Some(1));
        assert_eq!(cell_of(&layout, &bl_b).map(|c| c.depth), Some(1));
        assert_eq!(layout.len(), 4, "every node in the forest is placed");
    }

    /// A malformed buffer with a pure cycle `a -> b -> a` has no root. The layout
    /// must terminate (no infinite walk) and place nothing, since nothing is
    /// reachable from a root.
    #[test]
    fn cycle_terminates_without_placing_unreachable() {
        let agent = AgentId::new("robot_1");
        let a = FrameId::base_link(agent.clone());
        let b = FrameId::odom(agent.clone());

        let layout = tree_layout(&[edge(a.clone(), b.clone()), edge(b.clone(), a.clone())]);

        assert!(cell_of(&layout, &a).is_none());
        assert!(cell_of(&layout, &b).is_none());
        assert!(layout.is_empty());
    }
}
