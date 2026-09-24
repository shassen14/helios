//! Pure tree geometry for the 2D tf panel: topology in, abstract grid cells out.
//!
//! This is the panel's one non-trivial computation, deliberately lifted out of
//! any `bevy_ui` system into a plain function over `helios_core` types so it is
//! Tier-1 testable (no `App`). It answers only *where each frame sits* — a
//! `(depth, x)` cell per node. Turning a cell into pixels, drawing connector
//! lines, and hanging staleness badges off edges are all downstream renderer
//! concerns that never enter here.
//!
//! It takes the edge set alone (`TfBuffer::edges()` gives exactly this). Sample
//! stamps and rates are a separate overlay fed by `dynamic_edge_stats()`; they
//! do not affect geometry.
//!
//! The placement is a **tidy tree**: leaves are laid left-to-right at successive
//! whole-number `x`, and every parent is centred over the span of its children.
//! A frame with three sensors below it therefore sits above their midpoint, not
//! hard-left of them — the difference between a legible tree and a lopsided one.

use helios_core::spatial::{transforms::tf::stamped::FrameEdge, FrameId};

use std::collections::{HashMap, HashSet};

/// A node's abstract position in the panel grid, before any pixel mapping.
///
/// `depth` is the number of parent-ward hops from a root (a root is `depth 0`).
/// `x` is a horizontal coordinate in *slot units*, not an ordinal: leaves take
/// successive whole numbers left-to-right, and an internal node takes the midpoint
/// of its children's `x`, so a parent is centred over its subtree and may land on a
/// fraction (e.g. `1.5` above children at `1` and `2`). The renderer maps `depth`→row
/// and `x`→column; this type carries no units and no styling.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LayoutCell {
    pub depth: usize,
    pub x: f32,
}

/// Places every frame reachable from a root onto the panel grid.
///
/// A **root** is a frame that appears as a `parent` but never as a `child` — the
/// top of a per-agent estimated tree. There may be more than one: the
/// multi-agent panel lays out N unjoined per-agent graphs at once, so a forest
/// is expected input, not an error. Roots take `depth 0`, and because leaf `x`
/// runs continuously across the whole forest, each root's subtree occupies its
/// own horizontal band with no overlap.
///
/// The result must be **deterministic**: the same edge set in any order yields the
/// same layout. `edges()` and the buffer's backing `HashMap` are both unordered, so
/// the implementation imposes its own order — `FrameId` is `Eq + Hash` but **not
/// `Ord`**, so roots and siblings are sorted by their `Display` string, never by
/// hash-map iteration order, and both the placement *and the returned order* fall out
/// of that single deterministic walk.
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
    // Sort siblings once here so the recursive walk is a pure post-order over a
    // fixed order — the sole source of determinism for both x and output order.
    for kids in children_of.values_mut() {
        kids.sort_by_key(|id| id.to_string());
    }

    let mut layout: Vec<(FrameId, LayoutCell)> = Vec::new();
    let mut visited: HashSet<FrameId> = HashSet::new();
    let mut next_leaf_x = 0.0;

    for root in &roots {
        place_subtree(
            root,
            0,
            &children_of,
            &mut visited,
            &mut next_leaf_x,
            &mut layout,
        );
    }

    layout
}

/// Recursively places one subtree in post-order and returns the `x` assigned to its
/// root, so the caller can centre over it. A leaf claims the next free column; an
/// internal node claims the midpoint of its children's columns. The `visited` guard
/// makes a cycle finite: a frame reached a second time returns `None` and is neither
/// re-placed nor counted toward its parent's centre. A node whose children were *all*
/// cycle-skipped is treated as a leaf, so it still gets a column.
fn place_subtree(
    frame: &FrameId,
    depth: usize,
    children_of: &HashMap<FrameId, Vec<FrameId>>,
    visited: &mut HashSet<FrameId>,
    next_leaf_x: &mut f32,
    out: &mut Vec<(FrameId, LayoutCell)>,
) -> Option<f32> {
    if !visited.insert(frame.clone()) {
        return None;
    }

    let child_xs: Vec<f32> = children_of
        .get(frame)
        .into_iter()
        .flatten()
        .filter_map(|child| place_subtree(child, depth + 1, children_of, visited, next_leaf_x, out))
        .collect();

    let x = match (child_xs.first(), child_xs.last()) {
        (Some(first), Some(last)) => (first + last) / 2.0,
        _ => {
            // A leaf, or an internal node all of whose children were cycle-skipped.
            let x = *next_leaf_x;
            *next_leaf_x += 1.0;
            x
        }
    };

    out.push((frame.clone(), LayoutCell { depth, x }));
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::prelude::AgentId;

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
    /// distinct, adjacent columns; the parent is centred over them. This is the
    /// tidy-tree invariant that fixes the lopsided fan: `odom` sits at `0.5`, the
    /// midpoint of children at `0` and `1`.
    #[test]
    fn parent_is_centred_over_its_children() {
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
        let parent = cell_of(&layout, &odom).expect("odom is placed");

        assert_eq!(a.depth, 1);
        assert_eq!(b.depth, 1);
        assert_ne!(a.x, b.x, "siblings may not share a column");
        assert_eq!(
            [a.x.min(b.x), a.x.max(b.x)],
            [0.0, 1.0],
            "sibling columns are adjacent whole numbers",
        );
        assert_eq!(parent.depth, 0);
        assert_eq!(
            parent.x, 0.5,
            "the parent sits at the midpoint of its children"
        );
    }

    /// Three children spread `0, 1, 2`, and the parent lands at the middle child's
    /// column (`1`) — the midpoint of `0` and `2`. The regression guard that a wide
    /// fan stays symmetric under its parent rather than hanging off one side.
    #[test]
    fn parent_centres_over_an_odd_fan() {
        let agent = AgentId::new("robot_1");
        let base_link = FrameId::base_link(agent.clone());
        let gps = FrameId::sensor(agent.clone(), "gps");
        let imu = FrameId::sensor(agent.clone(), "imu");
        let mag = FrameId::sensor(agent.clone(), "mag");

        let layout = tree_layout(&[
            edge(gps.clone(), base_link.clone()),
            edge(imu.clone(), base_link.clone()),
            edge(mag.clone(), base_link.clone()),
        ]);

        let parent = cell_of(&layout, &base_link).expect("base_link is placed");
        assert_eq!(
            parent.x, 1.0,
            "the parent centres over three children at 0,1,2"
        );
    }

    /// The load-bearing invariant: layout is a pure function of the edge *set*,
    /// not the edge *order*. Feed the same edges reversed and the placement must
    /// be identical — otherwise the panel jitters as `edges()` reorders frame to
    /// frame.
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
    /// depth 0, take separate columns (leaf `x` runs continuously across the
    /// forest), and every node is placed — the layout is a forest, never rejected
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
        assert_ne!(
            cell_of(&layout, &od_a).unwrap().x,
            cell_of(&layout, &od_b).unwrap().x,
            "the two roots occupy separate columns",
        );
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
