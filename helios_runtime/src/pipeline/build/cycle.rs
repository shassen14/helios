//! Finding the loops among the nodes the level sort could not place.
//!
//! When the sort gets stuck, every remaining node is waiting on another
//! remaining node, but not all of them are in a loop: some are only downstream
//! of one. [`find_cycles`] builds the graph of same-tick reads among the stuck
//! nodes, splits it into strongly connected components with Tarjan's
//! algorithm, and reports each component that is a loop.

use crate::{ChannelKey, CycleEdge, PipelineBuildError, PipelineNode};

use std::collections::HashMap;

/// Returns one [`PipelineBuildError::Cycle`] per loop among `stuck`, the nodes
/// the level sort could not place.
///
/// A loop is a set of two or more nodes in which each can reach every other by
/// following same-tick reads, or a single node that reads its own output in
/// the same tick. A stuck node that is only downstream of a loop is in no
/// error. Each error lists its nodes by name and only the edges between them;
/// errors are sorted by their first node's name, so the result does not depend
/// on the order of `stuck`.
///
/// Every stuck node waits on another stuck node, so following those waits
/// always comes back round to some node: a non-empty `stuck` always yields at
/// least one error.
pub(super) fn find_cycles(stuck: &[Box<dyn PipelineNode>]) -> Vec<PipelineBuildError> {
    let mut nodes: Vec<&dyn PipelineNode> = stuck.iter().map(|n| n.as_ref()).collect();

    // From here on a node is named by its position in `nodes`. Sorting by name
    // first means position order is name order.
    nodes.sort_by(|a, b| a.name().cmp(b.name()));

    let n = nodes.len();

    // The stuck node that writes each channel. Wiring allows one writer per
    // channel, so no entry is overwritten.
    let mut writer: HashMap<&ChannelKey, usize> = HashMap::new();

    for (i, node) in nodes.iter().enumerate() {
        for channel in node.port_descriptor().outputs() {
            writer.insert(channel, i);
        }
    }

    // `adjacency[p]` lists each (consumer, channel) that reads a channel node
    // `p` writes, in consumer order. An input with no entry in `writer` comes
    // from outside the stuck set and is already satisfied, so it is no edge.
    let mut adjacency: Vec<Vec<(usize, &ChannelKey)>> = vec![Vec::new(); n];

    for (consumer, node) in nodes.iter().enumerate() {
        for channel in node.port_descriptor().same_tick_inputs() {
            if let Some(&producer) = writer.get(channel) {
                adjacency[producer].push((consumer, channel));
            }
        }
    }

    let components = SccSearch::components(&adjacency);

    // A single-node component is a loop only if the node reads its own
    // output; otherwise it is a node downstream of a loop.
    let mut loops: Vec<Vec<usize>> = components
        .into_iter()
        .filter(|c| match c.as_slice() {
            [v] => adjacency[*v].iter().any(|&(w, _)| w == *v),
            _ => true,
        })
        .collect();

    // Positions follow name order, so sorting by position sorts by name. No
    // node is in two loops, so the first positions never tie.
    for l in &mut loops {
        l.sort();
    }
    loops.sort_by_key(|l| l[0]);

    let mut errors = Vec::with_capacity(loops.len());

    for l in &loops {
        let mut member = vec![false; n];
        for &i in l {
            member[i] = true;
        }

        let participants: Vec<String> = l.iter().map(|&i| nodes[i].name().to_string()).collect();

        // Members are walked in order and each adjacency list is in consumer
        // order, so edges come out sorted by producer, then consumer. Edges
        // leaving the loop are dropped.
        let mut edges = Vec::new();
        for &p in l {
            for &(c, channel) in &adjacency[p] {
                if member[c] {
                    edges.push(CycleEdge {
                        producer: nodes[p].name().to_string(),
                        consumer: nodes[c].name().to_string(),
                        channel: channel.clone(),
                    });
                }
            }
        }

        errors.push(PipelineBuildError::Cycle {
            participants,
            edges,
        });
    }

    errors
}

/// The state of one run of Tarjan's strongly connected components search
/// over an adjacency list, nodes named by position.
///
/// The search is depth-first. Each node is numbered in the order it is first
/// reached, and tracks the lowest number it can get back to through nodes not
/// yet assigned to a component. A node that cannot get back past its own
/// number is the first-reached node of its component, and closes it.
struct SccSearch {
    /// When each node was first reached; `None` until then.
    index: Vec<Option<usize>>,
    /// The lowest `index` each node is known to reach through nodes still on
    /// `stack`. Starts at the node's own index and only decreases.
    low: Vec<usize>,
    /// Whether each node is on `stack`.
    on_stack: Vec<bool>,
    /// Nodes reached but not yet assigned to a component, in the order
    /// reached. A node stays here after its own visit returns, until the
    /// first-reached node of its component closes it.
    stack: Vec<usize>,
    /// The next `index` to hand out.
    counter: usize,
    /// Each closed component, in the order closed.
    components: Vec<Vec<usize>>,
}

impl SccSearch {
    /// Splits the graph into strongly connected components, every node in
    /// exactly one. `adjacency[p]` lists the edges leaving node `p`; only the
    /// target of each edge is read. A node in no loop is a component of its
    /// own.
    fn components(adjacency: &[Vec<(usize, &ChannelKey)>]) -> Vec<Vec<usize>> {
        let n = adjacency.len();
        let mut search = SccSearch {
            index: vec![None; n],
            low: vec![0; n],
            on_stack: vec![false; n],
            stack: Vec::new(),
            counter: 0,
            components: Vec::new(),
        };

        for v in 0..n {
            if search.index[v].is_none() {
                search.visit(v, adjacency);
            }
        }

        search.components
    }

    /// Reaches `v`, searches every node reachable from it that is not yet
    /// reached, then closes `v`'s component if `v` was its first-reached node.
    ///
    /// Recursion depth is at most the number of nodes, which here is the stuck
    /// nodes of one pipeline.
    fn visit(&mut self, v: usize, adjacency: &[Vec<(usize, &ChannelKey)>]) {
        // Number `v` and put it on the stack, awaiting its component.
        let v_index = self.counter;
        self.index[v] = Some(v_index);
        self.low[v] = v_index;
        self.counter += 1;
        self.stack.push(v);
        self.on_stack[v] = true;

        // For each edge v → w:
        // - w not yet reached: search it; whatever w gets back to, v does too.
        // - w reached and still on the stack: w is open in this search, so v
        //   gets back to w's index.
        // - w reached and off the stack: w is in a closed component that
        //   cannot get back to v, so the edge plays no part.
        for &(w, _) in &adjacency[v] {
            match self.index[w] {
                None => {
                    self.visit(w, adjacency);
                    self.low[v] = self.low[v].min(self.low[w]);
                }
                Some(w_index) => {
                    if self.on_stack[w] {
                        self.low[v] = self.low[v].min(w_index);
                    }
                }
            }
        }

        // Nothing reachable from `v` gets back past it, so `v` is its
        // component's first-reached node. Everything above it on the stack was
        // reached from `v` and gets back no further, so it is the rest of the
        // component.
        if self.low[v] == v_index {
            let mut component = Vec::new();

            while let Some(x) = self.stack.pop() {
                self.on_stack[x] = false;
                component.push(x);
                if x == v {
                    break;
                }
            }

            self.components.push(component);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::channel::InternalChannel;

    /// Builds an adjacency list over `n` nodes from (producer, consumer)
    /// pairs. The search reads only the target of each edge, so every edge
    /// shares one channel.
    fn adjacency<'a>(
        n: usize,
        edges: &[(usize, usize)],
        channel: &'a ChannelKey,
    ) -> Vec<Vec<(usize, &'a ChannelKey)>> {
        let mut adjacency = vec![Vec::new(); n];
        for &(p, c) in edges {
            adjacency[p].push((c, channel));
        }
        adjacency
    }

    /// Components with each one's nodes sorted, so tests compare membership
    /// rather than pop order.
    fn sorted_components(adjacency: &[Vec<(usize, &ChannelKey)>]) -> Vec<Vec<usize>> {
        let mut components = SccSearch::components(adjacency);
        for c in &mut components {
            c.sort();
        }
        components
    }

    #[test]
    fn loop_closes_after_its_downstream_node() {
        // 0 → 1 → 2 → 0 is a loop; 3 only reads from 2. 3 closes first, on
        // its own, then 0 closes the loop, popping 2 and 1 above it.
        let ch: ChannelKey = InternalChannel::of::<u32>().into();
        let adj = adjacency(4, &[(0, 1), (1, 2), (2, 0), (2, 3)], &ch);

        assert_eq!(SccSearch::components(&adj), vec![vec![3], vec![2, 1, 0]]);
    }

    #[test]
    fn back_edge_uses_the_target_index_not_its_position() {
        // 2 is reached second (index 1) and 1 third (index 2). 1's edge back
        // to 2 must lower 1's low to 2's index, 1. Reading low at position 1
        // instead would leave it at 2 and split the loop {1, 2}.
        let ch: ChannelKey = InternalChannel::of::<u32>().into();
        let adj = adjacency(3, &[(0, 2), (2, 1), (1, 2)], &ch);

        assert_eq!(sorted_components(&adj), vec![vec![1, 2], vec![0]]);
    }

    #[test]
    fn closed_component_is_not_joined_by_a_later_edge_into_it() {
        // 0 is visited first and closes alone. 1 and 2 form a loop and 2 also
        // feeds 0, which is off the stack by then and stays out of the loop.
        let ch: ChannelKey = InternalChannel::of::<u32>().into();
        let adj = adjacency(3, &[(1, 2), (2, 1), (2, 0)], &ch);

        assert_eq!(sorted_components(&adj), vec![vec![0], vec![1, 2]]);
    }

    #[test]
    fn graph_without_edges_gives_one_component_per_node() {
        let ch: ChannelKey = InternalChannel::of::<u32>().into();
        let adj = adjacency(3, &[], &ch);

        assert_eq!(SccSearch::components(&adj), vec![vec![0], vec![1], vec![2]]);
    }

    #[test]
    fn self_edge_is_a_component_of_its_own() {
        // The search alone cannot tell a self-loop from a lone node; the
        // self-edge check in `find_cycles` does that.
        let ch: ChannelKey = InternalChannel::of::<u32>().into();
        let adj = adjacency(2, &[(0, 0), (0, 1)], &ch);

        assert_eq!(SccSearch::components(&adj), vec![vec![1], vec![0]]);
    }
}
