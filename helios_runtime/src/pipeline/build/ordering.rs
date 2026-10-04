//! Ordering nodes into levels: each node runs in a later level than the
//! producers of its same-tick inputs, required or optional. Runs only after the
//! wiring checks pass.

use super::cycle::find_cycles;

use crate::{ChannelKey, NodeId, PipelineBuildError, PipelineNode};

use std::collections::HashSet;

/// Sorts `remaining` into levels and assigns each node its [`NodeId`].
///
/// `produced` holds the channels already written before any node runs: the
/// body's channels and the outside inputs. A node joins the first level in
/// which every same-tick input is in `produced`; that level's outputs are then
/// added for the next. Within a level, nodes are sorted by name, so levels and
/// ids do not depend on the order nodes were added. Ids run from `0` in
/// level-major order, the order the pipeline indexes its rate timers by.
///
/// If the sort gets stuck, the remaining nodes are each waiting on another
/// remaining node. One [`PipelineBuildError::Cycle`] is returned per loop
/// among them, leaving out nodes that are only downstream of a loop.
pub(super) fn order_into_levels(
    mut remaining: Vec<Box<dyn PipelineNode>>,
    mut produced: HashSet<ChannelKey>,
) -> Result<Vec<Vec<(NodeId, Box<dyn PipelineNode>)>>, Vec<PipelineBuildError>> {
    // Kahn's algorithm (level-by-level form). Each iteration pulls out
    // every node whose same-tick inputs are already produced, assigns
    // it a NodeId, and pushes it into the current level. The level's
    // outputs then enter `produced` so the next iteration can advance.
    let mut levels: Vec<Vec<(NodeId, Box<dyn PipelineNode>)>> = Vec::new();
    let mut next_id: NodeId = 0;

    while !remaining.is_empty() {
        let (mut ready, still_waiting): (Vec<_>, Vec<_>) =
            remaining.into_iter().partition(|node| {
                node.port_descriptor()
                    .same_tick_inputs()
                    .all(|channel| produced.contains(channel))
            });

        remaining = still_waiting;

        // No node is ready but some remain, so the sort is stuck. Wiring gave
        // every input a supplier, so each stuck node waits on another stuck
        // node, and there is at least one loop among them. Finding none
        // would be a bug here, not in the pipeline.
        if ready.is_empty() {
            let errors = find_cycles(&remaining);

            if errors.is_empty() {
                let nodes = remaining.iter().map(|n| n.name().to_string()).collect();
                return Err(vec![PipelineBuildError::StuckWithoutCycle { nodes }]);
            }

            return Err(errors);
        }

        // Promote this level's outputs into `produced` so the next
        // iteration can advance.
        produced.extend(
            ready
                .iter()
                .flat_map(|node| node.port_descriptor().outputs().iter())
                .cloned(),
        );

        // Sort by name so ids don't depend on the order nodes were added.
        // Assign NodeIds in level-major order as we go — this is the
        // same order the rate-timer array will be indexed by at tick
        // time, so the two stay in lockstep without a second pass.
        ready.sort_by(|a, b| a.name().cmp(b.name()));
        let mut level: Vec<(NodeId, Box<dyn PipelineNode>)> = Vec::with_capacity(ready.len());
        for node in ready {
            level.push((next_id, node));
            next_id += 1;
        }

        levels.push(level);
    }
    Ok(levels)
}
