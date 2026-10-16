//! Ordering nodes into levels: each node runs in a later level than every
//! producer of its same-tick inputs, required or optional. A keyed channel
//! may have several producers, and its readers wait for all of them. Runs
//! only after the wiring checks pass.

use super::{cycle::find_cycles, declaration::NodeDeclaration};

use crate::{ChannelKey, NodeId, PipelineBuildError};

use std::collections::{HashMap, HashSet};

/// Sorts `remaining` into levels and assigns each node its [`NodeId`].
///
/// `produced` holds the channels already written before any node runs: the
/// body's channels and the outside inputs. A node joins the first level in
/// which every same-tick input is in `produced` and no node still unplaced
/// writes it; that level's outputs are then added for the next. Within a level, nodes are sorted by name, so levels and
/// ids do not depend on the order nodes were added. Ids run from `0` in
/// level-major order, the order the pipeline indexes its rate timers by.
///
/// If the sort gets stuck, the remaining nodes are each waiting on another
/// remaining node. One [`PipelineBuildError::Cycle`] is returned per loop
/// among them, leaving out nodes that are only downstream of a loop.
pub(super) fn order_into_levels(
    mut remaining: Vec<NodeDeclaration>,
    mut produced: HashSet<ChannelKey>,
) -> Result<Vec<Vec<(NodeId, NodeDeclaration)>>, Vec<PipelineBuildError>> {
    // Kahn's algorithm (level-by-level form). Each iteration pulls out
    // every node whose same-tick inputs are already produced, assigns
    // it a NodeId, and pushes it into the current level. The level's
    // outputs then enter `produced` so the next iteration can advance.
    let mut levels: Vec<Vec<(NodeId, NodeDeclaration)>> = Vec::new();
    let mut next_id: NodeId = 0;

    // How many unplaced nodes write each channel. Only a keyed channel has
    // more than one, and a reader waits until the last of them is placed.
    let mut unplaced_writers: HashMap<ChannelKey, usize> = HashMap::new();
    for node in &remaining {
        for channel in distinct_outputs(node) {
            *unplaced_writers.entry(channel.clone()).or_default() += 1;
        }
    }

    while !remaining.is_empty() {
        let (mut ready, still_waiting): (Vec<_>, Vec<_>) =
            remaining.into_iter().partition(|node| {
                node.port_descriptor().same_tick_inputs().all(|channel| {
                    produced.contains(channel) && !unplaced_writers.contains_key(channel)
                })
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

        // Promote this level's outputs into `produced`, and count its nodes
        // off their channels' unplaced writers, so the next iteration can
        // advance.
        for node in &ready {
            for channel in distinct_outputs(node) {
                if let Some(count) = unplaced_writers.get_mut(channel) {
                    *count -= 1;
                    if *count == 0 {
                        unplaced_writers.remove(channel);
                    }
                }
                produced.insert(channel.clone());
            }
        }

        // Sort by name so ids don't depend on the order nodes were added.
        // Assign NodeIds in level-major order as we go — this is the
        // same order the rate-timer array will be indexed by at tick
        // time, so the two stay in lockstep without a second pass.
        ready.sort_by(|a, b| a.name().cmp(b.name()));
        let mut level: Vec<(NodeId, NodeDeclaration)> = Vec::with_capacity(ready.len());
        for node in ready {
            level.push((next_id, node));
            next_id += 1;
        }

        levels.push(level);
    }
    Ok(levels)
}

/// The channels `node` writes, each once: a node may declare several outputs
/// on one keyed channel, and is still one writer of it.
fn distinct_outputs(node: &NodeDeclaration) -> HashSet<&ChannelKey> {
    node.port_descriptor().outputs().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::pipeline::NodePlan;
    use crate::port::{ChannelKind, Keyed, PortDescriptor, PortDescriptorBuilder};

    struct Edge {
        child: &'static str,
    }

    impl Keyed for Edge {
        type Instance = &'static str;

        fn instance(&self) -> &&'static str {
            &self.child
        }
    }

    fn edges() -> ChannelKey {
        ChannelKey::keyed::<Edge>(ChannelKind::Internal, "edges")
    }

    fn plain(name: &'static str) -> ChannelKey {
        ChannelKey::internal::<u32>(name)
    }

    fn declared(name: &str, descriptor: PortDescriptor) -> NodeDeclaration {
        NodePlan::new(name, descriptor, Box::new(|_, _| Err("never built".into()))).into()
    }

    /// Each level's node names, in order.
    fn level_names(nodes: Vec<NodeDeclaration>) -> Vec<Vec<String>> {
        let levels = order_into_levels(nodes, HashSet::new()).expect("no loop");
        levels
            .iter()
            .map(|level| {
                level
                    .iter()
                    .map(|(_, node)| node.name().to_string())
                    .collect()
            })
            .collect()
    }

    /// The localizer writes its edge a level after the relay writes its own,
    /// so the reader waits for the localizer, not just the first writer.
    #[test]
    fn a_reader_of_a_keyed_channel_waits_for_every_writer() {
        let nodes = vec![
            declared(
                "source",
                PortDescriptorBuilder::algorithm()
                    .output(plain("a"))
                    .build(),
            ),
            declared(
                "relay",
                PortDescriptorBuilder::algorithm()
                    .output_keyed::<Edge>(edges(), ["base"])
                    .build(),
            ),
            declared(
                "localizer",
                PortDescriptorBuilder::algorithm()
                    .input(plain("a"))
                    .output_keyed::<Edge>(edges(), ["odom"])
                    .build(),
            ),
            declared(
                "reader",
                PortDescriptorBuilder::algorithm().input(edges()).build(),
            ),
        ];

        assert_eq!(
            level_names(nodes),
            [vec!["relay", "source"], vec!["localizer"], vec!["reader"]]
        );
    }

    /// A node declaring two outputs on one keyed channel is one writer of it:
    /// placing it releases the channel's readers.
    #[test]
    fn a_writer_declaring_one_keyed_channel_twice_is_counted_once() {
        let nodes = vec![
            declared(
                "relay",
                PortDescriptorBuilder::algorithm()
                    .output_keyed::<Edge>(edges(), ["base"])
                    .output_keyed::<Edge>(edges(), ["wheel"])
                    .build(),
            ),
            declared(
                "reader",
                PortDescriptorBuilder::algorithm().input(edges()).build(),
            ),
        ];

        assert_eq!(level_names(nodes), [vec!["relay"], vec!["reader"]]);
    }
}
