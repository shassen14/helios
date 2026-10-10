//! What a watching host calls on [`AutonomyPipeline`]: the catalog of what
//! each node can report, and the watch set that picks what it does report.

use super::{observable_catalog, ScheduledNode};

use crate::{
    observe::watch::{WatchError, WatchSet},
    port::Observable,
    AutonomyPipeline,
};

impl AutonomyPipeline {
    /// Iterates over every leaf a watcher can ask for, paired with the name of
    /// the node that reports it.
    ///
    /// Nodes come in [`NodeId`](crate::NodeId) order. Within a node, the
    /// leaves it declares come first, in declaration order, then the leaves
    /// the pipeline adds to every node.
    ///
    /// A host formats its own paths from these pairs and looks its requests
    /// up in them, rather than parsing paths apart. A comparison across runs
    /// reads each [`Observable`]'s determinism to skip wall-clock leaves.
    pub fn observables(&self) -> impl Iterator<Item = (&str, Observable)> + '_ {
        self.levels.iter().flatten().flat_map(|scheduled| {
            let name = scheduled.node.name();
            observable_catalog(scheduled.node.as_ref())
                .into_iter()
                .map(move |observable| (name, observable))
        })
    }

    /// Replaces the whole watch set: from now on, exactly the leaves `set`
    /// names record what their nodes emit. A built pipeline watches nothing.
    ///
    /// All or nothing: if any entry names a node that doesn't exist, or a leaf
    /// its node doesn't declare, every such entry is reported and the previous
    /// watch set stays in place.
    ///
    /// Observations already recorded are kept until the next drain, including
    /// those of leaves this call stops watching, so drain before changing the
    /// watch set.
    pub fn watch(&mut self, set: &WatchSet) -> Result<(), Vec<WatchError>> {
        let leaves = match set {
            WatchSet::Everything => {
                for scheduled in self.levels.iter_mut().flatten() {
                    scheduled.observations.set_all_watched(true);
                }
                return Ok(());
            }
            WatchSet::Leaves(leaves) => leaves,
        };

        let errors: Vec<WatchError> = leaves
            .iter()
            .filter_map(|entry| match self.scheduled(&entry.node) {
                None => Some(WatchError::UnknownNode {
                    node: entry.node.clone(),
                    leaf: entry.leaf.clone(),
                }),
                Some(scheduled) if !scheduled.observations.declares(&entry.leaf) => {
                    Some(WatchError::UndeclaredLeaf {
                        node: entry.node.clone(),
                        leaf: entry.leaf.clone(),
                    })
                }
                Some(_) => None,
            })
            .collect();

        if !errors.is_empty() {
            return Err(errors);
        }

        // Walk the nodes rather than the entries: each node is visited once,
        // and no node has to be looked up by name while `levels` is borrowed
        // mutably.
        for scheduled in self.levels.iter_mut().flatten() {
            scheduled.observations.set_all_watched(false);

            let name = scheduled.node.name();
            for entry in leaves.iter().filter(|entry| entry.node == name) {
                let declared = scheduled.observations.set_watched(&entry.leaf, true);
                debug_assert!(declared, "leaf \"{}\" was checked above", entry.leaf);
            }
        }

        Ok(())
    }

    /// The node named `name`, if any. Node names are unique; the build
    /// refuses a pipeline where they are not.
    fn scheduled(&self, name: &str) -> Option<&ScheduledNode> {
        self.levels
            .iter()
            .flatten()
            .find(|scheduled| scheduled.node.name() == name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        observe::watch::WatchedLeaf,
        pipeline::autonomy_pipeline::TICK_DURATION_LEAF,
        port::{Determinism, MockNodePortDescriptor, PortBus, PortDescriptor},
        prelude::{PipelineNode, TickContext},
        PipelineBuilder,
    };

    use helios_core::prelude::TfProvider;

    const FIRST: &str = "first";
    const SECOND: &str = "second";
    const NIS: &str = "aiding.gps.nis";
    const DROPPED: &str = "aiding.gps.dropped";
    const UNDECLARED: &str = "aiding.gps.undeclared";
    const MISSING_NODE: &str = "missing";

    /// A node that only declares leaves; it never runs in these tests.
    struct Declaring {
        name: String,
        descriptor: PortDescriptor,
    }

    impl PipelineNode for Declaring {
        fn name(&self) -> &str {
            &self.name
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {}
    }

    fn declaring(name: &str, leaves: &[&str]) -> Box<dyn PipelineNode> {
        let descriptor = leaves
            .iter()
            .fold(MockNodePortDescriptor::new(), |builder, leaf| {
                builder.observable(*leaf, Determinism::Reproducible)
            })
            .build();
        Box::new(Declaring {
            name: name.to_string(),
            descriptor,
        })
    }

    /// `first` declares NIS and the drop count; `second` declares NIS only.
    fn pipeline() -> AutonomyPipeline {
        PipelineBuilder::new()
            .add_node(declaring(FIRST, &[NIS, DROPPED]))
            .add_node(declaring(SECOND, &[NIS]))
            .build()
            .expect("two independent nodes build")
    }

    fn leaves(pairs: &[(&str, &str)]) -> WatchSet {
        WatchSet::Leaves(
            pairs
                .iter()
                .map(|(node, leaf)| WatchedLeaf::new(*node, *leaf))
                .collect(),
        )
    }

    /// Every `(node, leaf)` in the catalog, in node-id order.
    fn catalog(pipeline: &AutonomyPipeline) -> Vec<(String, String)> {
        pipeline
            .observables()
            .map(|(node, observable)| (node.to_string(), observable.leaf_name().to_string()))
            .collect()
    }

    /// Every `(node, leaf)` currently watched, in node-id order.
    fn watched(pipeline: &AutonomyPipeline) -> Vec<(String, String)> {
        catalog(pipeline)
            .into_iter()
            .filter(|(node, leaf)| {
                pipeline
                    .scheduled(node)
                    .and_then(|scheduled| scheduled.observations.watched_leaf(leaf))
                    .is_some()
            })
            .collect()
    }

    fn pairs(pairs: &[(&str, &str)]) -> Vec<(String, String)> {
        pairs
            .iter()
            .map(|(node, leaf)| (node.to_string(), leaf.to_string()))
            .collect()
    }

    #[test]
    fn observables_list_every_node_in_order_each_ending_with_tick_duration() {
        assert_eq!(
            catalog(&pipeline()),
            pairs(&[
                (FIRST, NIS),
                (FIRST, DROPPED),
                (FIRST, TICK_DURATION_LEAF),
                (SECOND, NIS),
                (SECOND, TICK_DURATION_LEAF),
            ])
        );
    }

    #[test]
    fn built_pipeline_watches_nothing() {
        assert!(watched(&pipeline()).is_empty());
    }

    #[test]
    fn everything_watches_every_leaf_on_every_node() {
        let mut pipeline = pipeline();
        pipeline.watch(&WatchSet::Everything).expect("cannot fail");

        assert_eq!(watched(&pipeline), catalog(&pipeline));
    }

    #[test]
    fn leaves_watch_exactly_the_listed_pairs() {
        let mut pipeline = pipeline();
        pipeline
            .watch(&leaves(&[(FIRST, NIS), (SECOND, TICK_DURATION_LEAF)]))
            .expect("both pairs are declared");

        assert_eq!(
            watched(&pipeline),
            pairs(&[(FIRST, NIS), (SECOND, TICK_DURATION_LEAF)])
        );
    }

    #[test]
    fn replacing_the_set_unwatches_what_it_drops() {
        let mut pipeline = pipeline();
        pipeline.watch(&leaves(&[(FIRST, NIS)])).expect("declared");
        pipeline.watch(&leaves(&[(SECOND, NIS)])).expect("declared");

        assert_eq!(watched(&pipeline), pairs(&[(SECOND, NIS)]));
    }

    #[test]
    fn default_set_after_everything_watches_nothing() {
        let mut pipeline = pipeline();
        pipeline.watch(&WatchSet::Everything).expect("cannot fail");
        pipeline
            .watch(&WatchSet::default())
            .expect("empty set is valid");

        assert!(watched(&pipeline).is_empty());
    }

    #[test]
    fn unknown_node_and_undeclared_leaf_are_refused_together_and_change_nothing() {
        let mut pipeline = pipeline();
        pipeline.watch(&leaves(&[(FIRST, NIS)])).expect("declared");

        let errors = pipeline
            .watch(&leaves(&[
                (SECOND, NIS),
                (FIRST, UNDECLARED),
                (MISSING_NODE, NIS),
            ]))
            .expect_err("two entries are unknown");

        assert_eq!(
            errors,
            vec![
                WatchError::UndeclaredLeaf {
                    node: FIRST.to_string(),
                    leaf: UNDECLARED.to_string(),
                },
                WatchError::UnknownNode {
                    node: MISSING_NODE.to_string(),
                    leaf: NIS.to_string(),
                },
            ]
        );
        assert_eq!(watched(&pipeline), pairs(&[(FIRST, NIS)]));
    }

    #[test]
    fn leaf_of_another_node_is_undeclared_on_the_named_one() {
        let mut pipeline = pipeline();

        let errors = pipeline
            .watch(&leaves(&[(SECOND, DROPPED)]))
            .expect_err("only first declares the drop count");

        assert_eq!(
            errors,
            vec![WatchError::UndeclaredLeaf {
                node: SECOND.to_string(),
                leaf: DROPPED.to_string(),
            }]
        );
    }
}
