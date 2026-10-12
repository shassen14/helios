//! What a watching host calls on [`AutonomyPipeline`]: the catalog of what
//! each node can report, the watch set that picks what it does report, and
//! the drain that collects what was reported.

use super::{observable_catalog, ScheduledNode};

use crate::{
    observe::{
        observation::Observation,
        watch::{WatchError, WatchSet},
    },
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
    /// reads each [`Observable`]'s determinism to skip nondeterministic leaves.
    pub fn observables(&self) -> impl Iterator<Item = (&str, Observable)> + '_ {
        self.levels.iter().flatten().flat_map(|scheduled| {
            let name = scheduled.node.name();
            observable_catalog(scheduled.node.as_ref())
                .into_iter()
                .map(move |observable| (name, observable))
        })
    }

    /// Moves everything recorded since the last drain onto the end of `out`,
    /// leaving every node's buffer empty.
    ///
    /// Observations come grouped by node, in [`NodeId`](crate::NodeId) order,
    /// and in emit order within a node; a caller wanting one timeline sorts
    /// by timestamp. Appending lets a host reuse one list every tick, and
    /// drain several pipelines into it.
    ///
    /// The host must drain after every tick: a node with watched leaves
    /// records on every run, and nothing else empties its buffer.
    pub fn drain_into(&self, out: &mut Vec<Observation>) {
        for scheduled in self.levels.iter().flatten() {
            scheduled.observations.drain_into(out);
        }
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
        observe::{
            observation::{ObservedValue, ObservedValueKind},
            watch::WatchedLeaf,
        },
        pipeline::autonomy_pipeline::TICK_DURATION_LEAF,
        port::{
            ChannelKey, Determinism, InternalChannel, MockNodePortDescriptor, PortBus,
            PortDescriptor,
        },
        prelude::{Health, PipelineNode, Stamped, TickContext},
        ErasedStamped, PipelineBuilder,
    };

    use helios_core::{
        prelude::{FrameId, MonotonicTime, TfProvider},
        spatial::transforms::ErasedTransform,
    };

    use std::sync::Arc;

    const FIRST: &str = "first";
    const SECOND: &str = "second";
    const WRITER: &str = "writer";
    const NIS: &str = "aiding.gps.nis";
    const DROPPED: &str = "aiding.gps.dropped";
    const UNDECLARED: &str = "aiding.gps.undeclared";
    const MISSING_NODE: &str = "missing";
    /// The writer's output instance, and the tap leaf it is watched at.
    const ESTIMATE: &str = "odom/estimate";
    const ESTIMATE_TAP: &str = "odom.estimate";
    const STEP: f64 = 0.01;
    /// How far before the tick's `now` a written value is stamped, so a tap's
    /// timestamp can be told apart from the tick's.
    const MEASUREMENT_LAG: f64 = 0.5;
    /// A rate whose period, 1/30 s, falls between whole steps: the node runs on
    /// the fourth tick and then not before the eighth.
    const SLOWER_THAN_STEP_HZ: f64 = 30.0;

    /// A node that, each run, emits every leaf it declares once, valued and
    /// stamped with the tick's `now`, then writes each of its outputs when
    /// `now` is one of `writes_at`. The written value is `now`, stamped
    /// [`MEASUREMENT_LAG`] earlier.
    struct Emitting {
        name: String,
        descriptor: PortDescriptor,
        writes_at: Vec<f64>,
    }

    impl PipelineNode for Emitting {
        fn name(&self) -> &str {
            &self.name
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, bus: &PortBus, _tf: &dyn TfProvider, tick: TickContext) {
            for observable in self.descriptor.observables() {
                tick.emit(observable.leaf_name(), tick.now, tick.now.0);
            }
            if !self.writes_at.contains(&tick.now.0) {
                return;
            }
            for output in self.descriptor.outputs() {
                let stamped = Stamped {
                    value: tick.now.0,
                    timestamp: MonotonicTime(tick.now.0 - MEASUREMENT_LAG),
                    health: Health::Ok,
                    producer: tick.node_id,
                };
                bus.write(output.clone(), stamped)
                    .expect("the bus has a slot for every declared output");
            }
        }
    }

    fn emitting(name: &str, leaves: &[&str]) -> Box<dyn PipelineNode> {
        Box::new(Emitting {
            name: name.to_string(),
            descriptor: declaring(leaves).build(),
            writes_at: vec![],
        })
    }

    /// A node declaring `leaves` and outputting [`ESTIMATE`], written on the
    /// runs at `writes_at`.
    fn writing(leaves: &[&str], writes_at: &[f64]) -> Box<dyn PipelineNode> {
        writing_with(declaring(leaves), writes_at)
    }

    /// [`writing`] gated to `hz`.
    fn writing_at_rate(hz: f64, writes_at: &[f64]) -> Box<dyn PipelineNode> {
        writing_with(declaring(&[]).rate_hz(hz), writes_at)
    }

    fn writing_with(
        descriptor: MockNodePortDescriptor,
        writes_at: &[f64],
    ) -> Box<dyn PipelineNode> {
        Box::new(Emitting {
            name: WRITER.to_string(),
            descriptor: descriptor.output_internal(estimate()).build(),
            writes_at: writes_at.to_vec(),
        })
    }

    fn declaring(leaves: &[&str]) -> MockNodePortDescriptor {
        leaves
            .iter()
            .fold(MockNodePortDescriptor::new(), |builder, leaf| {
                builder.observable(*leaf, Determinism::Deterministic)
            })
    }

    fn estimate() -> InternalChannel {
        InternalChannel::named::<f64>(ESTIMATE)
    }

    /// `first` declares NIS and the drop count; `second` declares NIS only.
    fn pipeline() -> AutonomyPipeline {
        PipelineBuilder::new()
            .add_node(emitting(FIRST, &[NIS, DROPPED]))
            .add_node(emitting(SECOND, &[NIS]))
            .build()
            .expect("two independent nodes build")
    }

    /// `writer` declares NIS and outputs [`ESTIMATE`], written on the runs at
    /// `writes_at`; `second` declares NIS and outputs nothing.
    fn tapped_pipeline(writes_at: &[f64]) -> AutonomyPipeline {
        built(writing(&[NIS], writes_at))
    }

    /// `writer` in a pipeline beside `second`. Neither reads the other, so they
    /// share a level, where nodes run in name order: `second` first.
    fn built(writer: Box<dyn PipelineNode>) -> AutonomyPipeline {
        PipelineBuilder::new()
            .add_node(writer)
            .add_node(emitting(SECOND, &[NIS]))
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

    struct NoTransforms;

    impl TfProvider for NoTransforms {
        fn get_transform(
            &self,
            _from: FrameId,
            _to: FrameId,
            _at: MonotonicTime,
        ) -> Option<ErasedTransform> {
            None
        }
    }

    /// Ticks once at each time in `times`. The nodes declare no rate, so
    /// every node runs every tick.
    fn run(pipeline: &AutonomyPipeline, times: &[f64]) {
        for t in times {
            pipeline.tick(MonotonicTime(*t), STEP, &NoTransforms);
        }
    }

    fn drained(pipeline: &AutonomyPipeline) -> Vec<Observation> {
        let mut out = Vec::new();
        pipeline.drain_into(&mut out);
        out
    }

    /// What an [`Emitting`] node records for `leaf` on the run at `t`.
    fn emitted(node: &str, leaf: &str, t: f64) -> Observation {
        Observation {
            node: node.into(),
            leaf: leaf.into(),
            timestamp: MonotonicTime(t),
            value: ObservedValue::Scalar(t),
        }
    }

    /// What a watched tap on [`ESTIMATE`] records for `written`: the bus
    /// value itself, at the writer's tap leaf, stamped with the value's time.
    fn tapped(written: Arc<dyn ErasedStamped>) -> Observation {
        Observation {
            node: WRITER.into(),
            leaf: ESTIMATE_TAP.into(),
            timestamp: written.timestamp(),
            value: ObservedValue::Typed(written),
        }
    }

    /// The value now in the writer's [`ESTIMATE`] slot.
    fn written(pipeline: &AutonomyPipeline) -> Arc<dyn ErasedStamped> {
        pipeline
            .bus()
            .read_erased_versioned(&ChannelKey::from(estimate()))
            .map(|(written, _)| written)
            .expect("the writer has written its output")
    }

    /// The `(node, leaf)` of each observation, in drained order.
    fn sources(observations: &[Observation]) -> Vec<(String, String)> {
        observations
            .iter()
            .map(|o| (o.node.to_string(), o.leaf.to_string()))
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

    #[test]
    fn drain_gives_each_node_in_id_order_in_emit_order_then_empties() {
        let mut pipeline = pipeline();
        pipeline
            .watch(&leaves(&[(FIRST, NIS), (FIRST, DROPPED), (SECOND, NIS)]))
            .expect("all declared");

        run(&pipeline, &[1.0, 2.0]);

        assert_eq!(
            drained(&pipeline),
            vec![
                emitted(FIRST, NIS, 1.0),
                emitted(FIRST, DROPPED, 1.0),
                emitted(FIRST, NIS, 2.0),
                emitted(FIRST, DROPPED, 2.0),
                emitted(SECOND, NIS, 1.0),
                emitted(SECOND, NIS, 2.0),
            ]
        );
        assert!(drained(&pipeline).is_empty());
    }

    #[test]
    fn nothing_watched_drains_nothing() {
        let pipeline = pipeline();
        run(&pipeline, &[1.0]);

        assert!(drained(&pipeline).is_empty());
    }

    #[test]
    fn unwatched_leaves_beside_a_watched_one_are_not_recorded() {
        let mut pipeline = pipeline();
        pipeline.watch(&leaves(&[(FIRST, NIS)])).expect("declared");

        run(&pipeline, &[1.0]);

        assert_eq!(drained(&pipeline), vec![emitted(FIRST, NIS, 1.0)]);
    }

    #[test]
    fn watched_tick_duration_is_one_sample_per_run_stamped_with_now() {
        let mut pipeline = pipeline();
        pipeline
            .watch(&leaves(&[(SECOND, TICK_DURATION_LEAF)]))
            .expect("every node has a duration");

        run(&pipeline, &[1.0, 2.0]);
        let observations = drained(&pipeline);

        assert_eq!(
            sources(&observations),
            pairs(&[(SECOND, TICK_DURATION_LEAF), (SECOND, TICK_DURATION_LEAF)])
        );
        let stamps: Vec<MonotonicTime> = observations.iter().map(|o| o.timestamp).collect();
        assert_eq!(stamps, vec![MonotonicTime(1.0), MonotonicTime(2.0)]);
        for observation in &observations {
            let ObservedValue::Scalar(seconds) = observation.value else {
                panic!("a duration is a scalar, got {:?}", observation.value);
            };
            assert!(seconds.is_finite() && seconds >= 0.0, "duration {seconds}");
        }
    }

    #[test]
    fn tick_duration_follows_the_nodes_own_emits() {
        let mut pipeline = pipeline();
        pipeline.watch(&WatchSet::Everything).expect("cannot fail");

        run(&pipeline, &[1.0]);

        assert_eq!(
            sources(&drained(&pipeline)),
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
    fn observables_list_a_tap_per_output_between_declared_leaves_and_tick_duration() {
        assert_eq!(
            catalog(&tapped_pipeline(&[])),
            pairs(&[
                (SECOND, NIS),
                (SECOND, TICK_DURATION_LEAF),
                (WRITER, NIS),
                (WRITER, ESTIMATE_TAP),
                (WRITER, TICK_DURATION_LEAF),
            ])
        );
    }

    #[test]
    fn watching_a_channel_the_node_does_not_output_is_refused() {
        let mut pipeline = tapped_pipeline(&[]);

        let errors = pipeline
            .watch(&leaves(&[(SECOND, ESTIMATE_TAP)]))
            .expect_err("only the writer outputs the estimate");

        assert_eq!(
            errors,
            vec![WatchError::UndeclaredLeaf {
                node: SECOND.to_string(),
                leaf: ESTIMATE_TAP.to_string(),
            }]
        );
    }

    // The first write to a slot moves its version from none to some, so this
    // also covers a channel that held nothing before the run.
    #[test]
    fn watched_tap_records_the_bus_value_itself_stamped_with_its_own_time() {
        let mut pipeline = tapped_pipeline(&[1.0]);
        pipeline
            .watch(&leaves(&[(WRITER, ESTIMATE_TAP)]))
            .expect("the writer outputs the estimate");

        run(&pipeline, &[1.0]);
        let observations = drained(&pipeline);

        // `Typed` compares by `Arc::ptr_eq`, so equality means the tap holds
        // the bus's own value, not a copy.
        assert_eq!(observations, vec![tapped(written(&pipeline))]);
        assert_eq!(
            observations[0].timestamp,
            MonotonicTime(1.0 - MEASUREMENT_LAG)
        );
    }

    #[test]
    fn run_that_leaves_its_output_unwritten_records_nothing() {
        let mut pipeline = tapped_pipeline(&[1.0, 3.0]);
        pipeline
            .watch(&leaves(&[(WRITER, ESTIMATE_TAP)]))
            .expect("the writer outputs the estimate");

        run(&pipeline, &[1.0]);
        let first = drained(&pipeline);
        run(&pipeline, &[2.0]);
        let unwritten = drained(&pipeline);
        run(&pipeline, &[3.0]);
        let second = drained(&pipeline);

        // On the run at 2.0 the slot still holds the first value; only its
        // unchanged version says the node didn't write it again.
        assert_eq!(first.len(), 1);
        assert!(unwritten.is_empty(), "recorded {unwritten:?}");
        assert_eq!(second, vec![tapped(written(&pipeline))]);
        assert_ne!(first, second);
    }

    #[test]
    fn rate_skipped_node_records_nothing() {
        let mut pipeline = built(writing_at_rate(SLOWER_THAN_STEP_HZ, &[4.0, 5.0, 6.0, 7.0]));
        pipeline
            .watch(&leaves(&[(WRITER, ESTIMATE_TAP)]))
            .expect("the writer outputs the estimate");

        // The node would write at 5.0–7.0, but its rate keeps it from running
        // after 4.0.
        run(&pipeline, &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]);

        assert_eq!(drained(&pipeline), vec![tapped(written(&pipeline))]);
    }

    #[test]
    fn unwatched_tap_records_nothing_beside_a_watched_leaf() {
        let mut pipeline = tapped_pipeline(&[1.0]);
        pipeline.watch(&leaves(&[(WRITER, NIS)])).expect("declared");

        run(&pipeline, &[1.0]);

        assert_eq!(drained(&pipeline), vec![emitted(WRITER, NIS, 1.0)]);
    }

    #[test]
    fn every_recorded_value_is_the_kind_its_leaf_promises() {
        let mut pipeline = tapped_pipeline(&[1.0]);
        pipeline.watch(&WatchSet::Everything).expect("cannot fail");
        let promised: Vec<(String, String, ObservedValueKind)> = pipeline
            .observables()
            .map(|(node, observable)| {
                (
                    node.to_string(),
                    observable.leaf_name().to_string(),
                    observable.kind(),
                )
            })
            .collect();

        run(&pipeline, &[1.0]);
        let observations = drained(&pipeline);

        // Both kinds are covered: declared leaves and durations are scalars,
        // the estimate tap is typed.
        assert!(observations
            .iter()
            .any(|o| matches!(o.value, ObservedValue::Scalar(_))));
        assert!(observations
            .iter()
            .any(|o| matches!(o.value, ObservedValue::Typed(_))));
        for observation in &observations {
            let kind = promised
                .iter()
                .find(|(node, leaf, _)| {
                    node.as_str() == observation.node.as_ref()
                        && leaf.as_str() == observation.leaf.as_ref()
                })
                .map(|(_, _, kind)| *kind);
            assert_eq!(
                kind,
                Some(observation.value.kind()),
                "{}.{} arrived as another kind than its catalog entry promises",
                observation.node,
                observation.leaf
            );
        }
    }

    #[test]
    fn everything_records_taps_after_the_nodes_emits_and_before_its_duration() {
        let mut pipeline = tapped_pipeline(&[1.0]);
        pipeline.watch(&WatchSet::Everything).expect("cannot fail");

        run(&pipeline, &[1.0]);

        assert_eq!(
            sources(&drained(&pipeline)),
            pairs(&[
                (SECOND, NIS),
                (SECOND, TICK_DURATION_LEAF),
                (WRITER, NIS),
                (WRITER, ESTIMATE_TAP),
                (WRITER, TICK_DURATION_LEAF),
            ])
        );
    }
}
