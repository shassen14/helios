//! What a watching host calls on [`AutonomyPipeline`]: the catalog of what
//! the pipeline can record, the watch set that picks what it does record,
//! and the drain that collects what was recorded.

use super::{observable_catalog, ScheduledNode};

use crate::{
    observe::{
        key::Subject,
        observation::Observation,
        watch::{WatchError, WatchSet},
    },
    port::Observable,
    AutonomyPipeline,
};

impl AutonomyPipeline {
    /// Iterates over every subject a watcher can ask for, with what it
    /// records.
    ///
    /// First the channels the body and the outside world write, in the order
    /// [`descriptions`](Self::descriptions) lists them, once per instance
    /// the writer declares on a keyed channel. Then the nodes, in
    /// [`NodeId`](crate::NodeId) order. For each node: the leaves it
    /// declares, in declaration order, then the leaves the pipeline adds to
    /// every node, as [`Subject::Emission`]s; then each channel it writes, in
    /// output order, once per instance it declares on a keyed channel. Every
    /// channel the pipeline holds is listed, read or not.
    ///
    /// A host spells its own paths from these subjects and looks its
    /// requests up in them, rather than parsing paths apart. A comparison
    /// across runs reads each [`Observable`]'s determinism to skip
    /// nondeterministic ones; a channel's is its writer's.
    pub fn observables(&self) -> impl Iterator<Item = (Subject, Observable)> + '_ {
        let host_written = self
            .host_writes
            .taps()
            .map(|tap| (tap.subject().clone(), tap.observable()));
        let nodes = self.levels.iter().flatten().flat_map(|scheduled| {
            let name = scheduled.node.name();
            let emissions =
                observable_catalog(scheduled.node.as_ref())
                    .into_iter()
                    .map(move |observable| {
                        (
                            Subject::emission(name, observable.name().clone()),
                            observable,
                        )
                    });
            let channels = scheduled
                .taps
                .iter()
                .map(|tap| (tap.subject().clone(), tap.observable()));
            emissions.chain(channels)
        });
        host_written.chain(nodes)
    }

    /// Moves everything recorded since the last drain onto the end of `out`,
    /// leaving every buffer empty. Each observation's key names this
    /// pipeline's agent.
    ///
    /// What the body and the outside world wrote comes first, as it was
    /// recorded before any node ran. Then observations come grouped by the
    /// node whose run recorded them, in [`NodeId`](crate::NodeId) order, and
    /// in record order within a node; a caller wanting one timeline sorts by
    /// timestamp. Appending lets a host reuse one list every tick, and drain
    /// several pipelines into it.
    ///
    /// The host must drain after every tick: a pipeline with anything
    /// watched records on every tick, and nothing else empties its buffers.
    pub fn drain_into(&self, out: &mut Vec<Observation>) {
        self.host_writes.drain_into(out);
        for scheduled in self.levels.iter().flatten() {
            scheduled.observations.drain_into(out);
        }
    }

    /// Replaces the whole watch set: from now on, exactly the subjects `set`
    /// names are recorded. A built pipeline watches nothing.
    ///
    /// All or nothing: if any entry names a node that doesn't exist, a leaf
    /// its node doesn't declare, or a channel instance no writer declares,
    /// every such entry is reported and the previous watch set stays in
    /// place.
    ///
    /// Observations already recorded are kept until the next drain, including
    /// those of subjects this call stops watching, so drain before changing
    /// the watch set. A channel the body or the outside world writes that
    /// starts being watched records the value it holds at the next tick, even
    /// one written before this call.
    pub fn watch(&mut self, set: &WatchSet) -> Result<(), Vec<WatchError>> {
        let subjects = match set {
            WatchSet::Everything => {
                self.host_writes.set_watched(|_| true);
                for scheduled in self.levels.iter_mut().flatten() {
                    scheduled.observations.set_all_watched(true);
                    for tap in &mut scheduled.taps {
                        tap.watched = true;
                    }
                }
                return Ok(());
            }
            WatchSet::Subjects(subjects) => subjects,
        };

        let errors: Vec<WatchError> = subjects
            .iter()
            .filter_map(|subject| self.refusal(subject))
            .collect();
        if !errors.is_empty() {
            return Err(errors);
        }

        self.host_writes
            .set_watched(|tap| subjects.contains(tap.subject()));
        // Walk the nodes rather than the entries: each node is visited once,
        // and no node has to be looked up by name while `levels` is borrowed
        // mutably.
        for scheduled in self.levels.iter_mut().flatten() {
            scheduled.observations.set_all_watched(false);

            let name = scheduled.node.name();
            for subject in subjects {
                if let Subject::Emission { node, leaf } = subject {
                    if node.as_ref() == name {
                        let declared = scheduled.observations.set_watched(leaf, true);
                        debug_assert!(declared, "leaf \"{leaf}\" was checked above");
                    }
                }
            }
            for tap in &mut scheduled.taps {
                tap.watched = subjects.contains(tap.subject());
            }
        }

        Ok(())
    }

    /// Why `subject` can't be watched, or `None` when it can.
    fn refusal(&self, subject: &Subject) -> Option<WatchError> {
        match subject {
            Subject::Emission { node, leaf } => match self.scheduled(node) {
                None => Some(WatchError::UnknownNode {
                    node: node.to_string(),
                    leaf: leaf.to_string(),
                }),
                Some(scheduled) if !scheduled.observations.declares(leaf) => {
                    Some(WatchError::UndeclaredLeaf {
                        node: node.to_string(),
                        leaf: leaf.to_string(),
                    })
                }
                Some(_) => None,
            },
            Subject::Channel { channel, instance } => {
                let declared = self
                    .levels
                    .iter()
                    .flatten()
                    .flat_map(|scheduled| &scheduled.taps)
                    .chain(self.host_writes.taps())
                    .any(|tap| tap.subject() == subject);
                (!declared).then(|| WatchError::UndeclaredChannel {
                    channel: channel.clone(),
                    instance: instance.clone(),
                })
            }
        }
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
            key::RecordedKey,
            observation::{ObservedValue, ObservedValueKind},
        },
        pipeline::autonomy_pipeline::TICK_DURATION_LEAF,
        port::{
            ChannelKey, ChannelKind, Determinism, InstanceKey, Keyed, PortBus, PortDescriptor,
            PortDescriptorBuilder,
        },
        prelude::{Health, PipelineNode, Stamped, TickContext},
        ErasedStamped, PipelineBuilder,
    };

    use helios_core::kernel::si_unit::SiUnit;
    use helios_core::{
        prelude::{AgentId, FrameId, MonotonicTime, TfProvider},
        spatial::transforms::ErasedTransform,
    };

    use std::sync::Arc;

    const AGENT: &str = "car";
    const FIRST: &str = "first";
    const SECOND: &str = "second";
    const WRITER: &str = "writer";
    const NIS: &str = "aiding.gps.nis";
    const DROPPED: &str = "aiding.gps.dropped";
    const UNDECLARED: &str = "aiding.gps.undeclared";
    const MISSING_NODE: &str = "missing";
    /// The writer's output channel.
    const ESTIMATE: &str = "odom.estimate";
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
                tick.emit(observable.name(), tick.now, tick.now.0);
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

    fn writing_with(descriptor: PortDescriptorBuilder, writes_at: &[f64]) -> Box<dyn PipelineNode> {
        Box::new(Emitting {
            name: WRITER.to_string(),
            descriptor: descriptor.output(estimate()).build(),
            writes_at: writes_at.to_vec(),
        })
    }

    fn declaring(leaves: &[&str]) -> PortDescriptorBuilder {
        leaves
            .iter()
            .fold(PortDescriptorBuilder::algorithm(), |builder, leaf| {
                builder.observable(*leaf, Determinism::Deterministic, SiUnit::Dimensionless)
            })
    }

    fn estimate() -> ChannelKey {
        ChannelKey::internal::<f64>(ESTIMATE)
    }

    /// `first` declares NIS and the drop count; `second` declares NIS only.
    fn pipeline() -> AutonomyPipeline {
        PipelineBuilder::new(AgentId::new(AGENT))
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
        PipelineBuilder::new(AgentId::new(AGENT))
            .add_node(writer)
            .add_node(emitting(SECOND, &[NIS]))
            .build()
            .expect("two independent nodes build")
    }

    fn emission(node: &str, leaf: &str) -> Subject {
        Subject::emission(node, leaf)
    }

    /// The [`ESTIMATE`] channel's subject.
    fn estimate_channel() -> Subject {
        Subject::channel(estimate(), None)
    }

    fn subjects(subjects: impl IntoIterator<Item = Subject>) -> WatchSet {
        WatchSet::Subjects(subjects.into_iter().collect())
    }

    /// Every subject in the catalog, in catalog order.
    fn catalog(pipeline: &AutonomyPipeline) -> Vec<Subject> {
        pipeline.observables().map(|(subject, _)| subject).collect()
    }

    /// Every subject currently watched, in catalog order.
    fn watched(pipeline: &AutonomyPipeline) -> Vec<Subject> {
        catalog(pipeline)
            .into_iter()
            .filter(|subject| match subject {
                Subject::Emission { node, leaf } => pipeline
                    .scheduled(node)
                    .and_then(|scheduled| scheduled.observations.watched_leaf(leaf))
                    .is_some(),
                Subject::Channel { .. } => pipeline
                    .levels
                    .iter()
                    .flatten()
                    .flat_map(|scheduled| &scheduled.taps)
                    .chain(pipeline.host_writes.taps())
                    .any(|tap| tap.subject() == subject && tap.watched),
            })
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

    fn key(subject: Subject) -> RecordedKey {
        RecordedKey {
            agent: AgentId::new(AGENT),
            subject,
        }
    }

    /// What an [`Emitting`] node records for `leaf` on the run at `t`.
    fn emitted(node: &str, leaf: &str, t: f64) -> Observation {
        Observation {
            key: key(emission(node, leaf)),
            timestamp: MonotonicTime(t),
            value: ObservedValue::Scalar(t),
        }
    }

    /// What a watched tap on [`ESTIMATE`] records for `written`: the bus
    /// value itself, under the channel's key, stamped with the value's time.
    fn tapped(written: Arc<dyn ErasedStamped>) -> Observation {
        Observation {
            key: key(estimate_channel()),
            timestamp: written.timestamp(),
            value: ObservedValue::Typed(written),
        }
    }

    /// The value now in the writer's [`ESTIMATE`] slot.
    fn written(pipeline: &AutonomyPipeline) -> Arc<dyn ErasedStamped> {
        pipeline
            .bus()
            .read_erased_versioned(&estimate())
            .map(|(written, _)| written)
            .expect("the writer has written its output")
    }

    /// The subject of each observation, in drained order.
    fn sources(observations: &[Observation]) -> Vec<Subject> {
        observations.iter().map(|o| o.key.subject.clone()).collect()
    }

    #[test]
    fn observables_list_every_node_in_order_each_ending_with_tick_duration() {
        assert_eq!(
            catalog(&pipeline()),
            vec![
                emission(FIRST, NIS),
                emission(FIRST, DROPPED),
                emission(FIRST, TICK_DURATION_LEAF),
                emission(SECOND, NIS),
                emission(SECOND, TICK_DURATION_LEAF),
            ]
        );
    }

    #[test]
    fn built_pipeline_watches_nothing() {
        assert!(watched(&tapped_pipeline(&[])).is_empty());
    }

    #[test]
    fn everything_watches_every_emission_and_channel() {
        let mut pipeline = tapped_pipeline(&[]);
        pipeline.watch(&WatchSet::Everything).expect("cannot fail");

        assert_eq!(watched(&pipeline), catalog(&pipeline));
    }

    #[test]
    fn subjects_watch_exactly_the_listed_ones() {
        let mut pipeline = tapped_pipeline(&[]);
        pipeline
            .watch(&subjects([
                emission(WRITER, NIS),
                emission(SECOND, TICK_DURATION_LEAF),
                estimate_channel(),
            ]))
            .expect("all are recorded");

        assert_eq!(
            watched(&pipeline),
            vec![
                emission(SECOND, TICK_DURATION_LEAF),
                emission(WRITER, NIS),
                estimate_channel(),
            ]
        );
    }

    #[test]
    fn replacing_the_set_unwatches_what_it_drops() {
        let mut pipeline = tapped_pipeline(&[]);
        pipeline
            .watch(&subjects([emission(WRITER, NIS), estimate_channel()]))
            .expect("recorded");
        pipeline
            .watch(&subjects([emission(SECOND, NIS)]))
            .expect("declared");

        assert_eq!(watched(&pipeline), vec![emission(SECOND, NIS)]);
    }

    #[test]
    fn default_set_after_everything_watches_nothing() {
        let mut pipeline = tapped_pipeline(&[]);
        pipeline.watch(&WatchSet::Everything).expect("cannot fail");
        pipeline
            .watch(&WatchSet::default())
            .expect("empty set is valid");

        assert!(watched(&pipeline).is_empty());
    }

    #[test]
    fn unknown_node_undeclared_leaf_and_undeclared_channel_are_refused_together() {
        let mut pipeline = pipeline();
        pipeline
            .watch(&subjects([emission(FIRST, NIS)]))
            .expect("declared");
        let undeclared = ChannelKey::internal::<f64>("truth.tf");

        let errors = pipeline
            .watch(&subjects([
                emission(SECOND, NIS),
                emission(FIRST, UNDECLARED),
                emission(MISSING_NODE, NIS),
                Subject::channel(undeclared.clone(), None),
            ]))
            .expect_err("three entries are unknown");

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
                WatchError::UndeclaredChannel {
                    channel: undeclared,
                    instance: None,
                },
            ]
        );
        assert_eq!(watched(&pipeline), vec![emission(FIRST, NIS)]);
    }

    #[test]
    fn leaf_of_another_node_is_undeclared_on_the_named_one() {
        let mut pipeline = pipeline();

        let errors = pipeline
            .watch(&subjects([emission(SECOND, DROPPED)]))
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
    fn a_channel_is_not_a_leaf_of_its_writer() {
        let mut pipeline = tapped_pipeline(&[]);

        let errors = pipeline
            .watch(&subjects([emission(WRITER, ESTIMATE)]))
            .expect_err("a channel is watched under its own key");

        assert_eq!(
            errors,
            vec![WatchError::UndeclaredLeaf {
                node: WRITER.to_string(),
                leaf: ESTIMATE.to_string(),
            }]
        );
    }

    #[test]
    fn drain_gives_each_node_in_id_order_in_emit_order_then_empties() {
        let mut pipeline = pipeline();
        pipeline
            .watch(&subjects([
                emission(FIRST, NIS),
                emission(FIRST, DROPPED),
                emission(SECOND, NIS),
            ]))
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
    fn every_drained_key_names_the_pipelines_agent() {
        let mut pipeline = tapped_pipeline(&[1.0]);
        pipeline.watch(&WatchSet::Everything).expect("cannot fail");

        run(&pipeline, &[1.0]);
        let observations = drained(&pipeline);

        assert!(!observations.is_empty());
        for observation in &observations {
            assert_eq!(observation.key.agent, *pipeline.agent());
        }
    }

    #[test]
    fn nothing_watched_drains_nothing() {
        let pipeline = tapped_pipeline(&[1.0]);
        run(&pipeline, &[1.0]);

        assert!(drained(&pipeline).is_empty());
    }

    #[test]
    fn unwatched_leaves_beside_a_watched_one_are_not_recorded() {
        let mut pipeline = pipeline();
        pipeline
            .watch(&subjects([emission(FIRST, NIS)]))
            .expect("declared");

        run(&pipeline, &[1.0]);

        assert_eq!(drained(&pipeline), vec![emitted(FIRST, NIS, 1.0)]);
    }

    #[test]
    fn watched_tick_duration_is_one_sample_per_run_stamped_with_now() {
        let mut pipeline = pipeline();
        pipeline
            .watch(&subjects([emission(SECOND, TICK_DURATION_LEAF)]))
            .expect("every node has a duration");

        run(&pipeline, &[1.0, 2.0]);
        let observations = drained(&pipeline);

        assert_eq!(
            sources(&observations),
            vec![
                emission(SECOND, TICK_DURATION_LEAF),
                emission(SECOND, TICK_DURATION_LEAF)
            ]
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
            vec![
                emission(FIRST, NIS),
                emission(FIRST, DROPPED),
                emission(FIRST, TICK_DURATION_LEAF),
                emission(SECOND, NIS),
                emission(SECOND, TICK_DURATION_LEAF),
            ]
        );
    }

    #[test]
    fn observables_list_a_nodes_channels_after_its_emissions() {
        assert_eq!(
            catalog(&tapped_pipeline(&[])),
            vec![
                emission(SECOND, NIS),
                emission(SECOND, TICK_DURATION_LEAF),
                emission(WRITER, NIS),
                emission(WRITER, TICK_DURATION_LEAF),
                estimate_channel(),
            ]
        );
    }

    /// A leaf may share a name with an output: one is an emission of the
    /// node, the other a channel, so the two are told apart by kind.
    #[test]
    fn a_leaf_named_like_an_output_is_its_own_subject() {
        let mut pipeline = built(writing(&[ESTIMATE], &[1.0]));
        pipeline
            .watch(&subjects([emission(WRITER, ESTIMATE), estimate_channel()]))
            .expect("both are recorded");

        run(&pipeline, &[1.0]);

        assert_eq!(
            drained(&pipeline),
            vec![emitted(WRITER, ESTIMATE, 1.0), tapped(written(&pipeline))]
        );
    }

    // --- keyed outputs ---

    /// The keyed output's name.
    const EDGES: &str = "edges";

    /// A value about one frame, keyed by the frame.
    struct FrameSample {
        frame: &'static str,
    }

    impl Keyed for FrameSample {
        type Instance = &'static str;

        fn instance(&self) -> &&'static str {
            &self.frame
        }
    }

    fn edges() -> ChannelKey {
        ChannelKey::keyed::<FrameSample>(ChannelKind::Internal, EDGES)
    }

    fn edge(frame: &'static str) -> Subject {
        Subject::channel(edges(), Some(InstanceKey::of::<FrameSample>(&frame)))
    }

    /// A node writing each of `frames` on [`edges`] every run.
    struct KeyedWriting {
        descriptor: PortDescriptor,
        frames: Vec<&'static str>,
    }

    impl PipelineNode for KeyedWriting {
        fn name(&self) -> &str {
            WRITER
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, bus: &PortBus, _tf: &dyn TfProvider, tick: TickContext) {
            for &frame in &self.frames {
                let stamped = Stamped {
                    value: FrameSample { frame },
                    timestamp: tick.now,
                    health: Health::Ok,
                    producer: tick.node_id,
                };
                bus.write_keyed(edges(), stamped)
                    .expect("every written frame is declared");
            }
        }
    }

    fn keyed_pipeline(frames: &[&'static str]) -> AutonomyPipeline {
        built(Box::new(KeyedWriting {
            descriptor: PortDescriptorBuilder::algorithm()
                .output_keyed::<FrameSample>(edges(), frames.iter().copied())
                .build(),
            frames: frames.to_vec(),
        }))
    }

    /// The value now in `frame`'s instance of [`edges`].
    fn written_edge(pipeline: &AutonomyPipeline, frame: &'static str) -> Observation {
        let (written, _) = pipeline
            .bus()
            .read_erased_instance_versioned(&edges(), &InstanceKey::of::<FrameSample>(&frame))
            .expect("the instance was written");
        Observation {
            key: key(edge(frame)),
            timestamp: written.timestamp(),
            value: ObservedValue::Typed(written),
        }
    }

    #[test]
    fn observables_list_a_keyed_output_once_per_instance() {
        let catalog = catalog(&keyed_pipeline(&["wheel", "base"]));

        assert_eq!(&catalog[catalog.len() - 2..], [edge("wheel"), edge("base")]);
    }

    /// Each instance is its own series, so watching several records each
    /// under its own key.
    #[test]
    fn watched_keyed_taps_record_each_instance_under_its_own_key() {
        let mut pipeline = keyed_pipeline(&["base", "wheel"]);
        pipeline
            .watch(&subjects([edge("base"), edge("wheel")]))
            .expect("the writer writes both instances");

        run(&pipeline, &[1.0]);

        assert_eq!(
            drained(&pipeline),
            vec![
                written_edge(&pipeline, "base"),
                written_edge(&pipeline, "wheel")
            ]
        );
    }

    #[test]
    fn an_unwatched_instance_beside_a_watched_one_records_nothing() {
        let mut pipeline = keyed_pipeline(&["base", "wheel"]);
        pipeline
            .watch(&subjects([edge("wheel")]))
            .expect("the writer writes the wheel's edge");

        run(&pipeline, &[1.0]);

        assert_eq!(drained(&pipeline), vec![written_edge(&pipeline, "wheel")]);
    }

    #[test]
    fn watching_an_instance_no_writer_declares_is_refused() {
        let mut pipeline = keyed_pipeline(&["base"]);

        let errors = pipeline
            .watch(&subjects([edge("wheel")]))
            .expect_err("no writer declares the wheel's edge");

        assert_eq!(
            errors,
            vec![WatchError::UndeclaredChannel {
                channel: edges(),
                instance: Some(InstanceKey::of::<FrameSample>(&"wheel")),
            }]
        );
    }

    // The first write to a slot moves its version from none to some, so this
    // also covers a channel that held nothing before the run.
    #[test]
    fn watched_tap_records_the_bus_value_itself_stamped_with_its_own_time() {
        let mut pipeline = tapped_pipeline(&[1.0]);
        pipeline
            .watch(&subjects([estimate_channel()]))
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
            .watch(&subjects([estimate_channel()]))
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
            .watch(&subjects([estimate_channel()]))
            .expect("the writer outputs the estimate");

        // The node would write at 5.0–7.0, but its rate keeps it from running
        // after 4.0.
        run(&pipeline, &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]);

        assert_eq!(drained(&pipeline), vec![tapped(written(&pipeline))]);
    }

    #[test]
    fn unwatched_tap_records_nothing_beside_a_watched_leaf() {
        let mut pipeline = tapped_pipeline(&[1.0]);
        pipeline
            .watch(&subjects([emission(WRITER, NIS)]))
            .expect("declared");

        run(&pipeline, &[1.0]);

        assert_eq!(drained(&pipeline), vec![emitted(WRITER, NIS, 1.0)]);
    }

    #[test]
    fn every_recorded_value_is_the_kind_its_subject_promises() {
        let mut pipeline = tapped_pipeline(&[1.0]);
        pipeline.watch(&WatchSet::Everything).expect("cannot fail");
        let promised: Vec<(Subject, ObservedValueKind)> = pipeline
            .observables()
            .map(|(subject, observable)| (subject, observable.kind()))
            .collect();

        run(&pipeline, &[1.0]);
        let observations = drained(&pipeline);

        // Both kinds are covered: declared leaves and durations are scalars,
        // the estimate is typed.
        assert!(observations
            .iter()
            .any(|o| matches!(o.value, ObservedValue::Scalar(_))));
        assert!(observations
            .iter()
            .any(|o| matches!(o.value, ObservedValue::Typed(_))));
        for observation in &observations {
            let kind = promised
                .iter()
                .find(|(subject, _)| *subject == observation.key.subject)
                .map(|(_, kind)| *kind);
            assert_eq!(
                kind,
                Some(observation.value.kind()),
                "{} arrived as another kind than its catalog entry promises",
                observation.key.path()
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
            vec![
                emission(SECOND, NIS),
                emission(SECOND, TICK_DURATION_LEAF),
                emission(WRITER, NIS),
                estimate_channel(),
                emission(WRITER, TICK_DURATION_LEAF),
            ]
        );
    }
}
