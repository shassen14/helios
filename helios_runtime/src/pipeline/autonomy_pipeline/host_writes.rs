//! What the body and the outside world write onto the bus, recorded while
//! watched.
//!
//! A host writes between ticks, outside any node's run, so no run brackets
//! the write the way a node's taps bracket its own. Instead the pipeline
//! compares each watched channel's version with the one it last recorded,
//! at the start of every tick, before any node runs. What it records is what
//! the brain saw that tick: a channel written twice between ticks keeps, and
//! records, its last value.

use super::tap::{tapped_instances, Tap};

use crate::{
    observe::observation::{Observation, ObservedValue},
    port::{ChannelDescription, Determinism, PortBus, Provenance, SlotVersion, Writer},
};

use helios_core::prelude::AgentId;

use std::sync::Mutex;

/// A tap on every channel instance the body or the outside world writes,
/// and what they recorded since the last drain.
///
/// The watched flags change only through `&mut self`, between ticks. What
/// was recorded, and the version each tap last recorded, change during a
/// tick through `&self`, so they sit behind locks; only the tick touches
/// them, so the locks are never contended.
pub(in crate::pipeline) struct HostWrites {
    taps: Vec<HostTap>,
    /// True when at least one tap is watched, so a pipeline watching no
    /// host-written channel pays one check per tick.
    any_watched: bool,
    recorded: Mutex<Vec<Observation>>,
}

impl HostWrites {
    /// An unwatched tap per instance of each channel in `descriptions` that
    /// no node writes, in description order, on `agent`'s pipeline.
    pub(in crate::pipeline) fn new(agent: &AgentId, descriptions: &[ChannelDescription]) -> Self {
        let taps = descriptions
            .iter()
            .filter_map(|description| {
                let determinism = host_determinism(description.writer())?;
                let channel = description.key();
                let taps = tapped_instances(channel, description.instances())
                    .into_iter()
                    .map(move |instance| HostTap {
                        tap: Tap::new(agent, channel, instance, determinism),
                        recorded_version: Mutex::default(),
                    });
                Some(taps)
            })
            .flatten()
            .collect();

        Self {
            taps,
            any_watched: false,
            recorded: Mutex::default(),
        }
    }

    /// Every tap, in description order.
    pub(in crate::pipeline) fn taps(&self) -> impl Iterator<Item = &Tap> {
        self.taps.iter().map(|host| &host.tap)
    }

    /// Records the value of each watched channel instance whose version
    /// changed since it was last recorded, stamped with the value's own time.
    ///
    /// A poisoned lock skips that tap, or drops the sample.
    pub(in crate::pipeline) fn record_changed(&self, bus: &PortBus) {
        if !self.any_watched {
            return;
        }
        for host in self.taps.iter().filter(|host| host.tap.watched) {
            let Some((value, version)) = host.tap.read(bus) else {
                continue;
            };
            let Ok(mut recorded_version) = host.recorded_version.lock() else {
                continue;
            };
            if *recorded_version == Some(version) {
                continue;
            }
            *recorded_version = Some(version);

            let Ok(mut recorded) = self.recorded.lock() else {
                continue;
            };
            recorded.push(Observation {
                key: host.tap.key.clone(),
                timestamp: value.timestamp(),
                value: ObservedValue::Typed(value),
            });
        }
    }

    /// Moves every recorded observation onto the end of `out`, in record
    /// order, leaving this buffer empty.
    ///
    /// A poisoned lock leaves `out` unchanged.
    pub(in crate::pipeline) fn drain_into(&self, out: &mut Vec<Observation>) {
        let Ok(mut recorded) = self.recorded.lock() else {
            return;
        };
        out.append(&mut recorded);
    }

    /// Sets each tap watched exactly when `watched` says so.
    ///
    /// A tap that starts being watched forgets what it last recorded, so the
    /// next tick records the value the channel holds even if it was written
    /// before watching began: a goal sent once at startup is part of what the
    /// brain saw. A tap that stays watched keeps its version, so nothing is
    /// recorded twice.
    pub(in crate::pipeline) fn set_watched(&mut self, watched: impl Fn(&Tap) -> bool) {
        for host in &mut self.taps {
            let now_watched = watched(&host.tap);
            if now_watched && !host.tap.watched {
                if let Ok(recorded_version) = host.recorded_version.get_mut() {
                    *recorded_version = None;
                }
            }
            host.tap.watched = now_watched;
        }
        self.any_watched = self.taps.iter().any(|host| host.tap.watched);
    }
}

/// One host-written channel instance, and the version it last recorded.
struct HostTap {
    tap: Tap,
    /// `None` until the first record since the tap began being watched.
    recorded_version: Mutex<Option<SlotVersion>>,
}

/// Whether the values `writer` puts on a channel repeat across runs given
/// the same seed, or `None` for a node, whose channels its own taps record.
///
/// The body's values are deterministic in simulation, exact or seeded. An
/// outside input is not: an operator's teleop intent never repeats, and the
/// pipeline cannot tell a scripted mission's goal from an operator's.
fn host_determinism(writer: &Writer) -> Option<Determinism> {
    match writer {
        Writer::Body {
            provenance: Provenance::Exact | Provenance::Simulated,
            ..
        } => Some(Determinism::Deterministic),
        Writer::Outside => Some(Determinism::Nondeterministic),
        Writer::Node { .. } => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        observe::{
            key::{RecordedKey, Subject},
            watch::{WatchError, WatchSet},
        },
        pipeline::autonomy_pipeline::TICK_DURATION_LEAF,
        port::{
            ChannelKey, ChannelKind, ErasedStamped, InstanceKey, Keyed, PortDescriptor,
            PortDescriptorBuilder,
        },
        prelude::{Health, PipelineNode, Stamped, TickContext},
        AutonomyPipeline, BodyCapabilities, BodyChannel, PipelineBuilder,
    };

    use helios_core::{
        prelude::{FrameId, MonotonicTime, TfProvider},
        spatial::transforms::ErasedTransform,
    };

    use std::sync::Arc;

    const AGENT: &str = "car";
    const GPS: &str = "gps";
    const EDGES: &str = "truth.edges";
    const GOAL: &str = "goal";
    const UNIT: &str = "rig";
    const IDLE: &str = "idle";
    const STEP: f64 = 0.01;
    /// How far before the tick a host value is stamped, so a recorded value's
    /// own time can be told apart from the tick's.
    const LAG: f64 = 0.25;

    /// A value about one frame, keyed by the frame.
    struct FrameSample {
        frame: &'static str,
    }

    impl Keyed for FrameSample {
        type Instance = &'static str;
        type Declared = &'static str;

        fn instance(&self) -> &&'static str {
            &self.frame
        }

        fn declared_frame(&self) -> Option<&FrameId> {
            None
        }
    }

    /// A node that does nothing, so a tick has a node run to order against.
    struct Idle {
        descriptor: PortDescriptor,
    }

    impl PipelineNode for Idle {
        fn name(&self) -> &str {
            IDLE
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {}
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

    fn gps() -> ChannelKey {
        ChannelKey::internal::<f64>(GPS)
    }

    fn edges() -> ChannelKey {
        ChannelKey::keyed::<FrameSample>(ChannelKind::Internal, EDGES)
    }

    fn goal() -> ChannelKey {
        ChannelKey::internal::<f64>(GOAL)
    }

    fn edge(frame: &'static str) -> Subject {
        Subject::channel(edges(), Some(InstanceKey::of::<FrameSample>(&frame)))
    }

    /// A body publishing [`GPS`] and the [`EDGES`] of frames `a` and `b`, an
    /// outside [`GOAL`], and one idle node.
    fn pipeline() -> AutonomyPipeline {
        PipelineBuilder::new(AgentId::new(AGENT))
            .with_body_capabilities(BodyCapabilities {
                publishes: vec![
                    BodyChannel::new(gps(), UNIT, Provenance::Simulated),
                    BodyChannel::new(edges(), UNIT, Provenance::Exact)
                        .with_instances::<FrameSample>(["a", "b"]),
                ],
                ..Default::default()
            })
            .with_outside_inputs(vec![goal()])
            .add_node(Box::new(Idle {
                descriptor: PortDescriptorBuilder::algorithm().build(),
            }))
            .build()
            .expect("a body, an outside input and an idle node build")
    }

    fn watch(pipeline: &mut AutonomyPipeline, subjects: impl IntoIterator<Item = Subject>) {
        pipeline
            .watch(&WatchSet::Subjects(subjects.into_iter().collect()))
            .expect("every subject is declared");
    }

    /// Writes `value` to `channel` the way a host does, stamped [`LAG`]
    /// before `t`.
    fn write(pipeline: &AutonomyPipeline, channel: ChannelKey, value: f64, t: f64) {
        pipeline
            .bus()
            .write(channel, stamped(value, t))
            .expect("the bus has a slot for every declared channel");
    }

    fn write_edge(pipeline: &AutonomyPipeline, frame: &'static str, t: f64) {
        pipeline
            .bus()
            .write_keyed(edges(), stamped(FrameSample { frame }, t))
            .expect("the body declares this edge");
    }

    fn stamped<T>(value: T, t: f64) -> Stamped<T> {
        Stamped {
            value,
            timestamp: MonotonicTime(t - LAG),
            health: Health::Ok,
            producer: 0,
        }
    }

    fn tick(pipeline: &AutonomyPipeline, t: f64) {
        pipeline.tick(MonotonicTime(t), STEP, &NoTransforms);
    }

    fn drained(pipeline: &AutonomyPipeline) -> Vec<Observation> {
        let mut out = Vec::new();
        pipeline.drain_into(&mut out);
        out
    }

    /// The subject and time of each observation, in drain order.
    fn recorded(observations: &[Observation]) -> Vec<(Subject, f64)> {
        observations
            .iter()
            .map(|o| (o.key.subject.clone(), o.timestamp.0))
            .collect()
    }

    /// The value now on [`GPS`], as a recording of it holds it.
    fn on_gps(pipeline: &AutonomyPipeline) -> Arc<dyn ErasedStamped> {
        pipeline
            .bus()
            .read_erased_versioned(&gps())
            .map(|(value, _)| value)
            .expect("written")
    }

    #[test]
    fn observables_list_host_written_channels_first_with_their_writers_determinism() {
        let pipeline = pipeline();

        let listed: Vec<(Subject, Determinism)> = pipeline
            .observables()
            .map(|(subject, observable)| (subject, observable.determinism()))
            .collect();

        assert_eq!(
            listed,
            vec![
                (Subject::channel(gps(), None), Determinism::Deterministic),
                (edge("a"), Determinism::Deterministic),
                (edge("b"), Determinism::Deterministic),
                (
                    Subject::channel(goal(), None),
                    Determinism::Nondeterministic
                ),
                (
                    Subject::emission(IDLE, TICK_DURATION_LEAF),
                    Determinism::Nondeterministic
                ),
            ]
        );
    }

    #[test]
    fn a_watched_body_channel_records_the_bus_value_itself_stamped_with_its_own_time() {
        let mut pipeline = pipeline();
        watch(&mut pipeline, [Subject::channel(gps(), None)]);

        write(&pipeline, gps(), 7.0, 1.0);
        tick(&pipeline, 1.0);

        let value = on_gps(&pipeline);
        let ObservedValue::Typed(ref recorded) = drained(&pipeline)[0].value else {
            panic!("a channel records its typed value");
        };
        assert!(Arc::ptr_eq(recorded, &value));
        assert_eq!(value.timestamp(), MonotonicTime(1.0 - LAG));
    }

    #[test]
    fn each_write_is_recorded_once_at_the_next_tick_and_only_the_last_between_ticks() {
        let mut pipeline = pipeline();
        watch(&mut pipeline, [Subject::channel(gps(), None)]);

        write(&pipeline, gps(), 1.0, 1.0);
        tick(&pipeline, 1.0);
        tick(&pipeline, 2.0);
        write(&pipeline, gps(), 2.0, 2.5);
        write(&pipeline, gps(), 3.0, 3.0);
        tick(&pipeline, 3.0);

        assert_eq!(
            recorded(&drained(&pipeline)),
            vec![
                (Subject::channel(gps(), None), 1.0 - LAG),
                (Subject::channel(gps(), None), 3.0 - LAG),
            ]
        );
    }

    #[test]
    fn a_value_held_before_watching_is_recorded_at_the_first_tick_and_never_again() {
        let mut pipeline = pipeline();
        write(&pipeline, goal(), 4.0, 0.5);
        watch(&mut pipeline, [Subject::channel(goal(), None)]);

        tick(&pipeline, 1.0);
        watch(&mut pipeline, [Subject::channel(goal(), None)]);
        tick(&pipeline, 2.0);

        assert_eq!(
            recorded(&drained(&pipeline)),
            vec![(Subject::channel(goal(), None), 0.5 - LAG)]
        );
    }

    #[test]
    fn each_instance_of_a_keyed_body_channel_records_under_its_own_key() {
        let mut pipeline = pipeline();
        watch(&mut pipeline, [edge("b")]);

        write_edge(&pipeline, "a", 1.0);
        write_edge(&pipeline, "b", 1.0);
        tick(&pipeline, 1.0);

        assert_eq!(recorded(&drained(&pipeline)), vec![(edge("b"), 1.0 - LAG)]);
    }

    #[test]
    fn an_unwatched_host_channel_records_nothing() {
        let mut pipeline = pipeline();
        watch(&mut pipeline, [Subject::channel(goal(), None)]);

        write(&pipeline, gps(), 1.0, 1.0);
        tick(&pipeline, 1.0);

        assert!(drained(&pipeline).is_empty());
    }

    #[test]
    fn host_writes_drain_before_what_the_nodes_recorded_and_name_the_agent() {
        let mut pipeline = pipeline();
        pipeline.watch(&WatchSet::Everything).expect("cannot fail");

        write(&pipeline, gps(), 1.0, 1.0);
        write(&pipeline, goal(), 2.0, 1.0);
        tick(&pipeline, 1.0);

        let observations = drained(&pipeline);
        assert_eq!(
            observations
                .iter()
                .map(|o| o.key.clone())
                .collect::<Vec<_>>(),
            [
                Subject::channel(gps(), None),
                Subject::channel(goal(), None),
                Subject::emission(IDLE, TICK_DURATION_LEAF),
            ]
            .map(|subject| RecordedKey {
                agent: AgentId::new(AGENT),
                subject,
            })
        );
    }

    #[test]
    fn an_instance_the_body_does_not_declare_is_refused() {
        let mut pipeline = pipeline();

        let errors = pipeline
            .watch(&WatchSet::Subjects([edge("c")].into_iter().collect()))
            .expect_err("the body declares no edge c");

        assert_eq!(
            errors,
            vec![WatchError::UndeclaredChannel {
                channel: edges(),
                instance: Some(InstanceKey::of::<FrameSample>(&"c")),
            }]
        );
    }
}
