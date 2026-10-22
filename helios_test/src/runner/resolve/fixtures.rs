//! Test fixtures for resolving: a pipeline of two declaring nodes, a body
//! channel and an outside input, and entries resolved against it.

use super::{
    plan::{resolve, EntryOwner, ResolvedPlan, SeriesSource},
    problems::{TargetProblem, UnusableEntry},
};
use crate::{
    assertion::{
        extract::standard_extractors, Assertion, AssertionTarget, AssertionValue, Bound, Condition,
        EntryName, InstanceSelector, Measure, Window,
    },
    collector::ObservationCollector,
    evaluation::standard_evaluators,
    statistics::within_run::Reducer,
};

use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::{AgentId, FrameId, TfProvider};
use helios_runtime::{
    observe::{
        key::{RecordedKey, Subject},
        path::{channel_path, node_path},
    },
    port::{
        ChannelKey, ChannelKind, Determinism, InstanceKey, Keyed, PortBus, PortDescriptor,
        PortDescriptorBuilder, Provenance,
    },
    prelude::{PipelineNode, TickContext},
    AutonomyPipeline, BodyCapabilities, BodyChannel, PipelineBuilder,
};

pub(super) const CAR: &str = "car";
pub(super) const ESTIMATOR: &str = "estimator";
pub(super) const PLANNER: &str = "planner";
/// A scalar leaf the estimator declares.
pub(super) const NIS: &str = "aiding.gps.nis";
/// A channel the estimator writes whose payload has an extractor.
pub(super) const SPEED: &str = "speed";
/// A channel the estimator writes whose payload has none.
pub(super) const LABEL: &str = "label";
/// A scalar leaf the estimator declares, and a channel the planner writes
/// whose payload has no extractor.
pub(super) const SHARED: &str = "shared";
/// A keyed channel the planner writes in two instances.
pub(super) const EDGES: &str = "edges";
/// A keyed channel the planner writes in one instance.
pub(super) const ROOT: &str = "root";
/// A channel sent from outside the robot, which no node writes.
pub(super) const GOAL: &str = "goal";
/// A channel the body publishes, which no node writes.
pub(super) const ODOMETRY: &str = "odometry";

/// A value about one frame, keyed by the frame. The tests' extractor
/// reads it as the frame's name, a string.
pub(super) struct FrameSample(&'static str);

impl Keyed for FrameSample {
    type Instance = &'static str;
    type Declared = &'static str;

    fn instance(&self) -> &&'static str {
        &self.0
    }

    fn declared_frame(&self) -> Option<&FrameId> {
        None
    }
}

pub(super) fn keyed(name: &'static str) -> ChannelKey {
    ChannelKey::keyed::<FrameSample>(ChannelKind::Internal, name)
}

/// Writes nothing; exists only to carry a name and a descriptor with
/// outputs and leaves.
pub(super) struct DeclaringNode {
    name: &'static str,
    descriptor: PortDescriptor,
}

impl PipelineNode for DeclaringNode {
    fn name(&self) -> &str {
        self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, _: &PortBus, _: &dyn TfProvider, _: TickContext) {}
}

/// An estimator declaring [`NIS`] and [`SHARED`] and writing [`SPEED`]
/// and [`LABEL`]; a planner writing [`SHARED`], two instances of
/// [`EDGES`] and one of [`ROOT`]; [`ODOMETRY`] from the body; and
/// [`GOAL`] sent from outside.
pub(super) fn pipeline() -> AutonomyPipeline {
    let estimator = PortDescriptorBuilder::algorithm()
        .outputs_from_slice(&[
            ChannelKey::internal::<f64>(SPEED),
            ChannelKey::internal::<String>(LABEL),
        ])
        .observable(NIS, Determinism::Deterministic, SiUnit::Dimensionless)
        .observable(SHARED, Determinism::Deterministic, SiUnit::Dimensionless)
        .build();
    let planner = PortDescriptorBuilder::algorithm()
        .outputs_from_slice(&[ChannelKey::internal::<String>(SHARED)])
        .output_keyed::<FrameSample>(keyed(EDGES), ["base", "wheel"])
        .output_keyed::<FrameSample>(keyed(ROOT), ["base"])
        .build();
    PipelineBuilder::new(AgentId::new(CAR))
        .with_body_capabilities(BodyCapabilities {
            publishes: vec![BodyChannel::new(
                ChannelKey::internal::<f64>(ODOMETRY),
                "wheel_encoder",
                Provenance::Simulated,
            )],
            ..Default::default()
        })
        .with_outside_inputs(vec![ChannelKey::internal::<f64>(GOAL)])
        .add_node(Box::new(DeclaringNode {
            name: ESTIMATOR,
            descriptor: estimator,
        }))
        .add_node(Box::new(DeclaringNode {
            name: PLANNER,
            descriptor: planner,
        }))
        .build()
        .expect("two nodes with no inputs build")
}

/// The car's key for `subject`.
pub(super) fn key(subject: Subject) -> RecordedKey {
    RecordedKey {
        agent: AgentId::new(CAR),
        subject,
    }
}

/// [`SPEED`] as the estimator declares it.
pub(super) fn speed_subject() -> Subject {
    Subject::channel(ChannelKey::internal::<f64>(SPEED), None)
}

pub(super) fn target(agent: &str, node: &str, leaf: &str) -> AssertionTarget {
    AssertionTarget::parse(node_path(agent, node, leaf)).expect("valid target")
}

/// A target on the car's estimator.
pub(super) fn estimator(leaf: &str) -> AssertionTarget {
    target(CAR, ESTIMATOR, leaf)
}

/// A target on the car's channel named `name`. Only the name is spelled,
/// so the key's type here is arbitrary.
pub(super) fn channel(name: &str) -> AssertionTarget {
    AssertionTarget::parse(channel_path(CAR, &ChannelKey::internal::<f64>(name)))
        .expect("valid target")
}

/// The name of the entry at `position` in the lists a test resolves,
/// counting from 1: assertions first, then measures.
pub(super) fn name_at(position: usize) -> EntryName {
    EntryName::parse(format!("e{position}")).expect("valid name")
}

/// An every-sample assertion on `target`. Its name is set by its
/// position when it is resolved.
pub(super) fn assertion(target: AssertionTarget, allow_nondeterministic: bool) -> Assertion {
    Assertion {
        allow_nondeterministic,
        ..reduced(target, None)
    }
}

/// An assertion on `target` reduced by `reduce`.
pub(super) fn reduced(target: AssertionTarget, reduce: Option<Reducer>) -> Assertion {
    Assertion {
        name: name_at(1),
        target,
        window: Window::default(),
        reduce,
        condition: Condition::LessThan(1.0),
        allow_nondeterministic: false,
    }
}

/// A measure on `target` reduced by `reduce`.
pub(super) fn measure(target: AssertionTarget, reduce: Reducer) -> Measure {
    Measure {
        name: name_at(1),
        target,
        window: Window::default(),
        reduce,
        allow_nondeterministic: false,
    }
}

pub(super) fn unusable(
    position: usize,
    target: AssertionTarget,
    problems: Vec<TargetProblem>,
) -> UnusableEntry {
    UnusableEntry {
        name: name_at(position),
        target,
        problems,
    }
}

/// `assertions` resolved against [`pipeline`] and a collector built for
/// `requested`.
pub(super) fn resolve_with(
    assertions: &[Assertion],
    requested: &[AssertionTarget],
) -> (
    ObservationCollector,
    Result<ResolvedPlan, Vec<UnusableEntry>>,
) {
    resolve_all(assertions, &[], requested)
}

/// `assertions` and `measures` resolved against [`pipeline`] and a
/// collector built for `requested`, each entry named by its position
/// ([`name_at`]).
pub(super) fn resolve_all(
    assertions: &[Assertion],
    measures: &[Measure],
    requested: &[AssertionTarget],
) -> (
    ObservationCollector,
    Result<ResolvedPlan, Vec<UnusableEntry>>,
) {
    let assertions: Vec<Assertion> = assertions
        .iter()
        .enumerate()
        .map(|(index, assertion)| Assertion {
            name: name_at(index + 1),
            ..assertion.clone()
        })
        .collect();
    let measures: Vec<Measure> = measures
        .iter()
        .enumerate()
        .map(|(index, measure)| Measure {
            name: name_at(assertions.len() + index + 1),
            ..measure.clone()
        })
        .collect();
    let pipeline = pipeline();
    let mut extractors = standard_extractors();
    extractors.register::<FrameSample, String>(|sample| sample.0.to_string(), None);
    let mut collector = ObservationCollector::new(requested, extractors);
    let resolved = resolve(
        &[],
        None,
        &assertions,
        &measures,
        &standard_evaluators(),
        &[(AgentId::new(CAR), &pipeline)],
        &mut collector,
    )
    .map_err(|unusable| unusable.entries);
    (collector, resolved)
}

/// The assertions the run can't judge, against a collector built for
/// them.
pub(super) fn errors(assertions: &[Assertion]) -> Vec<UnusableEntry> {
    let targets: Vec<AssertionTarget> = assertions.iter().map(|a| a.target.clone()).collect();
    resolve_with(assertions, &targets)
        .1
        .err()
        .unwrap_or_default()
}

/// The car's `channel`, at instance `instance`.
pub(super) fn frame(channel: &'static str, instance: &'static str) -> Subject {
    Subject::channel(
        keyed(channel),
        Some(InstanceKey::of::<FrameSample>(&instance)),
    )
}

/// The car's channel `name`, narrowed by `instance`.
pub(super) fn instance(name: &str, instance: InstanceSelector) -> AssertionTarget {
    channel(name).with_instance(instance)
}

pub(super) fn one(instance: &str) -> InstanceSelector {
    InstanceSelector::One(instance.into())
}

/// An every-sample assertion on `target` whose condition fits a string
/// series, so only the selection is under test.
pub(super) fn assertion_of_kind(target: AssertionTarget) -> Assertion {
    judging(
        target,
        None,
        Condition::NotEquals(AssertionValue::String("none".into())),
    )
}

/// An assertion on `target`, reduced by `reduce`, judged by `condition`.
pub(super) fn judging(
    target: AssertionTarget,
    reduce: Option<Reducer>,
    condition: Condition,
) -> Assertion {
    Assertion {
        condition,
        ..reduced(target, reduce)
    }
}

/// The key each entry of `plan` reads, in plan order.
pub(super) fn plan_keys(plan: &ResolvedPlan) -> Vec<(EntryOwner, RecordedKey)> {
    plan.entries
        .iter()
        .filter_map(|entry| match &entry.source {
            SeriesSource::Recorded { key, .. } => Some((entry.owner, key.clone())),
            SeriesSource::Derived { .. } => None,
        })
        .collect()
}

/// `target` over a window starting `after` the series `anchor` names.
pub(super) fn anchored(target: AssertionTarget, anchor: &AssertionTarget) -> Assertion {
    Assertion {
        window: Window {
            start: Some(Bound::After {
                after: anchor.clone(),
                plus: 0.0,
            }),
            end: None,
        },
        ..assertion(target, false)
    }
}
