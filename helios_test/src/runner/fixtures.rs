//! What the runner's tests build a run from: a run file, a pipeline whose
//! estimator declares one scalar leaf and writes one float channel and one
//! keyed channel of two instances, an evaluator kind comparing the two
//! channels, and a collector they are reported to.

use super::{state::AssertionState, Phase, Runner};
use crate::{
    assertion::extract::standard_extractors,
    collector::ObservationCollector,
    evaluation::{
        kind::{BoundInput, InputRejection},
        standard_evaluators,
        typed::{
            projections::number, Component, Input, NoParams, Typed, TypedEvaluator, TypedKind,
        },
        EvaluatorRegistry,
    },
    run::Run,
};
use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::{AgentId, MonotonicTime, TfProvider};
use helios_runtime::{
    observe::{
        key::{RecordedKey, Subject},
        observation::{Observation, ObservedValue},
        path::{channel_path, node_path},
        sink::ObservationSink,
    },
    port::{
        ChannelKey, ChannelKind, Determinism, InstanceKey, Keyed, PortBus, PortDescriptor,
        PortDescriptorBuilder, RefusedWrites,
    },
    prelude::{Health, PipelineNode, Stamped, TickContext},
    AutonomyPipeline, PipelineBuilder,
};

use std::sync::Arc;

pub(super) const CAR: &str = "car";
pub(super) const TRUCK: &str = "truck";
const ESTIMATOR: &str = "estimator";
/// The one leaf [`pipeline`] declares, in meters.
const LEAF: &str = "x";
/// The one channel [`pipeline`]'s estimator writes, carrying `f64`.
const SPEED: &str = "speed";
/// The keyed channel [`pipeline`]'s estimator writes, one instance per
/// wheel in [`WHEEL_NAMES`].
const WHEELS: &str = "wheels";
/// The instances of [`WHEELS`].
pub(super) const WHEEL_NAMES: [&str; 2] = ["front", "rear"];

/// One wheel's speed, keyed by the wheel. Read as its speed.
struct Wheel {
    wheel: &'static str,
    speed: f64,
}

impl Keyed for Wheel {
    type Instance = &'static str;

    fn instance(&self) -> &&'static str {
        &self.wheel
    }
}

/// The fixtures' evaluator kind, `lag`: how far [`SPEED`] (`actual`, the
/// base) falls short of one wheel's speed (`wanted`, held), in m/s. Refuses
/// a speed that isn't finite, and inputs from two agents.
struct Lag;

/// The name `lag` is registered under.
pub(super) const LAG: &str = "lag";

/// Its one metric, `wanted - actual`.
pub(super) const LAG_METRIC: &str = "shortfall";

impl TypedKind for Lag {
    const NAME: &'static str = LAG;
    type Params = NoParams;
    type Evaluator = LagEvaluator;

    fn inputs(&self) -> <LagEvaluator as TypedEvaluator>::Inputs {
        (
            Input::base("actual", |_, speed| number(speed)),
            Input::held("wanted", |_, wheel: &Wheel| number(&wheel.speed)),
        )
    }

    /// Known only once bound, as a kind's whose metrics come from what it
    /// reads, so setup, not load, checks the names an entry reads.
    fn menu(&self) -> Option<Vec<Component<f64>>> {
        None
    }

    fn bind(&self, _: NoParams, inputs: &[BoundInput]) -> Result<LagEvaluator, Vec<String>> {
        match inputs {
            [actual, wanted] if actual.agent == wanted.agent => Ok(LagEvaluator),
            _ => Err(vec!["lag compares one agent's speeds".to_owned()]),
        }
    }
}

struct LagEvaluator;

impl TypedEvaluator for LagEvaluator {
    type Inputs = (Input<Self, f64, f64>, Input<Self, Wheel, f64>);
    type Fit = ();
    type Error = f64;

    fn components(&self) -> Vec<Component<f64>> {
        vec![Component::each(
            LAG_METRIC,
            SiUnit::MetersPerSecond,
            |shortfall: &f64| Ok(*shortfall),
        )]
    }

    fn fit(&self, _: &[(&f64, &f64)]) -> Result<(), InputRejection> {
        Ok(())
    }

    fn compute(&self, (actual, wanted): (&f64, &f64), _: &()) -> Result<f64, InputRejection> {
        Ok(wanted - actual)
    }
}

/// The standard evaluator kinds and [`Lag`].
pub(super) fn evaluators() -> EvaluatorRegistry {
    let mut registry = standard_evaluators();
    registry.register(Typed(Lag));
    registry
}

/// A `lag` evaluation named `name` of `agent`'s speed against its front
/// wheel's, as an `[[evaluate]]` block.
pub(super) fn lag(name: &str, agent: &str) -> String {
    format!(
        "name = \"{name}\"\n\
         kind = \"{LAG}\"\n\
         [evaluate.inputs]\n\
         actual = \"{}\"\n\
         wanted = {{ path = \"{}\", instance = \"front\" }}",
        speed_target(agent),
        channel_path(agent, &wheels())
    )
}

/// The derived series of `lag` evaluation `name`.
pub(super) fn lag_target(name: &str) -> String {
    format!("eval.inline.{name}.{LAG_METRIC}")
}

/// Declares the scalar leaf [`LEAF`]; executes nothing.
struct Estimator(PortDescriptor);

impl PipelineNode for Estimator {
    fn name(&self) -> &str {
        ESTIMATOR
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.0
    }

    fn execute(&self, _: &PortBus, _: &dyn TfProvider, _: TickContext) {}
}

pub(super) fn at(secs: f64) -> MonotonicTime {
    MonotonicTime(secs)
}

/// The target on `agent`'s [`LEAF`].
pub(super) fn target(agent: &str) -> String {
    node_path(agent, ESTIMATOR, LEAF)
}

/// The target on `agent`'s [`SPEED`] channel.
pub(super) fn speed_target(agent: &str) -> String {
    channel_path(agent, &speed::<f64>())
}

/// The selector of every instance of `agent`'s [`WHEELS`], as TOML.
pub(super) fn each_wheel(agent: &str) -> String {
    format!(
        "{{ path = \"{}\", instance = \"*\" }}",
        channel_path(agent, &wheels())
    )
}

/// The key of `agent`'s `wheel`.
pub(super) fn wheel_key(agent: &str, wheel: &'static str) -> RecordedKey {
    RecordedKey {
        agent: AgentId::new(agent),
        subject: wheel_subject(wheel),
    }
}

/// The key of `agent`'s [`LEAF`].
pub(super) fn leaf_key(agent: &str) -> RecordedKey {
    RecordedKey {
        agent: AgentId::new(agent),
        subject: Subject::emission(ESTIMATOR, LEAF),
    }
}

/// The key of `agent`'s [`SPEED`].
pub(super) fn speed_key(agent: &str) -> RecordedKey {
    RecordedKey {
        agent: AgentId::new(agent),
        subject: Subject::channel(speed::<f64>(), None),
    }
}

fn wheels() -> ChannelKey {
    ChannelKey::keyed::<Wheel>(ChannelKind::Internal, WHEELS)
}

fn wheel_subject(wheel: &'static str) -> Subject {
    Subject::channel(wheels(), Some(InstanceKey::of::<Wheel>(&wheel)))
}

/// [`SPEED`] carrying `T`. The pipeline declares it carrying `f64`.
fn speed<T: Send + Sync + 'static>() -> ChannelKey {
    ChannelKey::internal::<T>(SPEED)
}

/// A `Run` from a termination body and zero or more assertion blocks.
/// Fields are private, so it is built through TOML. `termination` is an
/// inline-table body (comma-separated keys).
pub(super) fn run(termination: &str, assertions: &[&str]) -> Run {
    run_measuring(termination, assertions, &[])
}

/// As [`run`], with measure blocks too.
pub(super) fn run_measuring(termination: &str, assertions: &[&str], measures: &[&str]) -> Run {
    run_evaluating(termination, assertions, measures, &[])
}

/// As [`run_measuring`], with evaluation blocks too.
pub(super) fn run_evaluating(
    termination: &str,
    assertions: &[&str],
    measures: &[&str],
    evaluations: &[&str],
) -> Run {
    let mut text = format!(
        "schema_version = \"2.0\"\n\
         scenario = {{ from = \"s\" }}\n\
         observation = {{ from = \"o\" }}\n\
         termination = {{ {termination} }}\n"
    );
    for block in assertions {
        text.push_str("\n[[assert]]\n");
        text.push_str(block);
        text.push('\n');
    }
    for block in measures {
        text.push_str("\n[[measure]]\n");
        text.push_str(block);
        text.push('\n');
    }
    for block in evaluations {
        text.push_str("\n[[evaluate]]\n");
        text.push_str(block);
        text.push('\n');
    }
    toml::from_str(&text).expect("valid run toml")
}

/// An assertion named `name` that every sample of `agent`'s [`LEAF`] is
/// `value`.
pub(super) fn cont(name: &str, agent: &str, value: f64) -> String {
    assertion_on(name, &target(agent), None, value)
}

/// An assertion named `name` that the final sample of `agent`'s [`LEAF`] is
/// `value`.
pub(super) fn term(name: &str, agent: &str, value: f64) -> String {
    assertion_on(name, &target(agent), Some("\"final\""), value)
}

/// An assertion named `name` that the final sample of `agent`'s [`SPEED`]
/// channel is `value`.
pub(super) fn speed_term(name: &str, agent: &str, value: f64) -> String {
    assertion_on(name, &speed_target(agent), Some("\"final\""), value)
}

/// An assertion named `name` that `agent`'s [`LEAF`], reduced by `reduce`
/// (as written in TOML), is below `bound`.
pub(super) fn reduced_below(name: &str, agent: &str, reduce: &str, bound: f64) -> String {
    format!(
        "name = \"{name}\"\n\
         target = \"{}\"\n\
         reduce = {reduce}\n\
         condition = {{ less_than = {bound:?} }}",
        target(agent)
    )
}

/// An entry named `name` on `target` over `window`, both as written in
/// TOML, with `rest` (more key lines: a condition, a reducer).
pub(super) fn windowed(name: &str, target: &str, window: &str, rest: &str) -> String {
    format!(
        "name = \"{name}\"\n\
         target = \"{target}\"\n\
         window = {window}\n\
         {rest}"
    )
}

/// A measure named `name` of `agent`'s [`LEAF`], reduced by `reduce` (as
/// written in TOML).
pub(super) fn measure(name: &str, agent: &str, reduce: &str) -> String {
    format!(
        "name = \"{name}\"\n\
         target = \"{}\"\n\
         reduce = {reduce}",
        target(agent)
    )
}

// Exact equality on a float is refused at load, so "is `value`" is the
// one-point range. `{value:?}` forces a decimal point.
fn assertion_on(name: &str, target: &str, reduce: Option<&str>, value: f64) -> String {
    let reduce = reduce
        .map(|reduce| format!("reduce = {reduce}\n"))
        .unwrap_or_default();
    format!(
        "name = \"{name}\"\n\
         target = \"{target}\"\n\
         {reduce}\
         condition = {{ within_range = [{value:?}, {value:?}] }}"
    )
}

/// An estimator declaring [`LEAF`] and writing [`SPEED`] and [`WHEELS`].
pub(super) fn pipeline() -> AutonomyPipeline {
    let descriptor = PortDescriptorBuilder::algorithm()
        .outputs_from_slice(&[speed::<f64>()])
        .output_keyed::<Wheel>(wheels(), WHEEL_NAMES)
        .observable(LEAF, Determinism::Deterministic, SiUnit::Meters)
        .build();
    PipelineBuilder::new(AgentId::new(CAR))
        .add_node(Box::new(Estimator(descriptor)))
        .build()
        .expect("one node with no inputs builds")
}

/// `agent`'s bus refusing one write, to a channel no writer declares.
pub(super) fn refused_write(agent: &str) -> (AgentId, RefusedWrites) {
    let pipeline = pipeline();
    let undeclared = ChannelKey::internal::<f64>("undeclared");
    let stamped = Stamped {
        value: 0.0_f64,
        timestamp: at(0.0),
        health: Health::Ok,
        producer: 0,
    };
    let _ = pipeline.bus().write(undeclared, stamped);
    let refused = pipeline
        .bus()
        .refused_writes()
        .into_iter()
        .next()
        .expect("the write to an undeclared channel is refused");
    (AgentId::new(agent), refused)
}

/// A runner for `run`, set up against `agents` (each with [`pipeline`]),
/// with the collector it reads.
pub(super) fn ready(run: Run, agents: &[&str]) -> (Runner, ObservationCollector) {
    let (runner, collector, setup) = set_up(run, agents);
    setup.expect("every target is a declared leaf");
    (runner, collector)
}

/// As [`ready`], with what setup found, which may refuse the run.
pub(super) fn set_up(
    run: Run,
    agents: &[&str],
) -> (
    Runner,
    ObservationCollector,
    Result<(), super::setup::SetupError>,
) {
    let mut runner = Runner::with_evaluators(run, evaluators()).expect("valid run");
    let mut extractors = standard_extractors();
    extractors.register::<Wheel, f64>(|wheel| wheel.speed, Some(SiUnit::MetersPerSecond));
    let mut collector = runner.collector_reading(extractors);
    let pipelines: Vec<(AgentId, AutonomyPipeline)> = agents
        .iter()
        .map(|agent| (AgentId::new(*agent), pipeline()))
        .collect();
    let pairs: Vec<(AgentId, &AutonomyPipeline)> = pipelines
        .iter()
        .map(|(agent, pipeline)| (agent.clone(), pipeline))
        .collect();
    let setup = runner.on_pipeline_built(&pairs, &mut collector);
    (runner, collector, setup)
}

/// `agent`'s estimator reports `value` for [`LEAF`] at `secs`.
pub(super) fn report(collector: &mut ObservationCollector, agent: &str, secs: f64, value: f64) {
    observe(collector, agent, secs, ObservedValue::Scalar(value));
}

/// `agent`'s estimator reports a value for [`LEAF`] at `secs` that the
/// collector turns away: a channel payload no extractor reads.
pub(super) fn report_unreadable(collector: &mut ObservationCollector, agent: &str, secs: f64) {
    let unreadable = Stamped {
        value: String::new(),
        timestamp: at(secs),
        health: Health::Ok,
        producer: 0,
    };
    observe(
        collector,
        agent,
        secs,
        ObservedValue::Typed(Arc::new(unreadable)),
    );
}

/// `agent`'s [`SPEED`] reports `value` at `secs`, recorded under the key
/// the pipeline declares, as a tap carries it.
pub(super) fn report_speed(
    collector: &mut ObservationCollector,
    agent: &str,
    secs: f64,
    value: f64,
) {
    observe_under(
        collector,
        agent,
        Subject::channel(speed::<f64>(), None),
        secs,
        tapped(value, secs),
    );
}

/// `agent`'s `wheel` reports `value` at `secs`.
pub(super) fn report_wheel(
    collector: &mut ObservationCollector,
    agent: &str,
    wheel: &'static str,
    secs: f64,
    value: f64,
) {
    observe_under(
        collector,
        agent,
        wheel_subject(wheel),
        secs,
        tapped(
            Wheel {
                wheel,
                speed: value,
            },
            secs,
        ),
    );
}

/// `value` as a tap carries it: the bus's own stamped value.
fn tapped<T: Send + Sync + 'static>(value: T, secs: f64) -> ObservedValue {
    ObservedValue::Typed(Arc::new(Stamped {
        value,
        timestamp: at(secs),
        health: Health::Ok,
        producer: 0,
    }))
}

/// A value at `secs` under `agent`'s [`SPEED`] path but a key no series is
/// bound to: the channel carrying `bool`, which the pipeline never declared.
pub(super) fn report_unbound(collector: &mut ObservationCollector, agent: &str, secs: f64) {
    observe_under(
        collector,
        agent,
        Subject::channel(speed::<bool>(), None),
        secs,
        ObservedValue::Scalar(0.0),
    );
}

fn observe(collector: &mut ObservationCollector, agent: &str, secs: f64, value: ObservedValue) {
    observe_under(
        collector,
        agent,
        Subject::emission(ESTIMATOR, LEAF),
        secs,
        value,
    );
}

fn observe_under(
    collector: &mut ObservationCollector,
    agent: &str,
    subject: Subject,
    secs: f64,
    value: ObservedValue,
) {
    collector.receive(&[Observation {
        key: RecordedKey {
            agent: AgentId::new(agent),
            subject,
        },
        timestamp: at(secs),
        value,
    }]);
}

/// The state of each of `runner`'s assertions, then measures, in run-file
/// order; none unless the run is ready.
pub(super) fn states(runner: &Runner) -> Vec<&AssertionState> {
    match &runner.phase {
        Phase::Ready(plan) => plan.states().collect(),
        Phase::AwaitingPipelines | Phase::Aborted => Vec::new(),
    }
}
