//! What the runner's tests build a run from: a run file, a pipeline whose
//! estimator declares one scalar leaf and writes one float channel, and a
//! collector that leaf is reported to.

use super::{state::AssertionState, Phase, Runner};
use crate::{collector::ObservationCollector, run::Run};
use helios_core::prelude::{AgentId, MonotonicTime, TfProvider};
use helios_runtime::{
    observe::{
        key::{RecordedKey, Subject},
        observation::{Observation, ObservedValue},
        path::{channel_path, node_path},
        sink::ObservationSink,
    },
    port::{
        ChannelKey, Determinism, PortBus, PortDescriptor, PortDescriptorBuilder, RefusedWrites,
    },
    prelude::{Health, PipelineNode, Stamped, TickContext},
    AutonomyPipeline, PipelineBuilder,
};

use std::sync::Arc;

pub(super) const CAR: &str = "car";
pub(super) const TRUCK: &str = "truck";
const ESTIMATOR: &str = "estimator";
/// The one leaf [`pipeline`] declares.
const LEAF: &str = "x";
/// The one channel [`pipeline`]'s estimator writes, carrying `f64`.
const SPEED: &str = "speed";

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
fn target(agent: &str) -> String {
    node_path(agent, ESTIMATOR, LEAF)
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
    assertion_on(
        name,
        &channel_path(agent, &speed::<f64>()),
        Some("\"final\""),
        value,
    )
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

/// An estimator declaring [`LEAF`] and writing [`SPEED`].
pub(super) fn pipeline() -> AutonomyPipeline {
    let descriptor = PortDescriptorBuilder::algorithm()
        .outputs_from_slice(&[speed::<f64>()])
        .observable(LEAF, Determinism::Deterministic)
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
    let mut runner = Runner::new(run).expect("valid run");
    let mut collector = runner.collector();
    let pipelines: Vec<(AgentId, AutonomyPipeline)> = agents
        .iter()
        .map(|agent| (AgentId::new(*agent), pipeline()))
        .collect();
    let pairs: Vec<(AgentId, &AutonomyPipeline)> = pipelines
        .iter()
        .map(|(agent, pipeline)| (agent.clone(), pipeline))
        .collect();
    runner
        .on_pipeline_built(&pairs, &mut collector)
        .expect("every target is a declared leaf");
    (runner, collector)
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
/// the pipeline declares.
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
        ObservedValue::Scalar(value),
    );
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
