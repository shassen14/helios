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
    let mut text = format!(
        "schema_version = \"1.0\"\n\
         scenario = {{ from = \"s\" }}\n\
         observation = {{ from = \"o\" }}\n\
         termination = {{ {termination} }}\n"
    );
    for block in assertions {
        text.push_str("\n[[assert]]\n");
        text.push_str(block);
        text.push('\n');
    }
    toml::from_str(&text).expect("valid run toml")
}

/// A continuous assertion that `agent`'s [`LEAF`] equals `value`.
pub(super) fn cont(agent: &str, value: f64) -> String {
    assertion("continuous", agent, value)
}

/// A terminal assertion that `agent`'s [`LEAF`] equals `value`.
pub(super) fn term(agent: &str, value: f64) -> String {
    assertion("at_completion", agent, value)
}

/// A terminal assertion that `agent`'s [`SPEED`] channel equals `value`.
pub(super) fn speed_term(agent: &str, value: f64) -> String {
    assertion_on(
        "at_completion",
        &channel_path(agent, &speed::<f64>()),
        value,
    )
}

fn assertion(when: &str, agent: &str, value: f64) -> String {
    assertion_on(when, &target(agent), value)
}

// `{value:?}` forces a decimal point so TOML reads it as a float, not an int.
fn assertion_on(when: &str, target: &str, value: f64) -> String {
    format!(
        "when = \"{when}\"\n\
         target = \"{target}\"\n\
         condition = \"equals\"\n\
         value = {value:?}"
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

/// The state of each of `runner`'s assertions, in run-file order; none
/// unless the run is ready.
pub(super) fn states(runner: &Runner) -> Vec<&AssertionState> {
    match &runner.phase {
        Phase::Ready(plan) => plan.states().collect(),
        Phase::AwaitingPipelines | Phase::Aborted => Vec::new(),
    }
}
