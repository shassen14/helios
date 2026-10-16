//! What the runner's tests build a run from: a run file, a pipeline whose
//! estimator declares one scalar leaf, and a collector that leaf is reported
//! to.

use super::{state::AssertionState, Phase, Runner};
use crate::{collector::ObservationCollector, run::Run};
use helios_core::prelude::{AgentId, MonotonicTime, TfProvider};
use helios_runtime::{
    observe::{
        observation::{AgentObservation, Observation, ObservedValue},
        path::node_path,
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

// `{value:?}` forces a decimal point so TOML reads it as a float, not an int.
fn assertion(when: &str, agent: &str, value: f64) -> String {
    format!(
        "when = \"{when}\"\n\
         target = \"{}\"\n\
         condition = \"equals\"\n\
         value = {value:?}",
        target(agent)
    )
}

/// An estimator declaring [`LEAF`].
pub(super) fn pipeline() -> AutonomyPipeline {
    let descriptor = PortDescriptorBuilder::algorithm()
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
    let collector = runner.collector();
    let pipelines: Vec<(AgentId, AutonomyPipeline)> = agents
        .iter()
        .map(|agent| (AgentId::new(*agent), pipeline()))
        .collect();
    let pairs: Vec<(AgentId, &AutonomyPipeline)> = pipelines
        .iter()
        .map(|(agent, pipeline)| (agent.clone(), pipeline))
        .collect();
    runner
        .on_pipeline_built(&pairs, &collector)
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

fn observe(collector: &mut ObservationCollector, agent: &str, secs: f64, value: ObservedValue) {
    collector.receive(&[AgentObservation {
        agent: AgentId::new(agent),
        observation: Observation {
            node: ESTIMATOR.into(),
            leaf: LEAF.into(),
            timestamp: at(secs),
            value,
        },
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
