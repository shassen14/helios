// DAG execution engine tests (#35): builder validation, level ordering,
// rate gating, and tick semantics.

#![allow(dead_code)]

use std::sync::{
    atomic::{AtomicU32, Ordering},
    Arc,
};

use helios_core::data::primitives::MonotonicTime;
use helios_runtime::{
    pipeline::{PipelineBuildError, PipelineBuilder},
    port::{ChannelKey, PortBus, PortDescriptor},
    prelude::{Health, PipelineNode, Stamped, TickContext},
};

use crate::common::MockRuntime;

// =========================================================================
// == Dummy PipelineNode implementations ==
// =========================================================================

/// Writes a fixed u32 value to a single output channel.
struct ProducerNode {
    name: String,
    descriptor: PortDescriptor,
    output: ChannelKey,
    value: u32,
}

impl ProducerNode {
    fn new(name: &str, output: ChannelKey, value: u32) -> Self {
        Self {
            name: name.to_string(),
            descriptor: PortDescriptor {
                required_inputs: vec![],
                optional_inputs: vec![],
                outputs: vec![output.clone()],
                rate: None,
            },
            output,
            value,
        }
    }
}

impl PipelineNode for ProducerNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(
        &self,
        bus: &PortBus,
        _runtime: &dyn helios_runtime::prelude::AgentRuntime,
        tick: TickContext,
    ) {
        let stamped = Stamped {
            value: self.value,
            timestamp: tick.now,
            health: Health::Ok,
            producer: tick.node_id,
        };
        let _ = bus.write(self.output.clone(), stamped);
    }
}

/// Reads a u32 from `input`, applies a transform, writes to `output`.
/// Early-returns if the input is not present — this is how we detect
/// ordering violations (a chained node placed too early sees no input
/// and produces nothing).
struct TransformNode {
    name: String,
    descriptor: PortDescriptor,
    input: ChannelKey,
    output: ChannelKey,
    transform: fn(u32) -> u32,
}

impl TransformNode {
    fn new(name: &str, input: ChannelKey, output: ChannelKey, transform: fn(u32) -> u32) -> Self {
        Self {
            name: name.to_string(),
            descriptor: PortDescriptor {
                required_inputs: vec![input.clone()],
                optional_inputs: vec![],
                outputs: vec![output.clone()],
                rate: None,
            },
            input,
            output,
            transform,
        }
    }
}

impl PipelineNode for TransformNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(
        &self,
        bus: &PortBus,
        _runtime: &dyn helios_runtime::prelude::AgentRuntime,
        tick: TickContext,
    ) {
        let Some(input) = bus.read::<u32>(self.input.clone()) else {
            return;
        };
        let out = (self.transform)(input.value);
        let stamped = Stamped {
            value: out,
            timestamp: tick.now,
            health: Health::Ok,
            producer: tick.node_id,
        };
        let _ = bus.write(self.output.clone(), stamped);
    }
}

/// Reads two u32 inputs, sums them, writes to output.
struct JoinNode {
    name: String,
    descriptor: PortDescriptor,
    input_a: ChannelKey,
    input_b: ChannelKey,
    output: ChannelKey,
}

impl JoinNode {
    fn new(name: &str, input_a: ChannelKey, input_b: ChannelKey, output: ChannelKey) -> Self {
        Self {
            name: name.to_string(),
            descriptor: PortDescriptor {
                required_inputs: vec![input_a.clone(), input_b.clone()],
                optional_inputs: vec![],
                outputs: vec![output.clone()],
                rate: None,
            },
            input_a,
            input_b,
            output,
        }
    }
}

impl PipelineNode for JoinNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(
        &self,
        bus: &PortBus,
        _runtime: &dyn helios_runtime::prelude::AgentRuntime,
        tick: TickContext,
    ) {
        let Some(a) = bus.read::<u32>(self.input_a.clone()) else {
            return;
        };
        let Some(b) = bus.read::<u32>(self.input_b.clone()) else {
            return;
        };
        let stamped = Stamped {
            value: a.value + b.value,
            timestamp: tick.now,
            health: Health::Ok,
            producer: tick.node_id,
        };
        let _ = bus.write(self.output.clone(), stamped);
    }
}

/// Increments a shared counter on every execute. Used for rate-gating tests.
struct CountingNode {
    name: String,
    descriptor: PortDescriptor,
    counter: Arc<AtomicU32>,
}

impl CountingNode {
    fn new(name: &str, rate_hz: Option<f64>, counter: Arc<AtomicU32>) -> Self {
        Self {
            name: name.to_string(),
            descriptor: PortDescriptor {
                required_inputs: vec![],
                optional_inputs: vec![],
                outputs: vec![],
                rate: rate_hz,
            },
            counter,
        }
    }
}

impl PipelineNode for CountingNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(
        &self,
        _bus: &PortBus,
        _runtime: &dyn helios_runtime::prelude::AgentRuntime,
        _tick: TickContext,
    ) {
        self.counter.fetch_add(1, Ordering::Relaxed);
    }
}

/// Node that requires an input but produces no output. Used when we need
/// a consumer for a channel without writing anything back.
struct SinkNode {
    name: String,
    descriptor: PortDescriptor,
}

impl SinkNode {
    fn new(name: &str, input: ChannelKey) -> Self {
        Self {
            name: name.to_string(),
            descriptor: PortDescriptor {
                required_inputs: vec![input],
                optional_inputs: vec![],
                outputs: vec![],
                rate: None,
            },
        }
    }
}

impl PipelineNode for SinkNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(
        &self,
        _bus: &PortBus,
        _runtime: &dyn helios_runtime::prelude::AgentRuntime,
        _tick: TickContext,
    ) {
    }
}

// Tag types so each test can mint distinct ChannelKeys without colliding.
struct ChA;
struct ChB;
struct ChC;
struct ChD;
struct Sensor;

// =========================================================================
// == Build-time validation ==
// =========================================================================

#[test]
fn single_node_executes() {
    let out = ChannelKey::of::<ChA>();
    let pipeline = PipelineBuilder::new()
        .add_node(Box::new(ProducerNode::new("a", out.clone(), 42)))
        .build()
        .expect("build should succeed");

    pipeline.tick(&MockRuntime, 0.1);

    let value = pipeline
        .bus()
        .read::<u32>(out)
        .expect("output should exist");
    assert_eq!(value.value, 42);
}

#[test]
fn two_node_chain_level_ordering() {
    let a = ChannelKey::of::<ChA>();
    let b = ChannelKey::of::<ChB>();

    let pipeline = PipelineBuilder::new()
        .add_node(Box::new(ProducerNode::new("a", a.clone(), 7)))
        .add_node(Box::new(TransformNode::new(
            "b",
            a.clone(),
            b.clone(),
            |v| v * 2,
        )))
        .build()
        .expect("build should succeed");

    pipeline.tick(&MockRuntime, 0.1);

    // If B were placed in level 0 alongside A, B would early-return (A's
    // output not yet on the bus when B reads). Output present means B ran
    // after A.
    let result = pipeline
        .bus()
        .read::<u32>(b)
        .expect("b output should exist");
    assert_eq!(result.value, 14);
}

#[test]
fn two_node_chain_with_builder_inserted_out_of_order() {
    // B added before A — toposort must still resolve A→B.
    let a = ChannelKey::of::<ChA>();
    let b = ChannelKey::of::<ChB>();

    let pipeline = PipelineBuilder::new()
        .add_node(Box::new(TransformNode::new(
            "b",
            a.clone(),
            b.clone(),
            |v| v + 1,
        )))
        .add_node(Box::new(ProducerNode::new("a", a.clone(), 10)))
        .build()
        .expect("build should succeed");

    pipeline.tick(&MockRuntime, 0.1);

    let result = pipeline
        .bus()
        .read::<u32>(b)
        .expect("b output should exist");
    assert_eq!(result.value, 11);
}

#[test]
fn three_node_diamond_resolves() {
    // A → (B, C) → D, with D summing B and C's outputs.
    let a = ChannelKey::of::<ChA>();
    let b = ChannelKey::of::<ChB>();
    let c = ChannelKey::of::<ChC>();
    let d = ChannelKey::of::<ChD>();

    let pipeline = PipelineBuilder::new()
        .add_node(Box::new(ProducerNode::new("a", a.clone(), 1)))
        .add_node(Box::new(TransformNode::new(
            "b",
            a.clone(),
            b.clone(),
            |v| v * 2,
        )))
        .add_node(Box::new(TransformNode::new(
            "c",
            a.clone(),
            c.clone(),
            |v| v * 3,
        )))
        .add_node(Box::new(JoinNode::new(
            "d",
            b.clone(),
            c.clone(),
            d.clone(),
        )))
        .build()
        .expect("build should succeed");

    pipeline.tick(&MockRuntime, 0.1);

    // A=1, B=A*2=2, C=A*3=3, D=B+C=5.
    let result = pipeline
        .bus()
        .read::<u32>(d)
        .expect("d output should exist");
    assert_eq!(result.value, 5);
}

#[test]
fn cycle_detected() {
    // A requires B's output, B requires A's output.
    let a_out = ChannelKey::named::<ChA>("a_out");
    let b_out = ChannelKey::named::<ChB>("b_out");

    let result = PipelineBuilder::new()
        .add_node(Box::new(TransformNode::new(
            "a",
            b_out.clone(),
            a_out.clone(),
            |v| v,
        )))
        .add_node(Box::new(TransformNode::new(
            "b",
            a_out.clone(),
            b_out.clone(),
            |v| v,
        )))
        .build();

    let Err(errors) = result else {
        panic!("should fail with cycle");
    };
    assert!(
        errors
            .iter()
            .any(|e| matches!(e, PipelineBuildError::Cycle)),
        "expected Cycle, got {:?}",
        errors
    );
}

#[test]
fn unsatisfied_input_detected() {
    let missing = ChannelKey::named::<ChA>("missing");
    let out = ChannelKey::of::<ChB>();

    let result = PipelineBuilder::new()
        .add_node(Box::new(TransformNode::new(
            "needs_missing",
            missing.clone(),
            out,
            |v| v,
        )))
        .build();

    let Err(errors) = result else {
        panic!("should fail with unsatisfied input");
    };
    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineBuildError::UnsatisfiedInput { node_name, channel }
                if node_name == "needs_missing" && *channel == missing
        )),
        "expected UnsatisfiedInput for 'needs_missing'/{missing:?}, got {errors:?}"
    );
}

#[test]
fn multiple_producers_detected() {
    let shared = ChannelKey::of::<ChA>();

    let result = PipelineBuilder::new()
        .add_node(Box::new(ProducerNode::new("a", shared.clone(), 1)))
        .add_node(Box::new(ProducerNode::new("b", shared.clone(), 2)))
        .build();

    let Err(errors) = result else {
        panic!("should fail with multiple producers");
    };
    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineBuildError::MultipleProducers { channel, .. } if *channel == shared
        )),
        "expected MultipleProducers, got {errors:?}"
    );
}

#[test]
fn external_channels_satisfy_required_inputs() {
    // A node requires a channel that no node produces. Declaring it via
    // with_external_channels must make the build succeed.
    let sensor_key = ChannelKey::named::<Sensor>("imu");

    let result = PipelineBuilder::new()
        .add_node(Box::new(SinkNode::new("consumer", sensor_key.clone())))
        .with_external_channels(vec![sensor_key])
        .build();

    assert!(result.is_ok(), "build should succeed: {:?}", result.err());
}

// =========================================================================
// == Rate gating ==
// =========================================================================

#[test]
fn rate_gated_node_fires_at_correct_interval() {
    // Period = 1 / 2 Hz = 0.5s. dt = 0.1s → fires on the 5th tick only.
    let counter = Arc::new(AtomicU32::new(0));
    let pipeline = PipelineBuilder::new()
        .add_node(Box::new(CountingNode::new(
            "rated",
            Some(2.0),
            counter.clone(),
        )))
        .build()
        .expect("build should succeed");

    for _ in 0..4 {
        pipeline.tick(&MockRuntime, 0.1);
    }
    assert_eq!(
        counter.load(Ordering::Relaxed),
        0,
        "must not fire before 0.5s elapsed"
    );

    pipeline.tick(&MockRuntime, 0.1);
    assert_eq!(counter.load(Ordering::Relaxed), 1, "must fire on 5th tick");
}

#[test]
fn rate_gated_node_does_not_double_fire() {
    // dt = 1.0s with period = 0.5s would cover two periods; node must
    // still fire exactly once (no catch-up).
    let counter = Arc::new(AtomicU32::new(0));
    let pipeline = PipelineBuilder::new()
        .add_node(Box::new(CountingNode::new(
            "rated",
            Some(2.0),
            counter.clone(),
        )))
        .build()
        .expect("build should succeed");

    pipeline.tick(&MockRuntime, 1.0);
    assert_eq!(
        counter.load(Ordering::Relaxed),
        1,
        "must fire exactly once on slow tick"
    );
}

#[test]
fn unrated_node_fires_every_tick() {
    // rate: None means fire every tick.
    let counter = Arc::new(AtomicU32::new(0));
    let pipeline = PipelineBuilder::new()
        .add_node(Box::new(CountingNode::new(
            "every_tick",
            None,
            counter.clone(),
        )))
        .build()
        .expect("build should succeed");

    for _ in 0..3 {
        pipeline.tick(&MockRuntime, 0.01);
    }
    assert_eq!(counter.load(Ordering::Relaxed), 3);
}

// =========================================================================
// == Tick semantics ==
// =========================================================================

#[test]
fn tick_preserves_externally_written_values() {
    // All bus slots are last-known-good — a value written from outside
    // the graph must still be readable after `tick()`.
    let sensor_key = ChannelKey::named::<Sensor>("imu");
    let pipeline = PipelineBuilder::new()
        .add_node(Box::new(SinkNode::new("consumer", sensor_key.clone())))
        .with_external_channels(vec![sensor_key.clone()])
        .build()
        .expect("build should succeed");

    let stamped = Stamped {
        value: 99u32,
        timestamp: MonotonicTime(0.0),
        health: Health::Ok,
        producer: 0,
    };
    pipeline
        .bus()
        .write(sensor_key.clone(), stamped)
        .expect("write should succeed");

    pipeline.tick(&MockRuntime, 0.1);

    let after = pipeline
        .bus()
        .read::<u32>(sensor_key)
        .expect("value should still be present after tick");
    assert_eq!(after.value, 99);
}
