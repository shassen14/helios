// DAG execution engine tests (#35): builder validation, level ordering,
// rate gating, and tick semantics.

#![allow(dead_code)]

use std::sync::{
    atomic::{AtomicBool, AtomicU32, Ordering},
    Arc, Mutex,
};

use helios_core::prelude::{AgentId, MonotonicTime};
use helios_core::spatial::id::FrameId;
use helios_core::spatial::transforms::tf::stamped::StampedTransform;
use helios_core::spatial::transforms::Convention;
use helios_runtime::{
    channels::tf::tf,
    pipeline::{
        ChannelUses, CycleEdge, MountLookupError, NodePlan, PipelineBuildError, PipelineBuilder,
        ReadMount,
    },
    port::{ChannelKey, InputNeed, InstanceKey, PortBus, PortDescriptor, PortDescriptorBuilder},
    prelude::{Health, PipelineNode, Stamped, TickContext},
    BodyCapabilities, BodyChannel, MountRefusal, Provenance, Writer,
};

use crate::common::MockRuntime;

/// The agent every pipeline in this file serves.
const AGENT: &str = "bot";
/// The unit every body channel in this file comes from.
const BODY_UNIT: &str = "test_unit";

/// Test-only sugar: every channel in this file is internal (algorithm-to-
/// algorithm wiring).
fn ikey_named<T: 'static>(name: &'static str) -> ChannelKey {
    ChannelKey::internal::<T>(name)
}

/// Oracle-channel sugar — only mocks/viz/tests may mint these.
fn okey_named<T: 'static>(name: &'static str) -> ChannelKey {
    ChannelKey::oracle::<T>(name)
}

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
            descriptor: PortDescriptorBuilder::algorithm()
                .outputs_from_slice(&[output.clone()])
                .build(),
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
        _tf: &dyn helios_core::prelude::TfProvider,
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
            descriptor: PortDescriptorBuilder::algorithm()
                .inputs_from_slices(&[input.clone()], &[])
                .outputs_from_slice(&[output.clone()])
                .build(),
            input,
            output,
            transform,
        }
    }

    /// Same node with `input` declared optional. `execute` is unchanged, so it
    /// still writes nothing when the input is absent.
    fn optional(
        name: &str,
        input: ChannelKey,
        output: ChannelKey,
        transform: fn(u32) -> u32,
    ) -> Self {
        Self {
            name: name.to_string(),
            descriptor: PortDescriptorBuilder::algorithm()
                .inputs_from_slices(&[], &[input.clone()])
                .outputs_from_slice(&[output.clone()])
                .build(),
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
        _tf: &dyn helios_core::prelude::TfProvider,
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
            descriptor: PortDescriptorBuilder::algorithm()
                .inputs_from_slices(&[input_a.clone(), input_b.clone()], &[])
                .outputs_from_slice(&[output.clone()])
                .build(),
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
        _tf: &dyn helios_core::prelude::TfProvider,
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
            descriptor: match rate_hz {
                Some(hz) => PortDescriptorBuilder::algorithm().rate_hz(hz),
                None => PortDescriptorBuilder::algorithm(),
            }
            .build(),
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
        _tf: &dyn helios_core::prelude::TfProvider,
        _tick: TickContext,
    ) {
        self.counter.fetch_add(1, Ordering::Relaxed);
    }
}

/// Node that reads one input, required or optional, and produces no output.
/// Used when we need a consumer for a channel without writing anything back.
struct SinkNode {
    name: String,
    descriptor: PortDescriptor,
}

impl SinkNode {
    fn new(name: &str, input: ChannelKey) -> Self {
        Self {
            name: name.to_string(),
            descriptor: // Mock builder: some tests sink an oracle channel.
            PortDescriptorBuilder::mock()
                .inputs_from_slices(&[input], &[])
                .build(),
        }
    }

    fn optional(name: &str, input: ChannelKey) -> Self {
        Self {
            name: name.to_string(),
            descriptor: PortDescriptorBuilder::mock()
                .inputs_from_slices(&[], &[input])
                .build(),
        }
    }

    /// A sink that resolves its input's mount through the tf tree, in the
    /// axes `convention` names.
    fn reading_mount_in(name: &str, input: ChannelKey, convention: Convention) -> Self {
        Self {
            name: name.to_string(),
            descriptor: PortDescriptorBuilder::algorithm()
                .inputs_from_slices(std::slice::from_ref(&input), &[])
                .reads_mount_in(&input, convention)
                .build(),
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
        _tf: &dyn helios_core::prelude::TfProvider,
        _tick: TickContext,
    ) {
    }
}

// Tag types so each test can mint distinct ChannelKeys without colliding. A
// test that writes or reads values keys its channels by the value's type
// instead, since the bus refuses a value of another type than its key's.
struct ChA;
struct ChB;
struct ChC;
struct ChD;
struct Sensor;
struct Truth;

// =========================================================================
// == Build-time validation ==
// =========================================================================

/// The participants and edges of every `Cycle` error, in the order returned.
fn cycles(errors: &[PipelineBuildError]) -> Vec<(Vec<&str>, Vec<CycleEdge>)> {
    errors
        .iter()
        .filter_map(|e| match e {
            PipelineBuildError::Cycle {
                participants,
                edges,
            } => Some((
                participants.iter().map(String::as_str).collect(),
                edges.clone(),
            )),
            _ => None,
        })
        .collect()
}

fn edge(producer: &str, consumer: &str, channel: &ChannelKey) -> CycleEdge {
    CycleEdge {
        producer: producer.to_string(),
        consumer: consumer.to_string(),
        channel: channel.clone(),
    }
}

#[test]
fn single_node_executes() {
    let out = ikey_named::<u32>("a");
    let pipeline = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(ProducerNode::new("a", out.clone(), 42)))
        .build()
        .expect("build should succeed");

    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);

    let value = pipeline
        .bus()
        .read::<u32>(out)
        .expect("output should exist");
    assert_eq!(value.value, 42);
}

#[test]
fn two_node_chain_level_ordering() {
    let a = ikey_named::<u32>("a");
    let b = ikey_named::<u32>("b");

    let pipeline = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(ProducerNode::new("a", a.clone(), 7)))
        .add_node(Box::new(TransformNode::new(
            "b",
            a.clone(),
            b.clone(),
            |v| v * 2,
        )))
        .build()
        .expect("build should succeed");

    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);

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
    let a = ikey_named::<u32>("a");
    let b = ikey_named::<u32>("b");

    let pipeline = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(TransformNode::new(
            "b",
            a.clone(),
            b.clone(),
            |v| v + 1,
        )))
        .add_node(Box::new(ProducerNode::new("a", a.clone(), 10)))
        .build()
        .expect("build should succeed");

    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);

    let result = pipeline
        .bus()
        .read::<u32>(b)
        .expect("b output should exist");
    assert_eq!(result.value, 11);
}

#[test]
fn three_node_diamond_resolves() {
    // A → (B, C) → D, with D summing B and C's outputs.
    let a = ikey_named::<u32>("a");
    let b = ikey_named::<u32>("b");
    let c = ikey_named::<u32>("c");
    let d = ikey_named::<u32>("d");

    let pipeline = PipelineBuilder::new(AgentId::new(AGENT))
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

    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);

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
    let a_out = ikey_named::<ChA>("a_out");
    let b_out = ikey_named::<ChB>("b_out");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
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
    assert_eq!(errors.len(), 1, "expected one Cycle, got {errors:?}");
    assert_eq!(
        cycles(&errors),
        [(
            vec!["a", "b"],
            vec![edge("a", "b", &a_out), edge("b", "a", &b_out)]
        )]
    );
}

#[test]
fn self_loop_is_a_cycle_of_one() {
    // The node is its channel's only writer, so wiring passes, but it cannot
    // run before itself.
    let own = ikey_named::<ChA>("a");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(TransformNode::new(
            "echo",
            own.clone(),
            own.clone(),
            |v| v,
        )))
        .build();

    let Err(errors) = result else {
        panic!("should fail with cycle");
    };
    assert_eq!(errors.len(), 1, "expected one Cycle, got {errors:?}");
    assert_eq!(
        cycles(&errors),
        [(vec!["echo"], vec![edge("echo", "echo", &own)])]
    );
}

#[test]
fn node_downstream_of_a_cycle_is_left_out() {
    // `c` reads `a`'s output. It is stuck too, but it is not in the loop, so
    // it is neither a participant nor an edge endpoint.
    let a_out = ikey_named::<ChA>("a_out");
    let b_out = ikey_named::<ChB>("b_out");
    let c_out = ikey_named::<ChC>("c");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
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
        .add_node(Box::new(TransformNode::new(
            "c",
            a_out.clone(),
            c_out,
            |v| v,
        )))
        .build();

    let Err(errors) = result else {
        panic!("should fail with cycle");
    };
    assert_eq!(errors.len(), 1, "expected one Cycle, got {errors:?}");
    assert_eq!(
        cycles(&errors),
        [(
            vec!["a", "b"],
            vec![edge("a", "b", &a_out), edge("b", "a", &b_out)]
        )]
    );
}

#[test]
fn separate_cycles_give_one_error_each_sorted_by_name() {
    // Two loops, {a, b} and {c, d}, joined by `m`, which reads from the first
    // and feeds the second. The {c, d} loop is added first; errors still come
    // back in name order, and `m` is in neither.
    let a_out = ikey_named::<ChA>("a_out");
    let b_out = ikey_named::<ChA>("b_out");
    let c_out = ikey_named::<ChA>("c_out");
    let d_out = ikey_named::<ChA>("d_out");
    let m_out = ikey_named::<ChA>("m_out");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(TransformNode::new(
            "c",
            d_out.clone(),
            c_out.clone(),
            |v| v,
        )))
        .add_node(Box::new(JoinNode::new(
            "d",
            c_out.clone(),
            m_out.clone(),
            d_out.clone(),
        )))
        .add_node(Box::new(TransformNode::new(
            "m",
            a_out.clone(),
            m_out.clone(),
            |v| v,
        )))
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
        panic!("should fail with cycles");
    };
    assert_eq!(errors.len(), 2, "expected two Cycles, got {errors:?}");
    assert_eq!(
        cycles(&errors),
        [
            (
                vec!["a", "b"],
                vec![edge("a", "b", &a_out), edge("b", "a", &b_out)]
            ),
            (
                vec!["c", "d"],
                vec![edge("c", "d", &c_out), edge("d", "c", &d_out)]
            ),
        ]
    );
}

#[test]
fn cycle_error_does_not_depend_on_insertion_order() {
    // A three-node loop a → b → c → a, added in two different orders.
    let a_out = ikey_named::<ChA>("a_out");
    let b_out = ikey_named::<ChA>("b_out");
    let c_out = ikey_named::<ChA>("c_out");

    let nodes = [
        ("a", c_out.clone(), a_out.clone()),
        ("b", a_out.clone(), b_out.clone()),
        ("c", b_out.clone(), c_out.clone()),
    ];
    let build = |order: [usize; 3]| {
        let mut builder = PipelineBuilder::new(AgentId::new(AGENT));
        for i in order {
            let (name, input, output) = nodes[i].clone();
            builder = builder.add_node(Box::new(TransformNode::new(name, input, output, |v| v)));
        }
        let Err(errors) = builder.build() else {
            panic!("should fail with cycle");
        };
        cycles(&errors)
            .into_iter()
            .map(|(names, edges)| {
                let names: Vec<String> = names.into_iter().map(str::to_string).collect();
                (names, edges)
            })
            .collect::<Vec<_>>()
    };

    let expected = vec![(
        vec!["a".to_string(), "b".to_string(), "c".to_string()],
        vec![
            edge("a", "b", &a_out),
            edge("b", "c", &b_out),
            edge("c", "a", &c_out),
        ],
    )];
    assert_eq!(build([0, 1, 2]), expected);
    assert_eq!(build([2, 0, 1]), expected);
}

#[test]
fn cycle_message_names_each_read_in_the_loop() {
    let a_out = ikey_named::<ChA>("a_out");
    let b_out = ikey_named::<ChB>("b_out");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(TransformNode::new(
            "a",
            b_out.clone(),
            a_out.clone(),
            |v| v,
        )))
        .add_node(Box::new(TransformNode::new("b", a_out, b_out, |v| v)))
        .build();

    let Err(errors) = result else {
        panic!("should fail with cycle");
    };
    let message = errors[0].to_string();
    assert!(message.contains("[a, b]"), "got {message}");
    assert!(message.contains("\"b\" reads"), "got {message}");
    assert!(message.contains("from \"a\""), "got {message}");
    assert!(message.contains("\"a\" reads"), "got {message}");
    assert!(message.contains("from \"b\""), "got {message}");
}

#[test]
fn unsatisfied_input_detected() {
    let missing = ikey_named::<ChA>("missing");
    let out = ikey_named::<ChB>("b");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
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
            PipelineBuildError::UnsatisfiedInput { node_name, channel, need }
                if node_name == "needs_missing" && *channel == missing && *need == InputNeed::Required
        )),
        "expected UnsatisfiedInput for 'needs_missing'/{missing:?}, got {errors:?}"
    );
}

#[test]
fn multiple_suppliers_detected() {
    let shared = ikey_named::<ChA>("a");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(ProducerNode::new("a", shared.clone(), 1)))
        .add_node(Box::new(ProducerNode::new("b", shared.clone(), 2)))
        .build();

    let Err(errors) = result else {
        panic!("should fail with multiple suppliers");
    };
    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineBuildError::MultipleSuppliers { channel, first, second }
                if *channel == shared
                    && *first == Writer::Node { node: "a".to_string() }
                    && *second == Writer::Node { node: "b".to_string() }
        )),
        "expected MultipleSuppliers naming nodes a and b, got {errors:?}"
    );
}

#[test]
fn misspelled_optional_input_is_rejected() {
    // An optional input still needs a supplier: optional means the node runs
    // without a value, not that the wiring goes unchecked. A misspelled aiding
    // channel must fail the build instead of leaving the node silently unaided.
    let misspelled = ikey_named::<Sensor>("gps.primry");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(SinkNode::optional("ekf", misspelled.clone())))
        .build();

    let Err(errors) = result else {
        panic!("should fail: nothing supplies the optional input");
    };
    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineBuildError::UnsatisfiedInput { node_name, channel, need }
                if node_name == "ekf" && *channel == misspelled && *need == InputNeed::Optional
        )),
        "expected UnsatisfiedInput (optional) for 'ekf'/{misspelled:?}, got {errors:?}"
    );
}

#[test]
fn optional_input_satisfied_by_body_capability_builds() {
    let gps = ikey_named::<Sensor>("gps.primary");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(SinkNode::optional("ekf", gps.clone())))
        .with_body_capabilities(BodyCapabilities {
            publishes: vec![BodyChannel::new(gps, BODY_UNIT, Provenance::Exact)],
            ..Default::default()
        })
        .build();

    assert!(result.is_ok(), "build should succeed: {:?}", result.err());
}

#[test]
fn node_and_body_supplying_one_channel_is_rejected() {
    // The node-vs-node check alone would pass this: one channel written by
    // both a node and the body has two writers racing one slot.
    let shared = ikey_named::<ChA>("a");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(ProducerNode::new("a", shared.clone(), 1)))
        .with_body_capabilities(BodyCapabilities {
            publishes: vec![BodyChannel::new(
                shared.clone(),
                BODY_UNIT,
                Provenance::Exact,
            )],
            ..Default::default()
        })
        .build();

    let Err(errors) = result else {
        panic!("should fail: node and body both supply the channel");
    };
    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineBuildError::MultipleSuppliers { channel, first, second }
                if *channel == shared
                    && *first == Writer::Body {
            unit: BODY_UNIT.to_string(),
            provenance: Provenance::Exact,
        }
                    && *second == Writer::Node { node: "a".to_string() }
        )),
        "expected MultipleSuppliers naming the body then node a, got {errors:?}"
    );
}

#[test]
fn node_and_outside_input_supplying_one_channel_is_rejected() {
    let shared = ikey_named::<ChA>("a");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(ProducerNode::new("a", shared.clone(), 1)))
        .with_outside_inputs(vec![shared.clone()])
        .build();

    let Err(errors) = result else {
        panic!("should fail: node and outside input both supply the channel");
    };
    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineBuildError::MultipleSuppliers { channel, first, second }
                if *channel == shared
                    && *first == Writer::Outside
                    && *second == Writer::Node { node: "a".to_string() }
        )),
        "expected MultipleSuppliers naming the outside input then node a, got {errors:?}"
    );
}

#[test]
fn missing_input_is_not_also_reported_as_a_cycle() {
    // `downstream` waits on `upstream`, which waits on a channel nobody
    // supplies. The one real error is the missing input; the stranded
    // downstream node must not also show up as a cycle.
    let missing = ikey_named::<ChA>("missing");
    let mid = ikey_named::<ChB>("b");
    let out = ikey_named::<ChC>("c");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(TransformNode::new(
            "upstream",
            missing,
            mid.clone(),
            |v| v,
        )))
        .add_node(Box::new(TransformNode::new("downstream", mid, out, |v| v)))
        .build();

    let Err(errors) = result else {
        panic!("should fail with unsatisfied input");
    };
    assert_eq!(
        errors.len(),
        1,
        "expected only the missing input, got {errors:?}"
    );
    assert!(
        matches!(errors[0], PipelineBuildError::UnsatisfiedInput { .. }),
        "expected UnsatisfiedInput, got {errors:?}"
    );
}

#[test]
fn duplicate_node_name_is_rejected_once() {
    // Three nodes share one name. The within-level order is by name, so a
    // shared name would leave their order to insertion; it is reported once.
    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(ProducerNode::new(
            "dup",
            ikey_named::<ChA>("a"),
            0,
        )))
        .add_node(Box::new(ProducerNode::new(
            "dup",
            ikey_named::<ChB>("b"),
            0,
        )))
        .add_node(Box::new(ProducerNode::new(
            "dup",
            ikey_named::<ChC>("c"),
            0,
        )))
        .build();

    let Err(errors) = result else {
        panic!("should fail: node names repeat");
    };
    let duplicates: Vec<&str> = errors
        .iter()
        .filter_map(|e| match e {
            PipelineBuildError::DuplicateNodeName { name } => Some(name.as_str()),
            _ => None,
        })
        .collect();
    assert_eq!(duplicates, ["dup"], "got {errors:?}");
}

#[test]
fn empty_or_dotted_node_names_are_refused_once_each() {
    // A node name is one part of every watcher path under the node. Two nodes
    // sharing a dotted name give one malformed-name error (and one duplicate).
    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(ProducerNode::new(
            "a.b",
            ikey_named::<ChA>("a"),
            0,
        )))
        .add_node(Box::new(ProducerNode::new(
            "a.b",
            ikey_named::<ChB>("b"),
            0,
        )))
        .add_node(Box::new(ProducerNode::new("", ikey_named::<ChC>("c"), 0)))
        .add_node(Box::new(ProducerNode::new(
            "fine",
            ikey_named::<ChD>("d"),
            0,
        )))
        .build();

    let Err(errors) = result else {
        panic!("should fail: two node names are not path segments");
    };
    let malformed: Vec<&str> = errors
        .iter()
        .filter_map(|e| match e {
            PipelineBuildError::MalformedNodeName { name } => Some(name.as_str()),
            _ => None,
        })
        .collect();
    assert_eq!(malformed, ["a.b", ""], "got {errors:?}");
}

#[test]
fn malformed_channel_names_are_refused_once_each() {
    // A slash, an empty level and an unnamed channel. The slashed channel's
    // reader is not also reported as unsupplied.
    let slashed = ikey_named::<ChA>("gps/fix");
    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(ProducerNode::new("gps", slashed.clone(), 0)))
        .add_node(Box::new(SinkNode::new("reader", slashed)))
        .add_node(Box::new(ProducerNode::new(
            "trailing",
            ikey_named::<ChB>("gps."),
            0,
        )))
        .add_node(Box::new(ProducerNode::new(
            "unnamed",
            ChannelKey::internal_of::<ChC>(),
            0,
        )))
        .build();

    let Err(errors) = result else {
        panic!("should fail: three channel names are malformed");
    };
    let malformed: Vec<&str> = errors
        .iter()
        .filter_map(|e| match e {
            PipelineBuildError::MalformedChannelName { name, .. } => Some(name.as_str()),
            _ => None,
        })
        .collect();
    assert_eq!(malformed, ["", "gps.", "gps/fix"], "got {errors:?}");
    assert_eq!(errors.len(), 3, "nothing else reported: {errors:?}");
}

#[test]
fn body_channel_with_a_malformed_name_names_the_body_and_its_reader() {
    let truth = okey_named::<Truth>("oracle/pose");
    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(SinkNode::new("mock_oracle", truth.clone())))
        .with_body_capabilities(BodyCapabilities {
            publishes: vec![BodyChannel::new(
                truth.clone(),
                BODY_UNIT,
                Provenance::Exact,
            )],
            ..Default::default()
        })
        .build();

    let Err(errors) = result else {
        panic!("should fail: the body's channel name holds a '/'");
    };
    let expected = [ChannelUses {
        key: truth,
        writers: vec![Writer::Body {
            unit: BODY_UNIT.to_string(),
            provenance: Provenance::Exact,
        }],
        readers: vec!["mock_oracle".to_string()],
    }];
    assert!(
        matches!(
            errors.as_slice(),
            [PipelineBuildError::MalformedChannelName { name, uses }]
                if name == "oracle/pose" && *uses == expected
        ),
        "got {errors:?}"
    );
}

#[test]
fn reader_and_writer_disagreeing_on_type_are_reported_together() {
    // One error showing both channels, instead of "nothing supplies" the
    // reader's.
    let written = ikey_named::<ChA>("estimate");
    let read = ikey_named::<ChB>("estimate");
    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(ProducerNode::new("ekf", written.clone(), 0)))
        .add_node(Box::new(SinkNode::new("controller", read.clone())))
        .build();

    let Err(errors) = result else {
        panic!("should fail: one name, two types");
    };
    let expected = [
        ChannelUses {
            key: written,
            writers: vec![Writer::Node {
                node: "ekf".to_string(),
            }],
            readers: vec![],
        },
        ChannelUses {
            key: read,
            writers: vec![],
            readers: vec!["controller".to_string()],
        },
    ];
    assert!(
        matches!(
            errors.as_slice(),
            [PipelineBuildError::ChannelNameReused { name, uses }]
                if name == "estimate" && *uses == expected
        ),
        "got {errors:?}"
    );
    let message = errors[0].to_string();
    assert!(
        message.contains("written by node \"ekf\"")
            && message.contains("read by node \"controller\""),
        "{message}"
    );
}

#[test]
fn one_name_across_kinds_is_refused() {
    // The body's measurement channel and a node's internal channel share a name
    // and a type; the kind still makes them two channels.
    let sensor = ChannelKey::measurement::<ChA>("imu");
    let internal = ikey_named::<ChA>("imu");
    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(ProducerNode::new("relay", internal, 0)))
        .with_body_capabilities(BodyCapabilities {
            publishes: vec![BodyChannel::new(sensor, BODY_UNIT, Provenance::Exact)],
            ..Default::default()
        })
        .build();

    let Err(errors) = result else {
        panic!("should fail: one name, two kinds");
    };
    assert!(
        matches!(
            errors.as_slice(),
            [PipelineBuildError::ChannelNameReused { name, uses }]
                if name == "imu" && uses.len() == 2
        ),
        "got {errors:?}"
    );
}

#[test]
fn level_order_does_not_depend_on_insertion_order() {
    // Three independent producers share level 0. Within a level nodes are
    // sorted by name, so either insertion order yields the same order (and
    // the same node ids).
    let build = |names: [&str; 3]| {
        let keys = [
            ikey_named::<ChA>("a"),
            ikey_named::<ChB>("b"),
            ikey_named::<ChC>("c"),
        ];
        let mut builder = PipelineBuilder::new(AgentId::new(AGENT));
        for (name, key) in names.into_iter().zip(keys) {
            builder = builder.add_node(Box::new(ProducerNode::new(name, key, 0)));
        }
        let pipeline = builder.build().expect("build should succeed");
        pipeline
            .channels()
            .map(|(name, _)| name.to_string())
            .collect::<Vec<_>>()
    };

    assert_eq!(build(["c", "a", "b"]), ["a", "b", "c"]);
    assert_eq!(build(["b", "c", "a"]), ["a", "b", "c"]);
}

#[test]
fn optional_input_producer_runs_first() {
    // The reader's name sorts before the producer's, so if the optional edge
    // did not order them, both would share level 0 and the reader would run
    // first, see nothing, and write nothing on the first tick.
    let source = ikey_named::<u32>("a");
    let out = ikey_named::<u32>("b");

    let pipeline = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(ProducerNode::new("b_producer", source.clone(), 7)))
        .add_node(Box::new(TransformNode::optional(
            "a_reader",
            source,
            out.clone(),
            |v| v + 1,
        )))
        .build()
        .expect("build should succeed");

    let order: Vec<&str> = pipeline.channels().map(|(name, _)| name).collect();
    assert_eq!(order, ["b_producer", "a_reader"]);

    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);
    let result = pipeline
        .bus()
        .read::<u32>(out)
        .expect("reader should see this tick's value on the first tick");
    assert_eq!(result.value, 8);
}

#[test]
fn cycle_through_optional_input_is_detected() {
    // `a` requires `b`'s output; `b` reads `a`'s output only optionally. The
    // optional edge is still same-tick, so the two wait on each other.
    let a_out = ikey_named::<ChA>("a_out");
    let b_out = ikey_named::<ChB>("b_out");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(TransformNode::new(
            "a",
            b_out.clone(),
            a_out.clone(),
            |v| v,
        )))
        .add_node(Box::new(TransformNode::optional("b", a_out, b_out, |v| v)))
        .build();

    let Err(errors) = result else {
        panic!("should fail with cycle");
    };
    assert!(
        errors
            .iter()
            .any(|e| matches!(e, PipelineBuildError::Cycle { .. })),
        "expected Cycle, got {errors:?}"
    );
}

#[test]
fn body_capabilities_satisfy_required_inputs() {
    // A node requires a channel that no node produces. Declaring it as a
    // body-published channel via with_body_capabilities must make the build
    // succeed.
    let sensor_key = ikey_named::<Sensor>("imu");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(SinkNode::new("consumer", sensor_key.clone())))
        .with_body_capabilities(BodyCapabilities {
            publishes: vec![BodyChannel::new(sensor_key, BODY_UNIT, Provenance::Exact)],
            ..Default::default()
        })
        .build();

    assert!(result.is_ok(), "build should succeed: {:?}", result.err());
}

#[test]
fn malformed_unit_is_reported_once_for_all_its_channels() {
    // Errors name a body channel's writer by its unit, so a unit follows the
    // node-name grammar. An IMU's two channels share one unit; a bad name is
    // one mistake, reported once.
    let accel = ikey_named::<Sensor>("imu.accel");
    let gyro = ikey_named::<Truth>("imu.gyro");
    let unit = "imu.front";

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(SinkNode::new("consumer", accel.clone())))
        .with_body_capabilities(BodyCapabilities {
            publishes: vec![
                BodyChannel::new(accel, unit, Provenance::Simulated),
                BodyChannel::new(gyro, unit, Provenance::Simulated),
            ],
            ..Default::default()
        })
        .build();

    let Err(errors) = result else {
        panic!("should fail: the unit name holds a '.'");
    };
    assert!(
        matches!(
            errors.as_slice(),
            [PipelineBuildError::MalformedUnitName { unit: reported }] if reported == unit
        ),
        "expected one MalformedUnitName for {unit:?}, got {errors:?}"
    );
}

#[test]
fn oracle_input_satisfied_by_body_capability() {
    // A mock-style node declares an Oracle channel as a required input.
    // When the body advertises it, the build succeeds.
    let oracle_key = okey_named::<Truth>("oracle.pose");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(SinkNode::new("mock_oracle", oracle_key.clone())))
        .with_body_capabilities(BodyCapabilities {
            publishes: vec![BodyChannel::new(oracle_key, BODY_UNIT, Provenance::Exact)],
            ..Default::default()
        })
        .build();

    assert!(result.is_ok(), "build should succeed: {:?}", result.err());
}

#[test]
fn oracle_input_without_body_capability_errors() {
    // Same node, but the body does not advertise the oracle channel. The
    // build must fail with UnsatisfiedBodyCapabilities naming the node —
    // not the generic UnsatisfiedInput. Oracle channels are never
    // produced by a node, so the only possible supplier is the body.
    let oracle_key = okey_named::<Truth>("oracle.pose");

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(SinkNode::new("mock_oracle", oracle_key.clone())))
        .with_body_capabilities(BodyCapabilities {
            publishes: vec![],
            ..Default::default()
        })
        .build();

    let Err(errors) = result else {
        panic!("should fail: body does not advertise the oracle channel");
    };
    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineBuildError::UnsatisfiedBodyCapabilities { node_name, channel_key, .. }
                if node_name == "mock_oracle" && *channel_key == oracle_key
        )),
        "expected UnsatisfiedBodyCapabilities for 'mock_oracle'/{oracle_key:?}, got {errors:?}"
    );
}

// =========================================================================
// == Rate gating ==
// =========================================================================

#[test]
fn rate_gated_node_fires_at_correct_interval() {
    // Period = 1 / 2 Hz = 0.5s. dt = 0.1s → fires on the 5th tick only.
    let counter = Arc::new(AtomicU32::new(0));
    let pipeline = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(CountingNode::new(
            "rated",
            Some(2.0),
            counter.clone(),
        )))
        .build()
        .expect("build should succeed");

    for _ in 0..4 {
        pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);
    }
    assert_eq!(
        counter.load(Ordering::Relaxed),
        0,
        "must not fire before 0.5s elapsed"
    );

    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);
    assert_eq!(counter.load(Ordering::Relaxed), 1, "must fire on 5th tick");
}

#[test]
fn rate_gated_node_does_not_double_fire() {
    // dt = 1.0s with period = 0.5s would cover two periods; node must
    // still fire exactly once (no catch-up).
    let counter = Arc::new(AtomicU32::new(0));
    let pipeline = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(CountingNode::new(
            "rated",
            Some(2.0),
            counter.clone(),
        )))
        .build()
        .expect("build should succeed");

    pipeline.tick(MonotonicTime(0.0), 1.0, &MockRuntime);
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
    let pipeline = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(CountingNode::new(
            "every_tick",
            None,
            counter.clone(),
        )))
        .build()
        .expect("build should succeed");

    for _ in 0..3 {
        pipeline.tick(MonotonicTime(0.0), 0.01, &MockRuntime);
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
    let sensor_key = ikey_named::<u32>("imu");
    let pipeline = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(SinkNode::new("consumer", sensor_key.clone())))
        .with_body_capabilities(BodyCapabilities {
            publishes: vec![BodyChannel::new(
                sensor_key.clone(),
                BODY_UNIT,
                Provenance::Exact,
            )],
            ..Default::default()
        })
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

    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);

    let after = pipeline
        .bus()
        .read::<u32>(sensor_key)
        .expect("value should still be present after tick");
    assert_eq!(after.value, 99);
}

// =========================================================================
// == Keyed channels ==
// =========================================================================

/// Declares its ports and writes nothing: these tests only build.
struct DeclaringNode {
    name: String,
    descriptor: PortDescriptor,
}

impl PipelineNode for DeclaringNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, _bus: &PortBus, _tf: &dyn helios_core::prelude::TfProvider, _: TickContext) {}
}

/// A node writing the edges of `children` on the transform tree's channel.
fn edge_writer(name: &str, children: &[FrameId]) -> Box<dyn PipelineNode> {
    Box::new(DeclaringNode {
        name: name.to_string(),
        descriptor: PortDescriptorBuilder::algorithm()
            .output_keyed::<StampedTransform>(tf(), children.to_vec())
            .build(),
    })
}

fn bot_frame(leaf: &str) -> FrameId {
    FrameId::agent(AgentId::new(AGENT), leaf)
}

/// A relay owning `base_link`'s edge and a localizer owning `odom`'s share
/// the one `tf` channel: a keyed channel has a writer per instance, not one
/// supplier.
#[test]
fn writers_of_distinct_tf_instances_share_the_channel() {
    let agent = AgentId::new(AGENT);
    PipelineBuilder::new(agent.clone())
        .add_node(edge_writer("relay", &[FrameId::base_link(agent.clone())]))
        .add_node(edge_writer("localizer", &[FrameId::odom(agent)]))
        .build()
        .expect("each instance has one writer");
}

#[test]
fn two_writers_of_one_tf_instance_are_refused() {
    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(edge_writer(
            "arm",
            &[bot_frame("wrist"), bot_frame("elbow")],
        ))
        .add_node(edge_writer("gripper", &[bot_frame("wrist")]))
        .build();
    let Err(errors) = result else {
        panic!("wrist has two writers");
    };

    let wrist = InstanceKey::of::<StampedTransform>(&bot_frame("wrist"));
    assert!(
        matches!(
            errors.as_slice(),
            [PipelineBuildError::InstanceWrittenTwice { channel, instance, first, second }]
                if *channel == tf()
                    && *instance == wrist
                    && *first == Writer::Node { node: "arm".to_string() }
                    && *second == Writer::Node { node: "gripper".to_string() }
        ),
        "{errors:?}"
    );
}

/// A keyed channel declared as a plain output names no instances, so every
/// write would be refused; the build says so instead.
#[test]
fn a_keyed_output_without_instances_is_refused() {
    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(Box::new(DeclaringNode {
            name: "relay".to_string(),
            descriptor: PortDescriptorBuilder::algorithm().output(tf()).build(),
        }))
        .build();
    let Err(errors) = result else {
        panic!("the relay names no instances");
    };

    assert!(
        matches!(
            errors.as_slice(),
            [PipelineBuildError::KeyedWithoutInstances { channel, .. }] if *channel == tf()
        ),
        "{errors:?}"
    );
}

/// A loop through each of two writers of one keyed channel reports both
/// writers' reads, not only one of them: the fuser reads `tf`, which the
/// localizer and the relay each write from a channel the fuser writes.
#[test]
fn a_loop_through_each_writer_of_a_keyed_channel_names_both() {
    let b = ikey_named::<u32>("b");
    let c = ikey_named::<u32>("c");
    let declaring = |name: &str, descriptor: PortDescriptor| -> Box<dyn PipelineNode> {
        Box::new(DeclaringNode {
            name: name.to_string(),
            descriptor,
        })
    };

    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_node(declaring(
            "fuser",
            PortDescriptorBuilder::algorithm()
                .input(tf())
                .output(b.clone())
                .output(c.clone())
                .build(),
        ))
        .add_node(declaring(
            "localizer",
            PortDescriptorBuilder::algorithm()
                .input(b.clone())
                .output_keyed::<StampedTransform>(tf(), [bot_frame("odom")])
                .build(),
        ))
        .add_node(declaring(
            "relay",
            PortDescriptorBuilder::algorithm()
                .input(c.clone())
                .output_keyed::<StampedTransform>(tf(), [bot_frame("base_link")])
                .build(),
        ))
        .build();
    let Err(errors) = result else {
        panic!("the fuser reads what it feeds");
    };

    assert_eq!(
        cycles(&errors),
        [(
            vec!["fuser", "localizer", "relay"],
            vec![
                edge("fuser", "localizer", &b),
                edge("fuser", "relay", &c),
                edge("localizer", "fuser", &tf()),
                edge("relay", "fuser", &tf()),
            ]
        )]
    );
}

// =========================================================================
// == Planned nodes ==
// =========================================================================

/// A plan for a [`ProducerNode`]. The node keeps the descriptor it is handed.
fn producer_plan(name: &str, output: ChannelKey, value: u32) -> NodePlan {
    let descriptor = ProducerNode::new(name, output.clone(), value).descriptor;
    let name = name.to_string();
    NodePlan::new(
        name.clone(),
        descriptor,
        Box::new(move |descriptor, _| {
            let mut node = ProducerNode::new(&name, output, value);
            node.descriptor = descriptor;
            Ok(Box::new(node) as Box<dyn PipelineNode>)
        }),
    )
}

/// A plan for a [`TransformNode`]. The node keeps the descriptor it is
/// handed.
fn transform_plan(
    name: &str,
    input: ChannelKey,
    output: ChannelKey,
    transform: fn(u32) -> u32,
) -> NodePlan {
    let descriptor = TransformNode::new(name, input.clone(), output.clone(), transform).descriptor;
    let name = name.to_string();
    NodePlan::new(
        name.clone(),
        descriptor,
        Box::new(move |descriptor, _| {
            let mut node = TransformNode::new(&name, input, output, transform);
            node.descriptor = descriptor;
            Ok(Box::new(node) as Box<dyn PipelineNode>)
        }),
    )
}

/// A plan named `name` declaring `descriptor`, built as a node that does
/// nothing and keeps the descriptor.
fn idle_plan(name: &str, descriptor: PortDescriptor) -> NodePlan {
    let built_name = name.to_string();
    NodePlan::new(
        name,
        descriptor,
        Box::new(move |descriptor, _| {
            let mut node = ProducerNode::new(&built_name, ikey_named::<u32>("unused"), 0);
            node.descriptor = descriptor;
            Ok(Box::new(node) as Box<dyn PipelineNode>)
        }),
    )
}

/// A plan reading a body channel's mount and one resolved through another
/// node's output is built with both: the body's stated mount, and for the
/// derived channel the mount of the input its writer's rule names.
#[test]
fn a_plan_is_built_with_the_mounts_its_inputs_resolve_to() {
    struct Scan;
    let agent = AgentId::new(AGENT);
    let lidar_mount = FrameId::agent(agent.clone(), "lidar");
    let range = ChannelKey::measurement::<Scan>("lidar");
    let cloud = ChannelKey::measurement::<Scan>("lidar.cloud");

    let flatten = idle_plan(
        "flatten",
        PortDescriptorBuilder::algorithm()
            .input(range.clone())
            .output_at_mount_of(cloud.clone(), range.clone())
            .build(),
    );
    let seen: Arc<Mutex<Vec<Result<FrameId, MountLookupError>>>> = Arc::default();
    let record = Arc::clone(&seen);
    let (range_key, lookup) = (range.clone(), cloud.clone());
    let reader = NodePlan::new(
        "reader",
        PortDescriptorBuilder::algorithm()
            .input(cloud.clone())
            .build(),
        Box::new(move |descriptor, mounts| {
            let mut record = record.lock().expect("not poisoned");
            record.push(mounts.mount(&lookup).cloned());
            record.push(mounts.mount(&range_key).cloned());
            let mut node = ProducerNode::new("reader", ikey_named::<u32>("reader_out"), 0);
            node.descriptor = descriptor;
            Ok(Box::new(node) as Box<dyn PipelineNode>)
        }),
    );
    let body = BodyCapabilities {
        publishes: vec![
            BodyChannel::new(range.clone(), BODY_UNIT, Provenance::Simulated)
                .with_mount(lidar_mount.clone()),
        ],
        ..Default::default()
    };

    let pipeline = PipelineBuilder::new(agent)
        .add_plan(reader)
        .add_plan(flatten)
        .with_body_capabilities(body)
        .build()
        .expect("build should succeed");

    // The reader reads only the cloud, so the range's mount is not its to see.
    assert_eq!(
        *seen.lock().expect("not poisoned"),
        [
            Ok(lidar_mount.clone()),
            Err(MountLookupError::NotRead {
                reader: "reader".to_string(),
                channel: range,
            }),
        ]
    );
    let cloud_description = pipeline
        .descriptions()
        .iter()
        .find(|description| description.key() == &cloud)
        .expect("the cloud is described");
    assert_eq!(cloud_description.mount(), Some(&lidar_mount));
}

/// The pipeline lists the mounts of the channels its nodes read, each once,
/// with every convention a node reads it in: two readers of one channel and
/// two channels on one mount give one entry, a reader that never resolves the
/// mount adds no convention, and a mounted channel no node reads gives none.
#[test]
fn read_mounts_lists_each_read_mount_once_with_the_axes_it_is_read_in() {
    let agent = AgentId::new(AGENT);
    let imu_mount = FrameId::agent(agent.clone(), "imu");
    let accel = ChannelKey::measurement::<Sensor>("imu.accel");
    let gyro = ChannelKey::measurement::<Truth>("imu.gyro");
    let unread = ChannelKey::measurement::<ChA>("mag");
    let body = BodyCapabilities {
        publishes: vec![
            BodyChannel::new(accel.clone(), BODY_UNIT, Provenance::Simulated)
                .with_mount(imu_mount.clone()),
            BodyChannel::new(gyro.clone(), BODY_UNIT, Provenance::Simulated)
                .with_mount(imu_mount.clone()),
            BodyChannel::new(unread, BODY_UNIT, Provenance::Simulated)
                .with_mount(FrameId::agent(agent.clone(), "mag")),
        ],
        ..Default::default()
    };

    let pipeline = PipelineBuilder::new(agent)
        .add_node(Box::new(SinkNode::reading_mount_in(
            "first",
            accel.clone(),
            Convention::Flu,
        )))
        .add_node(Box::new(SinkNode::new("second", accel)))
        .add_node(Box::new(SinkNode::reading_mount_in(
            "third",
            gyro.clone(),
            Convention::Enu,
        )))
        .add_node(Box::new(SinkNode::reading_mount_in(
            "fourth",
            gyro,
            Convention::Flu,
        )))
        .with_body_capabilities(body)
        .build()
        .expect("build should succeed");

    assert_eq!(
        pipeline.read_mounts(),
        vec![ReadMount {
            frame: imu_mount,
            read_in: vec![Convention::Flu, Convention::Enu],
        }]
    );
}

/// A body channel mounted on another agent's frame is refused at build.
#[test]
fn a_body_mount_on_another_agent_is_refused() {
    let body = BodyCapabilities {
        publishes: vec![BodyChannel::new(
            ChannelKey::measurement::<u32>("gps"),
            BODY_UNIT,
            Provenance::Simulated,
        )
        .with_mount(FrameId::agent(AgentId::new("other"), "gps"))],
        ..Default::default()
    };

    let Err(errors) = PipelineBuilder::new(AgentId::new(AGENT))
        .with_body_capabilities(body)
        .build()
    else {
        panic!("expected the mount to be refused");
    };
    assert!(
        matches!(
            errors.as_slice(),
            [PipelineBuildError::RefusedBodyMount {
                refusal: MountRefusal::OtherAgent,
                ..
            }]
        ),
        "{errors:?}"
    );
}

/// A plan for a node named `name` writing `output`, whose constructor fails
/// with `reason` and records that it ran.
fn failing_plan(
    name: &str,
    output: ChannelKey,
    reason: &'static str,
    ran: Arc<AtomicBool>,
) -> NodePlan {
    let descriptor = ProducerNode::new(name, output, 0).descriptor;
    NodePlan::new(
        name,
        descriptor,
        Box::new(move |_, _| {
            ran.store(true, Ordering::Relaxed);
            Err(reason.into())
        }),
    )
}

/// Plans are wired and ordered on their declared ports like built nodes, then
/// built and run in that order.
#[test]
fn plans_are_ordered_on_their_declared_ports_then_built() {
    let a = ikey_named::<u32>("a");
    let b = ikey_named::<u32>("b");

    let pipeline = PipelineBuilder::new(AgentId::new(AGENT))
        .add_plan(transform_plan("b", a.clone(), b.clone(), |v| v * 3))
        .add_plan(producer_plan("a", a, 5))
        .build()
        .expect("build should succeed");

    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);

    let result = pipeline
        .bus()
        .read::<u32>(b)
        .expect("b output should exist");
    assert_eq!(result.value, 15);
}

/// Every failed construction is reported, each with its constructor's own
/// message.
#[test]
fn every_failed_construction_is_reported() {
    let ran = Arc::new(AtomicBool::new(false));
    let errors = match PipelineBuilder::new(AgentId::new(AGENT))
        .add_plan(failing_plan(
            "a",
            ikey_named::<u32>("a"),
            "no a",
            Arc::clone(&ran),
        ))
        .add_plan(failing_plan(
            "b",
            ikey_named::<u32>("b"),
            "no b",
            Arc::clone(&ran),
        ))
        .build()
    {
        Ok(_) => panic!("expected both constructions to fail"),
        Err(errors) => errors,
    };

    let failed: Vec<(&str, String)> = errors
        .iter()
        .map(|error| match error {
            PipelineBuildError::ConstructionFailed { node_name, .. } => {
                (node_name.as_str(), error.to_string())
            }
            other => panic!("expected ConstructionFailed, got {other}"),
        })
        .collect();
    assert_eq!(
        failed,
        [("a", "no a".to_string()), ("b", "no b".to_string())]
    );
}

/// A plan is built only once wiring passes, so a constructor never runs on a
/// graph that is refused.
#[test]
fn a_plan_is_not_built_when_wiring_fails() {
    let ran = Arc::new(AtomicBool::new(false));
    let result = PipelineBuilder::new(AgentId::new(AGENT))
        .add_plan(failing_plan(
            "a",
            ikey_named::<u32>("a"),
            "no a",
            Arc::clone(&ran),
        ))
        .add_node(Box::new(TransformNode::new(
            "b",
            ikey_named::<u32>("missing"),
            ikey_named::<u32>("b"),
            |v| v,
        )))
        .build();

    let Err(errors) = result else {
        panic!("expected the missing input to be refused");
    };
    assert!(
        matches!(
            errors.as_slice(),
            [PipelineBuildError::UnsatisfiedInput { .. }]
        ),
        "{errors:?}"
    );
    assert!(!ran.load(Ordering::Relaxed), "the constructor ran");
}

/// The plan's name is the one wiring checked, so a node built under another
/// is refused.
#[test]
fn a_plan_built_under_another_name_is_refused() {
    let output = ikey_named::<u32>("a");
    let descriptor = ProducerNode::new("planned_as", output.clone(), 0).descriptor;
    let plan = NodePlan::new(
        "planned_as",
        descriptor,
        Box::new(move |_, _| {
            Ok(Box::new(ProducerNode::new("built_as", output, 0)) as Box<dyn PipelineNode>)
        }),
    );

    let Err(errors) = PipelineBuilder::new(AgentId::new(AGENT))
        .add_plan(plan)
        .build()
    else {
        panic!("expected the misnamed node to be refused");
    };
    assert!(
        matches!(
            errors.as_slice(),
            [PipelineBuildError::ConstructedNodeMisnamed { node_name, built_name }]
                if node_name == "planned_as" && built_name == "built_as"
        ),
        "{errors:?}"
    );
}
