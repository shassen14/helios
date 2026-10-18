// Build checks on watcher paths: each declared leaf is a well-formed path
// outside the pipeline's own group, and within a node the leaves are
// distinct. Output channels are recorded under their own names, not their
// node's, so no leaf check involves them.

use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::AgentId;
use helios_runtime::{
    pipeline::{
        autonomy_pipeline::{PIPELINE_LEAF_GROUP, TICK_DURATION_LEAF},
        PipelineBuildError, PipelineBuilder,
    },
    port::{ChannelKey, Determinism, PortBus, PortDescriptor, PortDescriptorBuilder},
    prelude::{PipelineNode, TickContext},
};

use std::sync::Arc;

/// The agent every pipeline here serves.
const AGENT: &str = "car";

/// Distinct output types, so nodes in one pipeline never supply the same
/// channel.
struct ChA;
struct ChB;

/// Writes nothing; exists only to carry a descriptor with outputs and leaves.
struct DeclaringNode {
    name: String,
    descriptor: PortDescriptor,
}

impl DeclaringNode {
    fn new(name: &str, outputs: &[ChannelKey], leaves: &[&str]) -> Self {
        let descriptor = leaves
            .iter()
            .fold(
                PortDescriptorBuilder::algorithm().outputs_from_slice(outputs),
                |builder, leaf| {
                    builder.observable(*leaf, Determinism::Deterministic, SiUnit::Dimensionless)
                },
            )
            .build();
        Self {
            name: name.to_string(),
            descriptor,
        }
    }
}

impl PipelineNode for DeclaringNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, _: &PortBus, _: &dyn helios_core::prelude::TfProvider, _: TickContext) {}
}

fn named<T: 'static>(instance: impl Into<Arc<str>>) -> ChannelKey {
    ChannelKey::internal::<T>(instance)
}

fn build_errors(nodes: Vec<DeclaringNode>) -> Vec<PipelineBuildError> {
    let builder = nodes.into_iter().fold(
        PipelineBuilder::new(AgentId::new(AGENT)),
        |builder, node| builder.add_node(Box::new(node)),
    );
    match builder.build() {
        Ok(_) => vec![],
        Err(errors) => errors,
    }
}

/// `(node, leaf)` of every duplicate-leaf error.
fn duplicates(errors: &[PipelineBuildError]) -> Vec<(&str, &str)> {
    errors
        .iter()
        .filter_map(|e| match e {
            PipelineBuildError::DuplicateObservable { node_name, leaf } => {
                Some((node_name.as_str(), leaf.as_str()))
            }
            _ => None,
        })
        .collect()
}

/// `(node, leaf)` of every malformed-leaf error.
fn malformed(errors: &[PipelineBuildError]) -> Vec<(&str, &str)> {
    errors
        .iter()
        .filter_map(|e| match e {
            PipelineBuildError::MalformedObservable { node_name, leaf } => {
                Some((node_name.as_str(), leaf.as_str()))
            }
            _ => None,
        })
        .collect()
}

/// `(node, leaf)` of every reserved-group error.
fn reserved(errors: &[PipelineBuildError]) -> Vec<(&str, &str)> {
    errors
        .iter()
        .filter_map(|e| match e {
            PipelineBuildError::ReservedObservable { node_name, leaf } => {
                Some((node_name.as_str(), leaf.as_str()))
            }
            _ => None,
        })
        .collect()
}

/// The name of every reused-channel-name error.
fn reused_names(errors: &[PipelineBuildError]) -> Vec<&str> {
    errors
        .iter()
        .filter_map(|e| match e {
            PipelineBuildError::ChannelNameReused { name, .. } => Some(name.as_str()),
            _ => None,
        })
        .collect()
}

#[test]
fn leaf_declared_twice_is_rejected_once() {
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[named::<ChA>("a")],
        &["nis", "nis", "nis"],
    )]);

    assert_eq!(duplicates(&errors), [("ekf", "nis")], "got {errors:?}");
}

#[test]
fn node_declaring_the_tick_duration_leaf_is_rejected_as_reserved() {
    // Refused for its group only: not also reported as a duplicate of the
    // leaf the pipeline adds.
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[named::<ChA>("a")],
        &[TICK_DURATION_LEAF],
    )]);

    assert_eq!(
        reserved(&errors),
        [("ekf", TICK_DURATION_LEAF)],
        "got {errors:?}"
    );
    assert_eq!(errors.len(), 1, "got {errors:?}");
}

#[test]
fn leaves_in_the_pipeline_group_are_rejected() {
    let overrun = format!("{PIPELINE_LEAF_GROUP}.overrun");
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[named::<ChA>("a")],
        &[PIPELINE_LEAF_GROUP, &overrun],
    )]);

    assert_eq!(
        reserved(&errors),
        [("ekf", PIPELINE_LEAF_GROUP), ("ekf", overrun.as_str())],
        "got {errors:?}"
    );
}

#[test]
fn leaf_only_starting_with_the_group_name_builds() {
    let ticker = format!("{PIPELINE_LEAF_GROUP}er.count");
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[named::<ChA>("a")],
        &[&ticker],
    )]);

    assert!(
        errors.is_empty(),
        "only the group itself is reserved: {errors:?}"
    );
}

#[test]
fn malformed_leaves_are_rejected_once_each() {
    let bad = ["", ".nis", "nis.", "aiding..nis", "aiding/gps.nis"];
    let leaves: Vec<&str> = bad.iter().chain(bad.iter()).copied().collect();
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[named::<ChA>("a")],
        &leaves,
    )]);

    let expected: Vec<(&str, &str)> = bad.iter().map(|leaf| ("ekf", *leaf)).collect();
    assert_eq!(malformed(&errors), expected, "got {errors:?}");
    assert_eq!(errors.len(), bad.len(), "nothing else reported: {errors:?}");
}

#[test]
fn outputs_named_in_the_pipeline_group_build() {
    // A channel is recorded under its own name, never under its node, so it
    // can't collide with the leaves the pipeline adds to every node.
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[
            named::<ChA>(TICK_DURATION_LEAF),
            named::<ChB>(PIPELINE_LEAF_GROUP),
        ],
        &[],
    )]);

    assert!(errors.is_empty(), "got {errors:?}");
}

#[test]
fn output_under_a_malformed_name_is_reported_once() {
    // The channel-name check refuses the name, and nothing else reports it.
    let output = named::<ChA>("est/cov");
    let errors = build_errors(vec![DeclaringNode::new("ekf", &[output], &["est.cov"])]);

    assert!(
        matches!(
            errors.as_slice(),
            [PipelineBuildError::MalformedChannelName { name, .. }] if name == "est/cov"
        ),
        "got {errors:?}"
    );
}

#[test]
fn leaf_at_a_reused_output_name_reports_only_the_reuse() {
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[named::<ChA>("est.cov"), named::<ChB>("est.cov")],
        &["est.cov"],
    )]);

    assert_eq!(reused_names(&errors), ["est.cov"], "got {errors:?}");
    assert_eq!(errors.len(), 1, "got {errors:?}");
}

#[test]
fn one_output_name_on_two_nodes_is_rejected() {
    // Output paths include the node, but a channel name is the channel's
    // address on the whole agent.
    let errors = build_errors(vec![
        DeclaringNode::new("ekf", &[named::<ChA>("estimate")], &[]),
        DeclaringNode::new("ukf", &[named::<ChB>("estimate")], &[]),
    ]);

    assert_eq!(reused_names(&errors), ["estimate"], "got {errors:?}");
}

#[test]
fn leaf_equal_to_its_nodes_output_builds() {
    // The leaf is an emission of the node and the output a channel: two
    // subjects, spelled under `node` and `channel`.
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[named::<ChA>("nis")],
        &["nis"],
    )]);

    assert!(errors.is_empty(), "got {errors:?}");
}

#[test]
fn leaves_grouped_under_an_output_build() {
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[named::<ChA>("estimate")],
        &["estimate.cov", "estimate.nis"],
    )]);

    assert!(errors.is_empty(), "a shared prefix is a group: {errors:?}");
}

#[test]
fn leaf_beside_a_longer_leaf_builds() {
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[named::<ChA>("a")],
        &["nis", "nis.mean"],
    )]);

    assert!(errors.is_empty(), "a shared prefix is a group: {errors:?}");
}

#[test]
fn same_leaf_on_two_nodes_builds() {
    let errors = build_errors(vec![
        DeclaringNode::new("ekf", &[named::<ChA>("a")], &["nis"]),
        DeclaringNode::new("ukf", &[named::<ChB>("b")], &["nis"]),
    ]);

    assert!(errors.is_empty(), "paths include the node: {errors:?}");
}

#[test]
fn leaf_equal_to_another_nodes_output_builds() {
    let errors = build_errors(vec![
        DeclaringNode::new("ekf", &[named::<ChA>("nis")], &[]),
        DeclaringNode::new("monitor", &[named::<ChB>("b")], &["nis"]),
    ]);

    assert!(errors.is_empty(), "the check is per node: {errors:?}");
}
