// Build checks on watcher paths: each declared leaf is a well-formed path,
// leaves and output channels sit outside the pipeline's own group, within a
// node the leaves are distinct and no leaf has an output's name, and output
// names that the channel-name check refused are not reported again here.

use helios_runtime::{
    pipeline::{
        autonomy_pipeline::{PIPELINE_LEAF_GROUP, TICK_DURATION_LEAF},
        PipelineBuildError, PipelineBuilder,
    },
    port::{
        AlgorithmNodePortDescriptor, ChannelKey, Determinism, InternalChannel, PortBus,
        PortDescriptor,
    },
    prelude::{PipelineNode, TickContext},
};

use std::sync::Arc;

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
                AlgorithmNodePortDescriptor::new().outputs_from_slice(outputs),
                |builder, leaf| builder.observable(*leaf, Determinism::Deterministic),
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
    InternalChannel::named::<T>(instance).into()
}

fn build_errors(nodes: Vec<DeclaringNode>) -> Vec<PipelineBuildError> {
    let builder = nodes
        .into_iter()
        .fold(PipelineBuilder::new(), |builder, node| {
            builder.add_node(Box::new(node))
        });
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

/// `(node, leaf, channel)` of every leaf/output collision error.
fn collisions(errors: &[PipelineBuildError]) -> Vec<(&str, &str, &ChannelKey)> {
    errors
        .iter()
        .filter_map(|e| match e {
            PipelineBuildError::ObservableCollidesWithOutput {
                node_name,
                leaf,
                channel,
            } => Some((node_name.as_str(), leaf.as_str(), channel)),
            _ => None,
        })
        .collect()
}

/// `(node, channel, segment)` of every reserved-output-path error.
fn reserved_outputs(errors: &[PipelineBuildError]) -> Vec<(&str, &ChannelKey, &str)> {
    errors
        .iter()
        .filter_map(|e| match e {
            PipelineBuildError::ReservedOutputPath {
                node_name,
                channel,
                segment,
            } => Some((node_name.as_str(), channel, segment.as_str())),
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
fn output_at_the_tick_duration_path_is_rejected_as_reserved() {
    // Refused for its group only: not also reported as colliding with the
    // leaf the pipeline adds.
    let output = named::<ChA>("tick.duration");
    let errors = build_errors(vec![DeclaringNode::new("ekf", &[output.clone()], &[])]);

    assert_eq!(
        reserved_outputs(&errors),
        [("ekf", &output, TICK_DURATION_LEAF)],
        "got {errors:?}"
    );
    assert_eq!(errors.len(), 1, "got {errors:?}");
}

#[test]
fn outputs_in_the_pipeline_group_are_rejected() {
    let group = named::<ChA>(PIPELINE_LEAF_GROUP);
    let overrun = named::<ChB>(format!("{PIPELINE_LEAF_GROUP}.overrun"));
    let overrun_path = format!("{PIPELINE_LEAF_GROUP}.overrun");
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[group.clone(), overrun.clone()],
        &[],
    )]);

    assert_eq!(
        reserved_outputs(&errors),
        [
            ("ekf", &group, PIPELINE_LEAF_GROUP),
            ("ekf", &overrun, overrun_path.as_str())
        ],
        "got {errors:?}"
    );
}

#[test]
fn output_under_a_malformed_name_is_reported_once() {
    // The channel-name check refuses the name; the path checks skip it.
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
fn leaf_at_a_reused_output_name_is_not_compared() {
    // Two outputs under one name are refused by the channel-name check, so
    // the leaf is not also reported as colliding with each.
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[named::<ChA>("est.cov"), named::<ChB>("est.cov")],
        &["est.cov"],
    )]);

    assert_eq!(reused_names(&errors), ["est.cov"], "got {errors:?}");
    assert!(collisions(&errors).is_empty(), "got {errors:?}");
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
fn leaf_equal_to_a_named_output_is_rejected() {
    let output = named::<ChA>("nis");
    let errors = build_errors(vec![DeclaringNode::new("ekf", &[output.clone()], &["nis"])]);

    assert_eq!(
        collisions(&errors),
        [("ekf", "nis", &output)],
        "got {errors:?}"
    );
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
