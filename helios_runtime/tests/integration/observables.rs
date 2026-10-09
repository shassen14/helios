// Build checks on watchable leaves: each declared leaf is a well-formed path
// outside the pipeline's own group, and within a node each leaf is distinct
// and differs from every output channel's path segment.

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

/// Distinct output types, so nodes in one pipeline never supply the same
/// channel.
struct ChA;
struct ChB;
struct Estimate;

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
                |builder, leaf| builder.observable(*leaf, Determinism::Reproducible),
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

fn named<T: 'static>(instance: &'static str) -> ChannelKey {
    InternalChannel::named::<T>(instance).into()
}

fn unnamed<T: 'static>() -> ChannelKey {
    InternalChannel::of::<T>().into()
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

#[test]
fn leaf_declared_twice_is_rejected_once() {
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[unnamed::<ChA>()],
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
        &[unnamed::<ChA>()],
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
        &[unnamed::<ChA>()],
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
        &[unnamed::<ChA>()],
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
        &[unnamed::<ChA>()],
        &leaves,
    )]);

    let expected: Vec<(&str, &str)> = bad.iter().map(|leaf| ("ekf", *leaf)).collect();
    assert_eq!(malformed(&errors), expected, "got {errors:?}");
    assert_eq!(errors.len(), bad.len(), "nothing else reported: {errors:?}");
}

#[test]
fn output_at_the_tick_duration_path_collides() {
    // The pipeline's own leaves are checked against outputs too.
    let output = named::<ChA>("tick/duration");
    let errors = build_errors(vec![DeclaringNode::new("ekf", &[output.clone()], &[])]);

    assert_eq!(
        collisions(&errors),
        [("ekf", TICK_DURATION_LEAF, &output)],
        "got {errors:?}"
    );
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
fn leaf_equal_to_an_unnamed_outputs_type_segment_is_rejected() {
    let output = unnamed::<Estimate>();
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[output.clone()],
        &["estimate"],
    )]);

    assert_eq!(
        collisions(&errors),
        [("ekf", "estimate", &output)],
        "got {errors:?}"
    );
}

#[test]
fn leaf_equal_to_an_output_with_slashes_is_rejected() {
    // `/` in an instance becomes `.` in the path, so `est/cov` sits at
    // `est.cov`, the same path as the leaf.
    let output = named::<ChA>("est/cov");
    let errors = build_errors(vec![DeclaringNode::new(
        "ekf",
        &[output.clone()],
        &["est.cov"],
    )]);

    assert_eq!(
        collisions(&errors),
        [("ekf", "est.cov", &output)],
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
        &[unnamed::<ChA>()],
        &["nis", "nis.mean"],
    )]);

    assert!(errors.is_empty(), "a shared prefix is a group: {errors:?}");
}

#[test]
fn same_leaf_on_two_nodes_builds() {
    let errors = build_errors(vec![
        DeclaringNode::new("ekf", &[unnamed::<ChA>()], &["nis"]),
        DeclaringNode::new("ukf", &[unnamed::<ChB>()], &["nis"]),
    ]);

    assert!(errors.is_empty(), "paths include the node: {errors:?}");
}

#[test]
fn leaf_equal_to_another_nodes_output_builds() {
    let errors = build_errors(vec![
        DeclaringNode::new("ekf", &[named::<ChA>("nis")], &[]),
        DeclaringNode::new("monitor", &[unnamed::<ChB>()], &["nis"]),
    ]);

    assert!(errors.is_empty(), "the check is per node: {errors:?}");
}
