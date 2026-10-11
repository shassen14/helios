//! Wiring checks: node names are unique path segments, each node's watchable
//! leaves and output channels' paths are well formed, outside the pipeline's
//! own group and distinct, and every input has exactly one supplier. These
//! need only the declarations, not an order, so they run before the sort.

use crate::{
    observe::path::{channel_to_path_segment, is_path_segment, PATH_SEPARATOR},
    pipeline::autonomy_pipeline::PIPELINE_LEAF_GROUP,
    port::ChannelKind,
    BodyCapabilities, ChannelKey, PipelineBuildError, PipelineNode, Supplier,
};

use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

/// Checks the graph's wiring and returns every problem found. An empty list
/// means each node name is unique and fits one part of a watcher path, every
/// watchable leaf a node declares and every output channel's path is well
/// formed and outside the pipeline's own group, no node names a leaf twice,
/// puts two outputs at one path, or gives a leaf the path of one of its
/// outputs, and every input,
/// required or optional, has exactly one supplier: a node output, a body
/// channel, or a declared outside input.
pub(super) fn check_wiring(
    nodes: &[Box<dyn PipelineNode>],
    capabilities: &BodyCapabilities,
    outside_inputs: &[ChannelKey],
) -> Vec<PipelineBuildError> {
    let mut errors = vec![];
    check_node_names(nodes, &mut errors);
    check_observables(nodes, &mut errors);
    let supplier_of = map_suppliers(nodes, capabilities, outside_inputs, &mut errors);
    check_inputs_supplied(nodes, &supplier_of, &capabilities.name, &mut errors);
    errors
}

/// Errors, logs and the within-level order all identify a node by name, so a
/// shared name is reported once per name. A node's name is also one part of
/// every watcher path under it, so an empty or dotted name is reported, once
/// per name.
fn check_node_names(nodes: &[Box<dyn PipelineNode>], errors: &mut Vec<PipelineBuildError>) {
    let mut seen_names: HashSet<&str> = HashSet::new();
    let mut reported_names: HashSet<&str> = HashSet::new();
    let mut malformed_names: HashSet<&str> = HashSet::new();
    for node in nodes {
        let name = node.name();
        if !is_path_segment(name) && malformed_names.insert(name) {
            errors.push(PipelineBuildError::MalformedNodeName {
                name: name.to_string(),
            });
        }
        if !seen_names.insert(name) && reported_names.insert(name) {
            errors.push(PipelineBuildError::DuplicateNodeName {
                name: name.to_string(),
            });
        }
    }
}

/// A watcher addresses a node's leaves and its output channels by one dotted
/// path under that node, so each path must name exactly one value. Each
/// source of paths is checked for its own form, then against the others:
/// declared leaves and output paths must be well formed and stay out of the
/// pipeline's own group, output paths must differ from each other, and
/// declared leaves must differ from each other and from every output path.
/// The pipeline's own leaves need no comparison, since nothing else may sit
/// in their group. Checked per node: paths include the node name, so two
/// nodes never clash. Only exact matches are refused; a shared dotted prefix
/// is a group.
fn check_observables(nodes: &[Box<dyn PipelineNode>], errors: &mut Vec<PipelineBuildError>) {
    for node in nodes {
        let refused_leaves = check_declared_leaves(node.as_ref(), errors);
        let output_paths = check_output_paths(node.as_ref(), errors);
        check_leaves_distinct(node.as_ref(), &refused_leaves, &output_paths, errors);
    }
}

/// Each leaf the node declares must be a well-formed path outside the
/// pipeline's own group. Returns the refused leaves, each reported once, so
/// the later checks skip them.
fn check_declared_leaves(
    node: &dyn PipelineNode,
    errors: &mut Vec<PipelineBuildError>,
) -> HashSet<Arc<str>> {
    let node_name = node.name();
    let mut refused_leaves: HashSet<Arc<str>> = HashSet::new();
    for observable in node.port_descriptor().observables() {
        let leaf = observable.leaf_name();
        let error = if !is_well_formed_leaf(leaf) {
            PipelineBuildError::MalformedObservable {
                node_name: node_name.to_string(),
                leaf: leaf.to_string(),
            }
        } else if is_in_pipeline_group(leaf) {
            PipelineBuildError::ReservedObservable {
                node_name: node_name.to_string(),
                leaf: leaf.to_string(),
            }
        } else {
            continue;
        };
        if refused_leaves.insert(leaf.clone()) {
            errors.push(error);
        }
    }
    refused_leaves
}

/// Each output channel is watched at its path segment, which comes from the
/// channel's name in config, so it gets the same checks as a declared leaf:
/// well formed and outside the pipeline's own group. Two outputs at one path
/// are refused, each later one against the first. Returns the accepted paths
/// with their channels.
///
/// A channel listed twice is skipped after its first listing: it is one
/// channel with one path, and the supplier check reports the repeat.
fn check_output_paths<'a>(
    node: &'a dyn PipelineNode,
    errors: &mut Vec<PipelineBuildError>,
) -> Vec<(String, &'a ChannelKey)> {
    let node_name = node.name();
    let mut seen_channels: HashSet<&ChannelKey> = HashSet::new();
    let mut accepted: Vec<(String, &ChannelKey)> = vec![];
    for channel in node.port_descriptor().outputs() {
        if !seen_channels.insert(channel) {
            continue;
        }
        let segment = channel_to_path_segment(channel);
        let error = if !is_well_formed_leaf(&segment) {
            PipelineBuildError::MalformedOutputPath {
                node_name: node_name.to_string(),
                channel: channel.clone(),
                segment,
            }
        } else if is_in_pipeline_group(&segment) {
            PipelineBuildError::ReservedOutputPath {
                node_name: node_name.to_string(),
                channel: channel.clone(),
                segment,
            }
        } else if let Some((_, first)) = accepted.iter().find(|(path, _)| *path == segment) {
            PipelineBuildError::DuplicateOutputPath {
                node_name: node_name.to_string(),
                segment,
                first: (*first).clone(),
                second: channel.clone(),
            }
        } else {
            accepted.push((segment, channel));
            continue;
        };
        errors.push(error);
    }
    accepted
}

/// Each declared leaf the earlier check accepted must be named once and
/// differ from every accepted output path. A duplicate is reported once per
/// leaf.
fn check_leaves_distinct(
    node: &dyn PipelineNode,
    refused_leaves: &HashSet<Arc<str>>,
    output_paths: &[(String, &ChannelKey)],
    errors: &mut Vec<PipelineBuildError>,
) {
    let node_name = node.name();
    let mut seen_leaves: HashSet<Arc<str>> = HashSet::new();
    let mut reported_leaves: HashSet<Arc<str>> = HashSet::new();
    for observable in node.port_descriptor().observables() {
        let leaf = observable.leaf_name();
        if refused_leaves.contains(leaf) {
            continue;
        }
        if !seen_leaves.insert(leaf.clone()) {
            if reported_leaves.insert(leaf.clone()) {
                errors.push(PipelineBuildError::DuplicateObservable {
                    node_name: node_name.to_string(),
                    leaf: leaf.to_string(),
                });
            }
            continue;
        }
        for (segment, channel) in output_paths {
            if segment.as_str() == leaf.as_ref() {
                errors.push(PipelineBuildError::ObservableCollidesWithOutput {
                    node_name: node_name.to_string(),
                    leaf: leaf.to_string(),
                    channel: (*channel).clone(),
                });
            }
        }
    }
}

/// A leaf is one or more non-empty parts joined by `.`, with no `/`. An empty
/// part (`""`, `.nis`, `nis.`, `aiding..nis`) would mis-split in a glob or a
/// tree-shaped sink, and a `/` would read as a level that channel paths turn
/// into `.` but a leaf would keep.
fn is_well_formed_leaf(leaf: &str) -> bool {
    !leaf.contains('/') && leaf.split(PATH_SEPARATOR).all(|part| !part.is_empty())
}

/// The pipeline's group itself or any leaf under it (`tick`, `tick.overrun`),
/// but not a leaf that only starts with the same letters (`ticker`).
fn is_in_pipeline_group(leaf: &str) -> bool {
    match leaf.strip_prefix(PIPELINE_LEAF_GROUP) {
        Some(rest) => rest.is_empty() || rest.starts_with(PATH_SEPARATOR),
        None => false,
    }
}

/// Records who supplies each channel. The body and the outside inputs go in
/// first, so a conflict names the outside-world supplier before the node. Any
/// second supplier of a key is an error.
fn map_suppliers(
    nodes: &[Box<dyn PipelineNode>],
    capabilities: &BodyCapabilities,
    outside_inputs: &[ChannelKey],
    errors: &mut Vec<PipelineBuildError>,
) -> HashMap<ChannelKey, Supplier> {
    let mut supplier_of: HashMap<ChannelKey, Supplier> = HashMap::new();
    let body_supplies = capabilities.publishes.iter().map(|published| {
        (
            published.key.clone(),
            Supplier::Body(capabilities.name.clone()),
        )
    });
    let outside_supplies = outside_inputs
        .iter()
        .map(|key| (key.clone(), Supplier::OutsideInput));
    let node_supplies = nodes.iter().flat_map(|node| {
        node.port_descriptor()
            .outputs()
            .iter()
            .map(|output| (output.clone(), Supplier::Node(node.name().to_string())))
    });
    for (channel, supplier) in body_supplies.chain(outside_supplies).chain(node_supplies) {
        if let Some(first) = supplier_of.get(&channel) {
            errors.push(PipelineBuildError::MultipleSuppliers {
                channel,
                first: first.clone(),
                second: supplier,
            });
        } else {
            supplier_of.insert(channel, supplier);
        }
    }
    supplier_of
}

/// Every input, required or optional, needs a supplier. Optional only means
/// the node runs without a value.
fn check_inputs_supplied(
    nodes: &[Box<dyn PipelineNode>],
    supplier_of: &HashMap<ChannelKey, Supplier>,
    body_name: &str,
    errors: &mut Vec<PipelineBuildError>,
) {
    for node in nodes {
        for input in node.port_descriptor().inputs() {
            let channel = input.channel();
            if supplier_of.contains_key(channel) {
                continue;
            }
            let error = match channel.kind() {
                ChannelKind::Sensor | ChannelKind::Internal => {
                    PipelineBuildError::UnsatisfiedInput {
                        node_name: node.name().to_string(),
                        channel: channel.clone(),
                        need: input.need(),
                    }
                }
                // Only a body supplies oracle and health channels, so a
                // missing one is the body's gap and the error names it.
                ChannelKind::Health | ChannelKind::Oracle => {
                    PipelineBuildError::UnsatisfiedBodyCapabilities {
                        node_name: node.name().to_string(),
                        channel_key: channel.clone(),
                        body: body_name.to_string(),
                        need: input.need(),
                    }
                }
            };
            errors.push(error);
        }
    }
}
