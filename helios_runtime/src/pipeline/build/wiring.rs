//! Wiring checks: node names are unique, each node's watchable leaves are
//! well formed, outside the pipeline's own group, distinct and clear of its
//! output channels' paths, and every input has exactly one supplier. These need only the declarations, not an order, so
//! they run before the sort.

use crate::{
    observe::path::channel_to_path_segment,
    pipeline::autonomy_pipeline::{observable_catalog, PIPELINE_LEAF_GROUP},
    port::ChannelKind,
    BodyCapabilities, ChannelKey, PipelineBuildError, PipelineNode, Supplier,
};

use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

/// Checks the graph's wiring and returns every problem found. An empty list
/// means each node name is unique, every watchable leaf a node declares is
/// well formed and outside the pipeline's own group, no node names a leaf
/// twice or gives one the path of its own output channel, and every input,
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
/// shared name is reported once per name.
fn check_node_names(nodes: &[Box<dyn PipelineNode>], errors: &mut Vec<PipelineBuildError>) {
    let mut seen_names: HashSet<&str> = HashSet::new();
    let mut reported_names: HashSet<&str> = HashSet::new();
    for node in nodes {
        let name = node.name();
        if !seen_names.insert(name) && reported_names.insert(name) {
            errors.push(PipelineBuildError::DuplicateNodeName {
                name: name.to_string(),
            });
        }
    }
}

/// A watcher addresses a node's leaves and its output channels by one dotted
/// path under that node. Each leaf the node declares must be a well-formed
/// path and stay out of the pipeline's own group. Then, across the node's
/// whole catalog, each leaf must be distinct and differ from every output's
/// path segment. Checked per node: paths include the node name, so two nodes
/// never clash. Only exact matches are refused; a shared dotted prefix is a
/// group. Each problem with a leaf is reported once, and a leaf refused for
/// its form or group is not checked further.
fn check_observables(nodes: &[Box<dyn PipelineNode>], errors: &mut Vec<PipelineBuildError>) {
    for node in nodes {
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

        let outputs: Vec<(String, &ChannelKey)> = node
            .port_descriptor()
            .outputs()
            .iter()
            .map(|key| (channel_to_path_segment(key), key))
            .collect();

        let mut seen_leaves: HashSet<Arc<str>> = HashSet::new();
        let mut reported_leaves: HashSet<Arc<str>> = HashSet::new();
        for observable in observable_catalog(node.as_ref()) {
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
            for (segment, channel) in &outputs {
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
}

/// A leaf is one or more non-empty parts joined by `.`, with no `/`. An empty
/// part (`""`, `.nis`, `nis.`, `aiding..nis`) would mis-split in a glob or a
/// tree-shaped sink, and a `/` would read as a level that channel paths turn
/// into `.` but a leaf would keep.
fn is_well_formed_leaf(leaf: &str) -> bool {
    !leaf.contains('/') && leaf.split('.').all(|part| !part.is_empty())
}

/// The pipeline's group itself or any leaf under it (`tick`, `tick.overrun`),
/// but not a leaf that only starts with the same letters (`ticker`).
fn is_in_pipeline_group(leaf: &str) -> bool {
    match leaf.strip_prefix(PIPELINE_LEAF_GROUP) {
        Some(rest) => rest.is_empty() || rest.starts_with('.'),
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
