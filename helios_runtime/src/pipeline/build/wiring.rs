//! Wiring checks: node names are unique, and every input has exactly one
//! supplier. These need only the declarations, not an order, so they run
//! before the sort.

use crate::{
    port::ChannelKind, BodyCapabilities, ChannelKey, PipelineBuildError, PipelineNode, Supplier,
};

use std::collections::{HashMap, HashSet};

/// Checks the graph's wiring and returns every problem found. An empty list
/// means each node name is unique and every input, required or optional, has
/// exactly one supplier: a node output, a body channel, or a declared outside
/// input.
pub(super) fn check_wiring(
    nodes: &[Box<dyn PipelineNode>],
    capabilities: &BodyCapabilities,
    outside_inputs: &[ChannelKey],
) -> Vec<PipelineBuildError> {
    let mut errors = vec![];
    check_node_names(nodes, &mut errors);
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
