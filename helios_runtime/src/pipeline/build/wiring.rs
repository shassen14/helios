//! Wiring checks: node names are unique levels, channel names are well
//! formed and each names one channel, each node's watchable leaves are well
//! formed, and leaves and output channels sit outside the pipeline's own
//! group and apart, and every input has exactly one supplier. These need only
//! the declarations, not an order, so they run before the sort.

use crate::{
    channels::tf::is_tf_edge, pipeline::autonomy_pipeline::PIPELINE_LEAF_GROUP, port::ChannelKind,
    BodyCapabilities, ChannelKey, ChannelUses, PipelineBuildError, PipelineNode, Supplier,
};

use helios_core::kernel::identifier::{is_dotted_name, is_level, LEVEL_SEPARATOR};

use std::{
    collections::{BTreeMap, HashMap, HashSet},
    sync::Arc,
};

/// Checks the graph's wiring and returns every problem found. An empty list
/// means each node name is unique and fits one part of a watcher path, every
/// channel name is well formed and names one channel, every watchable leaf a
/// node declares is well formed, no leaf or output channel sits in the
/// pipeline's own group, no node names a leaf twice or gives a leaf the name
/// of one of its outputs, and every input, required or optional, has exactly
/// one supplier: a node output, a body channel, or a declared outside input.
pub(super) fn check_wiring(
    nodes: &[Box<dyn PipelineNode>],
    capabilities: &BodyCapabilities,
    outside_inputs: &[ChannelKey],
) -> Vec<PipelineBuildError> {
    let mut errors = vec![];
    check_node_names(nodes, &mut errors);
    let refused_names = check_channel_names(nodes, capabilities, outside_inputs, &mut errors);
    check_observables(nodes, &refused_names, &mut errors);
    let supplier_of = map_suppliers(nodes, capabilities, outside_inputs, &mut errors);
    check_inputs_supplied(
        nodes,
        &supplier_of,
        &refused_names,
        &capabilities.name,
        &mut errors,
    );
    errors
}

/// Errors, logs and the within-level order all identify a node by name, so a
/// shared name is reported once per name. A node's name is also one part of
/// every watcher path under it, so a name that isn't one level is reported,
/// once per name.
fn check_node_names(nodes: &[Box<dyn PipelineNode>], errors: &mut Vec<PipelineBuildError>) {
    let mut seen_names: HashSet<&str> = HashSet::new();
    let mut reported_names: HashSet<&str> = HashSet::new();
    let mut malformed_names: HashSet<&str> = HashSet::new();
    for node in nodes {
        let name = node.name();
        if !is_level(name) && malformed_names.insert(name) {
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

/// A channel's name is its one address on the agent, so every name the graph
/// sees must be well formed and name one channel: one kind and one payload
/// type. Covers the body's channels, the outside inputs, and every node's
/// inputs and outputs. Each refused name is reported once, with every channel
/// under it and who writes and reads each, so a reader and a writer that
/// disagree on the type are shown side by side. Returns the refused names,
/// so the supplier check doesn't report their inputs again.
///
/// Transform-edge channels keep their own spelling until the transform tree
/// gets a channel of its own, so their names are not checked for form.
fn check_channel_names(
    nodes: &[Box<dyn PipelineNode>],
    capabilities: &BodyCapabilities,
    outside_inputs: &[ChannelKey],
    errors: &mut Vec<PipelineBuildError>,
) -> HashSet<Arc<str>> {
    let uses_by_name = map_channel_uses(nodes, capabilities, outside_inputs);
    let mut refused_names: HashSet<Arc<str>> = HashSet::new();
    for (name, uses) in uses_by_name {
        let error = if !is_dotted_name(&name) && !uses.iter().all(|used| is_tf_edge(&used.key)) {
            PipelineBuildError::MalformedChannelName {
                name: name.to_string(),
                uses,
            }
        } else if uses.len() > 1 {
            PipelineBuildError::ChannelNameReused {
                name: name.to_string(),
                uses,
            }
        } else {
            continue;
        };
        refused_names.insert(name);
        errors.push(error);
    }
    refused_names
}

/// Every channel the graph sees, grouped by name, each with who writes and
/// reads it. Names sort so errors come out in one order every build; the
/// channels under a name keep the order the build met them.
fn map_channel_uses(
    nodes: &[Box<dyn PipelineNode>],
    capabilities: &BodyCapabilities,
    outside_inputs: &[ChannelKey],
) -> BTreeMap<Arc<str>, Vec<ChannelUses>> {
    let mut uses_by_name: BTreeMap<Arc<str>, Vec<ChannelUses>> = BTreeMap::new();
    for published in &capabilities.publishes {
        push_once(
            &mut uses_of(&mut uses_by_name, &published.key).writers,
            Supplier::Body(capabilities.name.clone()),
        );
    }
    for key in outside_inputs {
        push_once(
            &mut uses_of(&mut uses_by_name, key).writers,
            Supplier::OutsideInput,
        );
    }
    for node in nodes {
        let descriptor = node.port_descriptor();
        for output in descriptor.outputs() {
            push_once(
                &mut uses_of(&mut uses_by_name, output).writers,
                Supplier::Node(node.name().to_string()),
            );
        }
        for input in descriptor.inputs() {
            push_once(
                &mut uses_of(&mut uses_by_name, input.channel()).readers,
                node.name().to_string(),
            );
        }
    }
    uses_by_name
}

/// The entry for `key` under its name, added empty the first time `key` is
/// met.
fn uses_of<'a>(
    uses_by_name: &'a mut BTreeMap<Arc<str>, Vec<ChannelUses>>,
    key: &ChannelKey,
) -> &'a mut ChannelUses {
    let uses = uses_by_name.entry(key.instance().clone()).or_default();
    let index = match uses.iter().position(|used| used.key == *key) {
        Some(index) => index,
        None => {
            uses.push(ChannelUses {
                key: key.clone(),
                writers: vec![],
                readers: vec![],
            });
            uses.len() - 1
        }
    };
    &mut uses[index]
}

/// Adds `item` unless `list` already holds it, so a channel listed twice by
/// one node names that node once.
fn push_once<T: PartialEq>(list: &mut Vec<T>, item: T) {
    if !list.contains(&item) {
        list.push(item);
    }
}

/// A watcher addresses a node's leaves and its output channels by one dotted
/// path under that node, so each path must name exactly one value. Each
/// source of paths is checked for its own form, then against the others:
/// declared leaves must be well formed, leaves and output names must stay out
/// of the pipeline's own group, and declared leaves must differ from each
/// other and from every output name. Output names are checked for form and
/// against each other with every channel name, so an output under a refused
/// name is skipped here.
/// The pipeline's own leaves need no comparison, since nothing else may sit
/// in their group. Checked per node: paths include the node name, so two
/// nodes never clash. Only exact matches are refused; a shared dotted prefix
/// is a group.
fn check_observables(
    nodes: &[Box<dyn PipelineNode>],
    refused_names: &HashSet<Arc<str>>,
    errors: &mut Vec<PipelineBuildError>,
) {
    for node in nodes {
        let refused_leaves = check_declared_leaves(node.as_ref(), errors);
        let output_paths = check_output_paths(node.as_ref(), refused_names, errors);
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
        let error = if !is_dotted_name(leaf) {
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

/// Each output channel is also watched under its node, at its name, so it
/// must stay outside the pipeline's own group like a declared leaf. Returns
/// the accepted names with their channels.
///
/// A channel listed twice is skipped after its first listing: it is one
/// channel with one name, and the supplier check reports the repeat. A
/// channel under a refused name is skipped too, since its error is already
/// reported.
fn check_output_paths<'a>(
    node: &'a dyn PipelineNode,
    refused_names: &HashSet<Arc<str>>,
    errors: &mut Vec<PipelineBuildError>,
) -> Vec<(&'a Arc<str>, &'a ChannelKey)> {
    let node_name = node.name();
    let mut seen_channels: HashSet<&ChannelKey> = HashSet::new();
    let mut accepted: Vec<(&Arc<str>, &ChannelKey)> = vec![];
    for channel in node.port_descriptor().outputs() {
        let name = channel.instance();
        if !seen_channels.insert(channel) || refused_names.contains(name) {
            continue;
        }
        if is_in_pipeline_group(name) {
            errors.push(PipelineBuildError::ReservedOutputPath {
                node_name: node_name.to_string(),
                channel: channel.clone(),
                segment: name.to_string(),
            });
        } else {
            accepted.push((name, channel));
        }
    }
    accepted
}

/// Each declared leaf the earlier check accepted must be named once and
/// differ from every accepted output name. A duplicate is reported once per
/// leaf.
fn check_leaves_distinct(
    node: &dyn PipelineNode,
    refused_leaves: &HashSet<Arc<str>>,
    output_paths: &[(&Arc<str>, &ChannelKey)],
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
        for (name, channel) in output_paths {
            if *name == leaf {
                errors.push(PipelineBuildError::ObservableCollidesWithOutput {
                    node_name: node_name.to_string(),
                    leaf: leaf.to_string(),
                    channel: (*channel).clone(),
                });
            }
        }
    }
}

/// The pipeline's group itself or any leaf under it (`tick`, `tick.overrun`),
/// but not a leaf that only starts with the same letters (`ticker`).
fn is_in_pipeline_group(leaf: &str) -> bool {
    match leaf.strip_prefix(PIPELINE_LEAF_GROUP) {
        Some(rest) => rest.is_empty() || rest.starts_with(LEVEL_SEPARATOR),
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
/// the node runs without a value. An input under a name the channel-name
/// check refused is skipped: that error already lists who writes and reads it.
fn check_inputs_supplied(
    nodes: &[Box<dyn PipelineNode>],
    supplier_of: &HashMap<ChannelKey, Supplier>,
    refused_names: &HashSet<Arc<str>>,
    body_name: &str,
    errors: &mut Vec<PipelineBuildError>,
) {
    for node in nodes {
        for input in node.port_descriptor().inputs() {
            let channel = input.channel();
            if supplier_of.contains_key(channel) || refused_names.contains(channel.instance()) {
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
