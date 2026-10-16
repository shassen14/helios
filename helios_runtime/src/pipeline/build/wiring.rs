//! Wiring checks: node names are unique levels, the body's unit names are
//! levels, the body's mounts are its own frames, every frame has one owner,
//! output mount rules name inputs, each node reads and writes only the channel
//! kinds its class may, channel names are well formed and each names one
//! channel, each node's watchable leaves are well formed, distinct and
//! outside the pipeline's own group, every input has exactly one supplier, and every instance of a keyed channel has
//! one writer. These need only the declarations, not an order, so they run
//! before the sort.

use super::declaration::NodeDeclaration;
use super::instances::check_instances;
use super::kinds::check_kind_access;
use super::mounts::{check_body_mounts, check_mount_rules};
use super::owners::check_frame_owners;

use crate::{
    pipeline::autonomy_pipeline::PIPELINE_LEAF_GROUP,
    port::{ChannelDescription, ChannelKind, Writer},
    ChannelKey, ChannelUses, PipelineBuildError,
};

use helios_core::kernel::identifier::{is_dotted_name, is_level, LEVEL_SEPARATOR};
use helios_core::prelude::AgentId;

use std::{
    collections::{BTreeMap, HashMap, HashSet},
    sync::Arc,
};

/// Checks the graph's wiring and returns every problem found. An empty list
/// means each node name is unique and fits one part of a watcher path, every
/// unit the body declares is one level, every mount the body states is a frame
/// on `agent`'s body other than `base_link`, every frame the body or a node
/// owns has one owner, every output mounted at an input's mount names a
/// channel its node reads in the same tick, every node reads and writes only
/// the channel kinds its class may, every channel name is well formed and
/// names one channel, every watchable leaf a node declares is well formed and
/// outside the pipeline's own group, no node names a leaf twice, and every
/// input,
/// required or optional, has exactly one supplier: a node output, a body
/// channel, or a declared outside input, and every writer of a keyed channel
/// declares instances no other writer does. `descriptions` holds every
/// channel some writer declares.
pub(super) fn check_wiring(
    agent: &AgentId,
    nodes: &[NodeDeclaration],
    descriptions: &[ChannelDescription],
) -> Vec<PipelineBuildError> {
    let mut errors = vec![];
    check_node_names(nodes, &mut errors);
    check_unit_names(descriptions, &mut errors);
    check_body_mounts(agent, descriptions, &mut errors);
    check_frame_owners(agent, nodes, descriptions, &mut errors);
    check_mount_rules(nodes, &mut errors);
    check_kind_access(nodes, &mut errors);
    let refused_names = check_channel_names(nodes, descriptions, &mut errors);
    check_observables(nodes, &mut errors);
    let supplier_of = map_suppliers(descriptions, &mut errors);
    check_instances(descriptions, &mut errors);
    check_inputs_supplied(nodes, &supplier_of, &refused_names, &mut errors);
    errors
}

/// Errors, logs and the within-level order all identify a node by name, so a
/// shared name is reported once per name. A node's name is also one part of
/// every watcher path under it, so a name that isn't one level is reported,
/// once per name.
fn check_node_names(nodes: &[NodeDeclaration], errors: &mut Vec<PipelineBuildError>) {
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

/// Errors name a body channel's writer by its unit, so each unit must be one
/// level, as a node name is. Several channels share a unit, so a malformed
/// one is reported once.
fn check_unit_names(descriptions: &[ChannelDescription], errors: &mut Vec<PipelineBuildError>) {
    let mut malformed_units: HashSet<&str> = HashSet::new();
    for description in descriptions {
        if let Writer::Body { unit, .. } = description.writer() {
            if !is_level(unit) && malformed_units.insert(unit) {
                errors.push(PipelineBuildError::MalformedUnitName { unit: unit.clone() });
            }
        }
    }
}

/// A channel's name is its one address on the agent, so every name the graph
/// sees must be well formed and name one channel: one kind and one payload
/// type. Covers every described channel and every node's inputs. Each refused name is reported once, with every channel
/// under it and who writes and reads each, so a reader and a writer that
/// disagree on the type are shown side by side. Returns the refused names,
/// so the supplier check doesn't report their inputs again.
fn check_channel_names(
    nodes: &[NodeDeclaration],
    descriptions: &[ChannelDescription],
    errors: &mut Vec<PipelineBuildError>,
) -> HashSet<Arc<str>> {
    let uses_by_name = map_channel_uses(nodes, descriptions);
    let mut refused_names: HashSet<Arc<str>> = HashSet::new();
    for (name, uses) in uses_by_name {
        let error = if !is_dotted_name(&name) {
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
/// channels under a name keep the order the build met them: described
/// channels first, then channels only read.
fn map_channel_uses(
    nodes: &[NodeDeclaration],
    descriptions: &[ChannelDescription],
) -> BTreeMap<Arc<str>, Vec<ChannelUses>> {
    let mut uses_by_name: BTreeMap<Arc<str>, Vec<ChannelUses>> = BTreeMap::new();
    for description in descriptions {
        push_once(
            &mut uses_of(&mut uses_by_name, description.key()).writers,
            description.writer().clone(),
        );
    }
    for node in nodes {
        for input in node.port_descriptor().inputs() {
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
    let uses = uses_by_name.entry(key.name().clone()).or_default();
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

/// A watcher addresses a node's leaves by one dotted path under that node,
/// so each path must name exactly one value: declared leaves must be well
/// formed, outside the pipeline's own group, and distinct. The pipeline's own
/// leaves need no comparison, since nothing else may sit in their group.
/// Checked per node: paths include the node name, so two nodes never clash.
/// Only exact matches are refused; a shared dotted prefix is a group. A
/// node's output channels are recorded under their own names, not the
/// node's, so a leaf may share a name with one.
fn check_observables(nodes: &[NodeDeclaration], errors: &mut Vec<PipelineBuildError>) {
    for node in nodes {
        let refused_leaves = check_declared_leaves(node, errors);
        check_leaves_distinct(node, &refused_leaves, errors);
    }
}

/// Each leaf the node declares must be a well-formed path outside the
/// pipeline's own group. Returns the refused leaves, each reported once, so
/// the later checks skip them.
fn check_declared_leaves(
    node: &NodeDeclaration,
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

/// Each declared leaf the earlier check accepted must be named once. A
/// duplicate is reported once per leaf.
fn check_leaves_distinct(
    node: &NodeDeclaration,
    refused_leaves: &HashSet<Arc<str>>,
    errors: &mut Vec<PipelineBuildError>,
) {
    let node_name = node.name();
    let mut seen_leaves: HashSet<Arc<str>> = HashSet::new();
    let mut reported_leaves: HashSet<Arc<str>> = HashSet::new();
    for observable in node.port_descriptor().observables() {
        let leaf = observable.leaf_name();
        if refused_leaves.contains(leaf) || seen_leaves.insert(leaf.clone()) {
            continue;
        }
        if reported_leaves.insert(leaf.clone()) {
            errors.push(PipelineBuildError::DuplicateObservable {
                node_name: node_name.to_string(),
                leaf: leaf.to_string(),
            });
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

/// Records who writes each channel, in description order: the body and the
/// outside inputs come before the nodes, so a conflict names the
/// outside-world writer before the node. Any second writer of a key is an
/// error, except of a keyed channel, whose writers share it instance by
/// instance; the instance check judges those.
fn map_suppliers<'a>(
    descriptions: &'a [ChannelDescription],
    errors: &mut Vec<PipelineBuildError>,
) -> HashMap<&'a ChannelKey, &'a Writer> {
    let mut supplier_of: HashMap<&ChannelKey, &Writer> = HashMap::new();
    for description in descriptions {
        let channel = description.key();
        if let Some(first) = supplier_of.get(channel) {
            if channel.is_keyed() {
                continue;
            }
            errors.push(PipelineBuildError::MultipleSuppliers {
                channel: channel.clone(),
                first: (*first).clone(),
                second: description.writer().clone(),
            });
        } else {
            supplier_of.insert(channel, description.writer());
        }
    }
    supplier_of
}

/// Every input, required or optional, needs a supplier. Optional only means
/// the node runs without a value. An input under a name the channel-name
/// check refused is skipped: that error already lists who writes and reads it.
fn check_inputs_supplied(
    nodes: &[NodeDeclaration],
    supplier_of: &HashMap<&ChannelKey, &Writer>,
    refused_names: &HashSet<Arc<str>>,
    errors: &mut Vec<PipelineBuildError>,
) {
    for node in nodes {
        for input in node.port_descriptor().inputs() {
            let channel = input.channel();
            if supplier_of.contains_key(channel) || refused_names.contains(channel.name()) {
                continue;
            }
            let error = match channel.kind() {
                ChannelKind::Measurement | ChannelKind::Internal => {
                    PipelineBuildError::UnsatisfiedInput {
                        node_name: node.name().to_string(),
                        channel: channel.clone(),
                        need: input.need(),
                    }
                }
                // Only a body supplies oracle and health channels, so a
                // missing one is the body's gap.
                ChannelKind::Health | ChannelKind::Oracle => {
                    PipelineBuildError::UnsatisfiedBodyCapabilities {
                        node_name: node.name().to_string(),
                        channel_key: channel.clone(),
                        need: input.need(),
                    }
                }
            };
            errors.push(error);
        }
    }
}
