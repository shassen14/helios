//! Resolving the members a seam section names to the channels they write.

use crate::assembly::error::PipelineAssemblyError;
use crate::pipeline::node::PipelineNode;
use crate::port::{ChannelKey, InternalChannel};

use std::any::{type_name, TypeId};
use std::collections::HashSet;

/// The payload type a seam combines, known at run time: what a member's
/// output must be to join it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SeamType {
    id: TypeId,
    name: &'static str,
}

impl SeamType {
    pub(crate) fn of<T: 'static>() -> Self {
        Self {
            id: TypeId::of::<T>(),
            name: type_name::<T>(),
        }
    }
}

/// One [`PipelineAssemblyError::DuplicateSeamMember`] for each repeat in
/// `members`, in order.
pub(super) fn duplicates<'a>(
    seam: &str,
    members: impl IntoIterator<Item = &'a String>,
) -> Vec<PipelineAssemblyError> {
    let mut seen = HashSet::new();
    members
        .into_iter()
        .filter(|member| !seen.insert(member.as_str()))
        .map(|member| PipelineAssemblyError::DuplicateSeamMember {
            seam: seam.to_string(),
            member: member.clone(),
        })
        .collect()
}

/// The channel `member` writes the seam's type on: its one internal output of
/// type `ty`.
///
/// Fails if no node is named `member`, or if it has no such output or more
/// than one.
pub(super) fn member_output(
    seam: &str,
    ty: SeamType,
    member: &str,
    nodes: &[Box<dyn PipelineNode>],
) -> Result<InternalChannel, PipelineAssemblyError> {
    let Some(node) = nodes.iter().find(|node| node.name() == member) else {
        return Err(PipelineAssemblyError::UnknownSeamMember {
            seam: seam.to_string(),
            member: member.to_string(),
        });
    };

    let outputs = node.port_descriptor().outputs();
    let matching: Vec<&InternalChannel> = outputs
        .iter()
        .filter_map(|key| match key {
            ChannelKey::Internal(channel) if channel.type_id() == ty.id => Some(channel),
            _ => None,
        })
        .collect();

    match matching.as_slice() {
        [only] => Ok((*only).clone()),
        _ => Err(PipelineAssemblyError::SeamMemberOutputMismatch {
            seam: seam.to_string(),
            member: member.to_string(),
            expected: ty.name,
            matching: matching.len(),
        }),
    }
}

/// Resolves each of `members` with [`member_output`], keeping the channels
/// that resolve and pushing an error onto `errors` for each that doesn't.
pub(super) fn member_outputs(
    seam: &str,
    ty: SeamType,
    members: &[String],
    nodes: &[Box<dyn PipelineNode>],
    errors: &mut Vec<PipelineAssemblyError>,
) -> Vec<InternalChannel> {
    members
        .iter()
        .filter_map(|member| {
            member_output(seam, ty, member, nodes)
                .map_err(|err| errors.push(err))
                .ok()
        })
        .collect()
}
