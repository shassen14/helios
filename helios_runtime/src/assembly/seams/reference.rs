//! The reference seam: turns the `[reference]` section into the `Selector`
//! that writes the guidance reference the controllers track.
//!
//! Each member writes its reference on a channel of its own. The seam reads
//! the section's member list, finds each member's reference output among the
//! built nodes, and adds one `Selector` from those channels onto the resolved
//! [`control::reference`] channel. A lone member is the selector's `base` with
//! no `preferred` inputs, which forwards it every tick, so the shape is the
//! same for any number of members.
//!
//! The seam type is `BodyTwistRef`, the only reference type today. When a
//! second appears, the section names it and this pass dispatches on it.

use crate::assembly::error::PipelineAssemblyError;
use crate::channels::control;
use crate::config::{ArbitrationPolicyConfig, ReferenceSeamConfig};
use crate::nodes::combinators::{Selector, SelectorPolicy};
use crate::pipeline::node::PipelineNode;
use crate::port::{ChannelKey, InternalChannel};

use helios_core::control::BodyTwistRef;

use std::any::{type_name, TypeId};
use std::collections::HashSet;
use std::iter;

/// Node name for the `Selector` this seam adds. Raw identity for observability,
/// so a referenced const; this pass is its sole synthesizer.
pub(crate) const REFERENCE_ARBITER_NODE: &str = "reference_arbiter";

/// The seam's name in errors: its stack section.
const SEAM: &str = "reference";

/// Builds the reference `Selector` from `config`, resolving each member among
/// `nodes`. No section means no seam: `None`, and anything reading the
/// reference fails the build as an unsatisfied input.
///
/// Fails with one error per member that is unknown, named twice, or lacks
/// exactly one reference output.
pub(in crate::assembly) fn reference_selector(
    config: Option<&ReferenceSeamConfig>,
    nodes: &[Box<dyn PipelineNode>],
) -> Result<Option<Box<dyn PipelineNode>>, Vec<PipelineAssemblyError>> {
    let Some(config) = config else {
        return Ok(None);
    };

    let mut errors = vec![];
    let mut seen = HashSet::new();
    for member in iter::once(&config.base).chain(&config.preferred) {
        if !seen.insert(member.as_str()) {
            errors.push(PipelineAssemblyError::DuplicateSeamMember {
                seam: SEAM,
                member: member.clone(),
            });
        }
    }

    let base = member_reference(&config.base, nodes);
    let preferred: Vec<_> = config
        .preferred
        .iter()
        .map(|member| member_reference(member, nodes))
        .collect();

    let base = base.map_err(|err| errors.push(err)).ok();
    let preferred: Vec<InternalChannel> = preferred
        .into_iter()
        .filter_map(|channel| channel.map_err(|err| errors.push(err)).ok())
        .collect();

    match base {
        Some(base) if errors.is_empty() => Ok(Some(Box::new(Selector::<BodyTwistRef>::new(
            REFERENCE_ARBITER_NODE,
            preferred,
            base,
            control::reference::<BodyTwistRef>(),
            selector_policy(config),
        )))),
        _ => Err(errors),
    }
}

/// The channel member `member` writes its reference on: its one internal
/// `BodyTwistRef` output.
fn member_reference(
    member: &str,
    nodes: &[Box<dyn PipelineNode>],
) -> Result<InternalChannel, PipelineAssemblyError> {
    let Some(node) = nodes.iter().find(|node| node.name() == member) else {
        return Err(PipelineAssemblyError::UnknownSeamMember {
            seam: SEAM,
            member: member.to_string(),
        });
    };

    let matching: Vec<&InternalChannel> = node
        .port_descriptor()
        .outputs()
        .iter()
        .filter_map(|key| match key {
            ChannelKey::Internal(channel) if channel.type_id() == TypeId::of::<BodyTwistRef>() => {
                Some(channel)
            }
            _ => None,
        })
        .collect();

    match matching.as_slice() {
        [only] => Ok((*only).clone()),
        _ => Err(PipelineAssemblyError::SeamMemberOutputMismatch {
            seam: SEAM,
            member: member.to_string(),
            expected: type_name::<BodyTwistRef>(),
            matching: matching.len(),
        }),
    }
}

/// Translates the section's policy into the one the `Selector` applies.
fn selector_policy(config: &ReferenceSeamConfig) -> SelectorPolicy {
    match config.policy {
        ArbitrationPolicyConfig::FreshnessOverride => SelectorPolicy::FreshnessOverride {
            max_age: config.max_age_s,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::pipeline::node::TickContext;
    use crate::port::{AlgorithmNodePortDescriptor, PortBus, PortDescriptor};

    use helios_core::prelude::TfProvider;

    /// A node that does nothing; only its name and outputs matter here.
    struct StubNode {
        name: String,
        descriptor: PortDescriptor,
    }

    impl PipelineNode for StubNode {
        fn name(&self) -> &str {
            &self.name
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {}
    }

    /// A node named `name` writing `outputs`.
    fn stub(name: &str, outputs: Vec<InternalChannel>) -> Box<dyn PipelineNode> {
        let descriptor = outputs
            .into_iter()
            .fold(AlgorithmNodePortDescriptor::new(), |builder, output| {
                builder.output_internal(output)
            })
            .build();
        Box::new(StubNode {
            name: name.to_string(),
            descriptor,
        })
    }

    /// A node writing its reference on a channel named after itself, as every
    /// follower and teleop mapper does.
    fn contender(name: &str) -> Box<dyn PipelineNode> {
        stub(name, vec![InternalChannel::named::<BodyTwistRef>(name)])
    }

    fn reference_key(name: &str) -> ChannelKey {
        InternalChannel::named::<BodyTwistRef>(name).into()
    }

    fn section(base: &str, preferred: &[&str]) -> ReferenceSeamConfig {
        ReferenceSeamConfig {
            base: base.to_string(),
            preferred: preferred.iter().map(|name| name.to_string()).collect(),
            policy: ArbitrationPolicyConfig::FreshnessOverride,
            max_age_s: 0.25,
        }
    }

    fn errors_of(
        result: Result<Option<Box<dyn PipelineNode>>, Vec<PipelineAssemblyError>>,
    ) -> Vec<PipelineAssemblyError> {
        match result {
            Ok(_) => panic!("the seam must not build"),
            Err(errors) => errors,
        }
    }

    #[test]
    fn no_section_adds_no_selector() {
        let nodes = vec![contender("pure_pursuit")];

        let selector = reference_selector(None, &nodes).expect("no section is not an error");

        assert!(selector.is_none());
    }

    #[test]
    fn a_lone_base_is_forwarded_with_no_preferred_inputs() {
        let nodes = vec![contender("pure_pursuit")];

        let selector = reference_selector(Some(&section("pure_pursuit", &[])), &nodes)
            .expect("a lone base builds")
            .expect("a section adds a selector");
        let descriptor = selector.port_descriptor();

        assert_eq!(selector.name(), REFERENCE_ARBITER_NODE);
        assert_eq!(
            descriptor.required_inputs().cloned().collect::<Vec<_>>(),
            vec![reference_key("pure_pursuit")]
        );
        assert_eq!(descriptor.optional_inputs().count(), 0);
        assert_eq!(
            descriptor.outputs(),
            [ChannelKey::from(control::reference::<BodyTwistRef>())].as_slice()
        );
    }

    #[test]
    fn preferred_members_are_optional_inputs_in_priority_order() {
        let nodes = vec![
            contender("pure_pursuit"),
            contender("teleop"),
            contender("remote"),
        ];

        let selector = reference_selector(
            Some(&section("pure_pursuit", &["teleop", "remote"])),
            &nodes,
        )
        .expect("three members build")
        .expect("a section adds a selector");

        assert_eq!(
            selector
                .port_descriptor()
                .optional_inputs()
                .cloned()
                .collect::<Vec<_>>(),
            vec![reference_key("teleop"), reference_key("remote")]
        );
    }

    #[test]
    fn a_member_that_is_not_a_node_is_named() {
        let nodes = vec![contender("pure_pursuit")];

        let errors = errors_of(reference_selector(
            Some(&section("pure_pursuit", &["teleop"])),
            &nodes,
        ));

        assert!(
            matches!(
                errors.as_slice(),
                [PipelineAssemblyError::UnknownSeamMember { seam: "reference", member }]
                    if member == "teleop"
            ),
            "got {errors:?}"
        );
    }

    #[test]
    fn a_member_named_twice_is_rejected() {
        let nodes = vec![contender("pure_pursuit")];

        let errors = errors_of(reference_selector(
            Some(&section("pure_pursuit", &["pure_pursuit"])),
            &nodes,
        ));

        assert!(
            matches!(
                errors.as_slice(),
                [PipelineAssemblyError::DuplicateSeamMember { member, .. }]
                    if member == "pure_pursuit"
            ),
            "got {errors:?}"
        );
    }

    #[test]
    fn a_member_without_a_reference_output_is_rejected() {
        // A planner writes a `Path`, not a reference, so naming it as a member
        // is a mistake the seam reports rather than wiring nothing.
        let nodes = vec![stub(
            "local_path",
            vec![InternalChannel::named::<f64>("local_path")],
        )];

        let errors = errors_of(reference_selector(
            Some(&section("local_path", &[])),
            &nodes,
        ));

        assert!(
            matches!(
                errors.as_slice(),
                [PipelineAssemblyError::SeamMemberOutputMismatch { member, matching: 0, .. }]
                    if member == "local_path"
            ),
            "got {errors:?}"
        );
    }

    #[test]
    fn every_bad_member_is_reported_at_once() {
        let nodes = vec![contender("pure_pursuit")];

        let errors = errors_of(reference_selector(
            Some(&section("missing_base", &["missing_preferred"])),
            &nodes,
        ));

        assert_eq!(errors.len(), 2, "got {errors:?}");
    }

    #[test]
    fn freshness_policy_carries_the_configured_max_age() {
        match selector_policy(&section("pure_pursuit", &[])) {
            SelectorPolicy::FreshnessOverride { max_age } => {
                assert!((max_age - 0.25).abs() < f64::EPSILON);
            }
        }
    }
}
