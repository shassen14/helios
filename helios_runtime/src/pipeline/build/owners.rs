//! Frame owners: every frame the agent's graph names as someone's own has one
//! owner. The body owns its `base_link` and each frame it mounts a channel
//! at; a node owns each frame it defines, such as the root an estimator holds
//! its state in.

use super::declaration::NodeDeclaration;

use crate::{
    port::{ChannelDescription, Writer},
    FrameOwner, PipelineBuildError,
};

use helios_core::prelude::AgentId;
use helios_core::spatial::FrameId;

use std::collections::HashMap;

/// Refuses every frame with two owners: two nodes defining it, or a node
/// defining `agent`'s `base_link` or a frame the body mounts a channel at.
/// One error per extra owner, naming the first owner and the extra one. The
/// body's units share an owner, so two mounts at one frame are not refused
/// here.
pub(super) fn check_frame_owners(
    agent: &AgentId,
    nodes: &[NodeDeclaration],
    descriptions: &[ChannelDescription],
    errors: &mut Vec<PipelineBuildError>,
) {
    let mut owners: HashMap<FrameId, FrameOwner> = HashMap::new();
    owners.insert(FrameId::base_link(agent.clone()), FrameOwner::Body);
    for description in descriptions {
        if let (Writer::Body { unit, .. }, Some(mount)) =
            (description.writer(), description.mount())
        {
            owners
                .entry(mount.clone())
                .or_insert_with(|| FrameOwner::Unit { unit: unit.clone() });
        }
    }

    for node in nodes {
        for frame in node.port_descriptor().defined_frames() {
            let owner = FrameOwner::Node {
                node: node.name().to_string(),
            };
            match owners.get(frame) {
                Some(first) => errors.push(PipelineBuildError::FrameDefinedTwice {
                    frame: frame.clone(),
                    first: first.clone(),
                    second: owner,
                }),
                None => {
                    owners.insert(frame.clone(), owner);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::{ChannelKey, PortDescriptorBuilder, Provenance};
    use crate::BodyChannel;

    struct Reading;

    fn agent() -> AgentId {
        AgentId::new("car")
    }

    fn estimator(name: &str, root: FrameId) -> NodeDeclaration {
        let descriptor = PortDescriptorBuilder::algorithm()
            .defines_frame(root)
            .build();
        super::super::NodePlan::new(name, descriptor, Box::new(|_, _| Err("never built".into())))
            .into()
    }

    fn mounted(unit: &str, mount: FrameId) -> ChannelDescription {
        let key = ChannelKey::measurement::<Reading>(unit);
        ChannelDescription::body(
            &BodyChannel::new(key, unit, Provenance::Simulated).with_mount(mount),
        )
    }

    fn check(nodes: &[NodeDeclaration], descriptions: &[ChannelDescription]) -> Vec<String> {
        let mut errors = vec![];
        check_frame_owners(&agent(), nodes, descriptions, &mut errors);
        errors.iter().map(ToString::to_string).collect()
    }

    /// An estimator in `odom` beside a shadow in its own root, over a body
    /// with a mounted sensor: every frame has one owner.
    #[test]
    fn distinct_roots_and_mounts_pass() {
        let nodes = [
            estimator("primary", FrameId::odom(agent())),
            estimator("shadow", FrameId::agent(agent(), "shadow.odom")),
        ];
        let body = [mounted("gps", FrameId::agent(agent(), "gps"))];
        assert_eq!(check(&nodes, &body), Vec::<String>::new());
    }

    /// Two nodes defining one frame are refused once, naming both.
    #[test]
    fn a_frame_two_nodes_define_is_refused_naming_both() {
        let nodes = [
            estimator("primary", FrameId::odom(agent())),
            estimator("shadow", FrameId::odom(agent())),
        ];
        let errors = check(&nodes, &[]);
        assert_eq!(errors.len(), 1, "{errors:?}");
        assert!(errors[0].contains("agent.car.odom"), "{}", errors[0]);
        assert!(errors[0].contains("node \"primary\""), "{}", errors[0]);
        assert!(errors[0].contains("node \"shadow\""), "{}", errors[0]);
    }

    /// A node defining the body's `base_link`, or a frame the body mounts a
    /// channel at, is refused naming the body's owner first.
    #[test]
    fn a_node_defining_a_frame_the_body_owns_is_refused() {
        let nodes = [
            estimator("a", FrameId::base_link(agent())),
            estimator("b", FrameId::agent(agent(), "gps")),
        ];
        let body = [mounted("gps", FrameId::agent(agent(), "gps"))];
        let errors = check(&nodes, &body);
        assert_eq!(errors.len(), 2, "{errors:?}");
        assert!(
            errors[0].contains("by the body and by node \"a\""),
            "{}",
            errors[0]
        );
        assert!(
            errors[1].contains("by unit \"gps\" and by node \"b\""),
            "{}",
            errors[1]
        );
    }
}
