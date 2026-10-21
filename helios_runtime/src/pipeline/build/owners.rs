//! Frame owners: every frame the agent's graph names as someone's own has one
//! owner, and the owner gives its axis convention. The body owns its
//! `base_link`, each frame it mounts a channel at and every other frame it
//! lists; the host owns its world frames; a node owns each frame it defines,
//! such as the root an estimator holds its state in.

use super::declaration::NodeDeclaration;

use crate::{
    port::{ChannelDescription, Writer},
    FrameOwner, PipelineBuildError,
};

use helios_core::prelude::AgentId;
use helios_core::spatial::{transforms::Convention, FrameId};

use std::collections::HashMap;

/// Each owned frame with the axis convention its owner gives it. Only the
/// builder makes one; a built pipeline exposes it through
/// [`AutonomyPipeline::frame_conventions`](crate::AutonomyPipeline::frame_conventions).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FrameConventions {
    conventions: HashMap<FrameId, Convention>,
}

impl FrameConventions {
    /// `frame`'s axis convention, or `None` when its owner gives none or it
    /// has no owner.
    pub fn convention_of(&self, frame: &FrameId) -> Option<Convention> {
        self.conventions.get(frame).copied()
    }
}

/// Refuses every frame with two owners: two nodes defining it, a node
/// defining a frame the body owns (`agent`'s `base_link`, a frame it mounts a
/// channel at, or one of `body_frames`) or one of `host_frames`, or a host
/// frame the body owns. One error per extra owner, naming the first owner and
/// the extra one. The body's units share an owner, so two mounts at one frame
/// are not refused here.
///
/// Returns each owned frame's convention as its owner gives it: the body's
/// from `body_frames`, the host's from `host_frames`, a node's from its
/// definition. A body frame missing from `body_frames` has none.
pub(super) fn check_frame_owners(
    agent: &AgentId,
    nodes: &[NodeDeclaration],
    descriptions: &[ChannelDescription],
    body_frames: &HashMap<FrameId, Convention>,
    host_frames: &HashMap<FrameId, Convention>,
    errors: &mut Vec<PipelineBuildError>,
) -> FrameConventions {
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
    for frame in body_frames.keys() {
        owners.entry(frame.clone()).or_insert(FrameOwner::Body);
    }
    let mut conventions = body_frames.clone();

    // Sorted by spelling, so the errors come in one order every build.
    let mut host: Vec<_> = host_frames.iter().collect();
    host.sort_by_key(|(frame, _)| frame.to_string());
    let host = host
        .into_iter()
        .map(|(frame, convention)| (FrameOwner::Host, frame, *convention));
    let nodes = nodes.iter().flat_map(|node| {
        let owner = FrameOwner::Node {
            node: node.name().to_string(),
        };
        node.port_descriptor()
            .defined_frames()
            .iter()
            .map(move |(frame, convention)| (owner.clone(), frame, *convention))
    });

    for (owner, frame, convention) in host.chain(nodes) {
        match owners.get(frame) {
            Some(first) => errors.push(PipelineBuildError::FrameDefinedTwice {
                frame: frame.clone(),
                first: first.clone(),
                second: owner,
            }),
            None => {
                owners.insert(frame.clone(), owner);
                conventions.insert(frame.clone(), convention);
            }
        }
    }

    FrameConventions { conventions }
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
            .defines_frame(root, Convention::Enu)
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
        own(nodes, descriptions, &HashMap::new(), &HashMap::new()).1
    }

    fn own(
        nodes: &[NodeDeclaration],
        descriptions: &[ChannelDescription],
        body_frames: &HashMap<FrameId, Convention>,
        host_frames: &HashMap<FrameId, Convention>,
    ) -> (FrameConventions, Vec<String>) {
        let mut errors = vec![];
        let conventions = check_frame_owners(
            &agent(),
            nodes,
            descriptions,
            body_frames,
            host_frames,
            &mut errors,
        );
        (
            conventions,
            errors.iter().map(ToString::to_string).collect(),
        )
    }

    fn datum() -> FrameId {
        FrameId::global("datum")
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

    /// The body's frames, the host's datum and an estimator's root each take
    /// their owner's convention; a frame nobody owns has none.
    #[test]
    fn each_frame_takes_its_owner_s_convention() {
        let (base_link, gps) = (FrameId::base_link(agent()), FrameId::agent(agent(), "gps"));
        let body_frames = HashMap::from([
            (base_link.clone(), Convention::Flu),
            (gps.clone(), Convention::Flu),
        ]);
        let host_frames = HashMap::from([(datum(), Convention::Enu)]);
        let nodes = [estimator("primary", FrameId::odom(agent()))];

        let (conventions, errors) = own(
            &nodes,
            &[mounted("gps", gps.clone())],
            &body_frames,
            &host_frames,
        );

        assert_eq!(errors, Vec::<String>::new());
        assert_eq!(conventions.convention_of(&base_link), Some(Convention::Flu));
        assert_eq!(conventions.convention_of(&gps), Some(Convention::Flu));
        assert_eq!(conventions.convention_of(&datum()), Some(Convention::Enu));
        assert_eq!(
            conventions.convention_of(&FrameId::odom(agent())),
            Some(Convention::Enu)
        );
        assert_eq!(conventions.convention_of(&FrameId::map(agent())), None);
    }

    /// A frame the body lists without mounting a channel at it, such as a
    /// wheel's mount, is still the body's: a node defining it is refused.
    #[test]
    fn a_listed_body_frame_is_the_body_s() {
        let wheel = FrameId::agent(agent(), "wheel_fl");
        let body_frames = HashMap::from([(wheel.clone(), Convention::Flu)]);

        let (_, errors) = own(&[estimator("a", wheel)], &[], &body_frames, &HashMap::new());

        assert_eq!(errors.len(), 1, "{errors:?}");
        assert!(
            errors[0].contains("by the body and by node \"a\""),
            "{}",
            errors[0]
        );
    }

    /// A node defining the host's datum, and a datum the body also names, are
    /// each refused naming the host.
    #[test]
    fn a_host_frame_has_one_owner() {
        let host_frames = HashMap::from([(datum(), Convention::Enu)]);

        let (_, errors) = own(
            &[estimator("a", datum())],
            &[],
            &HashMap::new(),
            &host_frames,
        );
        assert_eq!(errors.len(), 1, "{errors:?}");
        assert!(
            errors[0].contains("by the host and by node \"a\""),
            "{}",
            errors[0]
        );

        let body_frames = HashMap::from([(datum(), Convention::Enu)]);
        let (_, errors) = own(&[], &[], &body_frames, &host_frames);
        assert_eq!(errors.len(), 1, "{errors:?}");
        assert!(
            errors[0].contains("by the body and by the host"),
            "{}",
            errors[0]
        );
    }
}
