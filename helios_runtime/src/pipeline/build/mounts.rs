//! Mounts, the frame each channel's values are measured in, stated by the
//! channel's writer: the body's checked against the agent, each node output's
//! [`MountRule`] checked against the node's inputs, the rules resolved down
//! the graph in level order, each mount a node reads through the tf tree
//! checked to exist, and [`InputMounts`], the mounts of one node's inputs
//! that a planned node is built with.

use super::declaration::NodeDeclaration;

use crate::{
    port::{ChannelDescription, MountRule, Writer},
    ChannelKey, FrameOwner, MountRefusal, NodeId, PipelineBuildError, PortDescriptor,
};

use helios_core::kernel::identifier::is_dotted_name;
use helios_core::prelude::AgentId;
use helios_core::spatial::id::FrameScope;
use helios_core::spatial::{transforms::Convention, FrameId};

use std::{
    collections::{HashMap, HashSet},
    error::Error,
};

/// The mounts of one planned node's declared inputs, as their writers state
/// them. The builder hands it to the node's constructor once the graph is
/// ordered and every mount resolved.
///
/// It holds only the node's own inputs, so a node can't build on a fact
/// about a channel it does not read.
#[derive(Debug)]
pub struct InputMounts {
    reader: String,
    inputs: HashMap<ChannelKey, ChannelDescription>,
}

impl InputMounts {
    /// The mounts of the inputs `descriptor` declares for node `reader`, from
    /// the resolved `descriptions`.
    pub(crate) fn new(
        reader: &str,
        descriptor: &PortDescriptor,
        descriptions: &HashMap<&ChannelKey, &ChannelDescription>,
    ) -> Self {
        let inputs = descriptor
            .inputs()
            .filter_map(|input| descriptions.get(input.channel()))
            .map(|description| (description.key().clone(), (*description).clone()))
            .collect();
        Self {
            reader: reader.to_string(),
            inputs,
        }
    }

    /// The mount of `channel`, one of the node's inputs.
    ///
    /// Fails, naming the node and the channel, if the node does not read
    /// `channel`, or if its writer states no mount: an outside input, a node
    /// output with no mount, or a body channel declared without one.
    pub fn mount(&self, channel: &ChannelKey) -> Result<&FrameId, MountLookupError> {
        let description = self
            .inputs
            .get(channel)
            .ok_or_else(|| MountLookupError::NotRead {
                reader: self.reader.clone(),
                channel: channel.clone(),
            })?;
        description
            .mount()
            .ok_or_else(|| MountLookupError::Unmounted {
                reader: self.reader.clone(),
                channel: channel.clone(),
                writer: description.writer().clone(),
            })
    }
}

/// Why [`InputMounts::mount`] has no mount for a channel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MountLookupError {
    /// The node asked for the mount of a channel it does not read.
    NotRead { reader: String, channel: ChannelKey },
    /// The channel's writer states no mount for it.
    Unmounted {
        reader: String,
        channel: ChannelKey,
        writer: Writer,
    },
}

impl Error for MountLookupError {}

impl std::fmt::Display for MountLookupError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MountLookupError::NotRead { reader, channel } => write!(
                f,
                "node \"{reader}\" asks for the mount of channel {channel}, which it does not \
                 read"
            ),
            MountLookupError::Unmounted {
                reader,
                channel,
                writer,
            } => write!(
                f,
                "node \"{reader}\" needs the mount of channel {channel}, but {writer} states \
                 none for it"
            ),
        }
    }
}

/// Every mount the body states must be a frame of `agent`'s own body other
/// than `base_link`, with a well-formed leaf: a mount is where an installed
/// part measures, so another agent's frame, a global frame or `base_link` is
/// a wiring mistake. A mount at a frame a node defines is refused with the
/// frame's owners. One error per channel.
///
/// Every mount that passes must also be in `body_frames`, where the body
/// gives it its convention; one error per frame left out, naming the first
/// unit mounted there.
pub(super) fn check_body_mounts(
    agent: &AgentId,
    descriptions: &[ChannelDescription],
    body_frames: &HashMap<FrameId, Convention>,
    errors: &mut Vec<PipelineBuildError>,
) {
    let mut unlisted = HashSet::new();
    for description in descriptions {
        let (Writer::Body { unit, .. }, Some(mount)) = (description.writer(), description.mount())
        else {
            continue;
        };
        if let Some(refusal) = refuse_body_mount(agent, mount) {
            errors.push(PipelineBuildError::RefusedBodyMount {
                unit: unit.clone(),
                channel: description.key().clone(),
                mount: mount.clone(),
                refusal,
            });
        } else if !body_frames.contains_key(mount) && unlisted.insert(mount) {
            errors.push(PipelineBuildError::FrameWithoutConvention {
                frame: mount.clone(),
                owner: FrameOwner::Unit { unit: unit.clone() },
            });
        }
    }
}

/// Why `mount` can't be a body mount of `agent`, if it can't.
fn refuse_body_mount(agent: &AgentId, mount: &FrameId) -> Option<MountRefusal> {
    match mount.scope() {
        FrameScope::Global => Some(MountRefusal::Global),
        FrameScope::Agent(owner) if owner != agent => Some(MountRefusal::OtherAgent),
        FrameScope::Agent(_) if mount.is_base_link() => Some(MountRefusal::BaseLink),
        FrameScope::Agent(_) if !is_dotted_name(mount.leaf().as_str()) => {
            Some(MountRefusal::MalformedLeaf)
        }
        FrameScope::Agent(_) => None,
    }
}

/// Every output a node mounts at an input's mount must name a channel the
/// node reads in the same tick, so the input's writer runs first and its
/// mount is resolved before the output's.
pub(super) fn check_mount_rules(nodes: &[NodeDeclaration], errors: &mut Vec<PipelineBuildError>) {
    for node in nodes {
        let descriptor = node.port_descriptor();
        for output in descriptor.output_ports() {
            let MountRule::FromInput(input) = output.mount() else {
                continue;
            };
            if !descriptor.same_tick_inputs().any(|read| read == input) {
                errors.push(PipelineBuildError::MountFromUnreadInput {
                    node_name: node.name().to_string(),
                    output: output.channel().clone(),
                    input: input.clone(),
                });
            }
        }
    }
}

/// Gives every node output the mount its rule names, walking `levels` in
/// order: a rule follows a same-tick input, whose writer sits in an earlier
/// level or outside the graph, so one pass resolves every rule. An output
/// at the mount of an input with none has none.
pub(super) fn resolve_mounts(
    levels: &[Vec<(NodeId, NodeDeclaration)>],
    descriptions: Vec<ChannelDescription>,
) -> Vec<ChannelDescription> {
    let mut mounts: HashMap<ChannelKey, Option<FrameId>> = descriptions
        .iter()
        .map(|description| (description.key().clone(), description.mount().cloned()))
        .collect();
    for (_, node) in levels.iter().flatten() {
        for output in node.port_descriptor().output_ports() {
            if let MountRule::FromInput(input) = output.mount() {
                let mount = mounts.get(input).cloned().flatten();
                mounts.insert(output.channel().clone(), mount);
            }
        }
    }

    descriptions
        .into_iter()
        .map(|description| match description.writer() {
            Writer::Node { .. } => {
                let mount = mounts.get(description.key()).cloned().flatten();
                description.with_mount(mount)
            }
            Writer::Body { .. } | Writer::Outside => description,
        })
        .collect()
}

/// Every input a node reads the mount of through the tf tree must have a
/// mount once `descriptions` are resolved. Without one, the node would look up
/// whatever frame each value names: nothing checks an unmounted channel's
/// values against a frame, and the host checks only stated mounts for reach
/// and axes. Reported once per node and channel.
pub(super) fn check_mount_readings(
    levels: &[Vec<(NodeId, NodeDeclaration)>],
    descriptions: &[ChannelDescription],
    errors: &mut Vec<PipelineBuildError>,
) {
    let mounted: HashSet<&ChannelKey> = descriptions
        .iter()
        .filter(|description| description.mount().is_some())
        .map(ChannelDescription::key)
        .collect();
    for (_, node) in levels.iter().flatten() {
        let mut reported: HashSet<&ChannelKey> = HashSet::new();
        for reading in node.port_descriptor().mount_readings() {
            let channel = reading.channel();
            if !mounted.contains(channel) && reported.insert(channel) {
                errors.push(PipelineBuildError::MountReadWithoutMount {
                    node_name: node.name().to_string(),
                    channel: channel.clone(),
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::{ChannelKey, PortDescriptorBuilder, Provenance};
    use crate::BodyChannel;

    use helios_core::spatial::transforms::Convention;

    const AGENT: &str = "car";
    const OTHER_AGENT: &str = "truck";
    const READER: &str = "ekf";

    struct Reading;

    fn agent() -> AgentId {
        AgentId::new(AGENT)
    }

    fn sensor(name: &str) -> ChannelKey {
        ChannelKey::measurement::<Reading>(name)
    }

    fn body(name: &str, mount: Option<FrameId>) -> ChannelDescription {
        let channel = BodyChannel::new(sensor(name), name, Provenance::Simulated);
        let channel = match mount {
            Some(mount) => channel.with_mount(mount),
            None => channel,
        };
        ChannelDescription::body(&channel)
    }

    fn mount_errors(
        descriptions: &[ChannelDescription],
        body_frames: &HashMap<FrameId, Convention>,
    ) -> Vec<PipelineBuildError> {
        let mut errors = vec![];
        check_body_mounts(&agent(), descriptions, body_frames, &mut errors);
        errors
    }

    /// The refusals of `descriptions` over a body listing every mount.
    fn refusals(descriptions: &[ChannelDescription]) -> Vec<MountRefusal> {
        let body_frames = descriptions
            .iter()
            .filter_map(|description| description.mount())
            .map(|mount| (mount.clone(), Convention::Flu))
            .collect();
        mount_errors(descriptions, &body_frames)
            .into_iter()
            .map(|error| match error {
                PipelineBuildError::RefusedBodyMount { refusal, .. } => refusal,
                other => panic!("unexpected error {other}"),
            })
            .collect()
    }

    /// A mount on the agent's own body other than `base_link` passes; a
    /// dotted leaf is well formed, as a channel name is.
    #[test]
    fn a_mount_on_the_agents_own_body_passes() {
        let descriptions = [
            body("gps", Some(FrameId::agent(agent(), "gps"))),
            body("imu.accel", Some(FrameId::agent(agent(), "imu.accel"))),
            body("battery", None),
        ];
        assert!(refusals(&descriptions).is_empty());
    }

    #[test]
    fn a_body_mount_off_the_agents_body_is_refused() {
        let descriptions = [
            body("a", Some(FrameId::agent(AgentId::new(OTHER_AGENT), "gps"))),
            body("b", Some(FrameId::global("datum"))),
            body("c", Some(FrameId::base_link(agent()))),
            body("d", Some(FrameId::agent(agent(), "gps/antenna"))),
        ];
        assert_eq!(
            refusals(&descriptions),
            [
                MountRefusal::OtherAgent,
                MountRefusal::Global,
                MountRefusal::BaseLink,
                MountRefusal::MalformedLeaf,
            ]
        );
    }

    /// The refusal names the unit, the channel and the frame.
    #[test]
    fn a_refused_mount_names_unit_channel_and_frame() {
        let errors = mount_errors(
            &[body("gps", Some(FrameId::base_link(agent())))],
            &HashMap::new(),
        );
        let message = errors[0].to_string();
        assert!(message.contains("unit \"gps\""), "{message}");
        assert!(message.contains(&sensor("gps").to_string()), "{message}");
        assert!(message.contains("agent.car.base_link"), "{message}");
    }

    /// A mount the body gives no convention is refused once, however many
    /// channels share it, naming the first unit there; a refused mount is
    /// reported only as refused.
    #[test]
    fn a_mount_without_a_convention_is_refused_once() {
        let imu = FrameId::agent(agent(), "imu");
        let descriptions = [
            body("imu.accel", Some(imu.clone())),
            body("imu.gyro", Some(imu.clone())),
            body("gps", Some(FrameId::base_link(agent()))),
        ];

        let errors = mount_errors(&descriptions, &HashMap::new());

        assert_eq!(errors.len(), 2, "{errors:?}");
        assert!(
            matches!(&errors[0], PipelineBuildError::FrameWithoutConvention { frame, owner: FrameOwner::Unit { unit } }
                if *frame == imu && unit == "imu.accel"),
            "{:?}",
            errors[0]
        );
        assert!(
            matches!(errors[1], PipelineBuildError::RefusedBodyMount { .. }),
            "{:?}",
            errors[1]
        );
    }

    fn deproject(input: &str, output: &str, rule_input: &str) -> NodeDeclaration {
        let descriptor = PortDescriptorBuilder::algorithm()
            .input(ChannelKey::measurement::<Reading>(input))
            .output_at_mount_of(
                ChannelKey::measurement::<Reading>(output),
                ChannelKey::measurement::<Reading>(rule_input),
            )
            .build();
        planned(output, descriptor)
    }

    fn planned(name: &str, descriptor: PortDescriptor) -> NodeDeclaration {
        let name = name.replace('.', "_");
        super::super::NodePlan::new(name, descriptor, Box::new(|_, _| Err("never built".into())))
            .into()
    }

    #[test]
    fn a_rule_naming_a_channel_the_node_does_not_read_is_refused() {
        let mut errors = vec![];
        check_mount_rules(
            &[
                deproject("lidar", "lidar.cloud", "lidar"),
                deproject("lidar", "radar.cloud", "radar"),
            ],
            &mut errors,
        );
        assert_eq!(errors.len(), 1);
        let PipelineBuildError::MountFromUnreadInput { output, input, .. } = &errors[0] else {
            panic!("unexpected error {}", errors[0]);
        };
        assert_eq!((output, input), (&sensor("radar.cloud"), &sensor("radar")));
    }

    /// A rule chain resolves in level order: a cloud at the lidar's mount,
    /// then a cropped cloud at the cloud's. An output at the mount of an
    /// unmounted input has none, and a body or outside channel keeps its own.
    #[test]
    fn rules_resolve_down_the_graph_in_level_order() {
        let lidar = FrameId::agent(agent(), "lidar");
        let levels = vec![
            vec![
                (0, deproject("lidar", "lidar.cloud", "lidar")),
                (1, deproject("radar", "radar.cloud", "radar")),
            ],
            vec![(2, deproject("lidar.cloud", "lidar.crop", "lidar.cloud"))],
        ];
        let descriptions = vec![
            body("lidar", Some(lidar.clone())),
            body("radar", None),
            ChannelDescription::outside(ChannelKey::internal::<Reading>("goal")),
            ChannelDescription::node(sensor("lidar.crop"), "lidar_crop"),
            ChannelDescription::node(sensor("lidar.cloud"), "lidar_cloud"),
            ChannelDescription::node(sensor("radar.cloud"), "radar_cloud"),
        ];

        let resolved = resolve_mounts(&levels, descriptions);
        let mounts: Vec<Option<&FrameId>> = resolved.iter().map(|d| d.mount()).collect();
        assert_eq!(
            mounts,
            [Some(&lidar), None, None, Some(&lidar), Some(&lidar), None]
        );
    }

    /// A node reading `channel`'s mount through the tree in both FLU and ENU.
    fn mount_reader(name: &str, channel: &str) -> NodeDeclaration {
        let channel = sensor(channel);
        let descriptor = PortDescriptorBuilder::algorithm()
            .input(channel.clone())
            .reads_mount_in(&channel, Convention::Flu)
            .reads_mount_in(&channel, Convention::Enu)
            .build();
        planned(name, descriptor)
    }

    /// A node reading the mount of a mounted input passes; one reading the
    /// mount of an unmounted input is refused once, however many axes it
    /// reads it in.
    #[test]
    fn a_mount_read_through_the_tree_must_exist() {
        let levels = vec![vec![
            (0, mount_reader("mapper", "lidar")),
            (1, mount_reader("tracker", "radar")),
        ]];
        let descriptions = [
            body("lidar", Some(FrameId::agent(agent(), "lidar"))),
            body("radar", None),
        ];

        let mut errors = vec![];
        check_mount_readings(&levels, &descriptions, &mut errors);

        assert_eq!(errors.len(), 1, "{errors:?}");
        let PipelineBuildError::MountReadWithoutMount { node_name, channel } = &errors[0] else {
            panic!("unexpected error {}", errors[0]);
        };
        assert_eq!((node_name.as_str(), channel), ("tracker", &sensor("radar")));
        assert_eq!(
            errors[0].to_string(),
            format!(
                "node \"tracker\" reads the mount of {radar} through the tf tree, but no writer \
                 of {radar} states a mount",
                radar = sensor("radar")
            )
        );
    }

    fn input_mounts(descriptions: &[ChannelDescription]) -> InputMounts {
        let descriptor = PortDescriptorBuilder::algorithm()
            .input(ChannelKey::measurement::<Reading>("gps"))
            .optional(ChannelKey::measurement::<Reading>("battery"))
            .build();
        let by_key = descriptions.iter().map(|d| (d.key(), d)).collect();
        InputMounts::new(READER, &descriptor, &by_key)
    }

    /// A node gets the mounts of the channels it reads, and only those.
    #[test]
    fn a_node_looks_up_the_mounts_of_its_own_inputs() {
        let gps = FrameId::agent(agent(), "gps");
        let mag = FrameId::agent(agent(), "mag");
        let mounts = input_mounts(&[
            body("gps", Some(gps.clone())),
            body("battery", None),
            body("mag", Some(mag)),
        ]);

        assert_eq!(mounts.mount(&sensor("gps")), Ok(&gps));
        assert_eq!(
            mounts.mount(&sensor("mag")),
            Err(MountLookupError::NotRead {
                reader: READER.to_string(),
                channel: sensor("mag"),
            })
        );
        let Err(unmounted) = mounts.mount(&sensor("battery")) else {
            panic!("the battery has no mount");
        };
        assert_eq!(
            unmounted.to_string(),
            format!(
                "node \"{READER}\" needs the mount of channel {}, but unit \"battery\" states \
                 none for it",
                sensor("battery")
            )
        );
    }
}
