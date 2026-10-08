//! The `[actuators]` seam's pass.

use super::super::members::{duplicates, member_output, SeamType};
use super::config::ActuatorSeamConfig;

use crate::assembly::error::PipelineAssemblyError;
use crate::body::BodyCapabilities;
use crate::channels::control;
use crate::nodes::combinators::Merge;
use crate::pipeline::node::PipelineNode;
use crate::port::ChannelKey;

use helios_core::control::actuators::{ActuatorCommand, ActuatorDrive};

use std::collections::{BTreeMap, HashSet};

/// Node name for the `Merge` this seam adds. Raw identity for observability,
/// so a referenced const; this pass is its sole synthesizer.
pub(crate) const ACTUATOR_MERGE_NODE: &str = "actuator_merge";

/// The seam's name in errors: its stack section.
const SEAM: &str = "actuators";

/// Builds the actuator `Merge` from `config`, resolving each member among
/// `nodes` and checking the actuators its `ActuatorCommand` output declares
/// against the other members and against `body`. No section means no seam:
/// `None`, and the body is never commanded.
///
/// Fails with every error found: a member that is unknown, named twice, lacks
/// exactly one `ActuatorCommand` output or declares no drives on it; an
/// actuator two members drive; an actuator the body lacks, or one that accepts
/// another setpoint kind.
pub(in crate::assembly) fn actuator_merge(
    config: Option<&ActuatorSeamConfig>,
    body: &BodyCapabilities,
    nodes: &[Box<dyn PipelineNode>],
) -> Result<Option<Box<dyn PipelineNode>>, Vec<PipelineAssemblyError>> {
    let Some(config) = config else {
        return Ok(None);
    };

    if config.members.is_empty() {
        return Err(vec![PipelineAssemblyError::EmptySeam {
            seam: SEAM.to_string(),
        }]);
    }

    let ty = SeamType::of::<ActuatorCommand>();
    let mut errors = duplicates(SEAM, &config.members);
    let mut seen = HashSet::new();
    let mut partials = Vec::new();
    let mut driven: BTreeMap<String, Vec<String>> = BTreeMap::new();

    for member in &config.members {
        if !seen.insert(member.as_str()) {
            continue;
        }
        let partial = match member_output(SEAM, ty, member, nodes) {
            Ok(partial) => partial,
            Err(err) => {
                errors.push(err);
                continue;
            }
        };

        let member_drives = declared_drives(member, &partial.clone().into(), nodes);
        partials.push(partial);
        if member_drives.is_empty() {
            errors.push(PipelineAssemblyError::UndeclaredDrives {
                member: member.clone(),
            });
            continue;
        }

        // A member listing one actuator twice still drives it once.
        let mut member_actuators = HashSet::new();
        for drive in member_drives {
            if member_actuators.insert(drive.actuator().as_str()) {
                driven
                    .entry(drive.actuator().as_str().to_string())
                    .or_default()
                    .push(member.clone());
            }
            errors.extend(body_disagreement(member, drive, body));
        }
    }

    for (actuator, mut members) in driven {
        if members.len() > 1 {
            members.sort();
            errors.push(PipelineAssemblyError::ActuatorDrivenTwice { actuator, members });
        }
    }

    if errors.is_empty() {
        Ok(Some(Box::new(Merge::new(
            ACTUATOR_MERGE_NODE,
            partials,
            control::actuators(),
        ))))
    } else {
        Err(errors)
    }
}

/// The drives `member` declares on its `output`, each listed once. `member`
/// is among `nodes`: the caller resolved `output` from it.
fn declared_drives<'a>(
    member: &str,
    output: &ChannelKey,
    nodes: &'a [Box<dyn PipelineNode>],
) -> Vec<&'a ActuatorDrive> {
    let mut unique = Vec::new();
    if let Some(node) = nodes.iter().find(|node| node.name() == member) {
        for drive in node.port_descriptor().drives(output) {
            if !unique.contains(&drive) {
                unique.push(drive);
            }
        }
    }
    unique
}

/// Why `body` can't take what `member` writes to one actuator, if it can't:
/// the body lacks the actuator, or the actuator accepts another setpoint kind.
fn body_disagreement(
    member: &str,
    drive: &ActuatorDrive,
    body: &BodyCapabilities,
) -> Option<PipelineAssemblyError> {
    match body.actuation.spec(drive.actuator()) {
        None => Some(PipelineAssemblyError::ActuatorNotOnBody {
            member: member.to_string(),
            actuator: drive.actuator().as_str().to_string(),
            body: body.name.clone(),
        }),
        Some(spec) if spec.kind() != drive.kind() => {
            Some(PipelineAssemblyError::ActuatorKindMismatch {
                member: member.to_string(),
                actuator: drive.actuator().as_str().to_string(),
                writes: drive.kind(),
                accepts: spec.kind(),
            })
        }
        Some(_) => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assembly::test_stub::{stub, stub_driving};
    use crate::port::InternalChannel;

    use helios_core::control::actuation_model::{ActuationModel, ActuatorSpec, SignConvention};
    use helios_core::control::actuators::{ActuatorId, SetpointKind};
    use helios_core::control::commands::DriveForce;

    /// A node writing its partial command on a channel named after itself, as
    /// every allocator does, driving `drives` as `(actuator, kind)` pairs.
    fn allocator(name: &str, drives: &[(&str, SetpointKind)]) -> Box<dyn PipelineNode> {
        stub_driving(
            name,
            drives
                .iter()
                .map(|(actuator, kind)| ActuatorDrive::new(ActuatorId::new(*actuator), *kind))
                .collect(),
        )
    }

    fn partial_key(name: &str) -> ChannelKey {
        InternalChannel::named::<ActuatorCommand>(name).into()
    }

    fn section(members: &[&str]) -> ActuatorSeamConfig {
        ActuatorSeamConfig {
            members: members.iter().map(|name| name.to_string()).collect(),
        }
    }

    /// The raycast car's body: a torque-driven wheel and a position-driven
    /// steer.
    fn car_body() -> BodyCapabilities {
        let spec = |id: &str, kind: SetpointKind| {
            ActuatorSpec::new(
                ActuatorId::new(id),
                kind,
                1.0,
                kind.value(0.0),
                SignConvention::Normal,
            )
        };
        BodyCapabilities {
            name: "car".to_string(),
            actuation: ActuationModel::new(vec![
                spec("wheels", SetpointKind::Torque),
                spec("steer_axle", SetpointKind::Position),
            ]),
            ..Default::default()
        }
    }

    /// The car's two allocators, as the proving ground wires them.
    fn car_drive() -> Box<dyn PipelineNode> {
        allocator("drive", &[("wheels", SetpointKind::Torque)])
    }

    fn car_steer() -> Box<dyn PipelineNode> {
        allocator("steer", &[("steer_axle", SetpointKind::Position)])
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
    fn no_section_adds_no_merge() {
        let nodes = vec![car_drive()];

        let merge = actuator_merge(None, &car_body(), &nodes).expect("no section is not an error");

        assert!(merge.is_none());
    }

    #[test]
    fn members_merge_onto_the_fixed_actuators_channel() {
        let nodes = vec![car_drive(), car_steer()];

        let merge = actuator_merge(Some(&section(&["drive", "steer"])), &car_body(), &nodes)
            .expect("the car's seam builds")
            .expect("a section adds a merge");

        assert_eq!(merge.name(), ACTUATOR_MERGE_NODE);
        assert_eq!(
            merge
                .port_descriptor()
                .required_inputs()
                .cloned()
                .collect::<Vec<_>>(),
            [partial_key("drive"), partial_key("steer")]
        );
        assert_eq!(
            merge.port_descriptor().outputs(),
            [ChannelKey::from(control::actuators())]
        );
    }

    #[test]
    fn an_empty_member_list_is_rejected() {
        let errors = errors_of(actuator_merge(Some(&section(&[])), &car_body(), &[]));

        assert!(
            matches!(errors.as_slice(), [PipelineAssemblyError::EmptySeam { seam }] if seam == SEAM),
            "got {errors:?}"
        );
    }

    #[test]
    fn unknown_duplicate_and_wrong_type_members_are_rejected() {
        let nodes = vec![
            car_drive(),
            stub("speed", vec![InternalChannel::named::<DriveForce>("speed")]),
        ];

        let errors = errors_of(actuator_merge(
            Some(&section(&["drive", "drive", "missing", "speed"])),
            &car_body(),
            &nodes,
        ));

        assert!(
            matches!(
                errors.as_slice(),
                [
                    PipelineAssemblyError::DuplicateSeamMember { member: dup, .. },
                    PipelineAssemblyError::UnknownSeamMember { member: unknown, .. },
                    PipelineAssemblyError::SeamMemberOutputMismatch { member: wrong, matching: 0, .. },
                ] if dup == "drive" && unknown == "missing" && wrong == "speed"
            ),
            "got {errors:?}"
        );
    }

    #[test]
    fn a_member_that_declares_no_drives_is_rejected() {
        // A node writing an `ActuatorCommand` without saying what it drives:
        // the collision and body checks would be blind to it.
        let nodes = vec![
            car_drive(),
            stub(
                "direct",
                vec![InternalChannel::named::<ActuatorCommand>("direct")],
            ),
        ];

        let errors = errors_of(actuator_merge(
            Some(&section(&["drive", "direct"])),
            &car_body(),
            &nodes,
        ));

        assert!(
            matches!(errors.as_slice(), [PipelineAssemblyError::UndeclaredDrives { member }] if member == "direct"),
            "got {errors:?}"
        );
    }

    #[test]
    fn a_controller_writing_actuator_commands_directly_can_be_a_member() {
        // Membership is by output type, not by being an allocator.
        let nodes = vec![allocator(
            "joint_pid",
            &[("shoulder", SetpointKind::Torque)],
        )];
        let body = BodyCapabilities {
            actuation: ActuationModel::new(vec![ActuatorSpec::new(
                ActuatorId::new("shoulder"),
                SetpointKind::Torque,
                1.0,
                SetpointKind::Torque.value(0.0),
                SignConvention::Normal,
            )]),
            ..Default::default()
        };

        let merge = actuator_merge(Some(&section(&["joint_pid"])), &body, &nodes)
            .expect("a lone direct member builds");

        assert!(merge.is_some());
    }

    #[test]
    fn an_actuator_driven_by_two_members_names_both() {
        let nodes = vec![
            allocator("front_axle", &[("wheels", SetpointKind::Torque)]),
            allocator("rear_axle", &[("wheels", SetpointKind::Torque)]),
        ];

        let errors = errors_of(actuator_merge(
            Some(&section(&["rear_axle", "front_axle"])),
            &car_body(),
            &nodes,
        ));

        assert!(
            matches!(
                errors.as_slice(),
                [PipelineAssemblyError::ActuatorDrivenTwice { actuator, members }]
                    if actuator == "wheels" && members.as_slice() == ["front_axle", "rear_axle"]
            ),
            "got {errors:?}"
        );
    }

    #[test]
    fn a_member_listing_one_actuator_twice_drives_it_once() {
        // A repeated declaration is not two members driving one actuator, and
        // is checked against the body once.
        let nodes = vec![
            allocator(
                "drive",
                &[
                    ("wheels", SetpointKind::Torque),
                    ("wheels", SetpointKind::Torque),
                ],
            ),
            car_steer(),
        ];

        let merge = actuator_merge(Some(&section(&["drive", "steer"])), &car_body(), &nodes)
            .expect("a repeated declaration builds");

        assert!(merge.is_some());
    }

    #[test]
    fn an_actuator_the_body_lacks_names_the_body() {
        let nodes = vec![allocator("drive", &[("traction", SetpointKind::Torque)])];

        let errors = errors_of(actuator_merge(
            Some(&section(&["drive"])),
            &car_body(),
            &nodes,
        ));

        assert!(
            matches!(
                errors.as_slice(),
                [PipelineAssemblyError::ActuatorNotOnBody { member, actuator, body }]
                    if member == "drive" && actuator == "traction" && body == "car"
            ),
            "got {errors:?}"
        );
    }

    #[test]
    fn a_setpoint_kind_the_actuator_does_not_accept_is_rejected() {
        // A torque into a position-driven steer.
        let nodes = vec![allocator("steer", &[("steer_axle", SetpointKind::Torque)])];

        let errors = errors_of(actuator_merge(
            Some(&section(&["steer"])),
            &car_body(),
            &nodes,
        ));

        assert!(
            matches!(
                errors.as_slice(),
                [PipelineAssemblyError::ActuatorKindMismatch {
                    member,
                    actuator,
                    writes: SetpointKind::Torque,
                    accepts: SetpointKind::Position,
                }] if member == "steer" && actuator == "steer_axle"
            ),
            "got {errors:?}"
        );
    }

    #[test]
    fn the_drives_of_a_node_outside_the_seam_are_not_checked() {
        // `shadow` names an actuator the body lacks, but drives nothing.
        let nodes = vec![
            car_drive(),
            car_steer(),
            allocator("shadow", &[("traction", SetpointKind::Velocity)]),
        ];

        let merge = actuator_merge(Some(&section(&["drive", "steer"])), &car_body(), &nodes)
            .expect("a non-member's drives are not checked");

        assert!(merge.is_some());
    }
}
