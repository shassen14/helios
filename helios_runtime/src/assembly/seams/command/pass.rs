//! The `[command]` seam's pass: one `Sum` per `[command.<fold>]` table.

use super::super::members::{duplicates, member_outputs};
use super::{CommandFoldConfig, CommandTypes};

use crate::assembly::error::PipelineAssemblyError;
use crate::assembly::registry::AutonomyRegistry;
use crate::pipeline::node::PipelineNode;

use std::collections::BTreeMap;

/// Builds one `Sum` per fold in `folds`, resolving each fold's type in
/// `registry` and each member among `nodes`.
///
/// Fails with one error per fold whose type is unknown or that names no
/// members, and one per member that is unknown, named twice in its fold, or
/// lacks exactly one output of the fold's type.
pub(in crate::assembly) fn command_sums(
    folds: &BTreeMap<String, CommandFoldConfig>,
    registry: &AutonomyRegistry,
    nodes: &[Box<dyn PipelineNode>],
) -> Result<Vec<Box<dyn PipelineNode>>, Vec<PipelineAssemblyError>> {
    let mut sums = vec![];
    let mut errors = vec![];

    for (fold_name, fold) in folds {
        match command_sum(fold_name, fold, registry, nodes) {
            Ok(sum) => sums.push(sum),
            Err(fold_errors) => errors.extend(fold_errors),
        }
    }

    if errors.is_empty() {
        Ok(sums)
    } else {
        Err(errors)
    }
}

/// The `Sum` for fold `fold_name`.
fn command_sum(
    fold_name: &str,
    fold: &CommandFoldConfig,
    registry: &AutonomyRegistry,
    nodes: &[Box<dyn PipelineNode>],
) -> Result<Box<dyn PipelineNode>, Vec<PipelineAssemblyError>> {
    let types = registry.extension::<CommandTypes>();
    let Some(command_type) = types.and_then(|types| types.get(&fold.type_name)) else {
        return Err(vec![PipelineAssemblyError::UnknownCommandType {
            fold: fold_name.to_string(),
            type_name: fold.type_name.clone(),
            registered: types.map(CommandTypes::names).unwrap_or_default(),
        }]);
    };

    let seam = format!("command.{fold_name}");
    if fold.required.is_empty() && fold.optional.is_empty() {
        return Err(vec![PipelineAssemblyError::EmptySeam { seam }]);
    }

    let ty = command_type.ty;
    let mut errors = duplicates(&seam, fold.required.iter().chain(&fold.optional));
    let required = member_outputs(&seam, ty, &fold.required, nodes, &mut errors);
    let optional = member_outputs(&seam, ty, &fold.optional, nodes, &mut errors);
    if !errors.is_empty() {
        return Err(errors);
    }

    Ok((command_type.build_sum)(fold_name, required, optional))
}

#[cfg(test)]
mod tests {
    use super::super::types::{BODY_TWIST_TYPE, DRIVE_FORCE_TYPE, STEER_ANGLE_TYPE};
    use super::*;

    use crate::assembly::test_stub::stub;
    use crate::port::{ChannelKey, InternalChannel};

    use helios_core::control::commands::{DriveForce, SteerAngle};

    use std::ops::Add;

    /// A node writing a command of type `T` on a channel named after itself, as
    /// every controller does.
    fn controller<T: 'static>(name: &str) -> Box<dyn PipelineNode> {
        stub(name, vec![InternalChannel::named::<T>(name)])
    }

    fn key<T: 'static>(name: &str) -> ChannelKey {
        InternalChannel::named::<T>(name).into()
    }

    fn fold(type_name: &str, required: &[&str], optional: &[&str]) -> CommandFoldConfig {
        let names = |names: &[&str]| names.iter().map(|name| name.to_string()).collect();
        CommandFoldConfig {
            type_name: type_name.to_string(),
            required: names(required),
            optional: names(optional),
        }
    }

    fn folds<const N: usize>(
        entries: [(&str, CommandFoldConfig); N],
    ) -> BTreeMap<String, CommandFoldConfig> {
        entries
            .into_iter()
            .map(|(name, fold)| (name.to_string(), fold))
            .collect()
    }

    /// The raycast car's control nodes: a drive feedback and feedforward leg,
    /// and a steer feedforward.
    fn car_controllers() -> Vec<Box<dyn PipelineNode>> {
        vec![
            controller::<DriveForce>("drive_speed"),
            controller::<DriveForce>("drive_ff"),
            controller::<SteerAngle>("steer_ff"),
        ]
    }

    fn build(
        folds: &BTreeMap<String, CommandFoldConfig>,
        nodes: &[Box<dyn PipelineNode>],
    ) -> Result<Vec<Box<dyn PipelineNode>>, Vec<PipelineAssemblyError>> {
        command_sums(folds, &AutonomyRegistry::default(), nodes)
    }

    fn errors_of(
        result: Result<Vec<Box<dyn PipelineNode>>, Vec<PipelineAssemblyError>>,
    ) -> Vec<PipelineAssemblyError> {
        match result {
            Ok(_) => panic!("the seam must not build"),
            Err(errors) => errors,
        }
    }

    #[test]
    fn no_folds_add_no_sum() {
        let sums = build(&BTreeMap::new(), &car_controllers()).expect("no folds is not an error");

        assert!(sums.is_empty());
    }

    #[test]
    fn each_fold_is_a_sum_named_after_it_with_its_members_by_role() {
        let config = folds([
            (
                "drive_cmd",
                fold(DRIVE_FORCE_TYPE, &["drive_speed"], &["drive_ff"]),
            ),
            ("steer_cmd", fold(STEER_ANGLE_TYPE, &[], &["steer_ff"])),
        ]);

        let sums = build(&config, &car_controllers()).expect("both folds build");

        let [drive, steer] = sums.as_slice() else {
            panic!("expected two sums, got {}", sums.len());
        };
        assert_eq!(drive.name(), "drive_cmd");
        let drive = drive.port_descriptor();
        assert_eq!(
            drive.required_inputs().cloned().collect::<Vec<_>>(),
            vec![key::<DriveForce>("drive_speed")]
        );
        assert_eq!(
            drive.optional_inputs().cloned().collect::<Vec<_>>(),
            vec![key::<DriveForce>("drive_ff")]
        );
        assert_eq!(drive.outputs(), [key::<DriveForce>("drive_cmd")].as_slice());

        assert_eq!(steer.name(), "steer_cmd");
        assert_eq!(steer.port_descriptor().required_inputs().count(), 0);
        assert_eq!(
            steer.port_descriptor().outputs(),
            [key::<SteerAngle>("steer_cmd")].as_slice()
        );
    }

    #[test]
    fn two_folds_may_share_a_type() {
        // A skid-steer: one drive fold per side.
        let nodes = vec![
            controller::<DriveForce>("left_speed"),
            controller::<DriveForce>("right_speed"),
        ];
        let config = folds([
            ("left_cmd", fold(DRIVE_FORCE_TYPE, &["left_speed"], &[])),
            ("right_cmd", fold(DRIVE_FORCE_TYPE, &["right_speed"], &[])),
        ]);

        let sums = build(&config, &nodes).expect("two folds of one type build");

        let outputs: Vec<ChannelKey> = sums
            .iter()
            .flat_map(|sum| sum.port_descriptor().outputs().to_vec())
            .collect();
        assert_eq!(
            outputs,
            vec![
                key::<DriveForce>("left_cmd"),
                key::<DriveForce>("right_cmd")
            ]
        );
    }

    #[test]
    fn a_type_registered_from_outside_folds_like_a_built_in() {
        // Stands in for a command type a downstream crate defines.
        #[derive(Clone)]
        struct Thrust(f64);

        impl Add for Thrust {
            type Output = Thrust;

            fn add(self, other: Thrust) -> Thrust {
                Thrust(self.0 + other.0)
            }
        }

        let mut registry = AutonomyRegistry::default();
        registry
            .extension_mut::<CommandTypes>()
            .register::<Thrust>("Thrust")
            .expect("Thrust is new");
        let nodes = vec![controller::<Thrust>("altitude")];
        let config = folds([("lift", fold("Thrust", &["altitude"], &[]))]);

        let sums = command_sums(&config, &registry, &nodes).expect("a registered type folds");

        assert_eq!(
            sums[0].port_descriptor().outputs(),
            [key::<Thrust>("lift")].as_slice()
        );
    }

    #[test]
    fn a_registry_without_command_types_lists_none() {
        let registry = AutonomyRegistry::empty();
        let config = folds([("drive_cmd", fold(DRIVE_FORCE_TYPE, &["drive_speed"], &[]))]);

        let errors = errors_of(command_sums(&config, &registry, &car_controllers()));

        assert!(
            matches!(
                errors.as_slice(),
                [PipelineAssemblyError::UnknownCommandType { registered, .. }]
                    if registered.is_empty()
            ),
            "got {errors:?}"
        );
    }

    #[test]
    fn an_unknown_type_lists_the_registered_ones() {
        let config = folds([("drive_cmd", fold("WheelTorque", &["drive_speed"], &[]))]);

        let errors = errors_of(build(&config, &car_controllers()));

        assert!(
            matches!(
                errors.as_slice(),
                [PipelineAssemblyError::UnknownCommandType { fold, type_name, registered }]
                    if fold == "drive_cmd"
                        && type_name == "WheelTorque"
                        && registered == &[BODY_TWIST_TYPE, DRIVE_FORCE_TYPE, STEER_ANGLE_TYPE]
            ),
            "got {errors:?}"
        );
    }

    #[test]
    fn a_member_of_the_wrong_type_is_named() {
        // The steer feedforward writes a `SteerAngle`, so it can't join a
        // `DriveForce` fold.
        let config = folds([(
            "drive_cmd",
            fold(DRIVE_FORCE_TYPE, &["drive_speed"], &["steer_ff"]),
        )]);

        let errors = errors_of(build(&config, &car_controllers()));

        assert!(
            matches!(
                errors.as_slice(),
                [PipelineAssemblyError::SeamMemberOutputMismatch {
                    seam,
                    member,
                    matching: 0,
                    ..
                }] if seam == "command.drive_cmd" && member == "steer_ff"
            ),
            "got {errors:?}"
        );
    }

    #[test]
    fn a_member_that_is_not_a_node_is_named() {
        let config = folds([("steer_cmd", fold(STEER_ANGLE_TYPE, &["steer_pid"], &[]))]);

        let errors = errors_of(build(&config, &car_controllers()));

        assert!(
            matches!(
                errors.as_slice(),
                [PipelineAssemblyError::UnknownSeamMember { seam, member }]
                    if seam == "command.steer_cmd" && member == "steer_pid"
            ),
            "got {errors:?}"
        );
    }

    #[test]
    fn a_member_both_required_and_optional_is_rejected() {
        let config = folds([(
            "drive_cmd",
            fold(DRIVE_FORCE_TYPE, &["drive_speed"], &["drive_speed"]),
        )]);

        let errors = errors_of(build(&config, &car_controllers()));

        assert!(
            matches!(
                errors.as_slice(),
                [PipelineAssemblyError::DuplicateSeamMember { member, .. }]
                    if member == "drive_speed"
            ),
            "got {errors:?}"
        );
    }

    #[test]
    fn a_fold_with_no_members_is_rejected() {
        let config = folds([("drive_cmd", fold(DRIVE_FORCE_TYPE, &[], &[]))]);

        let errors = errors_of(build(&config, &car_controllers()));

        assert!(
            matches!(
                errors.as_slice(),
                [PipelineAssemblyError::EmptySeam { seam }] if seam == "command.drive_cmd"
            ),
            "got {errors:?}"
        );
    }

    #[test]
    fn every_bad_fold_is_reported_at_once() {
        let config = folds([
            ("drive_cmd", fold(DRIVE_FORCE_TYPE, &["missing_drive"], &[])),
            ("steer_cmd", fold(STEER_ANGLE_TYPE, &["missing_steer"], &[])),
        ]);

        let errors = errors_of(build(&config, &car_controllers()));

        assert_eq!(errors.len(), 2, "got {errors:?}");
    }
}
