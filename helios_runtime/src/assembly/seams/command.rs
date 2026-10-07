//! The command seam: turns each `[command.<fold>]` table into a `Sum` that
//! writes the fold's command for an allocator to read.
//!
//! Each member writes its command on a channel of its own. For every fold, the
//! pass looks up the fold's `type` among the registered command types, finds
//! each member's output of that type among the built nodes, and adds a `Sum`
//! named after the fold, writing `T @ <fold name>`. Required members are the
//! sum's required inputs, so it publishes only once they all have; optional
//! members are folded in when present. A lone member is a one-input sum, which
//! forwards it, so the shape is the same for any number of members.
//!
//! A command type is an entry in the registry's table, not a variant here, so
//! any number of folds may share one and a type defined outside this crate is
//! one registration away.

use super::members::{duplicates, member_outputs, SeamType};

use crate::assembly::error::PipelineAssemblyError;
use crate::assembly::registry::AutonomyRegistry;
use crate::config::CommandFoldConfig;
use crate::nodes::combinators::Sum;
use crate::pipeline::node::PipelineNode;
use crate::port::InternalChannel;

use helios_core::control::commands::{BodyTwist, DriveForce, SteerAngle};

use std::collections::BTreeMap;
use std::ops::Add;

/// The `type` a fold states to sum body-frame velocity commands.
pub(crate) const BODY_TWIST_TYPE: &str = "BodyTwist";

/// The `type` a fold states to sum longitudinal drive forces.
pub(crate) const DRIVE_FORCE_TYPE: &str = "DriveForce";

/// The `type` a fold states to sum steer angles.
pub(crate) const STEER_ANGLE_TYPE: &str = "SteerAngle";

/// Builds a fold's `Sum` from its name and its required and optional inputs.
type BuildSum = fn(&str, Vec<InternalChannel>, Vec<InternalChannel>) -> Box<dyn PipelineNode>;

/// One entry in the registry's command-type table: what a member must write to
/// join a fold of this type, and how to build that fold's `Sum`.
pub(crate) struct CommandType {
    ty: SeamType,
    build_sum: BuildSum,
}

impl CommandType {
    /// The entry for command type `T`.
    pub(crate) fn of<T>() -> Self
    where
        T: Send + Sync + Clone + Add<Output = T> + 'static,
    {
        Self {
            ty: SeamType::of::<T>(),
            build_sum: build_sum::<T>,
        }
    }
}

/// A `Sum` named `fold` over `required` and `optional`, writing `T @ <fold>`.
fn build_sum<T>(
    fold: &str,
    required: Vec<InternalChannel>,
    optional: Vec<InternalChannel>,
) -> Box<dyn PipelineNode>
where
    T: Send + Sync + Clone + Add<Output = T> + 'static,
{
    Box::new(Sum::<T>::new(
        fold,
        required,
        optional,
        InternalChannel::named::<T>(fold),
    ))
}

/// Adds the built-in command types to `registry`.
pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry
        .register_command_type::<BodyTwist>(BODY_TWIST_TYPE)
        .expect("BodyTwist is registered once, by the default registry");
    registry
        .register_command_type::<DriveForce>(DRIVE_FORCE_TYPE)
        .expect("DriveForce is registered once, by the default registry");
    registry
        .register_command_type::<SteerAngle>(STEER_ANGLE_TYPE)
        .expect("SteerAngle is registered once, by the default registry");
}

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
    let Some(command_type) = registry.command_type(&fold.type_name) else {
        return Err(vec![PipelineAssemblyError::UnknownCommandType {
            fold: fold_name.to_string(),
            type_name: fold.type_name.clone(),
            registered: registry.command_type_names(),
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
    use super::*;

    use crate::assembly::test_stub::stub;
    use crate::port::ChannelKey;

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
            .register_command_type::<Thrust>("Thrust")
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
