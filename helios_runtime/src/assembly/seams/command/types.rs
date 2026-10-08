//! [`CommandTypes`], the registry extension listing the command types a fold
//! may name, and the built-in types the default registry adds.

use super::super::members::SeamType;

use crate::assembly::registry::AutonomyRegistry;
use crate::nodes::combinators::Sum;
use crate::pipeline::node::PipelineNode;
use crate::port::InternalChannel;

use helios_core::control::commands::{BodyTwist, DriveForce, SteerAngle};

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt::Display;
use std::ops::Add;

/// The `type` a fold states to sum body-frame velocity commands.
pub(crate) const BODY_TWIST_TYPE: &str = "BodyTwist";

/// The `type` a fold states to sum longitudinal drive forces.
pub(crate) const DRIVE_FORCE_TYPE: &str = "DriveForce";

/// The `type` a fold states to sum steer angles.
pub(crate) const STEER_ANGLE_TYPE: &str = "SteerAngle";

/// Builds a fold's `Sum` from its name and its required and optional inputs.
type BuildSum = fn(&str, Vec<InternalChannel>, Vec<InternalChannel>) -> Box<dyn PipelineNode>;

/// The command types a `[command.<fold>]` table may name in its `type` key:
/// type name → what the seam needs to fold it.
///
/// A registry extension. The default registry adds the built-in types; an
/// outside crate adds its own through
/// `registry.extension_mut::<CommandTypes>().register::<T>(name)`.
#[derive(Default)]
pub struct CommandTypes {
    // Sorted, so an unknown-type error lists the registered types in a stable
    // order.
    types: BTreeMap<String, CommandType>,
}

impl CommandTypes {
    /// The entry registered under `name`.
    pub(crate) fn get(&self, name: &str) -> Option<&CommandType> {
        self.types.get(name)
    }

    /// Every registered type name, sorted.
    pub(crate) fn names(&self) -> Vec<String> {
        self.types.keys().cloned().collect()
    }

    /// Registers command type `T` under `name`.
    ///
    /// A fold of this type sums its members' `T` outputs, so `T` must be
    /// addable. Any number of folds may share a type.
    ///
    /// Fails if `name` is already registered; the first registration is kept.
    pub fn register<T>(&mut self, name: impl Into<String>) -> Result<(), DuplicateCommandType>
    where
        T: Send + Sync + Clone + Add<Output = T> + 'static,
    {
        let name = name.into();
        if self.types.contains_key(&name) {
            return Err(DuplicateCommandType { name });
        }

        self.types.insert(name, CommandType::of::<T>());

        Ok(())
    }
}

/// A command type was registered twice.
#[derive(Debug)]
pub struct DuplicateCommandType {
    /// The type name that was already taken.
    pub name: String,
}

impl Display for DuplicateCommandType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "command type '{}' is already registered", self.name)
    }
}

impl Error for DuplicateCommandType {}

/// One entry in [`CommandTypes`]: what a member must write to
/// join a fold of this type, and how to build that fold's `Sum`.
pub(crate) struct CommandType {
    pub(super) ty: SeamType,
    pub(super) build_sum: BuildSum,
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
    let types = registry.extension_mut::<CommandTypes>();
    types
        .register::<BodyTwist>(BODY_TWIST_TYPE)
        .expect("BodyTwist is registered once, by the default registry");
    types
        .register::<DriveForce>(DRIVE_FORCE_TYPE)
        .expect("DriveForce is registered once, by the default registry");
    types
        .register::<SteerAngle>(STEER_ANGLE_TYPE)
        .expect("SteerAngle is registered once, by the default registry");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registering_a_command_type_twice_is_rejected() {
        let mut registry = AutonomyRegistry::default();
        let err = registry
            .extension_mut::<CommandTypes>()
            .register::<f64>(DRIVE_FORCE_TYPE)
            .expect_err("DriveForce is already a built-in");
        assert_eq!(err.name, DRIVE_FORCE_TYPE);
        assert_eq!(
            err.to_string(),
            "command type 'DriveForce' is already registered"
        );
    }
}
