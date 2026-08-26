use crate::config::structs::TopologyConfig;
use crate::core::transforms::TrackedFrame;
use crate::registry::contexts::TopologyBuildContext;

use helios_core::frames::transforms::Convention;

use avian3d::prelude::*;
use bevy::prelude::*;

/// Builds the general single-body topology: one dynamic rigid body at the start
/// pose, tagged with its FLU body frame. Future mount frames (wheels, sensors)
/// attach as child frames here; the L0 shim reads none, so this stays minimal.
#[allow(irrefutable_let_patterns)] // single variant today; a second makes the else-arm live
pub fn build_rigid_body_with_mount(ctx: &mut TopologyBuildContext) -> Result<(), String> {
    let TopologyConfig::RigidBodyWithMount { mass } = ctx.config else {
        return Err(format!(
            "RigidBodyWithMount builder received a `{}` topology",
            ctx.config.kind_str(),
        ));
    };

    ctx.commands.entity(ctx.entity).insert((
        ctx.start_transform,
        RigidBody::Dynamic,
        Mass(*mass),
        TrackedFrame(Convention::Flu),
        SleepingDisabled,
        LinearVelocity::default(),
        AngularVelocity::default(),
        InheritedVisibility::VISIBLE,
    ));

    Ok(())
}
