use crate::agents::vehicles::mounts::MountFrame;
use crate::config::structs::TopologyConfig;
use crate::core::transforms::TrackedFrame;
use crate::registry::contexts::TopologyBuildContext;

use helios_core::frames::transforms::Convention;

use avian3d::prelude::*;
use bevy::prelude::*;

/// Builds the general single-body topology: one dynamic rigid body at the start
/// pose tagged with its FLU body frame, plus one child frame entity per declared
/// mount. Each mount frame carries its FLU pose and a `MountFrame` name; the
/// visual and plant axes resolve those names to place parts and assign wheel
/// roles. A body that declares no mounts (the L0 shim) spawns none.
#[allow(irrefutable_let_patterns)] // single variant today; a second makes the else-arm live
pub fn build_rigid_body_with_mount(ctx: &mut TopologyBuildContext) -> Result<(), String> {
    let TopologyConfig::RigidBodyWithMount { mass, mounts } = ctx.config else {
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

    for mount in mounts {
        let frame = ctx
            .commands
            .spawn((
                mount.pose.to_bevy_local_transform(),
                TrackedFrame(Convention::Flu),
                MountFrame(mount.name.clone()),
            ))
            .id();

        ctx.commands.entity(ctx.entity).add_child(frame);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::config::structs::{MountConfig, Pose};

    use bevy::ecs::system::SystemState;
    use nalgebra::{UnitQuaternion, Vector3};

    fn build(config: &TopologyConfig, world: &mut World) -> Entity {
        let entity = world.spawn_empty().id();
        let mut state: SystemState<Commands> = SystemState::new(world);
        {
            let mut commands = state.get_mut(world).unwrap();
            let mut ctx = TopologyBuildContext {
                entity,
                commands: &mut commands,
                config,
                start_transform: Transform::IDENTITY,
            };
            build_rigid_body_with_mount(&mut ctx).unwrap();
        }
        state.apply(world);
        entity
    }

    fn mount(name: &str, translation: Vector3<f64>) -> MountConfig {
        MountConfig {
            name: name.to_string(),
            pose: Pose {
                translation,
                rotation: UnitQuaternion::identity(),
            },
        }
    }

    // Spawning posed, named child frames is an ECS effect, so this is a Tier-2
    // minimal-World test. It asserts each mount becomes a child of the body,
    // carrying its name and the FLU→Bevy image of its declared pose.
    #[test]
    fn builds_a_named_child_frame_per_mount() {
        let front_left = mount("wheel_fl", Vector3::new(1.3, 0.75, 0.0));
        let rear_right = mount("wheel_rr", Vector3::new(-1.3, -0.75, 0.0));
        let expected_fl = front_left.pose.to_bevy_local_transform().translation;
        let expected_rr = rear_right.pose.to_bevy_local_transform().translation;

        let config = TopologyConfig::RigidBodyWithMount {
            mass: 1500.0,
            mounts: vec![front_left, rear_right],
        };

        let mut world = World::new();
        let entity = build(&config, &mut world);

        let children = world
            .entity(entity)
            .get::<Children>()
            .expect("the body has mount children");
        assert_eq!(children.len(), 2);

        // Requiring TrackedFrame in the query also asserts every mount frame is
        // FLU-tagged: a frame missing it would not match and the count would fail.
        let mut placed = std::collections::HashMap::new();
        let mut query = world.query::<(&MountFrame, &Transform, &TrackedFrame)>();
        for (mount, transform, _flu) in query.iter(&world) {
            placed.insert(mount.0.clone(), transform.translation);
        }
        assert_eq!(placed.len(), 2);

        let close = |a: Vec3, b: Vec3| (a - b).length() < 1e-5;
        assert!(close(placed["wheel_fl"], expected_fl));
        assert!(close(placed["wheel_rr"], expected_rr));
    }

    // A body with no mounts spawns no child frames — the L0 car today.
    #[test]
    fn builds_no_frames_when_no_mounts() {
        let config = TopologyConfig::RigidBodyWithMount {
            mass: 1500.0,
            mounts: vec![],
        };

        let mut world = World::new();
        let entity = build(&config, &mut world);

        assert!(world.entity(entity).get::<Children>().is_none());
    }
}
