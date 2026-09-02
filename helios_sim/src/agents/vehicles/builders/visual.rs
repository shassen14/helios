use crate::config::structs::{MountConfig, VisualConfig};
use crate::registry::contexts::VisualBuildContext;

use bevy::prelude::*;

use std::f32::consts::FRAC_PI_2;

/// Builds the visual axis for a wheeled vehicle out of primitives: one chassis
/// box at the body origin, plus one wheel cylinder at every mount frame. Purely
/// cosmetic — the solver never reads these meshes. Collision geometry and the
/// plant's contact rays are separate axes with their own dims, so a drawn wheel
/// can differ from the collider and from the ray hub on purpose. Dims come from
/// config; wheel *positions* come from the same mount list the plant reads, so a
/// wheel renders exactly where its suspension acts. The wheel shape is authored
/// once and its handle cloned per mount. A body with no mounts draws its chassis
/// and no wheels.
#[allow(irrefutable_let_patterns)] // single variant today; a second makes the else-arm live
pub fn build_wheeled_primitives(ctx: &mut VisualBuildContext) -> Result<(), String> {
    let VisualConfig::WheeledPrimitives { chassis, wheel } = ctx.config else {
        return Err(format!(
            "WheeledPrimitives builder received a `{}` visual",
            ctx.config.kind_str(),
        ));
    };

    // Chassis: a box centred on the body origin, in the body's construction frame.
    let chassis_mesh = ctx.meshes.add(Cuboid::new(chassis.x, chassis.y, chassis.z));
    let chassis_material = ctx.materials.add(material(chassis.color));
    let chassis = ctx
        .commands
        .spawn((
            Mesh3d(chassis_mesh),
            MeshMaterial3d(chassis_material),
            Transform::default(),
        ))
        .id();
    ctx.commands.entity(ctx.entity).add_child(chassis);

    // Wheels: one shared cylinder shape, stamped below each mount's body-local pose.
    let wheel_mesh = ctx.meshes.add(Cylinder::new(wheel.radius, wheel.width));
    let wheel_material = ctx.materials.add(material(wheel.color));

    for mount in ctx.mounts {
        let entity = ctx
            .commands
            .spawn((
                Mesh3d(wheel_mesh.clone()),
                MeshMaterial3d(wheel_material.clone()),
                wheel_transform(mount, wheel.drop),
            ))
            .id();
        ctx.commands.entity(ctx.entity).add_child(entity);
    }

    Ok(())
}

/// Places a wheel below its mount: the mount pose lowered by `drop` (the strut top
/// is the mount; the hub hangs beneath it), then composed with a quarter-turn
/// about Z that lays the cylinder on its side so its axis runs along the vehicle's
/// lateral (left–right) axis. A bare [`Cylinder`] spins about +Y, which would
/// stand the wheel upright. The drop is subtracted in body FLU (up = +z) *before*
/// the frame conversion, so all axis handling stays inside `to_bevy_local_transform`.
fn wheel_transform(mount: &MountConfig, drop: f32) -> Transform {
    let mut pose = mount.pose;
    pose.translation.z -= drop as f64;
    let mut t = pose.to_bevy_local_transform();
    t.rotation *= Quat::from_rotation_z(FRAC_PI_2);
    t
}

/// A [`StandardMaterial`] of the given linear sRGBA. An alpha below 1.0 selects
/// `AlphaMode::Blend` so the surface renders translucent — opaque materials ignore
/// the alpha channel, which would hide anything drawn behind the chassis.
fn material(color: [f32; 4]) -> StandardMaterial {
    let [r, g, b, a] = color;
    let alpha_mode = if a < 1.0 {
        AlphaMode::Blend
    } else {
        AlphaMode::Opaque
    };
    StandardMaterial {
        base_color: Color::srgba(r, g, b, a),
        alpha_mode,
        ..default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::config::structs::{BoxVisual, Pose, WheelVisual};

    use bevy::ecs::system::SystemState;
    use nalgebra::{UnitQuaternion, Vector3};

    const OPAQUE_BLACK: [f32; 4] = [0.0, 0.0, 0.0, 1.0];
    const WHEEL_DROP: f32 = 0.3;

    fn wheeled(radius: f32) -> VisualConfig {
        VisualConfig::WheeledPrimitives {
            chassis: BoxVisual {
                x: 1.8,
                y: 0.8,
                z: 4.0,
                color: OPAQUE_BLACK,
            },
            wheel: WheelVisual {
                radius,
                width: 0.2,
                drop: WHEEL_DROP,
                color: OPAQUE_BLACK,
            },
        }
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

    // Building meshes needs the two `Assets` stores present; a bare `World` with
    // them inserted is enough for `add`, no `AssetPlugin` boot. Returns the world
    // and the built body entity for the caller to assert on.
    fn build(config: &VisualConfig, mounts: &[MountConfig]) -> (World, Entity) {
        let mut world = World::new();
        world.insert_resource(Assets::<Mesh>::default());
        world.insert_resource(Assets::<StandardMaterial>::default());
        let entity = world.spawn_empty().id();

        let mut state: SystemState<(
            Commands,
            ResMut<Assets<Mesh>>,
            ResMut<Assets<StandardMaterial>>,
        )> = SystemState::new(&mut world);
        {
            let (mut commands, mut meshes, mut materials) = state.get_mut(&mut world).unwrap();
            let mut ctx = VisualBuildContext {
                entity,
                commands: &mut commands,
                config,
                mounts,
                meshes: &mut meshes,
                materials: &mut materials,
            };
            build_wheeled_primitives(&mut ctx).unwrap();
        }
        state.apply(&mut world);
        (world, entity)
    }

    fn children(world: &World, entity: Entity) -> Vec<Entity> {
        world
            .entity(entity)
            .get::<Children>()
            .map(|c| c.iter().collect())
            .unwrap_or_default()
    }

    // The contract is an ECS effect — spawn a chassis child plus one wheel child
    // per mount, each carrying a mesh — so this is a Tier-2 minimal-World test. Four
    // mounts must yield five mesh children (chassis + four wheels).
    #[test]
    fn builds_chassis_plus_one_wheel_per_mount() {
        let mounts = [
            mount("fl", Vector3::new(1.25, 0.75, -0.1)),
            mount("fr", Vector3::new(1.25, -0.75, -0.1)),
            mount("rl", Vector3::new(-1.25, 0.75, -0.1)),
            mount("rr", Vector3::new(-1.25, -0.75, -0.1)),
        ];

        let (world, body) = build(&wheeled(0.3), &mounts);
        let kids = children(&world, body);

        assert_eq!(
            kids.len(),
            mounts.len() + 1,
            "chassis + one wheel per mount"
        );
        // Requiring Mesh3d also asserts every child is renderable — a stray
        // non-mesh child would fail the count.
        let with_mesh = kids
            .iter()
            .filter(|&&c| world.entity(c).get::<Mesh3d>().is_some())
            .count();
        assert_eq!(with_mesh, kids.len(), "every child carries a mesh");
    }

    // A mountless body still draws its chassis — one child, no wheels.
    #[test]
    fn builds_chassis_only_when_no_mounts() {
        let (world, body) = build(&wheeled(0.3), &[]);
        assert_eq!(children(&world, body).len(), 1);
    }

    // A wheel is placed `drop` below its mount's body-local pose, then laid on its
    // side. This pins the placement (mount pose lowered by `drop`) and the
    // cylinder-lay rotation independently of the helper: expected is re-derived from
    // the public pose conversion and the known quarter-turn.
    #[test]
    fn wheel_sits_below_its_mount_laid_on_its_side() {
        let m = mount("fl", Vector3::new(1.25, 0.75, -0.1));
        let mount_only = m.pose.to_bevy_local_transform();

        // Expected: the mount pose lowered by `drop` in FLU up (+z) before conversion.
        let mut dropped = m.pose;
        dropped.translation.z -= WHEEL_DROP as f64;
        let expected = dropped.to_bevy_local_transform();

        let (world, body) = build(&wheeled(0.3), std::slice::from_ref(&m));
        // Two children: the chassis at the origin and the one wheel. Pick the wheel
        // as the child whose translation is not the origin.
        let wheel = children(&world, body)
            .into_iter()
            .map(|c| *world.entity(c).get::<Transform>().expect("child transform"))
            .find(|t| t.translation != Vec3::ZERO)
            .expect("a wheel child offset from the origin");

        let close = |a: Vec3, b: Vec3| (a - b).length() < 1e-5;
        assert!(
            close(wheel.translation, expected.translation),
            "placed below the mount"
        );
        // The drop lowers the hub: FLU up (+z) maps to Bevy up (+y), so the wheel
        // sits below where the bare mount would place it.
        assert!(
            wheel.translation.y < mount_only.translation.y,
            "hangs below the mount"
        );
        // Equal rotations up to the quaternion double cover: |dot| ≈ 1. Compared by
        // dot, not `angle_between` (whose acos is float-noisy near identity).
        let expected_rot = expected.rotation * Quat::from_rotation_z(FRAC_PI_2);
        assert!(
            wheel.rotation.dot(expected_rot).abs() > 1.0 - 1e-6,
            "laid on its side"
        );
    }
}
