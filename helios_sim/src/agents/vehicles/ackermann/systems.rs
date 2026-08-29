use super::AckermannAssets;
use crate::agents::vehicles::builders::L0ShimPlantComponent;
use crate::agents::vehicles::components::ActuationModelComponent;
use crate::core::transforms::FluVector;
use crate::prelude::*;

use avian3d::prelude::*;

pub(super) fn setup_ackermann_assets(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.insert_resource(AckermannAssets {
        body_mesh: meshes.add(Cuboid::new(1.8, 0.8, 4.0)),
        body_material: materials.add(Color::srgba(0.7, 0.2, 0.2, 0.2)),
        wheel_mesh: meshes.add(Cylinder::new(0.3, 0.2)),
        wheel_material: materials.add(Color::srgb(0.1, 0.1, 0.1)),
    });
}

/// Residual cosmetic visual for the L0 car: a chassis mesh plus four wheel meshes,
/// spawned as children of the built body. Gated on `L0ShimPlantComponent` so only
/// L0 cars get car geometry. The dims are still hardcoded (duplicating `[collision]`);
/// this is the deferred visual axis, replaced wholesale by a visual builder that
/// reads mount frames and an asset config.
pub(super) fn attach_ackermann_visual(
    mut commands: Commands,
    query: Query<(Entity, &Name), With<L0ShimPlantComponent>>,
    assets: Res<AckermannAssets>,
) {
    for (entity, name) in &query {
        commands.entity(entity).with_children(|parent| {
            parent.spawn((
                Mesh3d(assets.body_mesh.clone()),
                MeshMaterial3d(assets.body_material.clone()),
                Name::new(format!("{}_Body", name)),
            ));

            let wheel_positions = [
                (Vec3::new(1.0, -0.2, 1.7), format!("{}_FR_Wheel", name)),
                (Vec3::new(-1.0, -0.2, 1.7), format!("{}_FL_Wheel", name)),
                (Vec3::new(1.0, -0.2, -1.7), format!("{}_RR_Wheel", name)),
                (Vec3::new(-1.0, -0.2, -1.7), format!("{}_RL_Wheel", name)),
            ];

            for (pos, wheel_name) in wheel_positions {
                parent.spawn((
                    Mesh3d(assets.wheel_mesh.clone()),
                    MeshMaterial3d(assets.wheel_material.clone()),
                    Transform::from_translation(pos)
                        .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
                    Name::new(wheel_name),
                ));
            }
        });
    }
}

/// Applies the pipeline's latest actuator command to physics.
///
/// `resolve` first enforces the body's actuator contract — clamping to each
/// actuator's limit, applying its sign convention, and substituting the
/// fail-safe value for any missing, wrong-kind, or non-finite setpoint — so the
/// command handed to the plant is always safe to apply. The [`L0ShimPlant`] fold
/// then turns that command into a single body-frame wrench (the L0 arcade model:
/// one rigid body, no articulated wheel joints). This system is the orchestration
/// around those two core calls: it rotates the FLU wrench into world space and
/// applies it as a `ConstantForce`/`ConstantTorque`, warning on any setpoint the
/// shim reports it cannot apply. Passive damping is left to Avian3D.
///
/// [`L0ShimPlant`]: helios_core::plant::L0ShimPlant
pub(super) fn drive_ackermann_cars(
    mut commands: Commands,
    mut query: Query<(
        Entity,
        &Transform,
        &L0ShimPlantComponent,
        &ActuationModelComponent,
        Option<&ActuatorCommandComponent>,
    )>,
) {
    for (entity, transform, plant, model, cmd_opt) in &mut query {
        let Some(cmd) = cmd_opt else {
            continue;
        };

        let resolved = model.0.resolve(&cmd.0);
        let folded = plant.0.fold(&resolved);

        for actuator in folded.unsupported() {
            warn!(
                ?actuator,
                "L0 shim received a force/torque setpoint it cannot apply; ignoring"
            );
        }

        // Rotate the FLU force and torque into Bevy world space before handing
        // them to Avian; conversions stay confined to `FluVector`. `into_inner`
        // drops the core FLU frame tag onto the sim's FLU newtype — same axis
        // order, no swap — and `From<FluVector>` performs the FLU → Bevy rotation.
        let wrench = folded.wrench();
        let force_bevy_local = Vec3::from(FluVector(wrench.force().into_inner()));
        let force_world = transform.rotation * force_bevy_local;

        let torque_bevy_local = Vec3::from(FluVector(wrench.torque().into_inner()));
        let torque_world = transform.rotation * torque_bevy_local;

        commands.entity(entity).insert((
            ConstantForce::new(force_world.x, force_world.y, force_world.z),
            ConstantTorque::new(torque_world.x, torque_world.y, torque_world.z),
        ));
    }
}
