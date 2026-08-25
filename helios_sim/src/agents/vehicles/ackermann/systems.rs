use super::{components::AckermannActuator, AckermannAssets};

use crate::{
    agents::vehicles::ackermann::components::ActuationModelComponent,
    config::structs::{CollisionConfig, PlantConfig, TopologyConfig},
    core::{
        components::GroundTruthState,
        transforms::{EnuBodyPose, FluVector},
    },
    prelude::*,
};

use helios_core::{control::actuators::SetpointValue, frames::transforms::Convention};

use avian3d::prelude::*;
use nalgebra::Vector3 as NaVec3;

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

#[allow(irrefutable_let_patterns)]
pub(super) fn process_ackermann_logic(
    mut commands: Commands,
    request_query: Query<(Entity, &SpawnAgentConfigRequest)>,
) {
    for (entity, request) in &request_query {
        if let PlantConfig::L0Shim {
            l0_force_gain,
            l0_yaw_gain,
            ..
        } = &request.0.vehicle.plant
        {
            commands.entity(entity).insert((
                AckermannActuator {
                    l0_force_gain: *l0_force_gain,
                    l0_yaw_gain: *l0_yaw_gain,
                },
                ActuationModelComponent(request.0.vehicle.actuation.clone()),
                TrackedFrame(Convention::Flu),
            ));
        }
    }
}

#[allow(clippy::type_complexity)]
pub(super) fn attach_ackermann_physics(
    mut commands: Commands,
    query: Query<
        (Entity, &Name, &GroundTruthState, &SpawnAgentConfigRequest),
        (With<AckermannActuator>, Without<RigidBody>),
    >,
    assets: Res<AckermannAssets>,
) {
    for (entity, name, ground_truth, request) in &query {
        let start_transform_bevy = Transform::from(EnuBodyPose(ground_truth.pose));

        // Source the rigid body from the embodiment axes: mass from topology,
        // passive damping from the L0 plant shim, collider dims + contact
        // friction from collision.
        let (
            TopologyConfig::RigidBodyWithMount { mass },
            PlantConfig::L0Shim {
                linear_damping,
                angular_damping,
                ..
            },
            CollisionConfig::Cuboid { x, y, z, friction },
        ) = (
            &request.0.vehicle.topology,
            &request.0.vehicle.plant,
            &request.0.vehicle.collision,
        );

        let mut entity_commands = commands.entity(entity);
        entity_commands
            .insert((
                start_transform_bevy,
                RigidBody::Dynamic,
                Collider::cuboid(*x, *y, *z),
                Mass(*mass),
                Friction::new(*friction),
                LinearDamping(*linear_damping),
                AngularDamping(*angular_damping),
                SleepingDisabled,
                LinearVelocity::default(),
                AngularVelocity::default(),
                InheritedVisibility::VISIBLE,
            ))
            .with_children(|parent| {
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
/// command reaching this system is always safe to apply directly.
///
/// This is the L0 arcade shim: a single rigid body, so the per-actuator
/// setpoints are folded into one chassis wrench rather than driven through
/// articulated wheel joints. It interprets by command space — a `Velocity`
/// (the drive) becomes forward force, a `Position` (the steer angle) becomes
/// yaw torque — then rotates that FLU wrench into world space and applies it as
/// a `ConstantForce`/`ConstantTorque`. Passive damping is left to Avian3D.
///
/// The velocity → force map is open-loop feedforward: steady-state speed is set
/// by force balancing drag and friction, not by the command, so actual speed
/// only approximates the requested velocity and no single gain tracks it across
/// the range. Closing that loop needs a brain-side speed controller emitting a
/// force setpoint; that is the next fidelity rung, not something this shim does.
pub(super) fn drive_ackermann_cars(
    mut commands: Commands,
    mut query: Query<(
        Entity,
        &Transform,
        &AckermannActuator,
        &ActuationModelComponent,
        Option<&ActuatorCommandComponent>,
    )>,
) {
    for (entity, transform, actuator, model, cmd_opt) in &mut query {
        let Some(cmd) = cmd_opt else {
            continue;
        };

        let resolved = model.0.resolve(&cmd.0);

        // Fold the resolved per-actuator setpoints into a single chassis wrench in
        // the body FLU frame (+X forward, +Z yaw-up). Interpreting by command
        // space is the L0 single-body shim: drive velocity → forward force, steer
        // angle → yaw torque. A force/torque setpoint has no place in this map and
        // is warned-and-ignored; the articulated model that drives per-wheel joints
        // is the next rung of the fidelity ladder.
        let mut force_flu = NaVec3::zeros();
        let mut torque_flu = NaVec3::zeros();

        for sp in resolved.setpoints() {
            match sp.value() {
                SetpointValue::Velocity(v) => force_flu.x += v * actuator.l0_force_gain as f64,
                SetpointValue::Position(v) => torque_flu.z += v * actuator.l0_yaw_gain as f64,
                SetpointValue::Force(_) | SetpointValue::Torque(_) => {
                    warn!(
                        actuator = ?sp.actuator(),
                        "L0 Ackermann shim received a force/torque setpoint it cannot apply; ignoring"
                    );
                }
            }
        }

        // Rotate both the FLU force and torque into Bevy world space before
        // handing them to Avian; conversions stay confined to `FluVector`.
        let force_bevy_local = Vec3::from(FluVector(force_flu));
        let force_world = transform.rotation * force_bevy_local;

        let torque_bevy_local = Vec3::from(FluVector(torque_flu));
        let torque_world = transform.rotation * torque_bevy_local;

        commands.entity(entity).insert((
            ConstantForce::new(force_world.x, force_world.y, force_world.z),
            ConstantTorque::new(torque_world.x, torque_world.y, torque_world.z),
        ));
    }
}
