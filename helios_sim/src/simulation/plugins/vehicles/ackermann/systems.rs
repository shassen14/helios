// helios_sim/src/simulation/plugins/vehicles/ackermann/systems.rs

use crate::{
    prelude::*,
    simulation::{
        core::{
            components::{ControlOutputComponent, GroundTruthState},
            transforms::{EnuBodyPose, EnuVector, FluVector},
        },
        registry::{AdapterBuildContext, AutonomyRegistry},
    },
};
use avian3d::prelude::*;
use nalgebra::Vector3 as NaVec3;

use super::{
    adapter::{AckermannAdapterComponent, DefaultAckermannAdapter},
    components::{AckermannActuator, AckermannParameters},
    AckermannAssets,
};

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

pub(super) fn process_ackermann_logic(
    mut commands: Commands,
    request_query: Query<(Entity, &SpawnAgentConfigRequest)>,
    registry: Res<AutonomyRegistry>,
) {
    for (entity, request) in &request_query {
        if let Vehicle::Ackermann {
            wheelbase,
            max_steering_angle,
            actuator,
            adapter: adapter_cfg,
            ..
        } = &request.0.vehicle
        {
            let adapter_key = adapter_cfg.kind_str();
            let ctx = AdapterBuildContext {
                agent_entity: entity,
                adapter_cfg: adapter_cfg.clone(),
            };
            let adapter_box = match registry.build_adapter(adapter_key, ctx) {
                Ok(a) => a,
                Err(e) => {
                    error!(
                        "Ackermann adapter '{}' failed for {:?}: {}. Falling back to Default.",
                        adapter_key, entity, e
                    );
                    Box::new(DefaultAckermannAdapter)
                }
            };

            commands.entity(entity).insert((
                AckermannParameters {
                    wheelbase: *wheelbase as f64,
                    max_steering_angle: max_steering_angle.to_radians(),
                },
                AckermannActuator {
                    max_force: actuator.max_force,
                    max_torque: actuator.max_torque,
                    max_speed: actuator.max_speed,
                },
                AckermannAdapterComponent(adapter_box),
                TrackedFrame,
            ));
        }
    }
}

pub(super) fn attach_ackermann_physics(
    mut commands: Commands,
    query: Query<
        (
            Entity,
            &Name,
            &GroundTruthState,
            &AckermannParameters,
            &SpawnAgentConfigRequest,
        ),
        Without<RigidBody>,
    >,
    assets: Res<AckermannAssets>,
) {
    for (entity, name, ground_truth, _params, request) in &query {
        let start_transform_bevy = Transform::from(EnuBodyPose(ground_truth.pose));

        // Read physics config from TOML; fall back to defaults if variant doesn't match.
        let physics = if let Vehicle::Ackermann { physics, .. } = &request.0.vehicle {
            physics.clone()
        } else {
            Default::default()
        };

        let mut entity_commands = commands.entity(entity);
        entity_commands
            .insert((
                start_transform_bevy,
                RigidBody::Dynamic,
                Collider::cuboid(1.8, 0.8, 4.0),
                Mass(physics.mass),
                Friction::new(physics.friction),
                LinearDamping(physics.linear_damping),
                AngularDamping(physics.angular_damping),
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

/// RUNTIME: Reads `ControlOutputComponent`, runs the adapter (Layer 2) to get an
/// `AckermannCommand`, then applies forces/torques via dumb kinematic mapping (Layer 3).
///
/// No embedded P controllers — passive damping is handled by Avian3D physics parameters.
pub(super) fn drive_ackermann_cars(
    mut commands: Commands,
    time: Res<Time>,
    mut tick: Local<u64>,
    mut query: Query<(
        Entity,
        &Transform,
        &LinearVelocity,
        &AngularVelocity,
        &AckermannParameters,
        &AckermannActuator,
        &Mass,
        &mut AckermannAdapterComponent,
        Option<&ControlOutputComponent>,
    )>,
) {
    *tick += 1;
    let log = *tick % 60 == 1;

    let dt = time.delta_secs();
    for (
        entity,
        transform,
        lin_vel,
        ang_vel,
        params,
        actuator,
        mass,
        mut adapter_comp,
        ctrl_output_opt,
    ) in &mut query
    {
        let Some(ctrl_output) = ctrl_output_opt else {
            if log {
                info!(
                    "[Actuation #{}] entity {entity:?} has no ControlOutputComponent — skipping",
                    *tick
                );
            }
            continue;
        };

        // Layer 2: adapter translates ControlOutput → AckermannCommand.
        let cmd = adapter_comp.0.adapt(
            &ctrl_output.0,
            params,
            actuator,
            transform,
            lin_vel,
            ang_vel,
            mass.0,
            dt,
        );

        // Layer 3: pure linear map — adapter has already resolved all kinematics.

        // Force: throttle_norm * max_force (FLU +X = Forward).
        let force_flu = NaVec3::new(
            cmd.throttle_norm as f64 * actuator.max_force as f64,
            0.0,
            0.0,
        );
        let force_bevy_local = Vec3::from(FluVector(force_flu));
        let force_world = transform.rotation * force_bevy_local;

        // Torque: steering_torque_norm * max_torque (FLU +Z = Yaw-up).
        let torque_flu = NaVec3::new(
            0.0,
            0.0,
            cmd.steering_torque_norm as f64 * actuator.max_torque as f64,
        );
        let torque_bevy_local = Vec3::from(FluVector(torque_flu));
        let torque_world = transform.rotation * torque_bevy_local;

        if log {
            let bevy_pos = transform.translation;
            let enu_pos = EnuVector::from(bevy_pos).0;
            let bevy_fwd = transform.forward();
            let enu_fwd = EnuVector::from(*bevy_fwd).0;
            info!(
                "[Actuation #{tick}] entity {entity:?}\n  \
                 bevy_pos       = ({bx:.2}, {by:.2}, {bz:.2})\n  \
                 enu_pos        = ({ex:.2}, {ey:.2}, {ez:.2})\n  \
                 bevy_fwd       = ({bfx:.3}, {bfy:.3}, {bfz:.3})\n  \
                 enu_fwd        = ({efx:.3}, {efy:.3}, {efz:.3})  heading={hdg:.1}°\n  \
                 throttle_norm  = {thr:.3}  steer_norm={steer:.3}\n  \
                 force_flu      = ({ffx:.1}, {ffy:.1}, {ffz:.1}) N\n  \
                 force_world    = ({fwx:.1}, {fwy:.1}, {fwz:.1}) N (Bevy)\n  \
                 torque_flu     = ({tfx:.1}, {tfy:.1}, {tfz:.1}) N·m\n  \
                 torque_world   = ({twx:.1}, {twy:.1}, {twz:.1}) N·m (Bevy)",
                tick = *tick,
                bx = bevy_pos.x,
                by = bevy_pos.y,
                bz = bevy_pos.z,
                ex = enu_pos.x,
                ey = enu_pos.y,
                ez = enu_pos.z,
                bfx = bevy_fwd.x,
                bfy = bevy_fwd.y,
                bfz = bevy_fwd.z,
                efx = enu_fwd.x,
                efy = enu_fwd.y,
                efz = enu_fwd.z,
                hdg = enu_fwd.y.atan2(enu_fwd.x).to_degrees(),
                thr = cmd.throttle_norm,
                steer = cmd.steering_torque_norm,
                ffx = force_flu.x,
                ffy = force_flu.y,
                ffz = force_flu.z,
                fwx = force_world.x,
                fwy = force_world.y,
                fwz = force_world.z,
                tfx = torque_flu.x,
                tfy = torque_flu.y,
                tfz = torque_flu.z,
                twx = torque_world.x,
                twy = torque_world.y,
                twz = torque_world.z,
            );
        }

        commands.entity(entity).insert((
            ExternalForce::new(force_world),
            ExternalTorque::new(torque_world),
        ));
    }
}
