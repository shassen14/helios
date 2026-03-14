// helios_sim/src/simulation/plugins/vehicles/ackermann/systems.rs

use crate::{
    prelude::*,
    simulation::{
        core::{
            components::{ControlOutputComponent, GroundTruthState},
            transforms::{enu_iso_to_bevy_transform, flu_vector_to_bevy_local_vector},
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
        let start_transform_bevy = enu_iso_to_bevy_transform(&ground_truth.pose);

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
        let force_bevy_local = flu_vector_to_bevy_local_vector(&force_flu);
        let force_world = transform.rotation * force_bevy_local;

        // Torque: steering_torque_norm * max_torque (FLU +Z = Yaw-up).
        let torque_flu = NaVec3::new(
            0.0,
            0.0,
            cmd.steering_torque_norm as f64 * actuator.max_torque as f64,
        );
        let torque_bevy_local = flu_vector_to_bevy_local_vector(&torque_flu);
        let torque_world = transform.rotation * torque_bevy_local;

        commands.entity(entity).insert((
            ExternalForce::new(force_world),
            ExternalTorque::new(torque_world),
        ));
    }
}
