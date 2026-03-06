// helios_sim/src/simulation/plugins/vehicles/ackermann.rs

use crate::{
    prelude::*,
    simulation::core::{
        app_state::SimulationSet,
        components::{ControlOutputComponent, GroundTruthState},
        transforms::{enu_iso_to_bevy_transform, flu_vector_to_bevy_local_vector},
    },
};
use avian3d::prelude::*;
use helios_core::control::ControlOutput;
use nalgebra::Vector3 as NaVec3;

// --- BEVY COMPONENTS for an Ackermann Vehicle ---

/// Static parameters for an Ackermann-steering vehicle.
/// Populated from config during `process_ackermann_logic`.
#[derive(Component, Clone)]
pub struct AckermannParameters {
    pub wheelbase: f64,
    pub max_steering_angle: f32,
}

/// Actuator physics limits for an Ackermann vehicle.
/// Read from the `[vehicle.actuator]` TOML section with sensible defaults.
#[derive(Component, Clone)]
pub struct AckermannActuator {
    pub max_force: f32,  // N
    pub max_torque: f32, // N·m
    pub max_speed: f32,  // m/s
}

/// A resource to hold shared assets, preventing them from being reloaded for every car.
#[derive(Resource)]
struct AckermannAssets {
    body_mesh: Handle<Mesh>,
    body_material: Handle<StandardMaterial>,
    wheel_mesh: Handle<Mesh>,
    wheel_material: Handle<StandardMaterial>,
}

// --- THE PLUGIN ---
pub struct AckermannCarPlugin;

impl Plugin for AckermannCarPlugin {
    fn build(&self, app: &mut App) {
        app
            .add_systems(OnEnter(AppState::SceneBuilding), setup_ackermann_assets)
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                (
                    process_ackermann_logic.in_set(SceneBuildSet::ProcessVehicle),
                    attach_ackermann_physics.in_set(SceneBuildSet::Physics),
                ),
            )
            .add_systems(
                FixedUpdate,
                drive_ackermann_cars.in_set(SimulationSet::Actuation),
            );
    }
}

// --- SYSTEMS ---

fn setup_ackermann_assets(
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

fn process_ackermann_logic(
    mut commands: Commands,
    request_query: Query<(Entity, &SpawnAgentConfigRequest)>,
) {
    for (entity, request) in &request_query {
        if let Vehicle::Ackermann {
            wheelbase,
            max_steering_angle,
            actuator,
            ..
        } = &request.0.vehicle
        {
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
                TrackedFrame,
            ));
        }
    }
}

fn attach_ackermann_physics(
    mut commands: Commands,
    query: Query<(Entity, &Name, &GroundTruthState, &AckermannParameters), Without<RigidBody>>,
    assets: Res<AckermannAssets>,
) {
    for (entity, name, ground_truth, _params) in &query {
        let start_transform_bevy = enu_iso_to_bevy_transform(&ground_truth.pose);

        let mut entity_commands = commands.entity(entity);
        entity_commands
            .insert((
                start_transform_bevy,
                RigidBody::Dynamic,
                Collider::cuboid(1.8, 0.8, 4.0),
                Mass(1500.0),
                Friction::new(0.7),
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

/// RUNTIME: Reads `ControlOutputComponent` and applies forces/torques in the Bevy world frame.
///
/// All FLU body-frame vectors are converted to Bevy local frame via `flu_vector_to_bevy_local_vector`,
/// then rotated into world space via the entity's transform quaternion.
/// No manual axis swaps — this is the only place the conversion happens.
fn drive_ackermann_cars(
    mut commands: Commands,
    query: Query<(
        Entity,
        &Transform,
        &LinearVelocity,
        &AngularVelocity,
        &AckermannParameters,
        &AckermannActuator,
        Option<&ControlOutputComponent>,
    )>,
) {
    for (entity, transform, lin_vel, ang_vel, params, actuator, ctrl_output_opt) in &query {
        let Some(ctrl_output) = ctrl_output_opt else {
            continue;
        };

        // Convert ControlOutput variant → (throttle_norm, steering_rad) in FLU / Ackermann convention.
        let (throttle_norm, steering_rad) =
            control_output_to_ackermann(&ctrl_output.0, params, actuator, lin_vel, ang_vel);

        // --- Driving Force (FLU +X = Forward) ---
        // Compute world forward speed from physics.
        let world_forward = transform.forward();
        let current_forward_speed = lin_vel.dot(*world_forward);
        let target_speed = throttle_norm * actuator.max_speed;
        let speed_error = target_speed - current_forward_speed;

        // Build force in FLU frame (+X forward), convert to Bevy local, rotate to world.
        let force_flu = NaVec3::new(speed_error as f64 * actuator.max_force as f64, 0.0, 0.0);
        let force_bevy_local = flu_vector_to_bevy_local_vector(&force_flu);
        let force_world = transform.rotation * force_bevy_local;

        // --- Steering Torque (FLU +Z = Yaw-up) ---
        let current_speed = lin_vel.length();
        let desired_yaw_rate =
            (current_speed / params.wheelbase as f32) * steering_rad.tan();
        let yaw_error = desired_yaw_rate - ang_vel.y;

        // Yaw torque in FLU frame (+Z = yaw up), convert to Bevy local (+Y = up in Bevy).
        let torque_flu = NaVec3::new(0.0, 0.0, yaw_error as f64 * actuator.max_torque as f64);
        let torque_bevy_local = flu_vector_to_bevy_local_vector(&torque_flu);
        let torque_world = transform.rotation * torque_bevy_local;

        commands.entity(entity).insert((
            ExternalForce::new(force_world),
            ExternalTorque::new(torque_world),
        ));
    }
}

/// Convert any `ControlOutput` variant into `(throttle_norm: f32, steering_rad: f32)`.
///
/// FLU body frame convention:
///   - throttle: normalised forward speed demand [-1, 1], positive = forward
///   - steering_rad: Ackermann wheel angle, positive = turn left (FLU convention)
fn control_output_to_ackermann(
    output: &ControlOutput,
    params: &AckermannParameters,
    actuator: &AckermannActuator,
    lin_vel: &LinearVelocity,
    _ang_vel: &AngularVelocity,
) -> (f32, f32) {
    match output {
        ControlOutput::BodyVelocity { linear, angular } => {
            // linear.x = desired forward speed (FLU), angular.z = desired yaw rate (FLU).
            let throttle = (linear.x as f32) / actuator.max_speed;
            let v = lin_vel.length().max(0.1);
            // steering = atan(yaw_rate * wheelbase / v)
            let steering = (angular.z as f32 * params.wheelbase as f32 / v).atan();
            (throttle.clamp(-1.0, 1.0), steering)
        }
        ControlOutput::Raw(u) => {
            // Ackermann Raw convention: u[0] = throttle [-1,1], u[1] = steering_rad.
            let throttle = u.get(0).copied().unwrap_or(0.0) as f32;
            let steering = u.get(1).copied().unwrap_or(0.0) as f32;
            (throttle.clamp(-1.0, 1.0), steering)
        }
        ControlOutput::RawActuators(signals) => {
            // Direct passthrough with clamping: [throttle, steering_rad].
            let throttle = signals.first().copied().unwrap_or(0.0) as f32;
            let steering = signals.get(1).copied().unwrap_or(0.0) as f32;
            (throttle.clamp(-1.0, 1.0), steering)
        }
        ControlOutput::Wrench { force, torque } => {
            // Project FLU forward force → throttle, FLU yaw torque → steering.
            let throttle = (force.x as f32) / (actuator.max_force).max(1.0);
            let yaw_torque = torque.z as f32;
            let steering = (yaw_torque / (actuator.max_torque).max(1.0)).atan();
            (throttle.clamp(-1.0, 1.0), steering)
        }
        ControlOutput::BodyAcceleration { linear, .. } => {
            // linear.x = desired forward acceleration (FLU).
            // Approximate throttle as accel / (max_force / mass). Use 1500 kg default.
            let approx_mass = 1500.0_f32;
            let throttle = (linear.x as f32 * approx_mass) / actuator.max_force.max(1.0);
            (throttle.clamp(-1.0, 1.0), 0.0)
        }
    }
}
