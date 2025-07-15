// helios_sim/src/simulation/plugins/vehicles/ackermann.rs

use crate::{
    prelude::*,
    simulation::core::{
        app_state::SimulationSet, topics::GroundTruthState, transforms::enu_iso_to_bevy_transform,
    },
}; // Use the internal prelude of the helios_sim crate
use avian3d::prelude::*;

// --- BEVY COMPONENTS for an Ackermann Vehicle ---

/// A component holding all the static, unchanging parameters of an
/// Ackermann-steering vehicle. Added by `process_ackermann_logic`.
#[derive(Component, Clone)]
pub struct AckermannParameters {
    pub wheelbase: f64,
    pub max_steering_angle: f32,
    // --- Actuator / Physics Parameters ---
    pub max_force: f32,
    pub max_torque: f32,
}

/// A "mailbox" component for control commands. The keyboard controller (or an AI)
/// writes to this, and the driving system reads from it. Added by `process_ackermann_logic`.
#[derive(Component, Default, Debug)]
pub struct VehicleControllerInput {
    /// The desired throttle, from -1.0 (full reverse) to 1.0 (full forward).
    pub throttle: f32,
    /// The desired steering angle, from -max_angle to +max_angle (radians).
    pub steering_angle: f32,
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
            // This system runs once at startup to load shared assets into a resource.
            .add_systems(OnEnter(AppState::SceneBuilding), setup_ackermann_assets)
            // These systems are part of the staged spawning pipeline.
            .add_systems(
                OnEnter(AppState::SceneBuilding),
                (
                    // This system reads the request and adds LOGICAL components like
                    // the dynamics model and controller mailbox.
                    process_ackermann_logic.in_set(SceneBuildSet::ProcessVehicle),
                    // This system runs AFTER the logic is attached. It adds the PHYSICAL
                    // RigidBody, Collider, and visual meshes.
                    attach_ackermann_physics.in_set(SceneBuildSet::Physics),
                ),
            )
            // This is the runtime system that makes the car drive.
            .add_systems(
                FixedUpdate,
                drive_ackermann_cars.in_set(SimulationSet::Actuation),
            );
    }
}

// --- SYSTEMS ---

/// STARTUP: Loads shared assets into the `AckermannAssets` resource.
fn setup_ackermann_assets(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.insert_resource(AckermannAssets {
        body_mesh: meshes.add(Cuboid::new(1.8, 0.8, 4.0)),
        body_material: materials.add(Color::srgb(0.7, 0.2, 0.2)),
        wheel_mesh: meshes.add(Cylinder::new(0.3, 0.2)),
        wheel_material: materials.add(Color::srgb(0.1, 0.1, 0.1)),
    });
}

/// SPAWNING (LOGIC): Reads the request and adds the logical components.
fn process_ackermann_logic(
    mut commands: Commands,
    request_query: Query<(Entity, &SpawnAgentConfigRequest)>,
) {
    for (entity, request) in &request_query {
        // This system only cares about agents configured as Ackermann vehicles.
        if let Vehicle::Ackermann {
            wheelbase,
            max_steering_angle,
            max_steering_rate,
            ..
        } = &request.0.vehicle
        {
            // Create the pure dynamics model from the core library.
            // let dynamics_model = AckermannKinematics {
            //     wheelbase: *wheelbase as f64,
            //     agent_handle: FrameHandle::from_entity(entity),
            // };

            // Insert all the logical components onto the main agent entity.
            commands.entity(entity).insert((
                AckermannParameters {
                    wheelbase: *wheelbase as f64,
                    max_steering_angle: max_steering_angle.to_radians(),
                    max_force: 5000.0,  // Tuning parameter
                    max_torque: 2500.0, // Tuning parameter
                },
                // The pure dynamics model, wrapped in the Bevy component.
                // DynamicsModel(Box::new(dynamics_model)),
                // The controller input "mailbox".
                VehicleControllerInput::default(),
                // Add the TrackedFrame component so the TfTree will see this entity.
                TrackedFrame,
            ));
        }
    }
}

/// SPAWNING (PHYSICS): Attaches the physical body and visual meshes.
fn attach_ackermann_physics(
    mut commands: Commands,
    // This query finds agents that have been processed by the logic system
    // but do not yet have a physical body.
    query: Query<(Entity, &Name, &GroundTruthState, &AckermannParameters), Without<RigidBody>>,
    assets: Res<AckermannAssets>,
) {
    for (entity, name, ground_truth, params) in &query {
        let start_transform_bevy = enu_iso_to_bevy_transform(&ground_truth.pose);

        let mut entity_commands = commands.entity(entity);
        entity_commands
            .insert((
                start_transform_bevy,
                RigidBody::Dynamic,
                // Use chassis size from config if available, otherwise use default
                Collider::cuboid(1.8, 0.8, 4.0),
                Mass(1500.0),
                Friction::new(0.7),
                // We need to disable sleeping for the driving forces to be applied consistently.
                SleepingDisabled,
                // Start with zero velocity.
                LinearVelocity::default(),
                AngularVelocity::default(),
                InheritedVisibility::VISIBLE,
            ))
            .with_children(|parent| {
                // Spawn the visual body mesh as a child.
                parent.spawn((
                    Mesh3d(assets.body_mesh.clone()),
                    MeshMaterial3d(assets.body_material.clone()),
                    Name::new(format!("{}_Body", name)),
                ));

                // Spawn the visual wheels as children.
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

/// RUNTIME: Reads controller input and applies forces/torques to the vehicle.
fn drive_ackermann_cars(
    mut commands: Commands,
    // Query for all the components needed to drive the car.
    query: Query<(
        Entity,
        &Transform,
        &LinearVelocity,
        &AngularVelocity,
        &AckermannParameters,
        &VehicleControllerInput,
    )>,
) {
    for (entity, transform, lin_vel, ang_vel, config, controller) in &query {
        // --- Steering Torque ---
        let current_speed = lin_vel.length();
        let desired_yaw_rate =
            (current_speed / config.wheelbase as f32) * controller.steering_angle.tan();
        let yaw_error = desired_yaw_rate - ang_vel.y;
        let torque_vector = Vec3::Y * yaw_error * config.max_torque;

        // --- Driving Force ---
        let world_forward_vector = transform.forward();
        let current_forward_speed = lin_vel.dot(*world_forward_vector);
        // A simple target speed based on throttle.
        let target_speed = controller.throttle * 20.0; // 20 m/s max speed
        let speed_error = target_speed - current_forward_speed;
        let force_vector = world_forward_vector * speed_error * config.max_force;

        // Apply forces and torques via commands. This is the correct way to interact with Avian3D.
        commands.entity(entity).insert((
            ExternalForce::new(force_vector),
            ExternalTorque::new(torque_vector),
        ));
    }
}
