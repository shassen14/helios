// src/simulation/plugins/vehicles/ackermann.rs

use crate::prelude::{AppState, GroundTruthState, Vehicle};
use crate::simulation::core::abstractions::DynamicsModel;
use crate::simulation::core::app_state::SceneBuildSet;
use crate::simulation::core::simulation_setup::SpawnAgentConfigRequest;
use crate::simulation::models::ackermann::AckermannKinematics;
use crate::simulation::utils::transforms::enu_iso_to_bevy_transform;
use avian3d::prelude::*;
use bevy::prelude::*;

// --- Plugin Definition ---
pub struct AckermannCarPlugin;

impl Plugin for AckermannCarPlugin {
    fn build(&self, app: &mut App) {
        // This plugin adds three systems to the OnEnter(SceneBuilding) schedule
        app.add_systems(OnEnter(AppState::SceneBuilding), setup_ackermann_assets);

        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            // We now have TWO systems that process the request.
            // One for logic, one for physics. Scheduled in order.
            (
                process_ackermann_logic.in_set(SceneBuildSet::ProcessVehicle),
                attach_ackermann_physics.in_set(SceneBuildSet::Physics),
            ),
        );

        // Add the runtime driving system as before
        app.add_systems(
            FixedUpdate,
            drive_ackermann_cars_by_velocity.run_if(in_state(AppState::Running)),
        );
    }
}

// --- Component Definitions for Ackermann Vehicles ---

/// A component holding all the static, unchanging parameters of an
/// Ackermann-steering vehicle.
#[derive(Component, Clone)]
pub struct AckermannParameters {
    // --- Kinematic / Geometric Parameters ---
    /// The distance between the front and rear axles in meters.
    pub wheelbase: f64,
    /// The maximum angle the inner wheel can turn, in radians.
    pub max_steering_angle: f32,

    // --- Actuator / Physics Parameters ---
    /// The maximum force the engine can apply for acceleration, in Newtons.
    pub max_force: f32,
    /// The maximum torque the steering mechanism can apply, in Newton-meters.
    pub max_torque: f32,
}

/// A component that acts as a "mailbox" for control commands.
/// Systems like a keyboard controller or an AI controller will write to this component,
/// and the vehicle's driving system will read from it.
#[derive(Component, Default, Debug)]
pub struct VehicleControllerInput {
    /// The desired linear acceleration, from -MAX to +MAX (m/s^2).
    pub linear_acceleration: f32,
    /// The desired steering angle, from -max_angle to +max_angle (radians).
    pub steering_angle: f32,
}

// --- Systems ---

fn process_ackermann_logic(
    mut commands: Commands,
    request_query: Query<(Entity, &SpawnAgentConfigRequest)>,
) {
    for (entity, request) in &request_query {
        if let Vehicle::Ackermann {
            wheelbase,
            max_steering_angle,
            ..
        } = &request.0.vehicle
        {
            let dynamics_model = AckermannKinematics {
                wheelbase: *wheelbase as f64,
                agent_entity: entity,
            };
            commands.entity(entity).insert((
                AckermannParameters {
                    wheelbase: *wheelbase as f64,
                    max_steering_angle: max_steering_angle.to_radians(),
                    max_force: 10000.0,
                    max_torque: 5000.0,
                },
                DynamicsModel(Box::new(dynamics_model)),
                VehicleControllerInput::default(),
            ));
        }
    }
}

fn attach_ackermann_physics(
    mut commands: Commands,
    // --- THE CORRECTED QUERY ---
    // We ask for all the data we need: Entity, Name, and GroundTruthState.
    // The filters (With/Without) remain the same.
    query: Query<
        (Entity, &Name, &GroundTruthState),
        (With<AckermannParameters>, Without<RigidBody>),
    >,
    // We now get the pre-loaded assets from the resource.
    assets: Res<AckermannAssets>,
) {
    // The loop now gives us all the data for each matching entity.
    for (entity, name, ground_truth) in &query {
        info!(
            "  -> Attaching physical body to Ackermann entity '{}' ({:?})",
            name, entity
        );

        // --- CALCULATION IS NOW INSIDE THE LOOP ---
        // This is now correct because `ground_truth` is specific to this entity.
        let start_transform_bevy = enu_iso_to_bevy_transform(&ground_truth.pose);

        let mut entity_commands = commands.entity(entity);

        entity_commands.insert((
            // We insert the start transform here. It's a component just like the others.
            start_transform_bevy,
            // All the RigidBody, Collider, etc. components go here
            RigidBody::Dynamic,
            Collider::cuboid(1.8, 0.8, 4.0),
            Mass(1500.0),
            Friction::new(0.7),
            SleepingDisabled,
            // These defaults are fine
            LinearVelocity::default(),
            AngularVelocity::default(),
            ExternalForce::default(),
            ExternalTorque::default(),
            InheritedVisibility::VISIBLE,
        ));

        entity_commands.with_children(|parent| {
            // Spawn the visual body mesh
            parent.spawn((
                // We CLONE the handles from the resource. This is very cheap.
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

// fn drive_ackermann_cars_by_velocity(
//     mut query: Query<(
//         &mut LinearVelocity,
//         &mut AngularVelocity,
//         &Transform, // To get current orientation
//         &AckermannParameters,
//         &VehicleControllerInput,
//     )>,
//     time: Res<Time<Fixed>>, // For time-based changes
// ) {
//     for (mut lin_vel, mut ang_vel, transform, config, controller) in query.iter_mut() {
//         // --- Linear Acceleration (directly apply) ---
//         // This is a direct application of the commanded acceleration.
//         // In a more complex model, you'd integrate the acceleration into velocity.
//         // Here, we're assuming the physics engine will handle the integration.
//         // If you were *not* using a physics engine, you'd update lin_vel directly.

//         // For a physics engine, you typically apply ExternalForce/ExternalTorque.
//         // Let's refine this to generate forces from the acceleration command.

//         // Convert commanded linear_acceleration to force (F=ma)
//         // This assumes a fixed mass for simplicity or queries for Mass component.
//         // For a robust system, you'd get Mass(m) component: let mass = Mass.0;
//         let mass = 1500.0; // Example mass
//         let world_forward_direction = transform.forward();
//         let force_vector = world_forward_direction * controller.linear_acceleration * mass;

//         // Apply force. This requires ExternalForce component.
//         // commands.entity(entity).insert(ExternalForce::new(force_vector));

//         // Let's simplify for now to directly set velocities for initial testing
//         // but mark this as a point for future physics-based force application.

//         let current_speed = lin_vel.length();
//         let target_speed = current_speed + controller.linear_acceleration * time.delta_secs();
//         let max_speed_cap = 20.0; // Arbitrary cap
//         let min_speed_cap = -5.0; // Arbitrary cap for reverse
//         let clamped_target_speed = target_speed.clamp(min_speed_cap, max_speed_cap);

//         let desired_lin_vel = transform.forward() * clamped_target_speed;
//         lin_vel.x = desired_lin_vel.x;
//         lin_vel.z = desired_lin_vel.z;
//         // lin_vel.y is left to gravity/physics

//         // --- Steering (Angular Velocity) ---
//         let current_speed_for_steering = lin_vel.length();
//         let desired_yaw_rate = (current_speed_for_steering / config.wheelbase as f32)
//             * controller.steering_angle.tan();
//         ang_vel.y = desired_yaw_rate;

//         println!("lin_vel: {:?}, ang_vel: {:?}", lin_vel, ang_vel);
//     }
// }

fn drive_ackermann_cars_by_velocity(
    // QUERY 1: Find all parent agents that have an Ackermann controller input.
    parent_query: Query<(&VehicleControllerInput, &Children)>,

    // QUERY 2: Find all the physics bodies that have Ackermann parameters.
    // This assumes the physics body is on a child. If it's on the parent,
    // the query would be different, but this pattern is more robust.
    // We will assume `attach_ackermann_physics` puts the RigidBody on the parent for now.

    // Let's go with the simpler pattern where the RigidBody is on the parent.
    mut vehicle_query: Query<(
        &mut LinearVelocity,
        &mut AngularVelocity,
        &Transform,
        &AckermannParameters,
        &VehicleControllerInput, // Get the controller input directly
    )>,
    time: Res<Time<Fixed>>,
) {
    // The query is now direct and simple.
    for (mut lin_vel, mut ang_vel, transform, config, controller) in &mut vehicle_query {
        // This logic is now correct because the query provides all necessary components.

        let current_speed = lin_vel.length();
        // Use controller.linear_acceleration (from your previous definition)
        let target_speed = current_speed + controller.linear_acceleration; // * time.delta_secs();
        let max_speed_cap = 20.0; // Arbitrary cap
        let min_speed_cap = -5.0; // Arbitrary cap for reverse
        let clamped_target_speed = target_speed.clamp(min_speed_cap, max_speed_cap);

        let desired_lin_vel = transform.forward() * clamped_target_speed;
        lin_vel.x = desired_lin_vel.x;
        lin_vel.z = desired_lin_vel.z;

        let current_speed_for_steering = lin_vel.length();
        let desired_yaw_rate = (current_speed_for_steering / config.wheelbase as f32)
            * controller.steering_angle.tan();
        ang_vel.y = desired_yaw_rate;
    }
}
#[derive(Resource)]
struct AckermannAssets {
    body_mesh: Handle<Mesh>,
    body_material: Handle<StandardMaterial>,
    wheel_mesh: Handle<Mesh>,
    wheel_material: Handle<StandardMaterial>,
}

// A startup system to load these assets once and place them in the resource.
fn setup_ackermann_assets(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    info!("[ASSETS] Loading shared assets for Ackermann vehicles.");
    commands.insert_resource(AckermannAssets {
        body_mesh: meshes.add(Cuboid::new(1.8, 0.8, 4.0)),
        body_material: materials.add(Color::srgb(0.7, 0.2, 0.2)),
        wheel_mesh: meshes.add(Cylinder::new(0.3, 0.2)),
        wheel_material: materials.add(Color::srgb(0.1, 0.1, 0.1)),
    });
}

// Startup system to spawn car entities based on the config.
// fn spawn_ackermann_cars(
//     mut commands: Commands,
//     mut meshes: ResMut<Assets<Mesh>>,
//     mut materials: ResMut<Assets<StandardMaterial>>,
//     query: Query<
//         (Entity, &Name, &AckermannParameters, &GroundTruthState),
//         With<SpawnRequestAckermann>,
//     >,
// ) {
//     info!("spawn_ackermann enter, but not in for loop");
//     for (entity, agent_name, params, ground_truth) in query.iter() {
//         info!(
//             "[SPAWN PASS 2] Fulfilling spawn request for Ackermann on entity {:?}",
//             entity
//         );

//         // --- Create Placeholder Visuals ---
//         // Define materials for the car body and wheels.
//         let body_material = materials.add(Color::srgb(0.7, 0.2, 0.2)); // Red
//         let wheel_material = materials.add(Color::srgb(0.1, 0.1, 0.1)); // Dark Gray

//         // Define the shape for the wheels.
//         let wheel_mesh: Handle<Mesh> = meshes.add(Cylinder::new(0.3, 0.2)); // 0.3 radius, 0.2 width
//         let car_body_mesh = meshes.add(Cuboid::new(1.8, 0.8, 4.0));

//         // Calculate the initial Bevy transform from the logical ground truth state.
//         let start_transform_bevy = enu_iso_to_bevy_transform(&ground_truth.pose);

//         // Use `commands.entity(entity).insert(...)` to ADD components to the EXISTING entity.
//         commands
//             .entity(entity)
//             .insert((
//                 // --- Attach Physical Components ---
//                 (
//                     start_transform_bevy,
//                     RigidBody::Dynamic,
//                     Collider::cuboid(1.8, 0.8, 4.0),
//                     Mass(1500.0),
//                     Friction::new(0.7),
//                     SleepingDisabled,
//                     LinearVelocity::default(),
//                     AngularVelocity::default(),
//                     ExternalForce::default(),
//                     ExternalTorque::default(),
//                     InheritedVisibility::VISIBLE,
//                 ),
//                 // --- Attach Vehicle-Specific Logic ---
//                 // The mathematical model for prediction and control.
//                 DynamicsModel(Box::new(AckermannKinematics {
//                     wheelbase: params.wheelbase,
//                 })),
//                 // The mailbox for receiving control inputs.
//                 VehicleControllerInput::default(),
//             ))
//             .with_children(|parent| {
//                 // --- Car Body ---
//                 println!("spawning children");
//                 parent.spawn((
//                     Mesh3d(car_body_mesh.clone()),
//                     MeshMaterial3d(body_material.clone()),
//                     // Transform::from_xyz(0.0, car_body_y_offset, 0.0),
//                     Name::new(format!("{}_Body", agent_name.clone())),
//                 ));

//                 let wheel_positions = [
//                     (
//                         Vec3::new(1.0, -0.2, 1.7),
//                         format!("{}_FR", agent_name.clone()),
//                     ),
//                     (
//                         Vec3::new(-1.0, -0.2, 1.7),
//                         format!("{}_FL", agent_name.clone()),
//                     ),
//                     (
//                         Vec3::new(1.0, -0.2, -1.7),
//                         format!("{}_RR", agent_name.clone()),
//                     ),
//                     (
//                         Vec3::new(-1.0, -0.2, -1.7),
//                         format!("{}_RL", agent_name.clone()),
//                     ),
//                 ];

//                 for (pos, name) in wheel_positions {
//                     parent.spawn((
//                         Mesh3d(wheel_mesh.clone()),
//                         MeshMaterial3d(wheel_material.clone()),
//                         Transform::from_translation(pos)
//                             .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
//                         Name::new(name),
//                     ));
//                 }
//             })
//             // --- IMPORTANT: Remove the request component so this system doesn't run again.
//             .remove::<SpawnRequestAckermann>();
//     }
// }

// / This system reads controller inputs and applies physically-based forces and torques.
// / All control logic is self-contained here.
// fn apply_ackermann_forces(
//     mut commands: Commands,
//     // The query now only needs components related to control and physics state.
//     // It no longer needs the DynamicsModel.
//     query: Query<(
//         Entity,
//         &Transform,
//         &LinearVelocity,
//         &AckermannParameters,
//         &VehicleControllerInput,
//     )>,
// ) {
//     for (entity, transform, lin_vel, config, controller) in query.iter() {
//         // --- 1. Calculate Local Forces and Torques ---
//         // This logic is now correctly placed inside the system.

//         // Forward/backward force based on throttle.
//         // We assume the car's local +X is its forward direction.
//         let local_force = Vec3::new(controller.throttle * config.max_force, 0.0, 0.0);

//         // Torque to steer the vehicle.
//         // This is a simplified model that a more advanced one could replace.
//         let forward_speed = transform.forward().dot(lin_vel.0);
//         let desired_yaw_rate =
//             (forward_speed / config.wheelbase as f32) * controller.steering_angle.tan();
//         // For simplicity, we directly command a torque based on desired yaw rate.
//         // A real controller would be a PID loop on yaw rate error.
//         let local_torque = Vec3::new(0.0, desired_yaw_rate * config.max_torque, 0.0);

//         // --- 2. Transform to World Frame ---
//         let world_force = transform.rotation * local_force;
//         let world_torque = transform.rotation * local_torque;

//         // --- 3. Apply to the Entity ---
//         // We use `commands.entity(entity).insert(...)` to apply the forces for this frame.
//         // This is a robust way to command the physics engine.
//         commands.entity(entity).insert((
//             ExternalForce::new(world_force),
//             ExternalTorque::new(world_torque),
//         ));
//     }
// }

// FixedUpdate system that applies forces to drive the car.
// fn drive_ackermann_cars(
//     mut query: Query<(
//         &mut ExternalForce,
//         &mut ExternalTorque,
//         &GlobalTransform,
//         &LinearVelocity,
//         &AngularVelocity,
//         &AckermannConfig,
//         &VehicleControllerInput,
//     )>,
//     time: Res<Time<Fixed>>,
// ) {
//     for (mut force, mut torque, transform, lin_vel, ang_vel, config, controller) in query.iter_mut()
//     {
//         // --- FORWARD/REVERSE FORCE ---
//         // Get the car's forward direction vector.
//         let forward_vector = transform.forward();
//         // Apply force based on throttle input.
//         let forward_force = forward_vector * controller.throttle * config.max_force;
//         // force.apply_force(forward_force);

//         // --- STEERING TORQUE ---
//         // This is a simple proportional controller to make the car turn.
//         // It tries to achieve the desired yaw rate for the current speed and steering angle.
//         let desired_yaw_rate =
//             (lin_vel.length() as f64 / config.wheelbase) * controller.steering_angle.tan() as f64;
//         let current_yaw_rate = ang_vel.y as f64;
//         let yaw_error = desired_yaw_rate - current_yaw_rate;

//         // Apply torque to correct the yaw rate.
//         let steering_torque = Vec3::new(
//             0.0,
//             (yaw_error as f32) * config.max_torque * time.delta_secs(),
//             0.0,
//         );
//         // torque.apply_torque(steering_torque);
//     }
// }
