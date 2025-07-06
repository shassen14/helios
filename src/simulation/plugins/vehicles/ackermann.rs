// src/simulation/plugins/vehicles/ackermann.rs

use crate::simulation::core::config::{SimulationConfig, Vehicle};
use crate::simulation::core::dynamics::DynamicsModel;
use crate::simulation::models::ackermann::AckermannKinematics;
use avian3d::prelude::*;
use bevy::prelude::*;

// --- Plugin Definition ---
pub struct AckermannCarPlugin;

impl Plugin for AckermannCarPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_ackermann_cars)
            // This single system now contains all the logic for driving.
            .add_systems(FixedUpdate, drive_ackermann_cars_by_velocity);
    }
}

// --- Components ---
#[derive(Component)]
pub struct AckermannConfig {
    pub wheelbase: f64,
    // Add actuator/control parameters here.
    pub max_force: f32,
    pub max_torque: f32,
}

#[derive(Component, Default)]
pub struct VehicleControllerInput {
    pub throttle: f32,
    pub steering_angle: f32,
}

// --- Systems ---

/// Startup system to spawn car entities based on the config.
fn spawn_ackermann_cars(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<SimulationConfig>,
) {
    for agent_config in &config.agents {
        if let Vehicle::Ackermann { wheelbase, .. } = &agent_config.vehicle {
            info!("Spawning Ackermann car: {}", &agent_config.name);

            // --- Create Placeholder Visuals ---
            // Define materials for the car body and wheels.
            let body_material = materials.add(Color::srgb(0.7, 0.2, 0.2)); // Red
            let wheel_material = materials.add(Color::srgb(0.1, 0.1, 0.1)); // Dark Gray

            // Define the shape for the wheels.
            let wheel_mesh: Handle<Mesh> = meshes.add(Cylinder::new(0.3, 0.2)); // 0.3 radius, 0.2 width
            let car_body_mesh = meshes.add(Cuboid::new(1.8, 0.8, 4.0));

            // Spawn the parent entity for the car's main body.
            // This entity will have the RigidBody, Collider, and our custom components.
            commands
                .spawn((
                    // Simulation Components use Simulation Frame data
                    Transform::from_xyz(-3.0, 10.0, -3.0),
                    RigidBody::Dynamic,
                    Collider::cuboid(1.8, 0.8, 4.0), // Match collider to visual shape
                    Mass(1500.0),
                    Friction::new(0.7),
                    Restitution::new(0.1),
                    // This is needed for the `drive_ackermann_cars` system to apply force.
                    ExternalForce::default(),
                    ExternalTorque::default(),
                    SleepingDisabled,
                    // LockedAxes::default(),
                    // --- Our Logic ---
                    DynamicsModel(Box::new(AckermannKinematics {
                        wheelbase: *wheelbase as f64,
                    })),
                    AckermannConfig {
                        wheelbase: *wheelbase as f64,
                        // max_speed: 20.0,
                        max_force: 10000.0,
                        max_torque: 5000.0,
                    },
                    VehicleControllerInput::default(),
                    Name::new(agent_config.name.clone()),
                ))
                .with_children(|parent| {
                    // --- Car Body ---
                    parent.spawn((
                        Mesh3d(car_body_mesh.clone()),
                        MeshMaterial3d(body_material.clone()),
                        // Transform::from_xyz(0.0, car_body_y_offset, 0.0),
                        Name::new(format!("{}_Body", agent_config.name.clone())),
                    ));

                    let wheel_positions = [
                        (
                            Vec3::new(1.0, -0.2, 1.7),
                            format!("{}_FR", agent_config.name.clone()),
                        ),
                        (
                            Vec3::new(-1.0, -0.2, 1.7),
                            format!("{}_FL", agent_config.name.clone()),
                        ),
                        (
                            Vec3::new(1.0, -0.2, -1.7),
                            format!("{}_RR", agent_config.name.clone()),
                        ),
                        (
                            Vec3::new(-1.0, -0.2, -1.7),
                            format!("{}_RL", agent_config.name.clone()),
                        ),
                    ];

                    for (pos, name) in wheel_positions {
                        parent.spawn((
                            Mesh3d(wheel_mesh.clone()),
                            MeshMaterial3d(wheel_material.clone()),
                            Transform::from_translation(pos)
                                .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
                            Name::new(name),
                        ));
                    }
                });
        }
    }
}

/// This system reads controller inputs and applies physically-based forces and torques.
/// All control logic is self-contained here.
fn apply_ackermann_forces(
    mut commands: Commands,
    // The query now only needs components related to control and physics state.
    // It no longer needs the DynamicsModel.
    query: Query<(
        Entity,
        &Transform,
        &LinearVelocity,
        &AckermannConfig,
        &VehicleControllerInput,
    )>,
) {
    for (entity, transform, lin_vel, config, controller) in query.iter() {
        // --- 1. Calculate Local Forces and Torques ---
        // This logic is now correctly placed inside the system.

        // Forward/backward force based on throttle.
        // We assume the car's local +X is its forward direction.
        let local_force = Vec3::new(controller.throttle * config.max_force, 0.0, 0.0);

        // Torque to steer the vehicle.
        // This is a simplified model that a more advanced one could replace.
        let forward_speed = transform.forward().dot(lin_vel.0);
        let desired_yaw_rate =
            (forward_speed / config.wheelbase as f32) * controller.steering_angle.tan();
        // For simplicity, we directly command a torque based on desired yaw rate.
        // A real controller would be a PID loop on yaw rate error.
        let local_torque = Vec3::new(0.0, desired_yaw_rate * config.max_torque, 0.0);

        // --- 2. Transform to World Frame ---
        let world_force = transform.rotation * local_force;
        let world_torque = transform.rotation * local_torque;

        // --- 3. Apply to the Entity ---
        // We use `commands.entity(entity).insert(...)` to apply the forces for this frame.
        // This is a robust way to command the physics engine.
        commands.entity(entity).insert((
            ExternalForce::new(world_force),
            ExternalTorque::new(world_torque),
        ));
    }
}

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

// fn drive_ackermann_cars_by_velocity(
//     mut query: Query<(
//         &mut LinearVelocity,
//         &mut AngularVelocity,
//         &Transform,
//         &AckermannConfig,
//         &VehicleControllerInput, // We will log this component
//     )>,
// ) {
//     for (mut lin_vel, mut ang_vel, transform, config, controller) in query.iter_mut() {
//         // --- ADD THIS LOGGING BLOCK ---
//         // Log the inputs this system is receiving.
//         // We only log if the inputs are non-zero to avoid spamming the console.
//         if controller.throttle != 0.0 || controller.steering_angle != 0.0 {
//             info!(
//                 throttle = controller.throttle,
//                 steering = controller.steering_angle,
//                 "Driving car with inputs."
//             );
//         }
//         // -----------------------------

//         let forward_vector = transform.forward();

//         // Let's also log the calculated velocities
//         let target_linear_velocity = forward_vector * controller.throttle * 10.0;
//         let target_angular_velocity_y =
//             (controller.throttle * 5.0 / config.wheelbase as f32) * controller.steering_angle.tan();

//         if target_linear_velocity.length() > 0.01 || target_angular_velocity_y.abs() > 0.01 {
//             debug!(
//                 lin_vel = ?target_linear_velocity,
//                 ang_vel_y = target_angular_velocity_y,
//                 "Applying new velocities."
//             );
//         }

//         // Apply the velocities
//         lin_vel.x = target_linear_velocity.x;
//         lin_vel.z = target_linear_velocity.z;
//         ang_vel.y = target_angular_velocity_y;
//     }
// }

fn drive_ackermann_cars_by_velocity(
    mut query: Query<(
        &mut LinearVelocity,
        &mut AngularVelocity,
        &Transform,
        &AckermannConfig,
        &VehicleControllerInput,
    )>,
) {
    for (mut lin_vel, mut ang_vel, transform, config, controller) in query.iter_mut() {
        // --- Steering ---
        // Calculate the desired turning speed (yaw rate).
        // If throttle is zero, turning speed should also be zero.
        let speed = lin_vel.length(); // Use current speed for more stable turning
        let turning_speed = (speed / config.wheelbase as f32) * controller.steering_angle.tan();
        ang_vel.y = turning_speed;

        // --- Throttle ---
        // Apply velocity in the direction the car is currently facing.
        let forward_vector = transform.forward();
        let target_velocity = forward_vector * controller.throttle * 20.0; // 20.0 is max speed

        // We only update the X and Z components of velocity to allow gravity to work on Y.
        lin_vel.x = target_velocity.x;
        lin_vel.z = target_velocity.z;
    }
}
