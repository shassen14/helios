// src/input_systems.rs

use crate::simulation::components::{ControlInput, KeyboardControlled};
use bevy::prelude::*;

/// System that reads keyboard input and updates the ControlInput for entities
/// marked with KeyboardControlled.
pub fn keyboard_control_system(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut query: Query<&mut ControlInput, With<KeyboardControlled>>,
    // Optional: Query dynamics model for limits if needed
    // mut query: Query<(&mut ControlInput, &DynamicsModel), With<KeyboardControlled>>,
) {
    // Define control parameters
    let max_acceleration = 2.0; // Max acceleration target m/s^2
    let max_braking = 3.0; // Max braking deceleration m/s^2
    let max_steering_angle_rad = 35.0f32.to_radians() as f64; // Max steer target rad
    let steering_rate = 3.0; // Rate of steering change rad/s
    let acceleration_rate = 5.0; // Rate of acceleration change m/s^3 (jerk)
    let deceleration_rate = 7.0; // Rate of deceleration/braking change m/s^3
    let steering_return_rate = 2.0; // Rate at which steering returns to center rad/s

    let dt = time.delta().as_secs_f64(); // Get delta time as f64

    // Use .single_mut() which returns Result
    if let Ok(mut control_input) = query.single_mut() {
        // Ensure control vector has the correct size [delta, a]
        if control_input.0.len() != 2 {
            if control_input.0.is_empty() || control_input.0.len() != 2 {
                println!("Warning: Initializing ControlInput for KeyboardControlled entity.");
                control_input.0 = nalgebra::DVector::from_vec(vec![0.0, 0.0]);
            } else {
                eprintln!("Warning: KeyboardControlled entity has unexpected ControlInput size.");
                return; // Skip if size is wrong but not empty
            }
        }

        let current_steering = control_input.0[0];
        let current_acceleration = control_input.0[1];

        let mut target_steering_direction = 0.0; // -1 for right, 0 for center, 1 for left
        let mut target_acceleration_direction = 0.0; // -1 for brake/reverse, 0 for coast, 1 for accelerate

        // --- Determine Target Directions ---
        if keyboard_input.pressed(KeyCode::KeyW) || keyboard_input.pressed(KeyCode::ArrowUp) {
            target_acceleration_direction += 1.0;
        }
        if keyboard_input.pressed(KeyCode::KeyS) || keyboard_input.pressed(KeyCode::ArrowDown) {
            target_acceleration_direction -= 1.0; // Request braking/reverse
        }

        if keyboard_input.pressed(KeyCode::KeyA) || keyboard_input.pressed(KeyCode::ArrowLeft) {
            target_steering_direction += 1.0; // Request left steer
        }
        if keyboard_input.pressed(KeyCode::KeyD) || keyboard_input.pressed(KeyCode::ArrowRight) {
            target_steering_direction -= 1.0; // Request right steer
        }

        // --- Smooth Steering Update ---
        let mut next_steering = current_steering;
        if target_steering_direction != 0.0 {
            // Move towards max steer angle in the target direction
            let target_steer = target_steering_direction * max_steering_angle_rad;
            let steer_change = steering_rate * dt;
            if target_steer > current_steering {
                next_steering = (current_steering + steer_change).min(target_steer);
            } else if target_steer < current_steering {
                next_steering = (current_steering - steer_change).max(target_steer);
            }
        } else {
            // Return to center if no steering input
            let return_change = steering_return_rate * dt;
            if current_steering > 0.0 {
                next_steering = (current_steering - return_change).max(0.0);
            } else if current_steering < 0.0 {
                next_steering = (current_steering + return_change).min(0.0);
            }
        }
        // Ensure clamp after update
        next_steering = next_steering.clamp(-max_steering_angle_rad, max_steering_angle_rad);

        // --- Smooth Acceleration Update ---
        let mut next_acceleration = current_acceleration;
        if target_acceleration_direction > 0.0 {
            // Accelerating forward
            let target_accel = max_acceleration;
            let accel_change = acceleration_rate * dt;
            next_acceleration = (current_acceleration + accel_change).min(target_accel);
        } else if target_acceleration_direction < 0.0 {
            // Braking/Reversing
            let target_brake = -max_braking; // Target deceleration/reverse accel
            let brake_change = deceleration_rate * dt; // Use different rate for braking maybe
            next_acceleration = (current_acceleration - brake_change).max(target_brake);
        } else {
            // Coasting (keys released) - gradually reduce acceleration/deceleration to zero
            let coast_change = deceleration_rate * dt; // Rate at which accel/decel decays
            if current_acceleration > 0.0 {
                next_acceleration = (current_acceleration - coast_change).max(0.0);
            } else if current_acceleration < 0.0 {
                next_acceleration = (current_acceleration + coast_change).min(0.0);
            }
        }

        // --- Update Control Input Vector ---
        control_input.0[0] = next_steering;
        control_input.0[1] = next_acceleration;
    }
}
