// src/models/bicycle_kinematic.rs

use crate::simulation::components::{ControlInput, ControlVector, StateVector}; // Use StateVector alias if preferred
use crate::simulation::traits::{Control, Dynamics, State};
use avian3d::prelude::{AngularVelocity, LinearVelocity};
use bevy::prelude::{GlobalTransform, Quat, Vec3 as BevyVec3};
use nalgebra::{DMatrix, DVector};
use std::any::Any;

/// Kinematic Bicycle Model for car-like vehicles.
/// Assumes rear-wheel drive, steering on front wheels.
/// State vector: [x, y, theta, v] (position_x, position_y, heading_angle, longitudinal_velocity)
/// Control input: [delta, a] (steering_angle, longitudinal_acceleration)
#[derive(Debug, Clone)] // Add Debug, Clone might be useful
pub struct BicycleKinematicModel {
    /// Distance between front and rear axles (meters).
    pub wheelbase: f64,
    /// Maximum allowable steering angle (radians).
    pub max_steer_angle: f64,
}

impl BicycleKinematicModel {
    pub fn new(wheelbase: f64, max_steer_angle_deg: f64) -> Self {
        assert!(wheelbase > 0.0, "Wheelbase must be positive.");
        Self {
            wheelbase,
            max_steer_angle: max_steer_angle_deg.to_radians(),
        }
    }

    const STATE_DIM: usize = 4;
    const CONTROL_DIM: usize = 2;

    //     pub fn calculate_forces_and_torques(
    //         &self,
    //         current_bevy_transform: &GlobalTransform, // Current pose in Bevy world
    //         current_bevy_linear_velocity: &LinearVelocity, // Current velocity in Bevy world
    //         current_bevy_angular_velocity: &AngularVelocity,
    //         control_input: &ControlInput, // [delta_enu, accel_enu]
    //         car_mass: f32,
    //     ) -> (BevyVec3, BevyVec3) {
    //         // (world_force, world_torque)
    //
    //         let desired_steering_angle_enu = control_input.0[0] as f32; // Assuming this is now ENU steering
    //         let desired_longitudinal_accel_enu = control_input.0[1] as f32;
    //
    //         // --- Calculate Target Thrust Force (in vehicle's Bevy forward direction) ---
    //         let thrust_magnitude = desired_longitudinal_accel_enu * car_mass;
    //         let bevy_forward_vector = current_bevy_transform.forward(); // Bevy's -Z local is forward
    //         let world_thrust_force = bevy_forward_vector * thrust_magnitude * 10.0;
    //
    //         // --- Calculate Torque for Steering ---
    //         // This is a simplification. Real steering applies forces at wheels, creating torque.
    //         // For a simple model, we can apply a torque to make the car yaw.
    //         // Proportional controller for yaw rate based on steering angle.
    //         let current_bevy_yaw_rate = current_bevy_angular_velocity.y; // Assuming angular.y is yaw rate in Bevy
    //
    //         // --- Simple Steering Torque ---
    //         // This part is still highly simplified and needs a proper control law (e.g., PID on yaw or yaw rate)
    //         // For a basic proportional control on yaw rate towards a target yaw rate implied by steering:
    //         let target_yaw_rate_gain = 2.0; // Tune this: how fast should it try to achieve steering
    //         let desired_bevy_yaw_rate = -desired_steering_angle_enu * target_yaw_rate_gain; // Negative because positive ENU steer (left) is positive Bevy Y rot,
    //         // but a direct map to yaw rate might need inversion depending on convention.
    //         // This needs careful thought: if desired_steering_angle_enu is + (left turn),
    //         // you want a positive torque around Bevy Y.
    //
    //         let yaw_rate_error = desired_bevy_yaw_rate - current_bevy_yaw_rate;
    //         let steering_torque_gain = 2000.0; // Tune this: N*m / (rad/s)
    //         let max_yaw_torque = 5000.0;
    //
    //         // A positive ENU steering angle (turn left, CCW from above) should result in
    //         // a positive torque around Bevy's Y-axis (up).
    //         // Let's directly map steering angle to torque for simplicity first, then refine.
    //         let simple_steering_torque_y = desired_steering_angle_enu * steering_torque_gain * 5.0; // Increased gain
    //         // The sign here determines if left steer = left turn.
    //         // If +ENU steer means turn left (CCW from top),
    //         // and +Bevy Y rotation is CCW from top, then sign is positive.
    //
    //         let world_steering_torque = BevyVec3::new(
    //             0.0,
    //             simple_steering_torque_y.clamp(-max_yaw_torque, max_yaw_torque),
    //             0.0,
    //         );
    //
    //         // println!("Calc Forces: Thrust Mag={:.2}, Steer Angle={:.2}rad -> TorqueY={:.2}, CurrentYawRate={:.2}",
    //         //    thrust_magnitude, desired_steering_angle_enu, world_steering_torque.y, current_bevy_yaw_rate);
    //
    //         (world_thrust_force, world_steering_torque)
    //     }
    // }

    pub fn calculate_world_forces_and_torques(
        &self,
        current_bevy_transform: &GlobalTransform, // Used to get current world orientation
        _current_bevy_linear_velocity: &LinearVelocity, // May be used for drag, speed-dependent effects
        _current_bevy_angular_velocity: &AngularVelocity, // May be used for damping, stability
        control_input: &ControlInput,
        car_mass: f32,
        // Pass model-specific parameters if not part of `self` or if they vary
        _wheelbase: f32, // May be used for more advanced steering force distribution
        steering_torque_gain: f32,
        max_yaw_torque: f32,
    ) -> (BevyVec3, BevyVec3) {
        // (world_force_to_apply, world_torque_to_apply)

        let desired_longitudinal_accel_input =
            control_input.0.get(1).cloned().unwrap_or(0.0) as f32;
        let desired_steering_input_enu = control_input.0.get(0).cloned().unwrap_or(0.0) as f32; // ENU steering angle

        // --- Calculate World Thrust Force ---
        // 1. Define local forward direction for the car model (e.g., -Z if using Bevy's convention for meshes)
        let local_forward_direction = BevyVec3::X; // Car's "nose" points along its local -Z

        // 2. Get current world orientation of the car
        let car_world_rotation: Quat = current_bevy_transform.compute_transform().rotation; // Or global_transform.rotation() if GlobalTransform is deref to Transform

        // 3. Transform local forward direction to world space
        let world_forward_direction = car_world_rotation * local_forward_direction;

        // 4. Calculate thrust magnitude and the world force vector
        let thrust_magnitude = desired_longitudinal_accel_input * car_mass;
        let world_thrust_force = world_forward_direction * thrust_magnitude;

        // --- Calculate World Steering Torque ---
        // We want to apply a torque that rotates the car around its local Y-axis (up).
        // 1. Define local up direction (axis of yaw rotation for the car model)
        let local_up_direction = BevyVec3::Y;

        // 2. Transform local up direction to world space (this gives the world axis to rotate around)
        let world_up_axis_for_steering = car_world_rotation * local_up_direction;

        // 3. Calculate torque magnitude
        // Positive ENU steering (turn left, CCW from ENU Z-up) should correspond to
        // positive torque around Bevy local Y (up), which then translates to a world torque.
        // If desired_steering_input_enu > 0 means "steer left", and a positive torque around
        // the car's local Y-axis causes a "left turn" visually.
        let torque_magnitude_around_local_y = desired_steering_input_enu * steering_torque_gain;
        let clamped_torque_magnitude =
            torque_magnitude_around_local_y.clamp(-max_yaw_torque, max_yaw_torque);

        // 4. The world torque vector
        let world_steering_torque = world_up_axis_for_steering * clamped_torque_magnitude;

        // println!("Calc World Forces: WorldThrust={:?}, WorldTorque={:?}", world_thrust_force, world_steering_torque);
        // println!("  LocalFwd={:?}, WorldFwd={:?}, CarRot={:?}", local_forward_direction, world_forward_direction, car_world_rotation);
        // println!("  LocalUp={:?}, WorldUpAxisSteer={:?}, SteerInput={:.2}, TorqueMag={:.2}", local_up_direction, world_up_axis_for_steering, desired_steering_input_enu, clamped_torque_magnitude);

        (world_thrust_force, world_steering_torque)
    }
}

impl Dynamics for BicycleKinematicModel {
    fn get_state_dim(&self) -> usize {
        Self::STATE_DIM
    }

    fn get_control_dim(&self) -> usize {
        Self::CONTROL_DIM
    }

    /// Calculates x_dot = f(x, u, t)
    /// x = [x, y, theta, v_x]
    /// u = [delta, a]
    /// x_dot = [v*cos(theta), v*sin(theta), v*tan(delta)/L, a]
    fn get_derivatives(&self, x: &State, u: &Control, _t: f64) -> State {
        // --- Input Validation & Clamping ---
        // Ensure state and control vectors have correct dimensions
        if x.nrows() != Self::STATE_DIM || u.nrows() != Self::CONTROL_DIM {
            // Log error and return zero derivatives
            eprintln!("Error: BicycleKinematicModel received incorrect state/control dimensions.");
            return StateVector::zeros(Self::STATE_DIM);
        }

        let sim_yaw = x[2];
        let v = x[3];
        let delta = u[0].clamp(-self.max_steer_angle, self.max_steer_angle);
        let a = u[1];

        let x_dot = v * sim_yaw.cos(); // Component along x_sim (Forward)
        let y_dot = v * sim_yaw.sin(); // Component along y_sim (Left)
        let yaw_dot = if self.wheelbase.abs() < 1e-6 {
            0.0
        } else {
            (v * delta.tan()) / self.wheelbase
        };
        let v_dot = a;

        StateVector::from_vec(vec![x_dot, y_dot, yaw_dot, v_dot])
    }

    /// Calculates Jacobians A = df/dx, B = df/du (Optional but useful for EKF/LQR)
    fn calculate_jacobian(&self, x: &State, u: &Control, _t: f64) -> (DMatrix<f64>, DMatrix<f64>) {
        // --- Input Validation & Clamping --- (same as get_derivatives)
        if x.nrows() != Self::STATE_DIM || u.nrows() != Self::CONTROL_DIM {
            eprintln!(
                "Error: BicycleKinematicModel Jacobian received incorrect state/control dimensions."
            );
            // Return zero matrices of appropriate size
            return (
                DMatrix::zeros(Self::STATE_DIM, Self::STATE_DIM),
                DMatrix::zeros(Self::STATE_DIM, Self::CONTROL_DIM),
            );
        }

        let sim_yaw = x[2];
        let v = x[3];
        let delta = u[0].clamp(-self.max_steer_angle, self.max_steer_angle);
        let l = self.wheelbase;

        // A = df/dx = [ [df1/dx, df1/dz, df1/dyaw, df1/dv],
        //               [df2/dx, df2/dz, df2/dyaw, df2/dv],
        //               [df3/dx, df3/dz, df3/dyaw, df3/dv],
        //               [df4/dx, df4/dz, df4/dyaw, df4/dv] ]
        // f1 = v*sin(yaw) -> df1/dyaw = v*cos(yaw), df1/dv = sin(yaw)
        // f2 = v*cos(yaw) -> df2/dyaw = -v*sin(yaw), df2/dv = cos(yaw)
        // f3 = v*tan(delta)/L -> df3/dv = tan(delta)/L
        // f4 = a -> all zero partials w.r.t state

        let mut a_mat = DMatrix::<f64>::zeros(Self::STATE_DIM, Self::STATE_DIM);
        a_mat[(0, 2)] = v * sim_yaw.cos(); // df1/dyaw
        a_mat[(0, 3)] = sim_yaw.sin(); // df1/dv
        a_mat[(1, 2)] = -v * sim_yaw.sin(); // df2/dyaw
        a_mat[(1, 3)] = sim_yaw.cos(); // df2/dv
        a_mat[(2, 3)] = if l.abs() < 1e-6 { 0.0 } else { delta.tan() / l }; // df3/dv

        // B = df/du = [ [df1/ddelta, df1/da],
        //               [df2/ddelta, df2/da],
        //               [df3/ddelta, df3/da],
        //               [df4/ddelta, df4/da] ]
        // f1, f2 partials w.r.t u are 0
        // f3 = v*tan(delta)/L -> df3/ddelta = v / (L * cos(delta)^2)
        // f4 = a -> df4/da = 1

        let mut b_mat = DMatrix::<f64>::zeros(Self::STATE_DIM, Self::CONTROL_DIM);
        let cos_delta_sq = delta.cos().powi(2);
        b_mat[(2, 0)] = if l.abs() < 1e-6 || cos_delta_sq < 1e-6 {
            0.0
        } else {
            v / (l * cos_delta_sq)
        }; // df3/ddelta
        b_mat[(3, 1)] = 1.0; // df4/da

        (a_mat, b_mat)
    }

    /// Calculates feedforward control `u_ff = [delta, a]` to achieve `x_dot_desired`.
    /// Solves x_dot_desired = f(x, u_ff, t) for u_ff.
    /// Input: x_dot_desired = [x_dot_d, y_dot_d, theta_dot_d, v_dot_d]
    fn calculate_feedforward_input(
        &self,
        x: &State,
        x_dot_desired: &State,
        _t: f64,
    ) -> Option<Control> {
        if x.nrows() != Self::STATE_DIM || x_dot_desired.nrows() != Self::STATE_DIM {
            eprintln!(
                "Error: BicycleKinematicModel feedforward received incorrect state dimensions."
            );
            return None;
        }

        let v = x[3]; // Current velocity

        // Desired acceleration 'a' is directly given by desired v_dot
        let a_ff = x_dot_desired[3];

        // Desired angular velocity yaw_dot_d = v * tan(delta) / L
        // Solve for delta: delta = atan(yaw_dot_d * L / v)
        let yaw_dot_d = x_dot_desired[2];
        let delta_ff = if v.abs() < 1e-3 {
            if yaw_dot_d.abs() < 1e-3 {
                0.0
            } else {
                return None;
            }
        } else {
            (yaw_dot_d * self.wheelbase / v).atan()
        };
        let delta_ff_clamped = delta_ff.clamp(-self.max_steer_angle, self.max_steer_angle);

        // Optional Consistency Check: Do a_ff and delta_ff_clamped produce desired x_dot/z_dot?
        // x_dot_check = v * x[2].sin() // Check if this matches x_dot_desired[0] approximately
        // z_dot_check = v * x[2].cos() // Check if this matches x_dot_desired[1] approximately
        // If not, the desired derivatives might be kinematically inconsistent.

        Some(ControlVector::from_vec(vec![delta_ff_clamped, a_ff]))
    }

    fn as_any(&self) -> &dyn Any {
        self // `self` (which is &BicycleKinematicModel) can be coerced to &dyn Any
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self // `self` (which is &mut BicycleKinematicModel) can be coerced to &mut dyn Any
    }
}
