use avian3d::prelude::{AngularVelocity, LinearVelocity};
use bevy::prelude::*;
use helios_core::control::{commands::BodyTwist, siso_pid::SisoPid};

use super::components::{AckermannActuator, AckermannCommand, AckermannParameters};

/// Translates a [`BodyTwist`] command into an `AckermannCommand`.
/// Owns all feedback loops and intermediate calculations; Layer 3 sees only the final command.
pub trait AckermannOutputAdapter: Send + Sync {
    #[allow(clippy::too_many_arguments)]
    fn adapt(
        &mut self,
        output: &BodyTwist,
        params: &AckermannParameters,
        actuator: &AckermannActuator,
        transform: &Transform,
        lin_vel: &LinearVelocity,
        ang_vel: &AngularVelocity,
        mass: f32,
        dt: f32,
    ) -> AckermannCommand;
}

/// Bevy component wrapping a boxed adapter so it can be queried per-entity.
#[derive(Component)]
pub struct AckermannAdapterComponent(pub Box<dyn AckermannOutputAdapter>);

// =========================================================================
// == DefaultAckermannAdapter — open-loop ==
// =========================================================================

/// Open-loop adapter: maps the commanded body twist directly to a steering demand
/// via Ackermann kinematics. No feedback controllers — passive damping is handled
/// by Avian3D `Friction` / damping params.
pub struct DefaultAckermannAdapter;

impl AckermannOutputAdapter for DefaultAckermannAdapter {
    fn adapt(
        &mut self,
        output: &BodyTwist,
        params: &AckermannParameters,
        actuator: &AckermannActuator,
        _transform: &Transform,
        lin_vel: &LinearVelocity,
        _ang_vel: &AngularVelocity,
        _mass: f32,
        _dt: f32,
    ) -> AckermannCommand {
        // The command is a body-FLU twist: linear.x is forward speed (m/s), angular.z
        // is yaw rate (rad/s). Recover the steering angle that produces that yaw rate
        // at the current speed, then apply Ackermann kinematics to get a speed-scaled
        // yaw-rate demand → normalised torque.
        let linear = output.linear();
        let angular = output.angular();

        let throttle_norm = ((linear.x() as f32) / actuator.max_speed).clamp(-1.0, 1.0);
        let v = lin_vel.length().max(0.1);
        let steering_rad = (angular.z() as f32 * params.wheelbase as f32 / v).atan();

        let yaw_rate_demand = (lin_vel.length() / params.wheelbase as f32) * steering_rad.tan();
        let max_yaw_rate =
            (actuator.max_speed / params.wheelbase as f32) * params.max_steering_angle.tan();
        let steering_torque_norm = (yaw_rate_demand / max_yaw_rate.max(0.01)).clamp(-1.0, 1.0);

        AckermannCommand {
            throttle_norm,
            steering_torque_norm,
        }
    }
}

// =========================================================================
// == DualSisoPidAdapter — closed-loop ==
// =========================================================================

/// Two SISO PID channels closing the loop on the commanded body twist:
///   - `longitudinal`: speed error (m/s) → raw force → `throttle_norm`
///   - `lateral`:      yaw-rate error (rad/s) → raw torque → `steering_torque_norm`
pub struct DualSisoPidAdapter {
    longitudinal: SisoPid,
    lateral: SisoPid,
}

impl DualSisoPidAdapter {
    pub fn new(longitudinal: SisoPid, lateral: SisoPid) -> Self {
        Self {
            longitudinal,
            lateral,
        }
    }
}

impl AckermannOutputAdapter for DualSisoPidAdapter {
    fn adapt(
        &mut self,
        output: &BodyTwist,
        _params: &AckermannParameters,
        actuator: &AckermannActuator,
        transform: &Transform,
        lin_vel: &LinearVelocity,
        ang_vel: &AngularVelocity,
        _mass: f32,
        dt: f32,
    ) -> AckermannCommand {
        let linear = output.linear();
        let angular = output.angular();

        // Longitudinal SISO: speed error → throttle_norm.
        // `transform.forward()` is the Bevy -Z forward direction for the vehicle,
        // which equals FLU +X projected into world space.
        let desired_speed = linear.x() as f32;
        let current_forward_speed = lin_vel.dot(*transform.forward());
        let speed_error = desired_speed - current_forward_speed;
        let raw_force = self.longitudinal.update(speed_error as f64, dt as f64) as f32;
        let throttle_norm = (raw_force / actuator.max_force).clamp(-1.0, 1.0);

        // Lateral SISO: yaw-rate error → steering_torque_norm.
        // `ang_vel.y` is Bevy Y-up yaw rate, which matches FLU Z yaw rate in sign convention.
        let desired_yaw_rate = angular.z() as f32;
        let current_yaw_rate = ang_vel.y;
        let yaw_error = desired_yaw_rate - current_yaw_rate;
        let raw_torque = self.lateral.update(yaw_error as f64, dt as f64) as f32;
        let steering_torque_norm = (raw_torque / actuator.max_torque).clamp(-1.0, 1.0);

        AckermannCommand {
            throttle_norm,
            steering_torque_norm,
        }
    }
}
