// helios_sim/src/simulation/plugins/vehicles/ackermann/adapter.rs

use avian3d::prelude::{AngularVelocity, LinearVelocity};
use bevy::prelude::*;
use helios_core::control::{siso_pid::SisoPid, ControlOutput};

use super::components::{AckermannActuator, AckermannCommand, AckermannParameters};

/// Translates a `ControlOutput` into an `AckermannCommand`.
/// Owns all feedback loops and intermediate calculations; Layer 3 sees only the final command.
pub trait AckermannOutputAdapter: Send + Sync {
    fn adapt(
        &mut self,
        output: &ControlOutput,
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

/// Open-loop adapter: direct port of the former `control_output_to_ackermann` free function.
/// No feedback controllers — passive damping is handled by Avian3D `Friction` / damping params.
pub struct DefaultAckermannAdapter;

impl AckermannOutputAdapter for DefaultAckermannAdapter {
    fn adapt(
        &mut self,
        output: &ControlOutput,
        params: &AckermannParameters,
        actuator: &AckermannActuator,
        _transform: &Transform,
        lin_vel: &LinearVelocity,
        _ang_vel: &AngularVelocity,
        mass: f32,
        _dt: f32,
    ) -> AckermannCommand {
        // Raw variants are direct normalised commands — no kinematic conversion.
        // The caller is already expressing a normalised intent, not a physical steering angle.
        match output {
            ControlOutput::Raw(u) => {
                return AckermannCommand {
                    throttle_norm: (u.get(0).copied().unwrap_or(0.0) as f32).clamp(-1.0, 1.0),
                    steering_torque_norm: (u.get(1).copied().unwrap_or(0.0) as f32)
                        .clamp(-1.0, 1.0),
                };
            }
            ControlOutput::RawActuators(signals) => {
                return AckermannCommand {
                    throttle_norm: (signals.first().copied().unwrap_or(0.0) as f32)
                        .clamp(-1.0, 1.0),
                    steering_torque_norm: (signals.get(1).copied().unwrap_or(0.0) as f32)
                        .clamp(-1.0, 1.0),
                };
            }
            _ => {}
        }

        // Physics-based variants: compute steering_rad first, then apply Ackermann kinematics
        // to convert it into a speed-scaled yaw rate → normalised torque demand.
        let (throttle_norm, steering_rad) = match output {
            ControlOutput::BodyVelocity { linear, angular } => {
                let throttle = (linear.x as f32) / actuator.max_speed;
                let v = lin_vel.length().max(0.1);
                let steering = (angular.z as f32 * params.wheelbase as f32 / v).atan();
                (throttle.clamp(-1.0, 1.0), steering)
            }
            ControlOutput::Wrench { force, torque } => {
                let throttle = (force.x as f32) / (actuator.max_force).max(1.0);
                let steering = (torque.z as f32 / (actuator.max_torque).max(1.0)).atan();
                (throttle.clamp(-1.0, 1.0), steering)
            }
            ControlOutput::BodyAcceleration { linear, .. } => {
                let throttle = (linear.x as f32 * mass) / actuator.max_force.max(1.0);
                (throttle.clamp(-1.0, 1.0), 0.0)
            }
            // Raw variants handled above; this arm is unreachable.
            _ => unreachable!(),
        };

        let yaw_rate_demand =
            (lin_vel.length() / params.wheelbase as f32) * steering_rad.tan();
        let max_yaw_rate =
            (actuator.max_speed / params.wheelbase as f32) * params.max_steering_angle.tan();
        let steering_torque_norm = (yaw_rate_demand / max_yaw_rate.max(0.01)).clamp(-1.0, 1.0);

        AckermannCommand { throttle_norm, steering_torque_norm }
    }
}

// =========================================================================
// == DualSisoPidAdapter — closed-loop ==
// =========================================================================

/// Two SISO PID channels:
///   - `longitudinal`: speed error (m/s) → raw force → `throttle_norm`
///   - `lateral`:      yaw-rate error (rad/s) → raw torque → `steering_torque_norm`
///
/// Only acts on `BodyVelocity` inputs. All other `ControlOutput` variants fall through
/// to the open-loop `DefaultAckermannAdapter`.
pub struct DualSisoPidAdapter {
    longitudinal: SisoPid,
    lateral: SisoPid,
    fallback: DefaultAckermannAdapter,
}

impl DualSisoPidAdapter {
    pub fn new(longitudinal: SisoPid, lateral: SisoPid) -> Self {
        Self { longitudinal, lateral, fallback: DefaultAckermannAdapter }
    }
}

impl AckermannOutputAdapter for DualSisoPidAdapter {
    fn adapt(
        &mut self,
        output: &ControlOutput,
        params: &AckermannParameters,
        actuator: &AckermannActuator,
        transform: &Transform,
        lin_vel: &LinearVelocity,
        ang_vel: &AngularVelocity,
        mass: f32,
        dt: f32,
    ) -> AckermannCommand {
        let ControlOutput::BodyVelocity { linear, angular } = output else {
            return self.fallback.adapt(output, params, actuator, transform, lin_vel, ang_vel, mass, dt);
        };

        // Longitudinal SISO: speed error → throttle_norm.
        // `transform.forward()` is the Bevy -Z forward direction for the vehicle,
        // which equals FLU +X projected into world space.
        let desired_speed = linear.x as f32;
        let current_forward_speed = lin_vel.dot(*transform.forward());
        let speed_error = desired_speed - current_forward_speed;
        let raw_force = self.longitudinal.update(speed_error as f64, dt as f64) as f32;
        let throttle_norm = (raw_force / actuator.max_force).clamp(-1.0, 1.0);

        // Lateral SISO: yaw-rate error → steering_torque_norm.
        // `ang_vel.y` is Bevy Y-up yaw rate, which matches FLU Z yaw rate in sign convention.
        let desired_yaw_rate = angular.z as f32;
        let current_yaw_rate = ang_vel.y;
        let yaw_error = desired_yaw_rate - current_yaw_rate;
        let raw_torque = self.lateral.update(yaw_error as f64, dt as f64) as f32;
        let steering_torque_norm = (raw_torque / actuator.max_torque).clamp(-1.0, 1.0);

        AckermannCommand { throttle_norm, steering_torque_norm }
    }
}
