use crate::control::{
    actuators::{ActuatorCommand, ActuatorId, ActuatorSetpoint, SetpointValue},
    allocation::Allocator,
    commands::DriveForce,
};

/// Maps a longitudinal [`DriveForce`] onto a single drive wheel's torque:
/// `τ = F · r`, where `r` is the wheel radius.
///
/// This is the inverse of the kinematic drive relation the Ackermann allocator
/// uses on the other side (`vx = ω · r`): there a *speed* is converted to a wheel
/// angular velocity; here a *force* is converted to a wheel torque. Commanding
/// torque rather than velocity is the higher-fidelity rung — it lets a speed→force
/// controller (feedback and feedforward summed) close the longitudinal loop through
/// the actuator's real command space.
///
/// It commands **one degree of freedom** — forward effort. A `DriveForce` carries
/// no yaw, so there is nothing to steer with, and the emitted command holds exactly
/// one setpoint (the drive actuator). A steered vehicle pairs this with a lateral
/// allocator that contributes the steer setpoint; on its own it drives in a straight
/// line.
///
/// Stateless and *raw*: the returned torque is unclamped. Saturation to the
/// actuator's limit, the sign convention, and the fail-safe belong to the host's
/// [`ActuationModel`](crate::control::actuation_model::ActuationModel), never here.
/// Unlike the steer inverse there is no low-speed singularity to guard — `τ = F · r`
/// is well-defined everywhere, including at a standstill.
pub struct WheelTorqueAllocator {
    /// Wheel radius `r` (m) — the lever arm converting drive force to wheel torque.
    wheel_radius: f64,
    /// The actuator that receives the wheel-torque setpoint.
    drive: ActuatorId,
}

impl Allocator for WheelTorqueAllocator {
    type In = DriveForce;
    type Inputs = ();

    /// Converts the commanded force to a wheel torque `τ = F · r` and returns a
    /// single-actuator command. The sign of the force carries through, so reverse
    /// thrust falls out for free. Never clamps; see the type doc.
    fn allocate(&mut self, command: &Self::In, _inputs: &Self::Inputs) -> ActuatorCommand {
        let torque = command.newtons() * self.wheel_radius;

        let setpoints = vec![ActuatorSetpoint::new(
            self.drive.clone(),
            SetpointValue::Torque(torque),
        )];

        ActuatorCommand::new(setpoints)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const WHEEL_RADIUS: f64 = 0.3;

    // Tests build the struct directly; a registry-facing constructor is the
    // allocator factory's concern, not this leaf's.
    fn allocator() -> WheelTorqueAllocator {
        WheelTorqueAllocator {
            wheel_radius: WHEEL_RADIUS,
            drive: ActuatorId::new("drive"),
        }
    }

    // The torque setpoint on the drive actuator, failing if it is absent or not a
    // torque.
    fn drive_torque(cmd: &ActuatorCommand) -> f64 {
        let setpoint = cmd
            .setpoints()
            .iter()
            .find(|sp| sp.actuator() == &ActuatorId::new("drive"))
            .expect("drive actuator present in command");
        let SetpointValue::Torque(value) = setpoint.value() else {
            panic!("drive setpoint must be a torque");
        };
        *value
    }

    #[test]
    fn force_scales_to_torque_by_wheel_radius() {
        // τ = F · r = 100 · 0.3 = 30.
        let cmd = allocator().allocate(&DriveForce::new(100.0), &());
        assert!((drive_torque(&cmd) - 30.0).abs() < 1e-12);
    }

    #[test]
    fn reverse_force_flips_torque_sign() {
        // The sign carries through unchanged: a braking / reversing force yields a
        // reverse-facing torque with no special case.
        let cmd = allocator().allocate(&DriveForce::new(-100.0), &());
        assert!((drive_torque(&cmd) + 30.0).abs() < 1e-12);
    }

    #[test]
    fn zero_force_commands_zero_torque() {
        // No singularity at rest, unlike the steer inverse: zero in, zero out.
        let cmd = allocator().allocate(&DriveForce::zero(), &());
        assert_eq!(drive_torque(&cmd), 0.0);
    }

    #[test]
    fn command_is_a_single_torque_setpoint() {
        // One degree of freedom: exactly one setpoint, on the drive actuator, in
        // the torque command space — never a steer setpoint, because a DriveForce
        // carries no yaw to steer with.
        let cmd = allocator().allocate(&DriveForce::new(5.0), &());
        assert_eq!(cmd.setpoints().len(), 1);
        let only = &cmd.setpoints()[0];
        assert_eq!(only.actuator(), &ActuatorId::new("drive"));
        assert!(matches!(only.value(), SetpointValue::Torque(_)));
    }
}
