use crate::control::{
    actuators::{ActuatorCommand, ActuatorId, ActuatorSetpoint, SetpointValue},
    allocation::Allocator,
    commands::SteerAngle,
};

/// Lifts a scalar [`SteerAngle`] onto a single steering actuator's position
/// setpoint — a one-to-one, pass-through allocation.
///
/// This is the lateral twin of
/// [`WheelTorqueAllocator`](crate::control::allocation::wheeled::drive_torque::WheelTorqueAllocator):
/// where that lift scales a drive force to a wheel torque through the wheel radius
/// (`τ = F·r`), this one has no lever arm — a steer *angle* already is the steering
/// joint's *position*, so the value passes through unchanged. The type still earns
/// its place: it names the steer actuator and packs the setpoint, the degenerate
/// one-input allocator that closes the decoupled lateral leg.
///
/// It commands **one degree of freedom** — the steer joint — and emits exactly one
/// setpoint. A driven vehicle pairs it with a longitudinal lift that contributes
/// the drive setpoint; the two partial commands merge into the one actuator
/// command.
///
/// Stateless and *raw*: the position is emitted unclamped. Saturation to the steer
/// limit, the sign convention, and the fail-safe belong to the host's
/// [`ActuationModel`](crate::control::actuation_model::ActuationModel), never here.
/// There is no low-speed singularity to guard — the ill-conditioned inverse lives
/// upstream in the bicycle-steer feedforward; this lift is well-defined for every
/// angle.
pub struct SteerPositionAllocator {
    /// The steering actuator that receives the position setpoint.
    steer: ActuatorId,
}

impl SteerPositionAllocator {
    /// Constructs the lift targeting the given steering actuator.
    pub fn new(steer: ActuatorId) -> Self {
        Self { steer }
    }
}

impl Allocator for SteerPositionAllocator {
    type In = SteerAngle;
    type Inputs = ();

    /// Packs the commanded angle as a single [`Position`](SetpointValue::Position)
    /// setpoint on the steer actuator. Identity in value — no radius, no scaling —
    /// and never clamped; see the type doc.
    fn allocate(&mut self, command: &Self::In, _inputs: &Self::Inputs) -> ActuatorCommand {
        let sp = ActuatorSetpoint::new(
            self.steer.clone(),
            SetpointValue::Position(command.radians()),
        );

        ActuatorCommand::new(vec![sp])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Tests build the struct directly; a registry-facing constructor is the
    // allocator factory's concern, not this leaf's.
    fn allocator() -> SteerPositionAllocator {
        SteerPositionAllocator {
            steer: ActuatorId::new("steer"),
        }
    }

    // The position setpoint on the steer actuator, failing if it is absent or not
    // a position.
    fn steer_position(cmd: &ActuatorCommand) -> f64 {
        let setpoint = cmd
            .setpoints()
            .iter()
            .find(|sp| sp.actuator() == &ActuatorId::new("steer"))
            .expect("steer actuator present in command");
        let SetpointValue::Position(value) = setpoint.value() else {
            panic!("steer setpoint must be a position");
        };
        *value
    }

    #[test]
    fn angle_passes_through_to_position() {
        // No lever arm, unlike the torque lift: the commanded angle is the joint
        // position setpoint unchanged.
        let cmd = allocator().allocate(&SteerAngle::new(0.3), &());
        assert_eq!(steer_position(&cmd), 0.3);
    }

    #[test]
    fn negative_angle_keeps_its_sign() {
        // A right-hand steer carries through with its sign intact.
        let cmd = allocator().allocate(&SteerAngle::new(-0.3), &());
        assert_eq!(steer_position(&cmd), -0.3);
    }

    #[test]
    fn zero_angle_commands_zero_position() {
        // A centered wheel maps to a zero position setpoint — no singularity here,
        // unlike the upstream inverse.
        let cmd = allocator().allocate(&SteerAngle::zero(), &());
        assert_eq!(steer_position(&cmd), 0.0);
    }

    #[test]
    fn command_is_a_single_position_setpoint() {
        // One degree of freedom: exactly one setpoint, on the steer actuator, in
        // the position command space.
        let cmd = allocator().allocate(&SteerAngle::new(0.2), &());
        assert_eq!(cmd.setpoints().len(), 1);
        let only = &cmd.setpoints()[0];
        assert_eq!(only.actuator(), &ActuatorId::new("steer"));
        assert!(matches!(only.value(), SetpointValue::Position(_)));
    }
}
