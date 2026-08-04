use crate::control::{
    actuators::{ActuatorCommand, ActuatorId, ActuatorSetpoint, SetpointValue},
    allocation::Allocator,
    commands::BodyTwist,
};

/// Maps a vehicle-level [`BodyTwist`] onto Ackermann actuator setpoints via the
/// bicycle model: a front steer angle and a rear wheel angular velocity.
///
/// Both outputs are kinematic feedforward — the steer angle inverts the yaw
/// kinematics `ωz = vx·tan(δ)/L`, and the wheel speed inverts `vx = ωwheel·r`.
/// The allocator is stateless and emits *raw* values: saturation to each
/// actuator's limit, the sign convention, and the fail-safe are the host's job,
/// declared once in the
/// [`ActuationModel`](crate::control::actuation_model::ActuationModel) and never
/// duplicated here.
pub struct KinematicAckermannAllocator {
    wheelbase: f64,    // L (m) — front-to-rear axle distance
    wheel_radius: f64, // r (m) — divides body speed to wheel angular speed (vx / r)
    drive: ActuatorId, // receives the wheel angular-velocity setpoint
    steer: ActuatorId, // receives the steer-angle position setpoint
}

impl KinematicAckermannAllocator {
    /// Forward-speed *magnitude* (m/s) below which the steer angle is centered
    /// rather than computed. The inverse `atan(L·ωz/vx)` is ill-conditioned as
    /// `vx → 0` (and `0/0 → NaN` at a standstill), so near zero the wheel holds
    /// straight instead of chasing a garbage angle.
    const MIN_STEER_SPEED: f64 = 0.1;

    pub fn new(wheelbase: f64, wheel_radius: f64, drive: ActuatorId, steer: ActuatorId) -> Self {
        Self {
            wheelbase,
            wheel_radius,
            drive,
            steer,
        }
    }
}

impl Allocator for KinematicAckermannAllocator {
    type In = BodyTwist;
    type Inputs = ();

    /// Reads forward speed and yaw rate off the body twist (FLU: `+X` forward,
    /// `+Z` up) and returns one setpoint per actuator — a steer-angle position
    /// and a wheel angular velocity. Never clamps; see the type doc.
    fn allocate(&mut self, command: &Self::In, _inputs: &Self::Inputs) -> ActuatorCommand {
        let vel_x = command.linear().x();
        let ang_vel_z = command.angular().z();

        // δ = atan(L·ωz / vx), guarded near vx = 0 where the inverse blows up.
        let steer_angle = if vel_x.abs() < Self::MIN_STEER_SPEED {
            0.0
        } else {
            (self.wheelbase * ang_vel_z / vel_x).atan()
        };

        // ωwheel = vx / r. Carries the sign of travel, so reverse falls out for free.
        let wheel_speed = vel_x / self.wheel_radius;

        ActuatorCommand::new(vec![
            ActuatorSetpoint::new(self.steer.clone(), SetpointValue::Position(steer_angle)),
            ActuatorSetpoint::new(self.drive.clone(), SetpointValue::Velocity(wheel_speed)),
        ])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::frames::conventions::FluVector;

    const WHEELBASE: f64 = 2.0;
    const WHEEL_RADIUS: f64 = 0.3;

    fn allocator() -> KinematicAckermannAllocator {
        KinematicAckermannAllocator::new(
            WHEELBASE,
            WHEEL_RADIUS,
            ActuatorId::new("drive"),
            ActuatorId::new("steer"),
        )
    }

    fn twist(vx: f64, wz: f64) -> BodyTwist {
        BodyTwist::new(FluVector::new(vx, 0.0, 0.0), FluVector::new(0.0, 0.0, wz))
    }

    // The setpoint for one actuator, failing if the command omits it.
    fn value_of(cmd: &ActuatorCommand, id: &str) -> SetpointValue {
        cmd.setpoints()
            .iter()
            .find(|sp| sp.actuator() == &ActuatorId::new(id))
            .expect("actuator present in command")
            .value()
            .clone()
    }

    #[test]
    fn straight_drive_centers_steer_and_scales_wheel_speed() {
        let cmd = allocator().allocate(&twist(6.0, 0.0), &());
        assert_eq!(value_of(&cmd, "steer"), SetpointValue::Position(0.0));
        assert_eq!(
            value_of(&cmd, "drive"),
            SetpointValue::Velocity(6.0 / WHEEL_RADIUS)
        );
    }

    #[test]
    fn command_is_total_and_correctly_typed() {
        // One setpoint per declared actuator, each in the actuator's command space.
        let cmd = allocator().allocate(&twist(3.0, 0.5), &());
        assert_eq!(cmd.setpoints().len(), 2);
        assert!(matches!(value_of(&cmd, "steer"), SetpointValue::Position(_)));
        assert!(matches!(value_of(&cmd, "drive"), SetpointValue::Velocity(_)));
    }

    #[test]
    fn left_yaw_gives_positive_steer_angle() {
        // Positive yaw rate (turning left, FLU +Z up) → positive steer angle.
        let cmd = allocator().allocate(&twist(4.0, 0.8), &());
        let SetpointValue::Position(delta) = value_of(&cmd, "steer") else {
            panic!("steer must be a position setpoint");
        };
        assert!(delta > 0.0, "expected positive steer, got {delta}");
        assert!((delta - (WHEELBASE * 0.8 / 4.0).atan()).abs() < 1e-12);
    }

    #[test]
    fn reverse_with_yaw_still_steers() {
        // Backing up (vx < 0) must not collapse the steer to center: the low-speed
        // guard is a magnitude threshold, not a signed one.
        let cmd = allocator().allocate(&twist(-5.0, 0.6), &());
        let SetpointValue::Position(delta) = value_of(&cmd, "steer") else {
            panic!("steer must be a position setpoint");
        };
        assert!(delta.is_finite());
        assert!(delta != 0.0, "reverse must still steer, got {delta}");
        assert_eq!(
            value_of(&cmd, "drive"),
            SetpointValue::Velocity(-5.0 / WHEEL_RADIUS)
        );
    }

    #[test]
    fn near_zero_speed_centers_steer_without_nan() {
        // At a standstill the atan inverse is 0/0; the guard yields a finite,
        // centered steer rather than NaN.
        let cmd = allocator().allocate(&twist(0.0, 0.7), &());
        assert_eq!(value_of(&cmd, "steer"), SetpointValue::Position(0.0));
    }
}
