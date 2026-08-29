use crate::control::actuators::{ActuatorCommand, ActuatorId, SetpointKind, SetpointValue};
use crate::control::commands::BodyWrench;
use crate::frames::quantities::FluVector;

use nalgebra::Vector3;

/// The L0 "arcade" plant: the lowest-fidelity map from a resolved actuator
/// command to the force and torque acting on a single rigid chassis.
///
/// A real car turns per-wheel torques and a steer angle into ground-contact
/// forces through tire and suspension dynamics. The L0 shim skips all of that:
/// it treats the whole vehicle as one body and folds the resolved setpoints
/// straight into a chassis wrench by command space — a drive `Velocity` becomes
/// forward force, a steer `Position` (the steer angle) becomes yaw torque. The
/// two gains below are the only tuning it has.
///
/// The map is deliberately crude and open-loop: the velocity → force term does
/// not track the commanded speed (steady-state speed is set by force-balancing
/// drag and friction, not by the command), so actual speed only approximates
/// what was asked. Closing that loop is a brain-side speed controller emitting a
/// force setpoint, not something this shim does. It is retired wholesale when a
/// raycast or dynamic plant supplies real per-wheel forces; until then it is
/// what lets an arcade car drive with no wheel articulation.
pub struct L0ShimPlant {
    /// Newtons of forward force per unit of commanded drive velocity (m/s).
    force_gain: f64,
    /// Newton-metres of yaw torque per unit of commanded steer angle (rad).
    yaw_gain: f64,
}

impl L0ShimPlant {
    pub fn new(force_gain: f64, yaw_gain: f64) -> Self {
        Self {
            force_gain,
            yaw_gain,
        }
    }

    /// Fold a resolved actuator command into a body-frame ([`Flu`](crate::frames::conventions::Flu))
    /// wrench.
    ///
    /// Each setpoint is interpreted by its command space: a `Velocity` adds
    /// forward (`+x`) force, a `Position` adds yaw (`+z`) torque, both scaled by
    /// the matching gain. Setpoints sharing a command space accumulate, so the
    /// fold is total over any number of drive or steer actuators.
    ///
    /// A `Force` or `Torque` setpoint has no place in this open-loop map — the
    /// shim derives force from *velocity*, not from a force setpoint — so it
    /// contributes nothing to the wrench and its actuator is reported in
    /// [`unsupported`](L0ShimWrench::unsupported) for the host to surface. The
    /// articulated plant that consumes wheel torques directly is the next rung
    /// of the fidelity ladder.
    ///
    /// Expects an already-resolved command (see
    /// [`ActuationModel::resolve`](crate::control::actuation_model::ActuationModel::resolve)):
    /// every value is finite and speaks its actuator's declared command space,
    /// so the fold never guards against `NaN` or substitutes a fail-safe.
    pub fn fold(&self, command: &ActuatorCommand) -> L0ShimWrench {
        let mut force = Vector3::zeros();
        let mut torque = Vector3::zeros();
        let mut unsupported = Vec::new();

        for sp in command.setpoints() {
            match sp.value() {
                SetpointValue::Velocity(v) => force.x += v * self.force_gain,
                SetpointValue::Position(v) => torque.z += v * self.yaw_gain,
                SetpointValue::Force(_) | SetpointValue::Torque(_) => {
                    unsupported.push(sp.actuator().clone())
                }
            }
        }

        L0ShimWrench {
            wrench: BodyWrench::new(FluVector::from_raw(force), FluVector::from_raw(torque)),
            unsupported,
        }
    }

    /// Whether this shim can apply a setpoint of the given command space.
    ///
    /// It folds `Velocity` into force and `Position` into torque; `Force` and
    /// `Torque` have no place in the open-loop map. This is the same partition
    /// [`fold`](Self::fold) applies per setpoint, exposed ahead of any command so
    /// a host can reject an incompatible actuation contract at spawn rather than
    /// warn-and-ignore every tick. The wildcard-free match keeps the two in step:
    /// a new [`SetpointKind`] fails to compile here until it is classified.
    pub fn accepts(&self, kind: SetpointKind) -> bool {
        match kind {
            SetpointKind::Velocity | SetpointKind::Position => true,
            SetpointKind::Force | SetpointKind::Torque => false,
        }
    }
}

/// The result of folding a command through an [`L0ShimPlant`]: the chassis
/// wrench, plus any actuators whose setpoint the shim could not apply.
pub struct L0ShimWrench {
    wrench: BodyWrench,
    unsupported: Vec<ActuatorId>,
}

impl L0ShimWrench {
    /// The folded body-frame wrench the host applies to the chassis.
    pub fn wrench(&self) -> BodyWrench {
        self.wrench
    }

    /// Actuators whose command space this shim cannot apply — a `Force` or
    /// `Torque` setpoint reaching an L0 car, which is a wiring or config
    /// mismatch. Each contributes nothing to the wrench; the host warns on them.
    pub fn unsupported(&self) -> &[ActuatorId] {
        &self.unsupported
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::control::actuators::ActuatorSetpoint;

    fn setpoint(id: &str, value: SetpointValue) -> ActuatorSetpoint {
        ActuatorSetpoint::new(ActuatorId::new(id), value)
    }

    // Distinct gains so a test that swapped force for torque, or the two axes,
    // would fail on the magnitude rather than pass by coincidence.
    fn plant() -> L0ShimPlant {
        L0ShimPlant::new(1000.0, 15000.0)
    }

    #[test]
    fn drive_velocity_becomes_forward_force() {
        // A drive Velocity maps to +x force scaled by force_gain, and touches no
        // other force axis and no torque.
        let out = plant().fold(&ActuatorCommand::new(vec![setpoint(
            "drive",
            SetpointValue::Velocity(3.0),
        )]));

        assert_eq!(out.wrench().force(), FluVector::new(3000.0, 0.0, 0.0));
        assert_eq!(out.wrench().torque(), FluVector::zeros());
        assert!(out.unsupported().is_empty());
    }

    #[test]
    fn steer_position_becomes_yaw_torque() {
        // A steer Position maps to +z yaw torque scaled by yaw_gain, and touches
        // no force.
        let out = plant().fold(&ActuatorCommand::new(vec![setpoint(
            "steer",
            SetpointValue::Position(0.1),
        )]));

        assert_eq!(out.wrench().force(), FluVector::zeros());
        assert_eq!(out.wrench().torque(), FluVector::new(0.0, 0.0, 1500.0));
        assert!(out.unsupported().is_empty());
    }

    #[test]
    fn drive_and_steer_fold_into_one_wrench() {
        // The car's two-actuator command: both contributions land in the same
        // wrench, force on +x and torque on +z independently.
        let out = plant().fold(&ActuatorCommand::new(vec![
            setpoint("drive", SetpointValue::Velocity(2.0)),
            setpoint("steer", SetpointValue::Position(-0.2)),
        ]));

        assert_eq!(out.wrench().force(), FluVector::new(2000.0, 0.0, 0.0));
        assert_eq!(out.wrench().torque(), FluVector::new(0.0, 0.0, -3000.0));
    }

    #[test]
    fn same_space_setpoints_accumulate() {
        // Two drive actuators sum on the forward axis — the fold is total over
        // however many actuators share a command space, not just one each.
        let out = plant().fold(&ActuatorCommand::new(vec![
            setpoint("drive_l", SetpointValue::Velocity(1.0)),
            setpoint("drive_r", SetpointValue::Velocity(4.0)),
        ]));

        assert_eq!(out.wrench().force(), FluVector::new(5000.0, 0.0, 0.0));
    }

    #[test]
    fn force_and_torque_setpoints_are_unsupported() {
        // The shim commands force from velocity, so a Force or Torque setpoint
        // has no place in the map: it contributes nothing and is reported for the
        // host to warn on, in the order the command listed the actuators.
        let out = plant().fold(&ActuatorCommand::new(vec![
            setpoint("drive", SetpointValue::Force(500.0)),
            setpoint("brake", SetpointValue::Torque(10.0)),
        ]));

        assert_eq!(out.wrench(), BodyWrench::zero());
        assert_eq!(
            out.unsupported(),
            &[ActuatorId::new("drive"), ActuatorId::new("brake")]
        );
    }

    #[test]
    fn accepts_agrees_with_fold() {
        // The spawn-time guard (`accepts`) and the per-tick fold must classify a
        // command space the same way, or a kind the guard passed would still be
        // warned-and-ignored at runtime. Fold a lone setpoint of each kind and
        // check `accepts` predicts exactly whether it lands in the wrench.
        let p = plant();
        for kind in [
            SetpointKind::Force,
            SetpointKind::Torque,
            SetpointKind::Position,
            SetpointKind::Velocity,
        ] {
            let folded = p.fold(&ActuatorCommand::new(vec![setpoint("a", kind.value(1.0))]));
            assert_eq!(
                p.accepts(kind),
                folded.unsupported().is_empty(),
                "accepts and fold disagree for {kind:?}"
            );
        }
    }

    #[test]
    fn empty_command_folds_to_zero() {
        // The cold-start / all-safe-state case: nothing to fold yields the zero
        // wrench and nothing unsupported.
        let out = plant().fold(&ActuatorCommand::new(vec![]));

        assert_eq!(out.wrench(), BodyWrench::zero());
        assert!(out.unsupported().is_empty());
    }
}
