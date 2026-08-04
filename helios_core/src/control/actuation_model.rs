//! A body's actuator contract: which actuators it exposes and how each behaves.
//!
//! [`ActuationModel`] is portable core vocabulary — the *same* model describes
//! the sim vehicle and the real one, loaded from the entity file. It is what
//! [`ActuatorCommand::validate`](crate::control::actuators::ActuatorCommand::validate)
//! checks a command against, and what the host reads to know each actuator's
//! saturation limit, fail-safe value, and sign convention.

use crate::control::actuators::{ActuatorId, SetpointKind, SetpointValue};

use serde::{Deserialize, Serialize};

/// Every actuator a body exposes, in declaration order.
#[derive(Clone, PartialEq, Debug, Deserialize, Serialize)]
pub struct ActuationModel {
    actuators: Vec<ActuatorSpec>,
}

impl ActuationModel {
    pub fn new(actuators: Vec<ActuatorSpec>) -> Self {
        Self { actuators }
    }

    /// The spec for one actuator by id, or `None` if this body has no such
    /// actuator.
    pub fn spec(&self, id: &ActuatorId) -> Option<&ActuatorSpec> {
        self.actuators.iter().find(|a| a.id == *id)
    }

    /// Every actuator spec, in declaration order.
    pub fn actuators(&self) -> &[ActuatorSpec] {
        &self.actuators
    }
}

/// The contract for one physical actuator: its identity, command space, and the
/// three safety-relevant facts the host needs to drive it — how far it may go,
/// what to command when the pipeline can't, and which way is positive.
#[derive(Clone, PartialEq, Debug, Deserialize, Serialize)]
pub struct ActuatorSpec {
    /// Stable identity, matched against each setpoint's actuator.
    id: ActuatorId,
    /// The one command space this actuator accepts (drive: torque, steer: position).
    kind: SetpointKind,
    /// Symmetric saturation bound: a setpoint is clamped to `|value| ≤ limit`
    /// (drive: max wheel torque; steer: max steering angle).
    limit: f64,
    /// The value to command on fail-safe — comms loss or a stalled pipeline. A
    /// hardware-safety requirement, not a nicety (drive: `Torque(0)` coast; steer:
    /// `Position(0)` center).
    safe_state: SetpointValue,
    /// Which physical direction counts as positive, so an allocator↔host
    /// disagreement can't silently invert the actuator.
    sign: SignConvention,
}

impl ActuatorSpec {
    pub fn kind(&self) -> SetpointKind {
        self.kind
    }

    pub fn id(&self) -> &ActuatorId {
        &self.id
    }
}

/// Which physical direction a positive setpoint drives an actuator.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Deserialize, Serialize)]
pub enum SignConvention {
    /// Positive setpoint → the actuator's natural positive direction.
    Normal,
    /// Positive setpoint → the actuator's natural negative direction.
    Inverted,
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::control::actuators::{ActuatorCommand, ActuatorCommandError, ActuatorSetpoint};

    // A two-actuator car: a torque-commanded drive and a position-commanded steer.
    // Constructed via struct literals (private fields are visible in-module) so the
    // tests need no public `ActuatorSpec` constructor.
    fn drive_spec() -> ActuatorSpec {
        ActuatorSpec {
            id: ActuatorId::new("drive"),
            kind: SetpointKind::Torque,
            limit: 100.0,
            safe_state: SetpointValue::Torque(0.0),
            sign: SignConvention::Normal,
        }
    }

    fn steer_spec() -> ActuatorSpec {
        ActuatorSpec {
            id: ActuatorId::new("steer"),
            kind: SetpointKind::Position,
            limit: 0.6,
            safe_state: SetpointValue::Position(0.0),
            sign: SignConvention::Normal,
        }
    }

    fn car_model() -> ActuationModel {
        ActuationModel::new(vec![drive_spec(), steer_spec()])
    }

    fn setpoint(id: &str, value: SetpointValue) -> ActuatorSetpoint {
        ActuatorSetpoint::new(ActuatorId::new(id), value)
    }

    #[test]
    fn well_formed_command_validates() {
        let cmd = ActuatorCommand::new(vec![
            setpoint("drive", SetpointValue::Torque(10.0)),
            setpoint("steer", SetpointValue::Position(0.2)),
        ]);
        assert!(cmd.validate(&car_model()).is_ok());
    }

    #[test]
    fn setpoint_for_undeclared_actuator_is_unknown() {
        let cmd = ActuatorCommand::new(vec![
            setpoint("drive", SetpointValue::Torque(10.0)),
            setpoint("headlights", SetpointValue::Force(1.0)),
            setpoint("steer", SetpointValue::Position(0.0)),
        ]);
        let errors = cmd
            .validate(&car_model())
            .expect_err("unknown actuator must fail");
        assert!(errors.iter().any(|e| matches!(
            e,
            ActuatorCommandError::UnknownActuator { actuator }
                if actuator == &ActuatorId::new("headlights")
        )));
    }

    #[test]
    fn wrong_command_space_is_a_kind_mismatch() {
        // Drive is torque-commanded; a Force is the wrong command space for it.
        let cmd = ActuatorCommand::new(vec![
            setpoint("drive", SetpointValue::Force(10.0)),
            setpoint("steer", SetpointValue::Position(0.0)),
        ]);
        let errors = cmd
            .validate(&car_model())
            .expect_err("kind mismatch must fail");
        assert!(errors.iter().any(|e| matches!(
            e,
            ActuatorCommandError::KindMismatch {
                expected: SetpointKind::Torque,
                found: SetpointKind::Force,
                ..
            }
        )));
    }

    #[test]
    fn non_finite_value_is_rejected() {
        // The open-loop steer inverse atan(wz·L / v) yields NaN at v ≈ 0.
        let cmd = ActuatorCommand::new(vec![
            setpoint("drive", SetpointValue::Torque(0.0)),
            setpoint("steer", SetpointValue::Position(f64::NAN)),
        ]);
        let errors = cmd
            .validate(&car_model())
            .expect_err("non-finite must fail");
        assert!(errors.iter().any(|e| matches!(
            e,
            ActuatorCommandError::NonFinite { actuator }
                if actuator == &ActuatorId::new("steer")
        )));
    }

    #[test]
    fn omitted_actuator_is_missing() {
        // Steer left out entirely: the command is not total over the model.
        let cmd = ActuatorCommand::new(vec![setpoint("drive", SetpointValue::Torque(10.0))]);
        let errors = cmd
            .validate(&car_model())
            .expect_err("missing actuator must fail");
        assert!(errors.iter().any(|e| matches!(
            e,
            ActuatorCommandError::MissingActuator { actuator }
                if actuator == &ActuatorId::new("steer")
        )));
    }

    #[test]
    fn repeated_actuator_is_a_duplicate() {
        // Two setpoints naming "drive": a malformed command a Vec can't prevent but
        // validate must catch, since the applier would otherwise pick one arbitrarily.
        let cmd = ActuatorCommand::new(vec![
            setpoint("drive", SetpointValue::Torque(10.0)),
            setpoint("drive", SetpointValue::Torque(20.0)),
            setpoint("steer", SetpointValue::Position(0.0)),
        ]);
        let errors = cmd
            .validate(&car_model())
            .expect_err("duplicate actuator must fail");
        assert!(errors.iter().any(|e| matches!(
            e,
            ActuatorCommandError::DuplicateActuator { actuator }
                if actuator == &ActuatorId::new("drive")
        )));
    }

    #[test]
    fn every_failure_is_reported_at_once() {
        // One unknown setpoint, and neither declared actuator covered: the single
        // ghost setpoint is unknown, and both drive and steer are missing.
        let cmd = ActuatorCommand::new(vec![setpoint("ghost", SetpointValue::Force(1.0))]);
        let errors = cmd
            .validate(&car_model())
            .expect_err("malformed command must fail");
        assert_eq!(errors.len(), 3);
    }

    #[test]
    fn model_round_trips_through_serde() {
        let model = car_model();
        let json = serde_json::to_string(&model).expect("serialize");
        let back: ActuationModel = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(model, back);
    }
}
