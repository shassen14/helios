//! A body's actuator contract: which actuators it exposes and how each behaves.
//!
//! [`ActuationModel`] is portable core vocabulary — the *same* model describes
//! the sim vehicle and the real one, loaded from the entity file. It is what
//! [`ActuatorCommand::validate`](crate::control::actuators::ActuatorCommand::validate)
//! checks a command against, and what the host reads to know each actuator's
//! saturation limit, fail-safe value, and sign convention.

use crate::control::actuators::{
    ActuatorCommand, ActuatorId, ActuatorSetpoint, SetpointKind, SetpointValue,
};

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

    /// Enforce this body's actuator contract on a command, yielding one that is
    /// always safe to apply.
    ///
    /// Unlike [`ActuatorCommand::validate`](crate::control::actuators::ActuatorCommand::validate),
    /// which *reports* whether a command is well-formed, `resolve` never fails —
    /// it is the fail-safe the host applies every tick, including when the command
    /// never passed validation (a cold-start empty command, a stale command after
    /// a pipeline stall, a buggy allocator). The result is therefore always *total*
    /// over the model: exactly one setpoint per declared actuator, in declaration
    /// order.
    ///
    /// For each declared actuator, the matching setpoint is used only when it is
    /// genuinely applicable — its value speaks the actuator's declared command
    /// space and is finite; when it is missing, of the wrong kind, or non-finite,
    /// the actuator's `safe_state` is substituted. An applicable value is passed
    /// through [`ActuatorSpec::enforce`], which applies the sign convention and
    /// clamps to the saturation limit. Setpoints naming actuators this body does
    /// not declare are simply never looked up, so they drop out.
    pub fn resolve(&self, command: &ActuatorCommand) -> ActuatorCommand {
        let setpoints = self
            .actuators
            .iter()
            .map(|spec| {
                let value = command
                    .setpoints()
                    .iter()
                    .find(|sp| sp.actuator() == spec.id())
                    .map(|sp| sp.value())
                    .filter(|v| v.kind() == spec.kind() && v.scalar().is_finite())
                    .map(|v| spec.enforce(v.scalar()))
                    .unwrap_or_else(|| spec.safe_state.clone());
                ActuatorSetpoint::new(spec.id.clone(), value)
            })
            .collect();

        ActuatorCommand::new(setpoints)
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
    /// Construct a spec directly. Production bodies deserialize their actuation
    /// model from the entity file; this is for programmatic construction — a
    /// hand-built body, or a test.
    pub fn new(
        id: ActuatorId,
        kind: SetpointKind,
        limit: f64,
        safe_state: SetpointValue,
        sign: SignConvention,
    ) -> Self {
        Self {
            id,
            kind,
            limit,
            safe_state,
            sign,
        }
    }

    pub fn kind(&self) -> SetpointKind {
        self.kind
    }

    pub fn id(&self) -> &ActuatorId {
        &self.id
    }

    /// Coerce a raw commanded scalar into a setpoint this actuator can safely
    /// accept: apply the sign convention, then clamp to the symmetric saturation
    /// bound `|value| ≤ limit`. The `.abs()` on `limit` guards the clamp — a
    /// negative limit in config would otherwise make `min > max` and panic. The
    /// result carries this actuator's declared command space.
    fn enforce(&self, scalar: f64) -> SetpointValue {
        let signed = match self.sign {
            SignConvention::Normal => scalar,
            SignConvention::Inverted => -scalar,
        };

        let bound = self.limit.abs();

        self.kind.value(signed.clamp(-bound, bound))
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

    // The resolved command is total, so every declared actuator is present; this
    // pulls one out by id to assert on its value.
    fn value_for<'a>(command: &'a ActuatorCommand, id: &str) -> &'a SetpointValue {
        command
            .setpoints()
            .iter()
            .find(|sp| sp.actuator() == &ActuatorId::new(id))
            .map(|sp| sp.value())
            .expect("resolve produces a total command, so the actuator must be present")
    }

    #[test]
    fn resolve_is_total_and_ordered_dropping_unknowns() {
        // Steer first, an undeclared actuator in the middle, drive last: resolve
        // must return exactly the model's actuators, in the model's declaration
        // order (drive, steer), and drop the ghost.
        let cmd = ActuatorCommand::new(vec![
            setpoint("steer", SetpointValue::Position(0.1)),
            setpoint("headlights", SetpointValue::Force(1.0)),
            setpoint("drive", SetpointValue::Torque(5.0)),
        ]);
        let resolved = car_model().resolve(&cmd);
        let ids: Vec<&ActuatorId> = resolved
            .setpoints()
            .iter()
            .map(|sp| sp.actuator())
            .collect();
        assert_eq!(
            ids,
            vec![&ActuatorId::new("drive"), &ActuatorId::new("steer")]
        );
    }

    #[test]
    fn in_range_command_passes_through_unchanged() {
        let cmd = ActuatorCommand::new(vec![
            setpoint("drive", SetpointValue::Torque(10.0)),
            setpoint("steer", SetpointValue::Position(0.2)),
        ]);
        let resolved = car_model().resolve(&cmd);
        assert_eq!(value_for(&resolved, "drive"), &SetpointValue::Torque(10.0));
        assert_eq!(value_for(&resolved, "steer"), &SetpointValue::Position(0.2));
    }

    #[test]
    fn over_limit_value_is_clamped_symmetrically() {
        // Drive limit is 100; both a large positive and a large negative torque
        // saturate at ±100.
        let over = car_model().resolve(&ActuatorCommand::new(vec![
            setpoint("drive", SetpointValue::Torque(500.0)),
            setpoint("steer", SetpointValue::Position(0.0)),
        ]));
        assert_eq!(value_for(&over, "drive"), &SetpointValue::Torque(100.0));

        let under = car_model().resolve(&ActuatorCommand::new(vec![
            setpoint("drive", SetpointValue::Torque(-500.0)),
            setpoint("steer", SetpointValue::Position(0.0)),
        ]));
        assert_eq!(value_for(&under, "drive"), &SetpointValue::Torque(-100.0));
    }

    #[test]
    fn missing_actuator_falls_back_to_safe_state() {
        // Steer omitted entirely: resolve fills it with the spec's safe_state.
        let cmd = ActuatorCommand::new(vec![setpoint("drive", SetpointValue::Torque(10.0))]);
        let resolved = car_model().resolve(&cmd);
        assert_eq!(value_for(&resolved, "steer"), &SetpointValue::Position(0.0));
    }

    #[test]
    fn wrong_kind_setpoint_falls_back_to_safe_state() {
        // Drive is torque-commanded; a Force is the wrong command space, so resolve
        // refuses to relabel it and drops to safe_state rather than apply it.
        let cmd = ActuatorCommand::new(vec![
            setpoint("drive", SetpointValue::Force(50.0)),
            setpoint("steer", SetpointValue::Position(0.0)),
        ]);
        let resolved = car_model().resolve(&cmd);
        assert_eq!(value_for(&resolved, "drive"), &SetpointValue::Torque(0.0));
    }

    #[test]
    fn non_finite_value_falls_back_to_safe_state() {
        // NaN survives `clamp` unchanged, so resolve must reject it up front and
        // substitute safe_state rather than pass a NaN torque to physics.
        let cmd = ActuatorCommand::new(vec![
            setpoint("drive", SetpointValue::Torque(f64::NAN)),
            setpoint("steer", SetpointValue::Position(0.0)),
        ]);
        let resolved = car_model().resolve(&cmd);
        assert_eq!(value_for(&resolved, "drive"), &SetpointValue::Torque(0.0));
    }

    #[test]
    fn empty_command_resolves_to_all_safe_states() {
        // The cold-start / stalled-pipeline case: no setpoints at all yields every
        // actuator's fail-safe value.
        let resolved = car_model().resolve(&ActuatorCommand::new(vec![]));
        assert_eq!(value_for(&resolved, "drive"), &SetpointValue::Torque(0.0));
        assert_eq!(value_for(&resolved, "steer"), &SetpointValue::Position(0.0));
    }

    #[test]
    fn inverted_sign_negates_before_clamping() {
        // A drive whose positive direction is inverted: the sign flips first, then
        // the flipped value is clamped to the same symmetric ±limit.
        let inverted_drive = ActuatorSpec {
            id: ActuatorId::new("drive"),
            kind: SetpointKind::Torque,
            limit: 100.0,
            safe_state: SetpointValue::Torque(0.0),
            sign: SignConvention::Inverted,
        };
        let model = ActuationModel::new(vec![inverted_drive, steer_spec()]);

        let in_range = model.resolve(&ActuatorCommand::new(vec![
            setpoint("drive", SetpointValue::Torque(10.0)),
            setpoint("steer", SetpointValue::Position(0.0)),
        ]));
        assert_eq!(value_for(&in_range, "drive"), &SetpointValue::Torque(-10.0));

        let clamped = model.resolve(&ActuatorCommand::new(vec![
            setpoint("drive", SetpointValue::Torque(500.0)),
            setpoint("steer", SetpointValue::Position(0.0)),
        ]));
        assert_eq!(value_for(&clamped, "drive"), &SetpointValue::Torque(-100.0));
    }
}
