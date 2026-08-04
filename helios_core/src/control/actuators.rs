//! The actuator setpoint vocabulary — the terminal command of the control
//! pipeline.
//!
//! An allocator emits an [`ActuatorCommand`]: one [`ActuatorSetpoint`] per
//! physical actuator, each a [`SetpointValue`] in that actuator's native command
//! space (torque, force, or position). The host applies these directly to
//! hardware or the sim; nothing downstream reinterprets them. This is the same
//! message a real vehicle receives, which is what lets a sim and a hardware
//! agent share one control pipeline.
//!
//! [`ActuatorCommand::validate`] checks a command against a body's
//! [`ActuationModel`] before it is trusted.

use std::{collections::HashSet, sync::Arc};

use serde::{Deserialize, Serialize};

use crate::control::actuation_model::ActuationModel;

/// A complete set of actuator setpoints — the control pipeline's final output.
///
/// The `setpoints` `Vec` is *semantically a map* keyed by [`ActuatorId`]: exactly
/// one setpoint per physical actuator. A `Vec` (not a `HashMap`) keeps the
/// command ordered and cheap to clone onto the bus each tick; the one-per-actuator
/// invariant is enforced by [`validate`](ActuatorCommand::validate) rather than by
/// the type.
#[derive(Clone, PartialEq, Debug)]
pub struct ActuatorCommand {
    setpoints: Vec<ActuatorSetpoint>,
}

impl ActuatorCommand {
    pub fn new(setpoints: Vec<ActuatorSetpoint>) -> Self {
        Self { setpoints }
    }

    /// Check this command against the body's actuator contract, returning *every*
    /// problem at once rather than the first.
    ///
    /// A command is valid when, for the given [`ActuationModel`], it: names only
    /// declared actuators ([`UnknownActuator`](ActuatorCommandError::UnknownActuator)),
    /// speaks each actuator's declared command space
    /// ([`KindMismatch`](ActuatorCommandError::KindMismatch)), carries a finite
    /// value ([`NonFinite`](ActuatorCommandError::NonFinite) — the open-loop steer
    /// inverse `atan(wz·L / v)` produces `NaN` at `v ≈ 0`), names each actuator at
    /// most once ([`DuplicateActuator`](ActuatorCommandError::DuplicateActuator)),
    /// and covers every declared actuator
    /// ([`MissingActuator`](ActuatorCommandError::MissingActuator)).
    pub fn validate(&self, model: &ActuationModel) -> Result<(), Vec<ActuatorCommandError>> {
        let mut errors = Vec::new();

        let mut seen_actuators = HashSet::new();

        for sp in &self.setpoints {
            match model.spec(sp.actuator()) {
                None => errors.push(ActuatorCommandError::UnknownActuator {
                    actuator: sp.actuator().clone(),
                }),
                Some(spec) => {
                    let found = sp.value().kind();
                    if found != spec.kind() {
                        errors.push(ActuatorCommandError::KindMismatch {
                            actuator: sp.actuator().clone(),
                            expected: spec.kind(),
                            found,
                        })
                    }

                    // NaN or infinite (the open-loop steer inverse can produce it).
                    if !sp.value().scalar().is_finite() {
                        errors.push(ActuatorCommandError::NonFinite {
                            actuator: sp.actuator().clone(),
                        })
                    }

                    // First sighting inserts; a repeat means two setpoints named
                    // the same actuator, which the applier could only resolve
                    // arbitrarily.
                    let is_new = seen_actuators.insert(spec.id());
                    if !is_new {
                        errors.push(ActuatorCommandError::DuplicateActuator {
                            actuator: spec.id().clone(),
                        });
                    }
                }
            }
        }

        for spec in model.actuators() {
            if !self.setpoints.iter().any(|sp| sp.actuator() == spec.id()) {
                errors.push(ActuatorCommandError::MissingActuator {
                    actuator: spec.id().clone(),
                })
            }
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }

    pub fn setpoints(&self) -> &[ActuatorSetpoint] {
        &self.setpoints
    }
}

/// A single way a command failed [`ActuatorCommand::validate`]. One command can
/// produce several.
#[derive(Clone, PartialEq, Debug)]
pub enum ActuatorCommandError {
    /// A setpoint spoke the wrong command space for its actuator — e.g. a `Force`
    /// sent to a torque-commanded drive.
    KindMismatch {
        actuator: ActuatorId,
        expected: SetpointKind,
        found: SetpointKind,
    },

    /// A setpoint named an actuator the model does not declare.
    UnknownActuator {
        actuator: ActuatorId,
    },

    /// A declared actuator received no setpoint. The command must be total.
    MissingActuator {
        actuator: ActuatorId,
    },

    /// A setpoint value was `NaN` or infinite.
    NonFinite {
        actuator: ActuatorId,
    },

    /// Two or more setpoints named the same actuator. The `Vec` cannot prevent
    /// this, so validation rejects it rather than let the applier pick arbitrarily.
    DuplicateActuator {
        actuator: ActuatorId,
    },
}

/// One actuator's commanded value: *which* actuator, and *what* to do.
#[derive(Clone, PartialEq, Debug)]
pub struct ActuatorSetpoint {
    actuator: ActuatorId,
    value: SetpointValue,
}

impl ActuatorSetpoint {
    pub fn new(actuator: ActuatorId, value: SetpointValue) -> Self {
        Self { actuator, value }
    }

    pub fn actuator(&self) -> &ActuatorId {
        &self.actuator
    }

    pub fn value(&self) -> &SetpointValue {
        &self.value
    }
}

/// Stable string identity of one physical actuator, shared by the allocator that
/// names it and the host that applies it — a reserved string whose two sides must
/// agree. Backed by `Arc<str>` so cloning it onto the bus every tick is cheap;
/// it serializes as (and loads from) a plain string, e.g. `"drive"`.
#[derive(Clone, PartialEq, Eq, Hash, Debug)]
pub struct ActuatorId(Arc<str>);

impl ActuatorId {
    pub fn new(id: impl Into<Arc<str>>) -> Self {
        Self(id.into())
    }
}

impl Serialize for ActuatorId {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.serialize_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for ActuatorId {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        Ok(ActuatorId::new(String::deserialize(deserializer)?))
    }
}

/// A commanded value in its native command space. The *variant* is the command
/// space: a `Torque(5.0)` and a `Force(5.0)` are not interchangeable despite the
/// equal number. Units are SI by convention (torque N·m, force N, position rad or
/// m per the actuator) and are not encoded in the type. Only the command spaces a
/// car exercises exist today; others (duty cycle, velocity, current) are added in
/// the commit that first constructs them.
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum SetpointValue {
    Force(f64),
    Torque(f64),
    Position(f64),
}

impl SetpointValue {
    /// This value's command space, magnitude discarded.
    pub fn kind(&self) -> SetpointKind {
        match self {
            SetpointValue::Force(_) => SetpointKind::Force,
            SetpointValue::Torque(_) => SetpointKind::Torque,
            SetpointValue::Position(_) => SetpointKind::Position,
        }
    }

    /// The signed scalar, command space discarded — for range and finiteness
    /// checks. Signed, not a magnitude: a reverse torque or a negative steer angle
    /// is legitimately below zero.
    pub fn scalar(&self) -> f64 {
        match self {
            SetpointValue::Force(v) | SetpointValue::Torque(v) | SetpointValue::Position(v) => *v,
        }
    }
}

/// The command space of a setpoint without its magnitude. An [`ActuatorSpec`]
/// declares the one kind it accepts; [`kind`](SetpointValue::kind) reports what a
/// value carries. The exhaustive, wildcard-free match in `kind` means adding a
/// [`SetpointValue`] variant fails to compile until this enum and every consumer
/// account for it.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum SetpointKind {
    Force,
    Torque,
    Position,
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::collections::HashSet;

    #[test]
    fn kind_reports_the_variant() {
        assert_eq!(SetpointValue::Force(1.0).kind(), SetpointKind::Force);
        assert_eq!(SetpointValue::Torque(1.0).kind(), SetpointKind::Torque);
        assert_eq!(SetpointValue::Position(1.0).kind(), SetpointKind::Position);
    }

    #[test]
    fn scalar_preserves_sign() {
        // Not a magnitude: reverse torque and a leftward steer angle are negative.
        assert_eq!(SetpointValue::Torque(-4.5).scalar(), -4.5);
        assert_eq!(SetpointValue::Position(0.3).scalar(), 0.3);
    }

    #[test]
    fn actuator_id_equal_when_string_equal() {
        assert_eq!(ActuatorId::new("drive"), ActuatorId::new("drive"));
        assert_ne!(ActuatorId::new("drive"), ActuatorId::new("steer"));
    }

    #[test]
    fn actuator_id_hashes_by_value_not_by_allocation() {
        // Two independently constructed ids back distinct `Arc`s but must collide
        // in a set — identity is the string, not the pointer.
        let mut set = HashSet::new();
        set.insert(ActuatorId::new("drive"));
        assert!(set.contains(&ActuatorId::new("drive")));
        assert!(!set.contains(&ActuatorId::new("steer")));
    }

    #[test]
    fn actuator_id_is_a_plain_string_on_the_wire() {
        // The hand-written serde routes through `String`, so an id serializes as a
        // bare string rather than an `Arc` wrapper — this is what keeps the entity
        // file readable (`id = "drive"`).
        let json = serde_json::to_string(&ActuatorId::new("drive")).expect("serialize");
        assert_eq!(json, "\"drive\"");
        let back: ActuatorId = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(back, ActuatorId::new("drive"));
    }

    #[test]
    fn setpoint_value_round_trips_through_serde() {
        let value = SetpointValue::Torque(2.5);
        let json = serde_json::to_string(&value).expect("serialize");
        let back: SetpointValue = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(value, back);
    }
}
