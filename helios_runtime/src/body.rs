//! The static brain/body I/O declaration: what a host (sim or hardware) offers
//! the autonomy pipeline at the bus seam.
//!
//! This is the portable contract that lets the *same* [`AutonomyPipeline`] run
//! against a simulation body or a `helios_hw` body — each host advertises what
//! it supplies, and the assembler adapts instead of assuming. It is distinct
//! from two neighbouring "capability"-shaped types:
//!
//! - [`AgentRuntime`](crate::runtime::AgentRuntime) is the *per-tick* contract
//!   (TF lookups, clock). `BodyCapabilities` is the *static* declaration made
//!   once at assembly time.
//! - [`CapabilitySet`](crate::validation::CapabilitySet) is the autonomy-stack
//!   feature set (which algorithm families are enabled). `BodyCapabilities`
//!   describes the host's I/O, not the brain's algorithms.

use crate::{port::ChannelKey, AutonomyStack};

use helios_core::control::{
    actuation_model::ActuationModel,
    actuators::{ActuatorId, SetpointKind},
};

/// How a value published onto a channel was produced.
///
/// Modelled as an enum (rather than a unit) because hardware will introduce
/// further variants — e.g. `Instrument` (read off a real sensor) or `Recorded`
/// (replayed from a log). Only `Exact` exists today; the others land with the
/// first hardware host.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Provenance {
    /// Ground truth, exact to the limits of the host (e.g. physics state in sim).
    #[default]
    Exact,
}

/// One channel the body fills on the bus, paired with how its value was produced.
#[derive(Clone, Debug)]
pub struct PublishedChannel {
    pub key: ChannelKey,
    pub provenance: Provenance,
}

/// Everything a host body offers the pipeline at the bus seam.
///
/// The discriminator is the *host*, not the robot's morphology: the same drone
/// has different capabilities in sim (publishes `oracle/*`) versus on hardware
/// (no oracle). Morphology lives elsewhere (vehicle plugin, dynamics, sensor
/// suite), never here.
#[derive(Clone, Debug, Default)]
pub struct BodyCapabilities {
    pub name: String,
    /// Channels the body writes to the bus: sensor channels, `oracle/*`
    /// ground-truth, and `health/*` driver/sensor status. The assembler reads
    /// this to seed the pipeline's external channels.
    pub publishes: Vec<PublishedChannel>,
    /// Whether the body consumes the control command from the bus. This is the
    /// one thing taken *off* the bus (a sink, not a published channel), so it
    /// can't live in `publishes`.
    pub consumes_control: bool,
}

/// Checks that every allocator's output setpoint kind agrees with the body
/// actuator it drives.
///
/// The allocator is the brain's side of the seam — it emits a setpoint in one
/// [`SetpointKind`]. The [`ActuationModel`] is the body's side — each actuator
/// declares the one kind it accepts. When they disagree, the pipeline drives the
/// wrong physical quantity into an actuator (a torque command into a
/// velocity-driven wheel), and nothing downstream can detect it. Both sides are
/// known only once a body is chosen, so the host runs this per agent at spawn; it
/// cannot fold into [`validate_autonomy_config`](crate::validation::validate_autonomy_config),
/// which validates the stack alone and never sees a body.
///
/// Reports every disagreement, not just the first, so one spawn surfaces them all.
pub fn check_actuation_agreement(
    stack: &AutonomyStack,
    model: &ActuationModel,
) -> Result<(), Vec<ActuatorKindMismatch>> {
    let mut mismatches = Vec::new();

    for (allocator, config) in &stack.allocators {
        let emits = config.output_kind();
        for actuator in config.actuator_ids() {
            match model.spec(&ActuatorId::new(actuator)) {
                None => mismatches.push(ActuatorKindMismatch::UnknownActuator {
                    allocator: allocator.clone(),
                    actuator: actuator.to_string(),
                }),
                Some(spec) if spec.kind() != emits => {
                    mismatches.push(ActuatorKindMismatch::KindMismatch {
                        allocator: allocator.clone(),
                        actuator: actuator.to_string(),
                        emits,
                        expects: spec.kind(),
                    })
                }
                Some(_) => {}
            }
        }
    }

    if mismatches.is_empty() {
        Ok(())
    } else {
        Err(mismatches)
    }
}

/// One way an allocator's output can fail to match the body it drives, reported
/// by [`check_actuation_agreement`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ActuatorKindMismatch {
    /// The allocator names an actuator the body's [`ActuationModel`] does not
    /// declare — usually a typo in the profile's actuator id.
    UnknownActuator { allocator: String, actuator: String },
    /// The allocator emits a setpoint kind the targeted actuator does not accept.
    KindMismatch {
        allocator: String,
        actuator: String,
        emits: SetpointKind,
        expects: SetpointKind,
    },
}

impl std::fmt::Display for ActuatorKindMismatch {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ActuatorKindMismatch::UnknownActuator {
                allocator,
                actuator,
            } => write!(
                f,
                "allocator `{allocator}` drives actuator `{actuator}`, which the body's actuation model does not declare"
            ),
            ActuatorKindMismatch::KindMismatch {
                allocator,
                actuator,
                emits,
                expects,
            } => write!(
                f,
                "allocator `{allocator}` emits {emits:?} but body actuator `{actuator}` accepts {expects:?}"
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::AllocatorConfig;
    use crate::port::InternalChannel;

    use helios_core::control::actuation_model::{ActuatorSpec, SignConvention};

    use std::collections::HashMap;

    #[test]
    fn default_is_empty_and_passive() {
        let caps = BodyCapabilities::default();
        assert!(caps.publishes.is_empty());
        assert!(!caps.consumes_control);
    }

    #[test]
    fn published_channel_records_key_and_provenance() {
        let key: ChannelKey = InternalChannel::of::<f64>().into();
        let caps = BodyCapabilities {
            name: String::default(),
            publishes: vec![PublishedChannel {
                key: key.clone(),
                provenance: Provenance::default(),
            }],
            consumes_control: true,
        };
        assert_eq!(caps.publishes.len(), 1);
        assert_eq!(caps.publishes[0].key, key);
        assert_eq!(caps.publishes[0].provenance, Provenance::Exact);
        assert!(caps.consumes_control);
    }

    /// A body with one actuator of the given id and kind. `kind.value(0.0)` gives
    /// a matching fail-safe, so the spec is self-consistent.
    fn body_with(id: &str, kind: SetpointKind) -> ActuationModel {
        ActuationModel::new(vec![ActuatorSpec::new(
            ActuatorId::new(id),
            kind,
            1.0,
            kind.value(0.0),
            SignConvention::Normal,
        )])
    }

    /// A stack whose one `WheelTorque` allocator drives an actuator named `drive`.
    fn stack_driving_drive() -> AutonomyStack {
        AutonomyStack {
            allocators: HashMap::from([(
                "drive".to_string(),
                AllocatorConfig::WheelTorque {
                    wheel_radius: 0.3,
                    drive: "drive".to_string(),
                },
            )]),
            ..Default::default()
        }
    }

    /// A `WheelTorque` allocator over a torque-driven actuator of the same name
    /// is the agreeing case — no mismatch.
    #[test]
    fn matching_kinds_agree() {
        let stack = stack_driving_drive();
        let model = body_with("drive", SetpointKind::Torque);
        assert!(check_actuation_agreement(&stack, &model).is_ok());
    }

    /// A torque-emitting allocator over a velocity-driven wheel is the exact bug
    /// this check exists to catch: the emitted kind and the accepted kind disagree.
    #[test]
    fn wrong_actuator_kind_is_a_mismatch() {
        let stack = stack_driving_drive();
        let model = body_with("drive", SetpointKind::Velocity);
        let errors = check_actuation_agreement(&stack, &model).unwrap_err();
        assert_eq!(
            errors,
            vec![ActuatorKindMismatch::KindMismatch {
                allocator: "drive".to_string(),
                actuator: "drive".to_string(),
                emits: SetpointKind::Torque,
                expects: SetpointKind::Velocity,
            }]
        );
    }

    /// An allocator that names an actuator the body never declares (here a typo:
    /// the body exposes `traction`, not `drive`) is reported distinctly from a
    /// kind mismatch.
    #[test]
    fn allocator_naming_an_absent_actuator_is_a_mismatch() {
        let stack = stack_driving_drive();
        let model = body_with("traction", SetpointKind::Torque);
        let errors = check_actuation_agreement(&stack, &model).unwrap_err();
        assert_eq!(
            errors,
            vec![ActuatorKindMismatch::UnknownActuator {
                allocator: "drive".to_string(),
                actuator: "drive".to_string(),
            }]
        );
    }
}
