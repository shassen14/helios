//! Plant models: the forward map from an actuator command to the forces it
//! produces on a body.
//!
//! A plant is world-truth — what the body *actually does* when actuated, the
//! motion dual of [`sensors`](crate::sensors) (observation-truth). It is what a
//! control law acts *on*, not part of the law, which is why it sits at the crate
//! root beside `sensors/` rather than inside [`control`](crate::control). The
//! forward force it computes is integrated by the host's physics engine; the
//! plant supplies the forces, the integrator supplies the motion.
//!
//! Fidelity is a ladder: [`RaycastWheelPlant`] is the rung in place today, a
//! per-wheel suspension-and-tire model. A cruder single-wrench rung or a richer
//! multi-body plant slots in beside it without changing this module's role.

pub mod raycast_wheels;

pub use raycast_wheels::{
    Axle, RaycastWheelPlant, SuspensionParams, TireParams, Wheel, WheelContact, WheelRay,
};

use crate::control::{actuators::ActuatorId, commands::BodyWrench};

/// What every plant returns: the body-frame wrench to apply, and the actuators
/// whose setpoint the plant could not honour.
///
/// It is shared across the fidelity ladder — each rung folds a command into a
/// wrench differently, but they all report their result in this one shape, so
/// the host applies forces and surfaces unsupported actuators the same way
/// regardless of which plant is installed. What lands in [`unsupported`] is
/// plant-specific: the setpoint kinds a given rung cannot apply (see each
/// plant's `accepts`), and the actuators are named in the order the command
/// listed them.
///
/// [`unsupported`]: PlantWrench::unsupported
pub struct PlantWrench {
    wrench: BodyWrench,
    unsupported: Vec<ActuatorId>,
}

impl PlantWrench {
    /// A plant's folded result: the body-frame wrench to apply and the actuators
    /// whose setpoint it could not honour. The one door every plant constructs
    /// through, so the shape's invariants live in a single place.
    pub fn new(wrench: BodyWrench, unsupported: Vec<ActuatorId>) -> Self {
        Self {
            wrench,
            unsupported,
        }
    }

    /// A plant contributing nothing this tick: a zero wrench and no unsupported
    /// actuators — the result an idle or fully airborne body folds to.
    pub fn zero() -> Self {
        Self::new(BodyWrench::zero(), Vec::new())
    }

    /// The folded body-frame ([`Flu`](crate::frames::conventions::Flu)) wrench
    /// the host rotates into world space and applies to the chassis.
    pub fn wrench(&self) -> BodyWrench {
        self.wrench
    }

    /// Actuators whose setpoint the plant could not apply — a command-space
    /// mismatch between the actuation contract and the installed plant (a config
    /// or wiring error). Each contributes nothing to the wrench; the host warns
    /// on them, one per actuator, in the command's actuator order.
    pub fn unsupported(&self) -> &[ActuatorId] {
        &self.unsupported
    }
}
