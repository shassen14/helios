//! The static brain/body I/O declaration: what the agent's body offers the
//! autonomy pipeline at the bus seam.
//!
//! The body is the robot's physical side, simulated or real: the sensors that
//! measure onto the bus and the actuators that take the control command off
//! it. The host, the process running the pipeline, describes the body it is
//! attached to, so the *same* [`AutonomyPipeline`] runs against a simulated or
//! a hardware body and the assembler adapts instead of assuming. Values sent
//! from outside the robot, such as mission goals and operator commands, are not
//! measurements of the body.
//!
//! It is distinct from the per-tick transform contract
//! [`TfProvider`](helios_core::prelude::TfProvider), passed into each node's
//! `execute` beside the bus. `BodyCapabilities` is the *static* declaration
//! made once at assembly time.

use crate::port::{ChannelKey, Provenance};

use helios_core::control::actuation_model::ActuationModel;

/// Everything the agent's body offers the pipeline at the bus seam: the
/// channels it measures, and the actuators that take the command back off the
/// bus. The host reads that command through
/// [`AutonomyPipeline::read_actuators`](crate::AutonomyPipeline::read_actuators).
///
/// This describes what one body carries, not the robot's morphology. Two bodies
/// of the same drone model can differ: a simulated one carries a perfect truth
/// sensor (oracle channels), a test vehicle may carry an RTK reference rig, and a
/// production unit carries no reference sensor at all. Morphology (vehicle
/// plugin, dynamics) lives elsewhere, never here.
#[derive(Clone, Debug, Default)]
pub struct BodyCapabilities {
    /// The body's name, used in build errors and the startup log.
    pub name: String,
    /// Channels the body measures onto the bus: sensor channels, oracle
    /// reference channels, and health driver/sensor status. The host writes
    /// them on the body's behalf; each counts as already written when the
    /// pipeline is built.
    pub publishes: Vec<BodyChannel>,
    /// The actuators the body exposes and the setpoint kind each accepts.
    /// The build checks every actuator the pipeline drives against it, so a
    /// misnamed actuator or a setpoint of the wrong kind fails the build
    /// instead of reaching the body. Empty for a body nothing drives.
    pub actuation: ActuationModel,
}

/// One channel the body publishes, as the host declares it: the channel and
/// how its values are produced.
///
/// The build completes each into a
/// [`ChannelDescription`](crate::port::ChannelDescription) whose writer is the
/// body, so a host never states a channel's writer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BodyChannel {
    key: ChannelKey,
    provenance: Provenance,
}

impl BodyChannel {
    pub fn new(key: impl Into<ChannelKey>, provenance: Provenance) -> Self {
        Self {
            key: key.into(),
            provenance,
        }
    }

    pub fn key(&self) -> &ChannelKey {
        &self.key
    }

    pub fn provenance(&self) -> Provenance {
        self.provenance
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_empty() {
        let caps = BodyCapabilities::default();
        assert!(caps.publishes.is_empty());
        assert_eq!(caps.actuation, ActuationModel::default());
    }
}
