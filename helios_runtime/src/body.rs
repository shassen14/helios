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

use crate::port::ChannelKey;

use helios_core::control::actuation_model::ActuationModel;

/// How a value published onto a channel was produced.
///
/// Modelled as an enum (rather than a unit) because hardware will introduce
/// further variants — e.g. `Instrument` (read off a real sensor) or `Recorded`
/// (replayed from a log). Only `Exact` exists today; the others land with the
/// first hardware host.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Provenance {
    /// Ground truth, exact to the limits of the simulation (e.g. physics state
    /// read by a perfect sensor in sim).
    #[default]
    Exact,
}

/// One channel the body fills on the bus, paired with how its value was produced.
#[derive(Clone, Debug)]
pub struct PublishedChannel {
    pub key: ChannelKey,
    pub provenance: Provenance,
}

/// Everything the agent's body offers the pipeline at the bus seam: the
/// channels it measures, and the actuators that take the command back off the
/// bus. The host reads that command through
/// [`AutonomyPipeline::read_actuators`](crate::AutonomyPipeline::read_actuators).
///
/// This describes what one body carries, not the robot's morphology. Two bodies
/// of the same drone model can differ: a simulated one carries a perfect truth
/// sensor (`oracle/*`), a test vehicle may carry an RTK reference rig, and a
/// production unit carries no reference sensor at all. Morphology (vehicle
/// plugin, dynamics) lives elsewhere, never here.
#[derive(Clone, Debug, Default)]
pub struct BodyCapabilities {
    /// The body's name, used in build errors and the startup log.
    pub name: String,
    /// Channels the body measures onto the bus: sensor channels, `oracle/*`
    /// reference channels, and `health/*` driver/sensor status. The host writes
    /// them on the body's behalf; each counts as already written when the
    /// pipeline is built.
    pub publishes: Vec<PublishedChannel>,
    /// The actuators the body exposes and the setpoint kind each accepts.
    /// The build checks every actuator the pipeline drives against it, so a
    /// misnamed actuator or a setpoint of the wrong kind fails the build
    /// instead of reaching the body. Empty for a body nothing drives.
    pub actuation: ActuationModel,
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::InternalChannel;

    #[test]
    fn default_is_empty() {
        let caps = BodyCapabilities::default();
        assert!(caps.publishes.is_empty());
        assert_eq!(caps.actuation, ActuationModel::default());
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
            actuation: ActuationModel::default(),
        };
        assert_eq!(caps.publishes.len(), 1);
        assert_eq!(caps.publishes[0].key, key);
        assert_eq!(caps.publishes[0].provenance, Provenance::Exact);
    }
}
