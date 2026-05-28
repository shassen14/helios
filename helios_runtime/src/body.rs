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

use crate::port::ChannelKey;

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
    /// Whether the body consumes `ControlOutput` from the bus. This is the one
    /// thing taken *off* the bus (a sink, not a published channel), so it can't
    /// live in `publishes`.
    pub consumes_control: bool,
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::InternalChannel;

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
}
