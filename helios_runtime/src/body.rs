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

use crate::port::{ChannelKey, InstanceKey, Keyed, Provenance};

use helios_core::control::actuation_model::ActuationModel;
use helios_core::spatial::{transforms::Convention, FrameId};

use std::{any::TypeId, collections::HashMap};

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
    /// Every channel the body measures onto the bus, read by the stack or
    /// not: measurement channels, oracle reference channels, and health
    /// driver/sensor status. The host writes them on the body's behalf; each
    /// counts as already written when the pipeline is built.
    ///
    /// It lists what is installed, as the body's config describes it, never
    /// what came up at boot. A device that fails is a health fact at run
    /// time; leaving it out here would refuse the build of a stack that reads
    /// it, even as an optional input.
    pub publishes: Vec<BodyChannel>,
    /// Each frame of the body, `base_link` and every mount, with its axis
    /// convention as the body's config declares it. The body owns each, so
    /// its convention here is the frame's one convention; a node defining
    /// one fails the build.
    pub frames: HashMap<FrameId, Convention>,
    /// The actuators the body exposes and the setpoint kind each accepts.
    /// The build checks every actuator the pipeline drives against it, so a
    /// misnamed actuator or a setpoint of the wrong kind fails the build
    /// instead of reaching the body. Empty for a body nothing drives.
    pub actuation: ActuationModel,
}

/// One channel the body publishes, as the host declares it: the typed
/// channel, the unit that produces it, how its values are produced, and the
/// frame its values are measured in, if they have one.
///
/// A unit is one installed part of the body, such as a sensor's installation
/// or the simulator's truth rig; an IMU's accelerometer and gyroscope channels
/// share one. Its name is one level, like an agent's or a node's.
///
/// The mount is the frame on the agent's body the unit measures in, such as
/// a sensor's installation, fixed for the run. A value with no frame, such
/// as a battery voltage or the oracle's pose (whose value names its own
/// frame), has none.
///
/// A keyed channel also states the instances the unit writes, such as the
/// frames a truth rig publishes the transforms of. The build refuses a keyed
/// body channel that states none, and an instance another writer also
/// writes.
///
/// A channel the unit writes at a nominal rate states it, shared by every
/// instance it lists: a sensor at its device's rate, a truth rig at the
/// host's. A value written once, such as a static edge, or only on an event
/// states none; a unit whose instances differ declares one channel for each
/// rate.
///
/// The build completes each into a
/// [`ChannelDescription`](crate::port::ChannelDescription) whose writer is the
/// body, so a host never states a channel's writer.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyChannel {
    key: ChannelKey,
    unit: String,
    provenance: Provenance,
    mount: Option<FrameId>,
    instances: Vec<InstanceKey>,
    rate_hz: Option<f64>,
}

impl BodyChannel {
    pub fn new(
        key: impl Into<ChannelKey>,
        unit: impl Into<String>,
        provenance: Provenance,
    ) -> Self {
        Self {
            key: key.into(),
            unit: unit.into(),
            provenance,
            mount: None,
            instances: Vec::new(),
            rate_hz: None,
        }
    }

    /// States the frame the channel's values are measured in. The build
    /// refuses a mount that is not one of this agent's frames off its spine.
    pub fn with_mount(mut self, mount: FrameId) -> Self {
        self.mount = Some(mount);
        self
    }

    /// States the instances of its keyed channel the unit writes, each
    /// spelled as `T`'s instances are.
    ///
    /// # Panics
    ///
    /// If the channel is not a keyed channel of `T`: a host bug, caught when
    /// the host describes its body.
    pub fn with_instances<T: Keyed>(
        mut self,
        instances: impl IntoIterator<Item = T::Instance>,
    ) -> Self {
        assert!(
            self.key.is_keyed() && self.key.type_id() == TypeId::of::<T>(),
            "instances must be stated on a keyed channel of {}, got {}",
            std::any::type_name::<T>(),
            self.key
        );
        self.instances = instances
            .into_iter()
            .map(|instance| InstanceKey::of::<T>(&instance))
            .collect();
        self
    }

    /// States that the unit writes the channel `hz` times a second,
    /// nominally.
    ///
    /// # Panics
    ///
    /// If `hz` is not finite and positive: a host bug, caught when the host
    /// describes its body.
    pub fn with_rate(mut self, hz: f64) -> Self {
        assert!(
            hz.is_finite() && hz > 0.0,
            "a rate must be finite and positive, got {hz} for {}",
            self.key
        );
        self.rate_hz = Some(hz);
        self
    }

    pub fn key(&self) -> &ChannelKey {
        &self.key
    }

    pub fn unit(&self) -> &str {
        &self.unit
    }

    pub fn provenance(&self) -> Provenance {
        self.provenance
    }

    pub fn mount(&self) -> Option<&FrameId> {
        self.mount.as_ref()
    }

    /// The instances of its keyed channel the unit writes. Empty for a
    /// channel that holds one value.
    pub fn instances(&self) -> &[InstanceKey] {
        &self.instances
    }

    /// How many times a second the unit writes the channel, nominally.
    /// `None` for a value written once or on an event.
    pub fn rate_hz(&self) -> Option<f64> {
        self.rate_hz
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::ChannelKind;

    struct Edge {
        child: &'static str,
    }

    impl Keyed for Edge {
        type Instance = &'static str;

        fn instance(&self) -> &&'static str {
            &self.child
        }
    }

    fn edges() -> ChannelKey {
        ChannelKey::keyed::<Edge>(ChannelKind::Oracle, "edges")
    }

    #[test]
    fn default_is_empty() {
        let caps = BodyCapabilities::default();
        assert!(caps.publishes.is_empty());
        assert!(caps.frames.is_empty());
        assert_eq!(caps.actuation, ActuationModel::default());
    }

    #[test]
    fn a_keyed_channel_keeps_its_instances_spelled() {
        let channel =
            BodyChannel::new(edges(), "rig", Provenance::Exact).with_instances::<Edge>(["base"]);
        assert_eq!(channel.instances(), [InstanceKey::of::<Edge>(&"base")]);
    }

    #[test]
    #[should_panic(expected = "keyed channel")]
    fn instances_on_a_single_value_channel_panic() {
        let single = ChannelKey::oracle::<Edge>("edge");
        let _ = BodyChannel::new(single, "rig", Provenance::Exact).with_instances::<Edge>(["base"]);
    }
}
