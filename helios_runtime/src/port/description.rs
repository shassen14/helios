//! [`ChannelDescription`]: what the pipeline knows about one channel, whoever
//! writes it, with its [`Writer`], its mount, and, for a body channel, the
//! unit that produces it and its [`Provenance`].
//!
//! Every writer's channels are described alike, and only the pipeline
//! builder makes descriptions. Each writer declares its channels through its
//! own door: the body as [`BodyChannel`](crate::body::BodyChannel)s in
//! [`BodyCapabilities`](crate::BodyCapabilities), outside inputs as keys, a
//! node as [`OutputPort`](crate::port::OutputPort)s. The builder chooses the
//! [`Writer`] from the door a channel came through, so no declaration can
//! state the wrong writer.
//!
//! A channel's mount, the frame its values are measured in, is stated by its
//! writer: the body states it with the channel, a node by its output's
//! [`MountRule`](crate::port::MountRule), which the builder resolves once the
//! graph is ordered. An outside input has none.
//!
//! A writer of a keyed channel also states the instances it writes: the body
//! with the channel, a node on its output port. Several writers may share a keyed channel, each
//! described with its own instances.
//!
//! A description carries the nominal rate its writer writes at: the body
//! states it with the channel, and the builder gives a node output its
//! node's. An outside input has none.

use crate::body::BodyChannel;
use crate::port::channel::ChannelKey;
use crate::port::keyed::InstanceKey;

use helios_core::spatial::FrameId;

/// One channel the pipeline sees, and who writes it.
///
/// Readers are not part of a description: who reads a channel is a fact of the
/// graph, not of the channel.
#[derive(Debug, Clone, PartialEq)]
pub struct ChannelDescription {
    key: ChannelKey,
    writer: Writer,
    mount: Option<FrameId>,
    instances: Vec<InstanceKey>,
    rate_hz: Option<f64>,
}

impl ChannelDescription {
    /// A channel the body publishes, as the host declared it.
    pub(crate) fn body(channel: &BodyChannel) -> Self {
        Self {
            key: channel.key().clone(),
            writer: Writer::Body {
                unit: channel.unit().to_string(),
                provenance: channel.provenance(),
            },
            mount: channel.mount().cloned(),
            instances: channel.instances().to_vec(),
            rate_hz: channel.rate_hz(),
        }
    }

    /// A channel sent from outside the robot, such as a mission goal or the
    /// operator's teleop intent.
    pub(crate) fn outside(key: ChannelKey) -> Self {
        Self {
            key,
            writer: Writer::Outside,
            mount: None,
            instances: Vec::new(),
            rate_hz: None,
        }
    }

    /// A channel node `node` writes, from one of its output ports. It has no
    /// mount until the builder resolves the output's rule, and no rate until
    /// the builder gives it the node's.
    pub(crate) fn node(key: ChannelKey, node: impl Into<String>) -> Self {
        Self {
            key,
            writer: Writer::Node { node: node.into() },
            mount: None,
            instances: Vec::new(),
            rate_hz: None,
        }
    }

    /// The description with `mount` as its mount, as the builder resolves it
    /// from a node output's rule.
    pub(crate) fn with_mount(mut self, mount: Option<FrameId>) -> Self {
        self.mount = mount;
        self
    }

    /// The description with `instances` as the instances of its keyed channel
    /// the writer writes, as a node's output port declares them. A body
    /// channel carries its own.
    pub(crate) fn with_instances(mut self, instances: Vec<InstanceKey>) -> Self {
        self.instances = instances;
        self
    }

    /// The description with `rate_hz` as the rate its node writes it at, as
    /// the builder derives it from the node's gate and the host's tick. A
    /// body channel carries its own.
    pub(crate) fn with_rate(mut self, rate_hz: Option<f64>) -> Self {
        self.rate_hz = rate_hz;
        self
    }

    pub fn key(&self) -> &ChannelKey {
        &self.key
    }

    pub fn writer(&self) -> &Writer {
        &self.writer
    }

    /// The frame the channel's values are measured in, fixed for the run, if
    /// its writer states one.
    pub fn mount(&self) -> Option<&FrameId> {
        self.mount.as_ref()
    }

    /// The instances of its keyed channel this writer writes. Empty for a
    /// channel that holds one value.
    pub fn instances(&self) -> &[InstanceKey] {
        &self.instances
    }

    /// How many times a second the writer writes the channel, nominally,
    /// the same for every instance it writes: a body channel's as the host
    /// stated it, a node output's from the node's gate or else the host's
    /// tick. `None` for an outside input, a value written once, or a node
    /// output when the host declared no tick rate.
    pub fn rate_hz(&self) -> Option<f64> {
        self.rate_hz
    }
}

/// Who writes a channel onto the bus.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Writer {
    /// The body publishes it: a sensor, an oracle reference, a health status.
    /// The host writes it on the body's behalf. `unit` names the installed
    /// part that produces it, such as a sensor's installation.
    Body {
        unit: String,
        provenance: Provenance,
    },
    /// A node in the graph writes it; carries the node's name.
    Node { node: String },
    /// It is sent from outside the robot by an operator or mission system.
    Outside,
}

/// Names the writer as build errors do. A pipeline serves one body, so a body
/// writer is named by its unit; the host prefixes every build error with the
/// agent.
impl std::fmt::Display for Writer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Writer::Body { unit, .. } => write!(f, "unit \"{unit}\""),
            Writer::Node { node } => write!(f, "node \"{node}\""),
            Writer::Outside => write!(f, "an outside input"),
        }
    }
}

/// How a value the body publishes was produced.
///
/// Hardware will add further variants, e.g. `Instrument` (read off a real
/// sensor) or `Recorded` (replayed from a log); they land with the first
/// hardware host.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Provenance {
    /// Ground truth, exact to the limits of the simulation (e.g. physics state
    /// read by a perfect sensor in sim).
    Exact,
    /// Produced by a model of a sensor: a forward model applied to the
    /// simulated world, with its noise and bias, or with them off.
    Simulated,
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::ChannelKey;

    use helios_core::prelude::AgentId;

    fn key() -> ChannelKey {
        ChannelKey::internal::<f64>("drive")
    }

    #[test]
    fn each_constructor_records_its_writer() {
        let channel = BodyChannel::new(key(), "imu", Provenance::Simulated);
        assert_eq!(
            ChannelDescription::body(&channel).writer(),
            &Writer::Body {
                unit: "imu".to_string(),
                provenance: Provenance::Simulated
            }
        );
        assert_eq!(ChannelDescription::body(&channel).key(), &key());
        assert_eq!(
            ChannelDescription::outside(key()).writer(),
            &Writer::Outside
        );
        assert_eq!(
            ChannelDescription::node(key(), "ekf").writer(),
            &Writer::Node {
                node: "ekf".to_string()
            }
        );
        assert_eq!(ChannelDescription::outside(key()).key(), &key());
    }

    /// A body channel's description carries the mount the host stated; an
    /// outside input and a node output have none until the builder resolves
    /// one.
    #[test]
    fn a_body_channel_keeps_its_mount() {
        let mount = FrameId::agent(AgentId::new("car"), "gps");
        let channel =
            BodyChannel::new(key(), "gps", Provenance::Simulated).with_mount(mount.clone());
        assert_eq!(ChannelDescription::body(&channel).mount(), Some(&mount));

        let unmounted = BodyChannel::new(key(), "battery", Provenance::Simulated);
        assert_eq!(ChannelDescription::body(&unmounted).mount(), None);
        assert_eq!(ChannelDescription::outside(key()).mount(), None);
        assert_eq!(ChannelDescription::node(key(), "ekf").mount(), None);
    }

    /// A body channel's description carries the rate the host stated; an
    /// outside input has none, and a node output none until the builder
    /// gives it one.
    #[test]
    fn a_body_channel_keeps_its_rate() {
        let gps = BodyChannel::new(key(), "gps", Provenance::Simulated).with_rate(10.0);
        assert_eq!(ChannelDescription::body(&gps).rate_hz(), Some(10.0));

        let unrated = BodyChannel::new(key(), "battery", Provenance::Simulated);
        assert_eq!(ChannelDescription::body(&unrated).rate_hz(), None);
        assert_eq!(ChannelDescription::outside(key()).rate_hz(), None);
        assert_eq!(ChannelDescription::node(key(), "ekf").rate_hz(), None);
    }

    /// A keyed body channel's description carries the instances the host
    /// stated, so the instance checks judge the body as they judge a node.
    #[test]
    fn a_body_channel_keeps_its_instances() {
        use crate::channels::truth_tf;

        use helios_core::spatial::transforms::tf::stamped::StampedTransform;

        let base_link = FrameId::base_link(AgentId::new("car"));
        let channel = BodyChannel::new(truth_tf(), "truth_rig", Provenance::Exact)
            .with_instances::<StampedTransform>([base_link.clone()]);
        assert_eq!(
            ChannelDescription::body(&channel).instances(),
            [InstanceKey::of::<StampedTransform>(&base_link)]
        );
    }

    #[test]
    fn writers_display_as_errors_name_them() {
        let body = Writer::Body {
            unit: "imu".to_string(),
            provenance: Provenance::Exact,
        };
        let node = Writer::Node {
            node: "ekf".to_string(),
        };
        assert_eq!(body.to_string(), "unit \"imu\"");
        assert_eq!(node.to_string(), "node \"ekf\"");
        assert_eq!(Writer::Outside.to_string(), "an outside input");
    }
}
