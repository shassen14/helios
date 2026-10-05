//! Where a node's sensor inputs may come from, and the check that every
//! sensor input has a source before the graph is built.

use super::error::PipelineAssemblyError;
use crate::pipeline::node::PipelineNode;
use crate::port::{ChannelKey, InputPort};

use std::collections::HashSet;

/// Where a node's sensor inputs may come from: channels the host publishes, and
/// channels a preprocessing node derives inside the graph.
pub(super) struct SensorInputs<'a> {
    pub(super) host: &'a HashSet<String>,
    pub(super) derived: &'a HashSet<ChannelKey>,
}

impl SensorInputs<'_> {
    /// Records a node's host-published sensor inputs as external, so they seed
    /// the topological sort, and reports any sensor input with no source.
    ///
    /// Every sensor input is checked, required and optional alike: an optional
    /// input on a channel nobody publishes fails the same silent way, its slot
    /// never filled and the node never using it.
    ///
    /// Only host channels are seeded. An internal input, or a sensor input a
    /// preprocessing node derives, must be produced inside the graph: seeding
    /// it would hide a missing producer, and would let the consumer become
    /// ready in the same level as its producer, leaving their order within a
    /// tick arbitrary. A sensor input that is neither host-published nor
    /// derived is an [`UnpublishedSensorInput`](PipelineAssemblyError::UnpublishedSensorInput):
    /// without the check the node builds, its slot stays empty, and it silently
    /// never works.
    pub(super) fn seed(
        &self,
        node: &dyn PipelineNode,
        external_channels: &mut Vec<ChannelKey>,
        errors: &mut Vec<PipelineAssemblyError>,
    ) {
        let sensor_inputs = node
            .port_descriptor()
            .inputs()
            .map(InputPort::channel)
            .filter(|key| matches!(key, ChannelKey::Sensor(_)))
            .filter(|key| !self.derived.contains(*key));

        for key in sensor_inputs {
            if self.host.contains(key.instance().as_ref()) {
                external_channels.push(key.clone());
            } else {
                errors.push(PipelineAssemblyError::UnpublishedSensorInput {
                    node_name: node.name().to_string(),
                    channel: key.instance().to_string(),
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::pipeline::node::TickContext;
    use crate::port::{AlgorithmNodePortDescriptor, PortBus, PortDescriptor, SensorChannel};

    use helios_core::prelude::TfProvider;

    /// A node that only declares ports; `seed` reads nothing else.
    struct PortsOnlyNode {
        descriptor: PortDescriptor,
    }

    impl PipelineNode for PortsOnlyNode {
        fn name(&self) -> &str {
            "reader"
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {}
    }

    fn optional_sensor_reader(channel: &str) -> PortsOnlyNode {
        PortsOnlyNode {
            descriptor: AlgorithmNodePortDescriptor::new()
                .optional_sensor(SensorChannel::named::<f64>(channel))
                .build(),
        }
    }

    /// An optional sensor input on a channel the host publishes is seeded like a
    /// required one, so the build counts it as supplied.
    #[test]
    fn optional_input_on_a_host_channel_is_seeded() {
        let host = HashSet::from(["gps".to_string()]);
        let derived = HashSet::new();
        let sensor_inputs = SensorInputs {
            host: &host,
            derived: &derived,
        };
        let mut external_channels = vec![];
        let mut errors = vec![];

        sensor_inputs.seed(
            &optional_sensor_reader("gps"),
            &mut external_channels,
            &mut errors,
        );

        assert!(errors.is_empty(), "unexpected errors: {errors:?}");
        assert_eq!(
            external_channels,
            vec![ChannelKey::from(SensorChannel::named::<f64>("gps"))]
        );
    }

    /// An optional sensor input nobody publishes would build with its slot
    /// never filled, so it is reported just as a required one is.
    #[test]
    fn optional_input_on_an_unpublished_channel_is_reported() {
        let host = HashSet::new();
        let derived = HashSet::new();
        let sensor_inputs = SensorInputs {
            host: &host,
            derived: &derived,
        };
        let mut external_channels = vec![];
        let mut errors = vec![];

        sensor_inputs.seed(
            &optional_sensor_reader("gps.typo"),
            &mut external_channels,
            &mut errors,
        );

        assert!(external_channels.is_empty());
        assert!(
            errors.iter().any(|e| matches!(
                e,
                PipelineAssemblyError::UnpublishedSensorInput { node_name, channel }
                    if node_name == "reader" && channel == "gps.typo"
            )),
            "expected UnpublishedSensorInput for `reader` / `gps.typo`, got {errors:?}"
        );
    }
}
