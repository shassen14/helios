//! Where a node's sensor inputs may come from: the sensor channels nodes derive
//! inside the graph, and the check that every sensor input has a source before
//! the graph is built.

use super::error::PipelineAssemblyError;

use crate::pipeline::node::PipelineNode;
use crate::port::{ChannelKey, InputPort};

use std::collections::HashSet;

/// Where a node's sensor inputs may come from: channels the host publishes, and
/// channels a node derives inside the graph.
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
    /// node derives, must be produced inside the graph: seeding
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

/// The sensor channels `nodes` write, which other nodes may read as they read
/// host channels.
///
/// Only sensor outputs count: [`SensorInputs::seed`] checks sensor inputs
/// alone, so an internal output could never be the source it looks for.
///
/// Fails with a [`SensorOutputShadowsHost`](PipelineAssemblyError::SensorOutputShadowsHost)
/// for every output named like a host channel, all reported at once.
pub(super) fn derived_channels(
    nodes: &[Box<dyn PipelineNode>],
    host: &HashSet<String>,
) -> Result<HashSet<ChannelKey>, Vec<PipelineAssemblyError>> {
    let mut channels = HashSet::new();
    let mut errors = Vec::new();

    for node in nodes {
        for key in node.port_descriptor().outputs() {
            if !matches!(key, ChannelKey::Sensor(_)) {
                continue;
            }

            if host.contains(key.instance().as_ref()) {
                errors.push(PipelineAssemblyError::SensorOutputShadowsHost {
                    node_name: node.name().to_string(),
                    channel: key.instance().to_string(),
                });
            } else {
                channels.insert(key.clone());
            }
        }
    }

    if errors.is_empty() {
        Ok(channels)
    } else {
        Err(errors)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::pipeline::node::TickContext;
    use crate::port::{
        AlgorithmNodePortDescriptor, InternalChannel, PortBus, PortDescriptor, SensorChannel,
    };

    use helios_core::prelude::TfProvider;

    /// A node that only declares ports; `seed` and `derived_channels` read
    /// nothing else.
    struct PortsOnlyNode {
        name: &'static str,
        descriptor: PortDescriptor,
    }

    impl PipelineNode for PortsOnlyNode {
        fn name(&self) -> &str {
            self.name
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {}
    }

    fn optional_sensor_reader(channel: &str) -> PortsOnlyNode {
        PortsOnlyNode {
            name: "reader",
            descriptor: AlgorithmNodePortDescriptor::new()
                .optional_sensor(SensorChannel::named::<f64>(channel))
                .build(),
        }
    }

    /// A node named `name` writing one sensor channel and one internal channel,
    /// both called `channel`.
    fn writer(name: &'static str, channel: &str) -> Box<dyn PipelineNode> {
        Box::new(PortsOnlyNode {
            name,
            descriptor: AlgorithmNodePortDescriptor::new()
                .output_sensor(SensorChannel::named::<f64>(channel))
                .output_internal(InternalChannel::named::<f64>(channel))
                .build(),
        })
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

    /// A node's sensor outputs are derived; its internal outputs are not, even
    /// under the same name.
    #[test]
    fn only_sensor_outputs_are_derived() {
        let derived = derived_channels(&[writer("deproject", "points")], &HashSet::new())
            .expect("no output shadows a host channel");
        assert_eq!(
            derived,
            HashSet::from([ChannelKey::from(SensorChannel::named::<f64>("points"))])
        );
    }

    /// Every output named like a host channel is reported, not just the first,
    /// and no derived set comes back.
    #[test]
    fn every_output_shadowing_a_host_channel_is_reported() {
        let host = HashSet::from(["front_lidar".to_string(), "rear_lidar".to_string()]);
        let nodes = [
            writer("front_deproject", "front_lidar"),
            writer("rear_deproject", "rear_lidar"),
            writer("accumulate", "points"),
        ];

        let errors = derived_channels(&nodes, &host).expect_err("two outputs shadow the host");

        let mut shadowing: Vec<(&str, &str)> = errors
            .iter()
            .map(|e| match e {
                PipelineAssemblyError::SensorOutputShadowsHost { node_name, channel } => {
                    (node_name.as_str(), channel.as_str())
                }
                other => panic!("expected SensorOutputShadowsHost, got {other}"),
            })
            .collect();
        shadowing.sort();
        assert_eq!(
            shadowing,
            [
                ("front_deproject", "front_lidar"),
                ("rear_deproject", "rear_lidar")
            ]
        );
    }
}
