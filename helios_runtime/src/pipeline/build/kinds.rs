//! Channel kinds: every node reads and writes only the kinds its class may,
//! by the table beside [`ChannelKind`](crate::port::ChannelKind). An
//! algorithm reading truth is refused here, so the same algorithm runs on
//! hardware, where no truth exists.

use super::declaration::NodeDeclaration;

use crate::PipelineBuildError;

/// Refuses every input whose kind the node's class may not read, and every
/// output whose kind it may not write, one error per channel, naming the
/// node, its class and the channel.
pub(super) fn check_kind_access(nodes: &[NodeDeclaration], errors: &mut Vec<PipelineBuildError>) {
    for node in nodes {
        let descriptor = node.port_descriptor();
        let class = descriptor.class();
        for input in descriptor.inputs() {
            let channel = input.channel();
            if !channel.kind().readable_by(class) {
                errors.push(PipelineBuildError::KindNotReadable {
                    node_name: node.name().to_string(),
                    class,
                    channel: channel.clone(),
                });
            }
        }
        for channel in descriptor.outputs() {
            if !channel.kind().writable_by(class) {
                errors.push(PipelineBuildError::KindNotWritable {
                    node_name: node.name().to_string(),
                    class,
                    channel: channel.clone(),
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::{ChannelKey, PortDescriptor, PortDescriptorBuilder};

    struct Pose;
    struct State;

    fn node(name: &str, descriptor: PortDescriptor) -> NodeDeclaration {
        super::super::NodePlan::new(name, descriptor, Box::new(|_, _| Err("never built".into())))
            .into()
    }

    fn check(nodes: &[NodeDeclaration]) -> Vec<String> {
        let mut errors = vec![];
        check_kind_access(nodes, &mut errors);
        errors.iter().map(ToString::to_string).collect()
    }

    /// A mock reading truth and an algorithm reading measurements and
    /// internals, each writing an internal: nothing is refused.
    #[test]
    fn each_class_reading_and_writing_its_kinds_passes() {
        let mock = PortDescriptorBuilder::mock()
            .input(ChannelKey::oracle::<Pose>("pose"))
            .optional(ChannelKey::oracle::<Pose>("twist"))
            .output(ChannelKey::internal::<State>("truth"))
            .build();
        let algorithm = PortDescriptorBuilder::algorithm()
            .input(ChannelKey::measurement::<Pose>("gps"))
            .optional(ChannelKey::internal::<State>("truth"))
            .output(ChannelKey::internal::<State>("estimate"))
            .output(ChannelKey::measurement::<Pose>("gps.filtered"))
            .build();
        assert_eq!(
            check(&[node("truth", mock), node("filter", algorithm)]),
            Vec::<String>::new()
        );
    }

    /// An algorithm reading truth, required or optional, is refused once per
    /// channel, naming the node, its class and the channel.
    #[test]
    fn an_algorithm_reading_truth_is_refused_per_channel() {
        let descriptor = PortDescriptorBuilder::algorithm()
            .input(ChannelKey::oracle::<Pose>("pose"))
            .optional(ChannelKey::oracle::<Pose>("twist"))
            .build();
        let errors = check(&[node("planner", descriptor)]);
        assert_eq!(errors.len(), 2, "{errors:?}");
        assert!(
            errors[0].contains("algorithm node \"planner\" reads channel [oracle]"),
            "{}",
            errors[0]
        );
        assert!(errors[0].contains("\"pose\""), "{}", errors[0]);
        assert!(errors[1].contains("\"twist\""), "{}", errors[1]);
    }

    /// No node reads device health yet, whatever its class.
    #[test]
    fn reading_health_is_refused_for_every_class() {
        let health = ChannelKey::health::<State>("gps.status");
        let nodes = [
            node(
                "filter",
                PortDescriptorBuilder::algorithm()
                    .input(health.clone())
                    .build(),
            ),
            node(
                "truth",
                PortDescriptorBuilder::mock().optional(health).build(),
            ),
        ];
        assert_eq!(check(&nodes).len(), 2);
    }

    /// Only a body writes truth or health, and a mock never writes a
    /// measurement, which an estimator would read as a device's.
    #[test]
    fn writing_a_kind_the_class_may_not_is_refused() {
        let algorithm = PortDescriptorBuilder::algorithm()
            .output(ChannelKey::oracle::<Pose>("pose"))
            .output(ChannelKey::health::<State>("status"))
            .build();
        let mock = PortDescriptorBuilder::mock()
            .output(ChannelKey::measurement::<Pose>("gps"))
            .build();
        let errors = check(&[node("filter", algorithm), node("truth", mock)]);
        assert_eq!(errors.len(), 3, "{errors:?}");
        assert!(
            errors[0].contains("algorithm node \"filter\" writes channel [oracle]"),
            "{}",
            errors[0]
        );
        assert!(
            errors[2].contains("mock node \"truth\" writes channel [measurement]"),
            "{}",
            errors[2]
        );
    }
}
