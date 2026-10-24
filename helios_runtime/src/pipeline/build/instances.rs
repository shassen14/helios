//! Keyed channels: every writer of a keyed channel declares the instances it
//! writes, each once, and no two writers declare one instance. Several
//! writers may share a keyed channel, as a localizer and an estimate relay
//! share the transform tree's, each owning its own edges. Every instance a
//! node reads by name is one some writer declares.

use super::declaration::NodeDeclaration;

use crate::{
    port::{ChannelDescription, InstanceKey, Writer},
    ChannelKey, PipelineBuildError,
};

use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

/// Refuses a writer of a keyed channel that declares no instances, once per
/// writer; an instance one writer declares twice, once per repeat; and an
/// instance a second writer declares, once per second writer, naming the
/// first. Writers are judged in description order: the body and
/// the outside inputs before the nodes.
pub(super) fn check_instances(
    descriptions: &[ChannelDescription],
    errors: &mut Vec<PipelineBuildError>,
) {
    let mut writer_of: HashMap<(&ChannelKey, &InstanceKey), &Writer> = HashMap::new();
    for description in descriptions {
        let channel = description.key();
        if !channel.is_keyed() {
            continue;
        }
        if description.instances().is_empty() {
            errors.push(PipelineBuildError::KeyedWithoutInstances {
                channel: channel.clone(),
                writer: description.writer().clone(),
            });
        }
        for instance in description.instances() {
            match writer_of.get(&(channel, instance)) {
                Some(first) if *first != description.writer() => {
                    errors.push(PipelineBuildError::InstanceWrittenTwice {
                        channel: channel.clone(),
                        instance: instance.clone(),
                        first: (*first).clone(),
                        second: description.writer().clone(),
                    });
                }
                Some(_) => {
                    errors.push(PipelineBuildError::InstanceDeclaredTwice {
                        channel: channel.clone(),
                        instance: instance.clone(),
                        writer: description.writer().clone(),
                    });
                }
                None => {
                    writer_of.insert((channel, instance), description.writer());
                }
            }
        }
    }
}

/// Refuses an instance a node reads by name that no writer of its channel
/// declares, once per instance per input, listing what the writers do
/// declare. Every writer is searched, since a keyed channel may have
/// several. An input with no writer at all, or under a name the
/// channel-name check refused, is left to the checks that report those.
pub(super) fn check_instances_read(
    nodes: &[NodeDeclaration],
    descriptions: &[ChannelDescription],
    refused_names: &HashSet<Arc<str>>,
    errors: &mut Vec<PipelineBuildError>,
) {
    for node in nodes {
        for input in node.port_descriptor().inputs() {
            let channel = input.channel();
            if input.instances().is_empty() || refused_names.contains(channel.name()) {
                continue;
            }
            let writers: Vec<&ChannelDescription> = descriptions
                .iter()
                .filter(|description| description.key() == channel)
                .collect();
            if writers.is_empty() {
                continue;
            }
            let declared: Vec<&InstanceKey> = writers
                .iter()
                .flat_map(|description| description.instances())
                .collect();
            for instance in input.instances() {
                if declared.contains(&instance) {
                    continue;
                }
                errors.push(PipelineBuildError::UnsatisfiedInstance {
                    node_name: node.name().to_string(),
                    channel: channel.clone(),
                    instance: instance.clone(),
                    need: input.need(),
                    declared: declared.iter().map(|&key| key.clone()).collect(),
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use super::super::NodePlan;

    use crate::port::{
        ChannelKind, DeclaredInstances, InputNeed, Keyed, PortDescriptorBuilder, Provenance,
    };
    use crate::BodyChannel;

    use helios_core::spatial::FrameId;

    struct Edge {
        child: &'static str,
    }

    impl Keyed for Edge {
        type Instance = &'static str;
        type Declared = &'static str;

        fn instance(&self) -> &&'static str {
            &self.child
        }

        fn declared_frame(&self) -> Option<&FrameId> {
            None
        }
    }

    fn edges() -> ChannelKey {
        ChannelKey::keyed::<Edge>(ChannelKind::Internal, "edges")
    }

    fn written_by(node: &str, children: &[&'static str]) -> ChannelDescription {
        ChannelDescription::node(edges(), node)
            .with_instances(DeclaredInstances::of::<Edge>(children.iter().copied()))
    }

    fn check(descriptions: &[ChannelDescription]) -> Vec<String> {
        let mut errors = vec![];
        check_instances(descriptions, &mut errors);
        errors.iter().map(ToString::to_string).collect()
    }

    /// A relay owning `base` and a localizer owning `odom` share the channel.
    #[test]
    fn writers_of_distinct_instances_share_a_keyed_channel() {
        assert_eq!(
            check(&[
                written_by("relay", &["base"]),
                written_by("localizer", &["odom"]),
            ]),
            Vec::<String>::new()
        );
    }

    #[test]
    fn a_second_writer_of_one_instance_is_refused_naming_both() {
        let errors = check(&[
            written_by("relay", &["base", "wheel"]),
            written_by("shadow", &["base"]),
        ]);
        assert_eq!(errors.len(), 1);
        assert!(errors[0].contains("instance \"base\""), "{errors:?}");
        assert!(errors[0].contains("node \"relay\""), "{errors:?}");
        assert!(errors[0].contains("node \"shadow\""), "{errors:?}");
    }

    #[test]
    fn one_writer_declaring_an_instance_twice_is_refused() {
        let errors = check(&[written_by("relay", &["base", "base"])]);
        assert_eq!(errors.len(), 1);
        assert!(
            errors[0].contains("node \"relay\" declares instance \"base\""),
            "{errors:?}"
        );
    }

    #[test]
    fn a_keyed_writer_without_instances_is_refused() {
        let errors = check(&[written_by("relay", &[])]);
        assert_eq!(errors.len(), 1);
        assert!(errors[0].contains("declares no instances"), "{errors:?}");
    }

    /// Node "oracle" reading `instance` of `channel`, `required` or not.
    fn reading(channel: &ChannelKey, instance: &'static str, required: bool) -> NodeDeclaration {
        let ports = PortDescriptorBuilder::mock();
        let ports = if required {
            ports.input(channel.clone())
        } else {
            ports.optional(channel.clone())
        };
        let descriptor = ports.reads_instance::<Edge>(channel, &instance).build();
        NodePlan::new(
            "oracle",
            descriptor,
            Box::new(|_, _| Err("never built".into())),
        )
        .into()
    }

    fn read_errors(
        nodes: &[NodeDeclaration],
        descriptions: &[ChannelDescription],
    ) -> Vec<PipelineBuildError> {
        let mut errors = vec![];
        check_instances_read(nodes, descriptions, &HashSet::new(), &mut errors);
        errors
    }

    #[test]
    fn an_instance_a_writer_declares_is_read() {
        assert!(read_errors(
            &[reading(&edges(), "base", true)],
            &[written_by("relay", &["base"])]
        )
        .is_empty());
    }

    /// The supplier map keeps the first writer only; the instance check
    /// searches them all.
    #[test]
    fn an_instance_a_second_writer_declares_is_read() {
        assert!(read_errors(
            &[reading(&edges(), "odom", true)],
            &[
                written_by("relay", &["base"]),
                written_by("localizer", &["odom"])
            ]
        )
        .is_empty());
    }

    #[test]
    fn a_required_instance_no_writer_declares_is_refused_listing_the_declared() {
        let errors = read_errors(
            &[reading(&edges(), "base", true)],
            &[
                written_by("relay", &["wheel"]),
                written_by("localizer", &["odom"]),
            ],
        );

        assert_eq!(errors.len(), 1);
        assert!(matches!(
            &errors[0],
            PipelineBuildError::UnsatisfiedInstance {
                need: InputNeed::Required,
                ..
            }
        ));
        assert_eq!(
            errors[0].to_string(),
            format!(
                "node \"oracle\" reads instance \"base\" of {} (required), which no writer \
                 declares; declared: \"wheel\", \"odom\"",
                edges()
            )
        );
    }

    /// Optional means the node runs without a value, not that the wiring is
    /// unchecked.
    #[test]
    fn an_optional_instance_no_writer_declares_is_refused() {
        let errors = read_errors(
            &[reading(&edges(), "base", false)],
            &[written_by("relay", &["wheel"])],
        );

        assert_eq!(errors.len(), 1);
        assert!(matches!(
            &errors[0],
            PipelineBuildError::UnsatisfiedInstance {
                need: InputNeed::Optional,
                ..
            }
        ));
    }

    /// Only a body supplies truth, so a missing instance is the body's gap:
    /// mocap naming its rigid body, not the agent's `base_link`.
    #[test]
    fn a_truth_instance_the_body_does_not_declare_is_its_gap() {
        let truth = ChannelKey::keyed::<Edge>(ChannelKind::Oracle, "truth.edges");
        let body = BodyChannel::new(truth.clone(), "mocap", Provenance::Exact)
            .with_instances::<Edge>(["mocap_body"]);
        let errors = read_errors(
            &[reading(&truth, "base_link", true)],
            &[ChannelDescription::body(&body)],
        );

        assert_eq!(errors.len(), 1);
        assert_eq!(
            errors[0].to_string(),
            format!(
                "node \"oracle\" reads instance \"base_link\" of {truth} (required), which \
                 the body does not publish; declared: \"mocap_body\""
            )
        );
    }

    /// An input nothing writes is the supplier check's to report.
    #[test]
    fn an_instance_of_an_unwritten_channel_is_left_to_the_supplier_check() {
        assert!(read_errors(&[reading(&edges(), "base", true)], &[]).is_empty());
    }

    /// Single-value channels are the supplier check's: two writers of one
    /// are not judged here.
    #[test]
    fn single_value_channels_are_not_judged() {
        let state = ChannelKey::internal::<Edge>("state");
        assert_eq!(
            check(&[
                ChannelDescription::node(state.clone(), "a"),
                ChannelDescription::node(state, "b"),
            ]),
            Vec::<String>::new()
        );
    }
}
