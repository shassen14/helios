//! Keyed channels: every writer of a keyed channel declares the instances it
//! writes, each once, and no two writers declare one instance. Several
//! writers may share a keyed channel, as a localizer and an estimate relay
//! share the transform tree's, each owning its own edges.

use crate::{
    port::{ChannelDescription, InstanceKey, Writer},
    ChannelKey, PipelineBuildError,
};

use std::collections::HashMap;

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

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::{ChannelKind, DeclaredInstances, Keyed};

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
