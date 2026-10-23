//! References, the world frame a body channel's values are expressed in or
//! relative to: each must be one of the host's world frames, which give it
//! its meaning and its axes. A node pins a frame it defines through an input
//! with one, and a node that defines a frame reads inputs relative to one
//! world frame at most.

use super::declaration::NodeDeclaration;

use crate::{
    port::{ChannelDescription, ChannelKey, Pin, Writer},
    PipelineBuildError,
};

use helios_core::spatial::{transforms::Convention, FrameId};

use std::collections::HashMap;

/// A pin the build resolved: node `node_name` places what `pin` names of a
/// frame it defines in `reference`, the world frame the pin's input is
/// relative to. Only the build makes one, once the pin's writer names a
/// world frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PinnedFrame {
    node_name: String,
    pin: Pin,
    reference: FrameId,
}

impl PinnedFrame {
    /// The node that declared the pin.
    pub fn node_name(&self) -> &str {
        &self.node_name
    }

    /// The pin as the node declared it: the frame, what of it, and the input
    /// that places it.
    pub fn pin(&self) -> &Pin {
        &self.pin
    }

    /// The world frame the pin places the frame in.
    pub fn reference(&self) -> &FrameId {
        &self.reference
    }
}

/// Every reference a body channel states must be one of the host's world
/// frames, `host_frames`. Only the host knows where its world is, so a
/// reference it doesn't declare means nothing the build can check. One error
/// per channel.
pub(super) fn check_references(
    descriptions: &[ChannelDescription],
    host_frames: &HashMap<FrameId, Convention>,
    errors: &mut Vec<PipelineBuildError>,
) {
    for description in descriptions {
        let (Writer::Body { unit, .. }, Some(reference)) =
            (description.writer(), description.reference())
        else {
            continue;
        };
        if !host_frames.contains_key(reference) {
            errors.push(PipelineBuildError::ReferenceNotWorldFrame {
                unit: unit.clone(),
                channel: description.key().clone(),
                reference: reference.clone(),
            });
        }
    }
}

/// Every pin a node declares must be through an input whose writer names a
/// world frame: a body channel's reference, or on a keyed channel the frame
/// the instance is expressed in, when that is one of `host_frames`. A node
/// output names none, so a pin through a mux or a converter is refused until
/// node outputs carry a reference. One error per pin. A pin through an input
/// no writer supplies is the supplier check's to report.
///
/// Returns each pin that names a world frame, with it.
pub(super) fn check_pins(
    nodes: &[NodeDeclaration],
    descriptions: &[ChannelDescription],
    host_frames: &HashMap<FrameId, Convention>,
    errors: &mut Vec<PipelineBuildError>,
) -> Vec<PinnedFrame> {
    let mut pinned = vec![];
    for node in nodes {
        for pin in node.port_descriptor().pins() {
            match pin_reference(pin, descriptions, host_frames) {
                PinReference::World(reference) => pinned.push(PinnedFrame {
                    node_name: node.name().to_string(),
                    pin: pin.clone(),
                    reference: reference.clone(),
                }),
                PinReference::Missing { writer, stated } => {
                    errors.push(PipelineBuildError::PinWithoutReference {
                        node_name: node.name().to_string(),
                        frame: pin.frame().clone(),
                        channel: pin.channel().clone(),
                        instance: pin.instance().cloned(),
                        writer: writer.clone(),
                        stated: stated.cloned(),
                    })
                }
                PinReference::Unwritten => {}
            }
        }
    }
    pinned
}

/// A node that defines a frame places it in one world frame, so the world
/// frames its inputs are relative to, its single-value inputs' references
/// and its pins' frames from `pinned`, must agree. One error per node, naming
/// each input with its world frame. A second datum then needs the host to
/// declare the static relating the two, which no host does yet.
pub(super) fn check_one_world(
    nodes: &[NodeDeclaration],
    descriptions: &[ChannelDescription],
    pinned: &[PinnedFrame],
    errors: &mut Vec<PipelineBuildError>,
) {
    for node in nodes {
        let descriptor = node.port_descriptor();
        if descriptor.defined_frames().is_empty() {
            continue;
        }
        let mut references: Vec<(ChannelKey, FrameId)> = vec![];
        let read = descriptor.inputs().filter_map(|input| {
            let reference = writer_of(input.channel(), descriptions)?.reference()?;
            Some((input.channel(), reference))
        });
        let pins = pinned
            .iter()
            .filter(|pinned| pinned.node_name == node.name())
            .map(|pinned| (pinned.pin.channel(), &pinned.reference));
        for (channel, reference) in read.chain(pins) {
            if !references.iter().any(|(seen, _)| seen == channel) {
                references.push((channel.clone(), reference.clone()));
            }
        }
        let first = references.first().map(|(_, reference)| reference);
        if references
            .iter()
            .any(|(_, reference)| Some(reference) != first)
        {
            errors.push(PipelineBuildError::ReferencesDisagree {
                node_name: node.name().to_string(),
                references,
            });
        }
    }
}

/// The world frame a pin's input names, or what its writer stated instead.
enum PinReference<'a> {
    /// The input is relative to this world frame.
    World(&'a FrameId),
    /// The input's writer names no world frame; `stated` is the frame an
    /// instance is expressed in when that isn't one.
    Missing {
        writer: &'a Writer,
        stated: Option<&'a FrameId>,
    },
    /// No writer supplies the input, or the instance read.
    Unwritten,
}

/// What `pin`'s input names: a single-value channel's reference as its
/// writer states it, or the frame a keyed channel's instance is expressed
/// in, which must be one of `host_frames`. A body reference outside them is
/// refused by [`check_references`] already, so it is taken as given here.
fn pin_reference<'a>(
    pin: &Pin,
    descriptions: &'a [ChannelDescription],
    host_frames: &HashMap<FrameId, Convention>,
) -> PinReference<'a> {
    let Some(instance) = pin.instance() else {
        let Some(description) = writer_of(pin.channel(), descriptions) else {
            return PinReference::Unwritten;
        };
        return match description.reference() {
            Some(reference) => PinReference::World(reference),
            None => PinReference::Missing {
                writer: description.writer(),
                stated: None,
            },
        };
    };
    let Some(description) = descriptions.iter().find(|description| {
        description.key() == pin.channel() && description.instances().contains(instance)
    }) else {
        return PinReference::Unwritten;
    };
    match description.expressed_in(instance) {
        Some(frame) if host_frames.contains_key(frame) => PinReference::World(frame),
        stated => PinReference::Missing {
            writer: description.writer(),
            stated,
        },
    }
}

/// The description of `channel`'s writer, the first if several write it:
/// the supplier check refuses a second.
fn writer_of<'a>(
    channel: &ChannelKey,
    descriptions: &'a [ChannelDescription],
) -> Option<&'a ChannelDescription> {
    descriptions
        .iter()
        .find(|description| description.key() == channel)
}

#[cfg(test)]
mod tests {
    use super::*;

    use super::super::NodePlan;

    use crate::channels::truth_tf;
    use crate::port::{Pinned, PortDescriptorBuilder, Provenance};
    use crate::BodyChannel;

    use helios_core::prelude::AgentId;
    use helios_core::spatial::transforms::tf::stamped::{FrameEdge, StampedTransform};

    struct Reading;

    fn datum() -> FrameId {
        FrameId::global("datum")
    }

    /// A second world frame, as an RTK base at its own datum would be.
    fn site() -> FrameId {
        FrameId::global("site")
    }

    fn base_link() -> FrameId {
        FrameId::base_link(AgentId::new("car"))
    }

    fn root() -> FrameId {
        FrameId::odom(AgentId::new("car"))
    }

    fn sensor(name: &str) -> ChannelKey {
        ChannelKey::measurement::<Reading>(name)
    }

    /// A body channel named `name`, relative to `reference` if given.
    fn body(name: &str, reference: Option<FrameId>) -> ChannelDescription {
        let channel = BodyChannel::new(sensor(name), name, Provenance::Simulated);
        let channel = match reference {
            Some(reference) => channel.with_reference(reference),
            None => channel,
        };
        ChannelDescription::body(&channel)
    }

    /// The body's `truth.tf`, its `base_link` instance expressed in `parent`.
    fn truth(parent: FrameId) -> ChannelDescription {
        let channel = BodyChannel::new(truth_tf(), "truth", Provenance::Exact)
            .with_instances::<StampedTransform>([FrameEdge {
                child: base_link(),
                parent,
            }]);
        ChannelDescription::body(&channel)
    }

    /// Node "ekf" with the ports `ports` declares, defining `root()`.
    fn estimator(ports: PortDescriptorBuilder) -> NodeDeclaration {
        let descriptor = ports.defines_frame(root(), Convention::Enu).build();
        NodePlan::new(
            "ekf",
            descriptor,
            Box::new(|_, _| Err("never built".into())),
        )
        .into()
    }

    /// An estimator pinning its root through `channel`.
    fn pinning(channel: &ChannelKey) -> NodeDeclaration {
        estimator(
            PortDescriptorBuilder::algorithm()
                .optional(channel.clone())
                .pins(root(), Pinned::Position, channel),
        )
    }

    /// An estimator pinning its root through its body's instance of
    /// `truth.tf`, as the mock oracle does.
    fn pinning_truth() -> NodeDeclaration {
        estimator(
            PortDescriptorBuilder::mock()
                .input(truth_tf())
                .pins_by_instance::<StampedTransform>(
                    root(),
                    Pinned::Position,
                    &truth_tf(),
                    &base_link(),
                ),
        )
    }

    /// The resolved pins and the pin and one-world errors, over a host
    /// declaring the datum and the site.
    fn anchoring(
        nodes: &[NodeDeclaration],
        descriptions: &[ChannelDescription],
    ) -> (Vec<PinnedFrame>, Vec<PipelineBuildError>) {
        let host_frames = HashMap::from([(datum(), Convention::Enu), (site(), Convention::Enu)]);
        let mut errors = vec![];
        let pinned = check_pins(nodes, descriptions, &host_frames, &mut errors);
        check_one_world(nodes, descriptions, &pinned, &mut errors);
        (pinned, errors)
    }

    fn anchoring_errors(
        nodes: &[NodeDeclaration],
        descriptions: &[ChannelDescription],
    ) -> Vec<PipelineBuildError> {
        anchoring(nodes, descriptions).1
    }

    /// The node, frame, what and world frame of each resolved pin.
    fn placed(pinned: &[PinnedFrame]) -> Vec<(&str, &FrameId, Pinned, &FrameId)> {
        pinned
            .iter()
            .map(|pinned| {
                (
                    pinned.node_name(),
                    pinned.pin().frame(),
                    pinned.pin().what(),
                    pinned.reference(),
                )
            })
            .collect()
    }

    fn gps(reference: FrameId) -> ChannelDescription {
        let channel = BodyChannel::new(
            ChannelKey::measurement::<Reading>("gps"),
            "gps",
            Provenance::Simulated,
        )
        .with_reference(reference);
        ChannelDescription::body(&channel)
    }

    fn reference_errors(descriptions: &[ChannelDescription]) -> Vec<PipelineBuildError> {
        let host_frames = HashMap::from([(datum(), Convention::Enu)]);
        let mut errors = vec![];
        check_references(descriptions, &host_frames, &mut errors);
        errors
    }

    #[test]
    fn a_reference_the_host_declares_passes() {
        assert!(reference_errors(&[gps(datum())]).is_empty());
    }

    #[test]
    fn a_channel_without_a_reference_passes() {
        let imu = BodyChannel::new(
            ChannelKey::measurement::<Reading>("imu"),
            "imu",
            Provenance::Simulated,
        );
        assert!(reference_errors(&[ChannelDescription::body(&imu)]).is_empty());
    }

    #[test]
    fn a_reference_the_host_does_not_declare_is_refused_naming_it() {
        let errors = reference_errors(&[gps(FrameId::global("site"))]);

        assert_eq!(errors.len(), 1);
        assert!(matches!(
            &errors[0],
            PipelineBuildError::ReferenceNotWorldFrame { unit, reference, .. }
                if unit == "gps" && *reference == FrameId::global("site")
        ));
        assert!(errors[0].to_string().ends_with(
            "is expressed relative to global.site, which is not one of the host's \
             world frames; the host declares each frame its values name"
        ));
    }

    #[test]
    fn a_pin_through_a_referenced_channel_is_placed_in_its_reference() {
        let (pinned, errors) = anchoring(&[pinning(&sensor("gps"))], &[body("gps", Some(datum()))]);

        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(
            placed(&pinned),
            [("ekf", &root(), Pinned::Position, &datum())]
        );
    }

    /// A body channel about the body alone places nothing.
    #[test]
    fn a_pin_through_an_unreferenced_body_channel_is_refused_naming_its_writer() {
        let (pinned, errors) = anchoring(&[pinning(&sensor("gps"))], &[body("gps", None)]);

        assert!(pinned.is_empty());

        assert_eq!(errors.len(), 1);
        assert!(matches!(
            &errors[0],
            PipelineBuildError::PinWithoutReference { node_name, frame, stated: None, .. }
                if node_name == "ekf" && *frame == root()
        ));
        assert!(errors[0].to_string().ends_with(
            "but unit \"gps\" states no world frame its values are relative to; a pinned \
             frame is placed in the world frame its input names"
        ));
    }

    /// A node output carries no reference, so a fix passed through a mux
    /// can't pin.
    #[test]
    fn a_pin_through_a_node_output_is_refused() {
        let errors = anchoring_errors(
            &[pinning(&sensor("gps"))],
            &[ChannelDescription::node(sensor("gps"), "mux")],
        );

        assert_eq!(errors.len(), 1);
        assert!(errors[0]
            .to_string()
            .contains("but node \"mux\" states no world frame"));
    }

    /// A pin no writer supplies is the supplier check's to report.
    #[test]
    fn a_pin_through_an_unwritten_input_is_left_to_the_supplier_check() {
        assert!(anchoring_errors(&[pinning(&sensor("gps"))], &[]).is_empty());
    }

    #[test]
    fn a_pin_by_instance_is_placed_in_the_instances_parent() {
        let (pinned, errors) = anchoring(&[pinning_truth()], &[truth(datum())]);

        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(
            placed(&pinned),
            [("ekf", &root(), Pinned::Position, &datum())]
        );
    }

    /// An instance expressed in a frame the host doesn't own as a world frame
    /// places nothing; the error names that frame.
    #[test]
    fn an_instance_in_no_world_frame_is_refused_naming_its_frame() {
        let mocap = FrameId::global("mocap");
        let errors = anchoring_errors(&[pinning_truth()], &[truth(mocap.clone())]);

        assert_eq!(errors.len(), 1);
        assert!(matches!(
            &errors[0],
            PipelineBuildError::PinWithoutReference { instance: Some(_), stated: Some(stated), .. }
                if *stated == mocap
        ));
        assert!(
            errors[0]
                .to_string()
                .contains("through instance \"agent.car.base_link\" of [oracle]"),
            "{}",
            errors[0]
        );
        assert!(errors[0].to_string().contains(
            "but unit \"truth\" states its values relative to global.mocap, which is not \
             one of the host's world frames"
        ));
    }

    /// A root pinned to the datum can't also read a channel relative to the
    /// site: the two are related by no static the host declares.
    #[test]
    fn inputs_relative_to_two_world_frames_are_refused_naming_each() {
        let gps = sensor("gps");
        let mag = sensor("mag");
        let node = estimator(
            PortDescriptorBuilder::algorithm()
                .optional(gps.clone())
                .optional(mag.clone())
                .pins(root(), Pinned::Position, &gps),
        );
        let errors = anchoring_errors(
            &[node],
            &[body("gps", Some(datum())), body("mag", Some(site()))],
        );

        assert_eq!(errors.len(), 1);
        let PipelineBuildError::ReferencesDisagree {
            node_name,
            references,
        } = &errors[0]
        else {
            panic!("expected disagreeing references, got {}", errors[0]);
        };
        assert_eq!(node_name, "ekf");
        assert_eq!(references, &[(gps, datum()), (mag, site())]);
    }

    /// Inputs relative to one world frame agree, however many there are.
    #[test]
    fn inputs_relative_to_one_world_frame_pass() {
        let gps = sensor("gps");
        let mag = sensor("mag");
        let node = estimator(
            PortDescriptorBuilder::algorithm()
                .optional(gps.clone())
                .optional(mag)
                .pins(root(), Pinned::Position, &gps),
        );
        let errors = anchoring_errors(
            &[node],
            &[body("gps", Some(datum())), body("mag", Some(datum()))],
        );
        assert!(errors.is_empty(), "{errors:?}");
    }
}
