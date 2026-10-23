//! Placements, where each frame a node defines sits in the world: anchored
//! to the world frame a pin of its position names, as a static edge from
//! that world frame, or free, with the reason nothing places it.

use super::declaration::NodeDeclaration;
use super::references::PinnedFrame;

use crate::port::Pinned;

use helios_core::prelude::MonotonicTime;
use helios_core::spatial::transforms::tf::stamped::StampedTransform;
use helios_core::spatial::transforms::{Convention, EdgeEnds, ErasedTransform};
use helios_core::spatial::FrameId;

use std::collections::HashMap;
use std::fmt::Display;

/// Where a frame node `node_name` defines sits in the world.
#[derive(Debug, Clone)]
pub struct FramePlacement {
    node_name: String,
    frame: FrameId,
    placement: Placement,
}

impl FramePlacement {
    /// The node that defines the frame.
    pub fn node_name(&self) -> &str {
        &self.node_name
    }

    /// The frame placed.
    pub fn frame(&self) -> &FrameId {
        &self.frame
    }

    /// Where it sits: anchored to a world frame, or free.
    pub fn placement(&self) -> &Placement {
        &self.placement
    }
}

impl Display for FramePlacement {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let FramePlacement {
            node_name,
            frame,
            placement,
        } = self;
        match placement {
            Placement::Anchored(edge) => write!(
                f,
                "node \"{node_name}\" anchors the position of {frame} at {}",
                edge.parent
            ),
            Placement::Free(Unanchored::NoPin) => write!(
                f,
                "node \"{node_name}\" reads no input that pins the position of {frame}"
            ),
        }
    }
}

/// Where a defined frame sits in the world.
#[derive(Debug, Clone)]
pub enum Placement {
    /// Its position is pinned to a world frame: the static edge from that
    /// world frame to it, at stamp 0. The two share an origin, since the
    /// node compares its input's values with positions in the frame
    /// directly.
    Anchored(StampedTransform),
    /// Nothing places it in the world.
    Free(Unanchored),
}

/// Why a defined frame is free in the world. A pin through an input no
/// writer supplies, or an instance no writer declares, is no reason: the
/// build refuses that input.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Unanchored {
    /// Its node pins its position through no input.
    NoPin,
}

/// Places every frame a node in `nodes` defines, in node and declaration
/// order: anchored at the world frame of its node's first pin of its
/// position in `pinned`, or free. `host_frames` gives each world frame's
/// axes; one a pin names but the host doesn't own is already refused by the
/// reference check, and the build stops on that, so its frame is left free
/// here.
pub(super) fn place_frames(
    nodes: &[NodeDeclaration],
    pinned: &[PinnedFrame],
    host_frames: &HashMap<FrameId, Convention>,
) -> Vec<FramePlacement> {
    let mut placements = vec![];
    for node in nodes {
        for (frame, convention) in node.port_descriptor().defined_frames() {
            let pins = pinned.iter().filter(|pinned| {
                pinned.node_name == node.name()
                    && pinned.pin.frame() == frame
                    && places_origin(pinned.pin.what())
            });
            placements.push(FramePlacement {
                node_name: node.name().to_string(),
                frame: frame.clone(),
                placement: place(frame, *convention, pins, host_frames),
            });
        }
    }
    placements
}

/// Whether a pin of `what` fixes a frame's origin in the world. A heading or
/// an altitude alone would leave it free to move, so each decides its
/// placement when its first model arrives.
fn places_origin(what: Pinned) -> bool {
    match what {
        Pinned::Position => true,
    }
}

/// `frame`, held in `convention`, anchored at the first of `pins` that names a
/// world frame, or free.
fn place<'a>(
    frame: &FrameId,
    convention: Convention,
    pins: impl Iterator<Item = &'a PinnedFrame>,
    host_frames: &HashMap<FrameId, Convention>,
) -> Placement {
    for pinned in pins {
        let reference = &pinned.reference;
        let Some(reference_convention) = host_frames.get(reference) else {
            continue;
        };
        let transform = ErasedTransform::coincident(EdgeEnds {
            child: convention,
            parent: *reference_convention,
        });
        return Placement::Anchored(StampedTransform {
            transform,
            child: frame.clone(),
            parent: reference.clone(),
            stamp: MonotonicTime(0.0),
        });
    }
    Placement::Free(Unanchored::NoPin)
}

#[cfg(test)]
mod tests {
    use super::*;

    use super::super::references::check_pins;
    use super::super::NodePlan;

    use crate::channels::truth_tf;
    use crate::port::{ChannelDescription, ChannelKey, PortDescriptorBuilder, Provenance};
    use crate::BodyChannel;

    use helios_core::prelude::AgentId;
    use helios_core::spatial::transforms::tf::stamped::FrameEdge;

    use nalgebra::Isometry3;

    struct Reading;

    fn agent() -> AgentId {
        AgentId::new("car")
    }

    fn datum() -> FrameId {
        FrameId::global("datum")
    }

    fn root() -> FrameId {
        FrameId::odom(agent())
    }

    fn gps() -> ChannelKey {
        ChannelKey::measurement::<Reading>("gps")
    }

    /// The GPS channel, relative to the datum.
    fn gps_relative_to_datum() -> ChannelDescription {
        let channel = BodyChannel::new(gps(), "gps", Provenance::Simulated).with_reference(datum());
        ChannelDescription::body(&channel)
    }

    /// The body's `truth.tf`, its `base_link` instance expressed in the datum.
    fn truth() -> ChannelDescription {
        let channel = BodyChannel::new(truth_tf(), "truth", Provenance::Exact)
            .with_instances::<StampedTransform>([FrameEdge {
                child: FrameId::base_link(agent()),
                parent: datum(),
            }]);
        ChannelDescription::body(&channel)
    }

    /// Node `name` with the ports `ports` declares, defining `frame` in ENU.
    fn estimator(name: &str, frame: FrameId, ports: PortDescriptorBuilder) -> NodeDeclaration {
        let descriptor = ports.defines_frame(frame, Convention::Enu).build();
        NodePlan::new(name, descriptor, Box::new(|_, _| Err("never built".into()))).into()
    }

    /// Node `name` defining `frame` and pinning its position through the GPS.
    fn pinning_gps(name: &str, frame: FrameId) -> NodeDeclaration {
        let ports = PortDescriptorBuilder::algorithm().optional(gps()).pins(
            frame.clone(),
            Pinned::Position,
            &gps(),
        );
        estimator(name, frame, ports)
    }

    /// Every defined frame's placement, over a host declaring the datum in
    /// ENU.
    fn placements(
        nodes: &[NodeDeclaration],
        descriptions: &[ChannelDescription],
    ) -> Vec<FramePlacement> {
        let host_frames = HashMap::from([(datum(), Convention::Enu)]);
        let mut errors = vec![];
        let pinned = check_pins(nodes, descriptions, &host_frames, &mut errors);
        assert!(errors.is_empty(), "{errors:?}");
        place_frames(nodes, &pinned, &host_frames)
    }

    /// The anchor edge, or a panic naming the placement.
    fn anchor(placement: &FramePlacement) -> &StampedTransform {
        match placement.placement() {
            Placement::Anchored(edge) => edge,
            Placement::Free(_) => panic!("expected an anchor, got: {placement}"),
        }
    }

    #[test]
    fn a_root_pinned_through_the_gps_is_anchored_at_the_datum() {
        let placements = placements(&[pinning_gps("ekf", root())], &[gps_relative_to_datum()]);

        assert_eq!(placements.len(), 1);
        let edge = anchor(&placements[0]);
        assert_eq!(edge.parent, datum());
        assert_eq!(edge.child, root());
        assert_eq!(edge.stamp, MonotonicTime(0.0));
        assert_eq!(edge.transform.isometry(), Isometry3::identity());
        assert_eq!(
            placements[0].to_string(),
            "node \"ekf\" anchors the position of agent.car.odom at global.datum"
        );
    }

    /// A dead-reckoning estimator's root is free, and says why.
    #[test]
    fn a_root_no_input_pins_is_free() {
        let imu = ChannelKey::measurement::<Reading>("imu");
        let imu_channel = BodyChannel::new(imu.clone(), "imu", Provenance::Simulated);
        let node = estimator("ekf", root(), PortDescriptorBuilder::algorithm().input(imu));

        let placements = placements(&[node], &[ChannelDescription::body(&imu_channel)]);

        assert_eq!(placements.len(), 1);
        assert!(matches!(
            placements[0].placement(),
            Placement::Free(Unanchored::NoPin)
        ));
        assert_eq!(
            placements[0].to_string(),
            "node \"ekf\" reads no input that pins the position of agent.car.odom"
        );
    }

    /// A shadow estimator in its own root is anchored beside the
    /// authoritative one, so each can be graded.
    #[test]
    fn a_shadow_root_is_anchored_too() {
        let shadow_root = FrameId::agent(agent(), "shadow.odom");
        let placements = placements(
            &[
                pinning_gps("ekf", root()),
                pinning_gps("shadow", shadow_root.clone()),
            ],
            &[gps_relative_to_datum()],
        );

        let anchored: Vec<(&str, &FrameId, &FrameId)> = placements
            .iter()
            .map(|placement| {
                let edge = anchor(placement);
                (placement.node_name(), &edge.child, &edge.parent)
            })
            .collect();
        assert_eq!(
            anchored,
            [
                ("ekf", &root(), &datum()),
                ("shadow", &shadow_root, &datum())
            ]
        );
    }

    /// The mock oracle's root sits at the parent of the truth instance it
    /// reads.
    #[test]
    fn a_root_pinned_by_instance_is_anchored_at_the_instances_parent() {
        let ports = PortDescriptorBuilder::mock()
            .input(truth_tf())
            .reads_instance::<StampedTransform>(&truth_tf(), &FrameId::base_link(agent()))
            .pins_by_instance::<StampedTransform>(
                root(),
                Pinned::Position,
                &truth_tf(),
                &FrameId::base_link(agent()),
            );
        let mock = estimator("mock", root(), ports);

        let placements = placements(&[mock], &[truth()]);
        assert_eq!(placements.len(), 1);
        assert_eq!(anchor(&placements[0]).parent, datum());
    }
}
