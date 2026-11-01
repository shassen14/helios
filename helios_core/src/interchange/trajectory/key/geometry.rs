//! [`GeometryKey`]: where something is, from a closed list of roles.

use crate::interchange::trajectory::anchor::{Anchor, Root};
use crate::interchange::trajectory::key::joint::JointGroup;
use crate::interchange::trajectory::name::KeyName;
use crate::kernel::manifold::{geometry::GeometryKind, rotation};
use crate::kernel::si_unit::SiUnit;

use std::iter;
use std::num::NonZeroUsize;

/// Components of a planar position: the reference frame's horizontal axes.
const PLANAR_DIM: NonZeroUsize = match NonZeroUsize::new(2) {
    Some(n) => n,
    None => panic!("a planar position has components"),
};
/// Components of a spatial position.
const SPATIAL_DIM: NonZeroUsize = match NonZeroUsize::new(3) {
    Some(n) => n,
    None => panic!("a spatial position has components"),
};
/// Components of a heading: one yaw angle.
const HEADING_DIM: NonZeroUsize = NonZeroUsize::MIN;

/// What a sample says about where something is. The role decides the kind
/// and the units, so neither is declared and neither can be wrong.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GeometryKey {
    /// Where an anchor is, in meters.
    Position {
        dim: PositionDim,
        anchor: Anchor,
        root: Root,
    },
    /// Which way an anchor faces about the up axis, in radians.
    Heading { anchor: Anchor, root: Root },
    /// An anchor's full 3D attitude. The body convention its yaw is read in
    /// is bound by the producer's config, not declared here.
    Orientation { anchor: Anchor, root: Root },
    /// Joint values, which have no anchor or root.
    JointGroup(JointGroup),
}

impl GeometryKey {
    /// What tells this key apart from every other key of a sample.
    pub fn id(&self) -> GeometryId {
        match self {
            Self::Position { anchor, .. } => GeometryId::Position(anchor.clone()),
            Self::Heading { anchor, .. } => GeometryId::Heading(anchor.clone()),
            Self::Orientation { anchor, .. } => GeometryId::Orientation(anchor.clone()),
            Self::JointGroup(group) => GeometryId::JointGroup(group.name().clone()),
        }
    }

    pub fn kind(&self) -> GeometryKind {
        match self {
            Self::Position { dim, .. } => GeometryKind::Euclidean {
                n: dim.components(),
            },
            Self::Heading { .. } => GeometryKind::Torus { n: HEADING_DIM },
            Self::Orientation { .. } => GeometryKind::Rotation3,
            Self::JointGroup(group) => group.kind(),
        }
    }

    /// The unit of each component of a step on this key, in order: one per
    /// `tangent_dim` of its kind.
    pub fn tangent_units(&self) -> Vec<SiUnit> {
        match self {
            Self::Position { dim, .. } => {
                iter::repeat_n(SiUnit::Meters, dim.components().get()).collect()
            }
            Self::Heading { .. } => iter::repeat_n(SiUnit::Radians, HEADING_DIM.get()).collect(),
            Self::Orientation { .. } => {
                iter::repeat_n(SiUnit::Radians, rotation::TANGENT_DIM).collect()
            }
            Self::JointGroup(group) => group.units().collect(),
        }
    }

    /// The anchor a spatial key describes; none for a joint group.
    pub fn anchor(&self) -> Option<&Anchor> {
        match self {
            Self::Position { anchor, .. }
            | Self::Heading { anchor, .. }
            | Self::Orientation { anchor, .. } => Some(anchor),
            Self::JointGroup(_) => None,
        }
    }

    /// What a spatial key's anchor is measured from; none for a joint group.
    pub fn root(&self) -> Option<&Root> {
        match self {
            Self::Position { root, .. }
            | Self::Heading { root, .. }
            | Self::Orientation { root, .. } => Some(root),
            Self::JointGroup(_) => None,
        }
    }
}

/// How many axes a position has.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PositionDim {
    /// The reference frame's horizontal axes: a ground vehicle's plan.
    Planar,
    Spatial,
}

impl PositionDim {
    pub fn components(self) -> NonZeroUsize {
        match self {
            Self::Planar => PLANAR_DIM,
            Self::Spatial => SPATIAL_DIM,
        }
    }
}

/// A geometry key's identity: its role and anchor, or its group's name.
/// Other keys name geometry by this, never by position in a list.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum GeometryId {
    Position(Anchor),
    Heading(Anchor),
    Orientation(Anchor),
    JointGroup(KeyName),
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::interchange::trajectory::key::joint::{Joint, JointType};
    use crate::spatial::id::{FrameName, BASE_LINK};

    fn anchor(spelling: &str) -> Anchor {
        Anchor::new(FrameName::new(spelling).expect("test anchors are leaves"))
    }

    fn position(dim: PositionDim, at: &str, root: Root) -> GeometryKey {
        GeometryKey::Position {
            dim,
            anchor: anchor(at),
            root,
        }
    }

    #[test]
    fn a_cars_plan_is_a_planar_position_and_a_heading_of_its_base() {
        let xy = position(PositionDim::Planar, BASE_LINK, Root::Reference);
        let heading = GeometryKey::Heading {
            anchor: anchor(BASE_LINK),
            root: Root::Reference,
        };

        assert_eq!(xy.kind(), GeometryKind::Euclidean { n: PLANAR_DIM });
        assert_eq!(xy.tangent_units(), [SiUnit::Meters; 2]);
        assert_eq!(heading.kind(), GeometryKind::Torus { n: HEADING_DIM });
        assert_eq!(heading.tangent_units(), [SiUnit::Radians]);

        // One anchor, two roles: two identities.
        assert_ne!(xy.id(), heading.id());
        assert_eq!(xy.anchor(), heading.anchor());
    }

    #[test]
    fn a_quadrotors_orientation_stores_four_and_steps_in_three_radians() {
        let attitude = GeometryKey::Orientation {
            anchor: anchor(BASE_LINK),
            root: Root::Reference,
        };

        assert_eq!(attitude.kind(), GeometryKind::Rotation3);
        assert_eq!(attitude.kind().storage_dim(), rotation::STORAGE_DIM);
        assert_eq!(
            attitude.tangent_units(),
            [SiUnit::Radians; rotation::TANGENT_DIM]
        );
        assert_eq!(
            position(PositionDim::Spatial, BASE_LINK, Root::Reference).tangent_units(),
            [SiUnit::Meters; 3]
        );
    }

    #[test]
    fn units_match_the_kinds_tangent_dim_for_every_role() {
        let group = JointGroup::new(
            KeyName::new("arm").expect("a valid name"),
            vec![Joint {
                name: KeyName::new("elbow").expect("a valid name"),
                joint_type: JointType::Prismatic,
            }],
        )
        .expect("a valid group");

        for key in [
            position(PositionDim::Planar, BASE_LINK, Root::Reference),
            position(PositionDim::Spatial, BASE_LINK, Root::Reference),
            GeometryKey::Heading {
                anchor: anchor(BASE_LINK),
                root: Root::Reference,
            },
            GeometryKey::Orientation {
                anchor: anchor(BASE_LINK),
                root: Root::Reference,
            },
            GeometryKey::JointGroup(group),
        ] {
            assert_eq!(
                key.tangent_units().len(),
                key.kind().tangent_dim(),
                "{key:?}"
            );
        }
    }

    #[test]
    fn a_tool_rooted_at_the_base_names_both_anchors() {
        let tool = position(
            PositionDim::Spatial,
            "tool0",
            Root::Anchor(anchor(BASE_LINK)),
        );

        assert_eq!(tool.anchor(), Some(&anchor("tool0")));
        assert_eq!(tool.root(), Some(&Root::Anchor(anchor(BASE_LINK))));
        assert_eq!(tool.id(), GeometryId::Position(anchor("tool0")));
    }

    #[test]
    fn a_legged_robots_named_points_are_distinct_positions() {
        let com = position(PositionDim::Spatial, "com", Root::Reference);
        let zmp = position(PositionDim::Planar, "zmp", Root::Reference);

        assert_ne!(com.id(), zmp.id());
    }

    #[test]
    fn a_joint_group_has_no_anchor_or_root_and_is_named_by_its_group() {
        let arm = KeyName::new("arm").expect("a valid name");
        let group = JointGroup::new(
            arm.clone(),
            vec![Joint {
                name: KeyName::new("elbow").expect("a valid name"),
                joint_type: JointType::Revolute { wraps: false },
            }],
        )
        .expect("a valid group");
        let key = GeometryKey::JointGroup(group);

        assert_eq!(key.anchor(), None);
        assert_eq!(key.root(), None);
        assert_eq!(key.id(), GeometryId::JointGroup(arm));
    }
}
