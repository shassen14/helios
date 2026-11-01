//! A joint group: the joints one geometry key holds, and why a list of
//! joints can be refused as one.

use crate::interchange::trajectory::name::KeyName;
use crate::kernel::manifold::geometry::GeometryKind;
use crate::kernel::si_unit::SiUnit;

use std::collections::HashSet;
use std::error::Error;
use std::fmt::{self, Display};
use std::num::NonZeroUsize;

/// How a joint moves. The minimum a key needs for its kind and units;
/// limits come with the robot description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType {
    /// Turns, in radians. A joint that wraps (a continuous wrist, a wheel)
    /// takes the short way round; one that doesn't is bounded short of a full
    /// turn either way, so it is flat.
    Revolute { wraps: bool },
    /// Slides, in meters, and never wraps.
    Prismatic,
}

impl JointType {
    pub fn unit(self) -> SiUnit {
        match self {
            Self::Revolute { .. } => SiUnit::Radians,
            Self::Prismatic => SiUnit::Meters,
        }
    }

    pub fn wraps(self) -> bool {
        matches!(self, Self::Revolute { wraps: true })
    }
}

/// One joint of a group: its name and how it moves.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Joint {
    pub name: KeyName,
    pub joint_type: JointType,
}

/// Named joints stored side by side as one value. Every joint wraps or none
/// does, so the group is one kind: an arm with both is two groups.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JointGroup {
    name: KeyName,
    joints: Vec<Joint>,
    kind: GeometryKind,
}

impl JointGroup {
    /// The group `name` of `joints`, in storage order. Refused when it has no
    /// joints, names one joint twice, or mixes joints that wrap with joints
    /// that don't.
    pub fn new(name: KeyName, joints: Vec<Joint>) -> Result<Self, JointGroupError> {
        let n = NonZeroUsize::new(joints.len()).ok_or(JointGroupError::Empty)?;

        let mut seen = HashSet::new();
        if let Some(twice) = joints.iter().find(|joint| !seen.insert(&joint.name)) {
            return Err(JointGroupError::DuplicateJoint(twice.name.clone()));
        }

        let wrapping = joints
            .iter()
            .filter(|joint| joint.joint_type.wraps())
            .count();
        let kind = if wrapping == 0 {
            GeometryKind::Euclidean { n }
        } else if wrapping == n.get() {
            GeometryKind::Torus { n }
        } else {
            return Err(JointGroupError::MixedWrapping);
        };

        Ok(Self { name, joints, kind })
    }

    pub fn name(&self) -> &KeyName {
        &self.name
    }

    pub fn joints(&self) -> &[Joint] {
        &self.joints
    }

    /// Flat when no joint wraps, a torus when every joint does.
    pub fn kind(&self) -> GeometryKind {
        self.kind
    }

    /// Each joint's unit, in storage order.
    pub fn units(&self) -> impl Iterator<Item = SiUnit> + '_ {
        self.joints.iter().map(|joint| joint.joint_type.unit())
    }
}

/// Why a list of joints is not a [`JointGroup`]. The caller holds the group's
/// name, so it names it beside this.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum JointGroupError {
    Empty,
    DuplicateJoint(KeyName),
    /// Some joints wrap and some don't; they are stored as two groups.
    MixedWrapping,
}

impl Display for JointGroupError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => write!(f, "it has no joints"),
            Self::DuplicateJoint(name) => write!(f, "it names joint '{name}' twice"),
            Self::MixedWrapping => write!(
                f,
                "some of its joints wrap and some don't; split them into two groups"
            ),
        }
    }
}

impl Error for JointGroupError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn name(spelling: &str) -> KeyName {
        KeyName::new(spelling).expect("test names are one level")
    }

    fn joint(spelling: &str, joint_type: JointType) -> Joint {
        Joint {
            name: name(spelling),
            joint_type,
        }
    }

    const BOUNDED: JointType = JointType::Revolute { wraps: false };
    const CONTINUOUS: JointType = JointType::Revolute { wraps: true };

    /// A six-axis arm whose joints all stop short of a full turn.
    fn bounded_arm() -> Vec<Joint> {
        [
            "shoulder_pan",
            "shoulder_lift",
            "elbow",
            "wrist_1",
            "wrist_2",
            "wrist_3",
        ]
        .into_iter()
        .map(|spelling| joint(spelling, BOUNDED))
        .collect()
    }

    #[test]
    fn a_group_with_no_wrapping_joint_is_flat_in_radians() {
        let arm = JointGroup::new(name("arm"), bounded_arm()).expect("a valid group");

        assert_eq!(arm.kind().tangent_dim(), bounded_arm().len());
        assert!(matches!(arm.kind(), GeometryKind::Euclidean { .. }));
        assert!(arm.units().all(|unit| unit == SiUnit::Radians));
    }

    #[test]
    fn a_group_whose_joints_all_wrap_is_a_torus() {
        // A diff-drive's two wheels turn without end.
        let wheels = JointGroup::new(
            name("wheels"),
            vec![joint("left", CONTINUOUS), joint("right", CONTINUOUS)],
        )
        .expect("a valid group");

        assert!(matches!(wheels.kind(), GeometryKind::Torus { .. }));
    }

    #[test]
    fn a_prismatic_joint_beside_a_bounded_one_is_flat_with_each_joints_unit() {
        // A SCARA's lift slides and its elbow turns; neither wraps.
        let scara = JointGroup::new(
            name("scara"),
            vec![joint("elbow", BOUNDED), joint("lift", JointType::Prismatic)],
        )
        .expect("a valid group");

        assert!(matches!(scara.kind(), GeometryKind::Euclidean { .. }));
        assert_eq!(
            scara.units().collect::<Vec<_>>(),
            [SiUnit::Radians, SiUnit::Meters]
        );
    }

    #[test]
    fn an_empty_group_is_refused() {
        assert_eq!(
            JointGroup::new(name("arm"), Vec::new()),
            Err(JointGroupError::Empty)
        );
    }

    #[test]
    fn a_joint_named_twice_is_refused_naming_it() {
        let mut joints = bounded_arm();
        joints.push(joint("elbow", BOUNDED));

        assert_eq!(
            JointGroup::new(name("arm"), joints),
            Err(JointGroupError::DuplicateJoint(name("elbow")))
        );
    }

    #[test]
    fn an_arm_with_a_continuous_wrist_is_refused_until_split() {
        let mut joints = bounded_arm();
        joints.push(joint("tool_spin", CONTINUOUS));

        assert_eq!(
            JointGroup::new(name("arm"), joints),
            Err(JointGroupError::MixedWrapping)
        );

        // Split, both halves are groups.
        assert!(JointGroup::new(name("arm"), bounded_arm()).is_ok());
        assert!(JointGroup::new(name("wrist"), vec![joint("tool_spin", CONTINUOUS)]).is_ok());
    }
}
