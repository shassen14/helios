//! `pose`: where one frame is, at each of its poses, in the root its edge
//! names: its position along each of the root's axes. Read with `final`, a
//! body's resting place; with `max`, how far it ranged along an axis.

use crate::evaluation::{
    kind::{BoundInput, InputRejection},
    typed::{
        projections::{rooted_pose, RootedPose},
        BaseFacts, Component, NoParams, OneInput,
    },
};

use helios_core::kernel::si_unit::SiUnit;
use helios_core::spatial::transforms::tf::stamped::StampedTransform;

/// _Per-step._ The `pose` kind, bound to one entry: reads one frame's pose,
/// the child of a transform edge, in the edge's parent. Its writer declares
/// that parent and the bus refuses an edge of another, so every pose of a
/// run is in one root.
pub struct Pose;

impl OneInput for Pose {
    const NAME: &'static str = "pose";
    /// The frame's pose: an edge of a transform tree, its instance the
    /// frame.
    const BASE: &'static str = "pose";
    type Params = NoParams;
    type BasePayload = StampedTransform;
    type BaseValue = RootedPose;
    type Fit = ();
    type State = ();
    type Output = RootedPose;

    fn keep_base(&self, edge: &StampedTransform) -> Result<RootedPose, InputRejection> {
        rooted_pose(edge)
    }

    fn menu() -> Option<Vec<Component<RootedPose>>> {
        Some(components())
    }

    /// The input must select one frame, and its series declare the root its
    /// poses are in, or `x`, `y` and `z` name no axes.
    fn bind(_params: NoParams, pose: &BoundInput) -> Result<Self, Vec<String>> {
        let mut problems = Vec::new();
        match pose.frame(Self::BASE) {
            Ok(Some(_)) => {}
            Ok(None) => problems.push(format!(
                "the {} has no instance: {} reads one frame's pose, so select the frame as \
                 its instance",
                Self::BASE,
                Self::NAME
            )),
            Err(problem) => problems.push(problem),
        }
        if pose.expressed_in.is_none() {
            problems.push(format!(
                "the {}'s series declares no root its poses are in, so x, y and z name no axes",
                Self::BASE
            ));
        }
        if problems.is_empty() {
            Ok(Self)
        } else {
            Err(problems)
        }
    }

    fn components(&self) -> Vec<Component<RootedPose>> {
        components()
    }

    fn fit(&self, _run: &[&RootedPose]) -> Result<(), InputRejection> {
        Ok(())
    }

    fn compute(
        &self,
        pose: &RootedPose,
        _facts: BaseFacts,
        _state: &mut (),
        _fit: &(),
    ) -> Result<RootedPose, InputRejection> {
        Ok(pose.clone())
    }
}

/// The frame's position along each axis of its root, in metres.
fn components() -> Vec<Component<RootedPose>> {
    vec![
        Component::each("x", SiUnit::Meters, |p: &RootedPose| {
            Ok(p.pose.isometry().translation.vector.x)
        }),
        Component::each("y", SiUnit::Meters, |p: &RootedPose| {
            Ok(p.pose.isometry().translation.vector.y)
        }),
        Component::each("z", SiUnit::Meters, |p: &RootedPose| {
            Ok(p.pose.isometry().translation.vector.z)
        }),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::evaluation::{kind::EvaluatorKind, typed::OneInputKind, Gap, GapWidth};

    use helios_core::prelude::{AgentId, FrameId, MonotonicTime};
    use helios_core::spatial::conventions::{Enu, Flu};
    use helios_core::spatial::transforms::{ErasedTransform, Transform};

    use nalgebra::Isometry3;

    fn input(instance: Option<&str>, root: Option<FrameId>) -> BoundInput {
        BoundInput {
            agent: AgentId::new("car"),
            instance: instance.map(str::to_owned),
            expressed_in: root,
            convention: None,
            placement: None,
            schema: None,
            rate_hz: None,
            writer: None,
        }
    }

    fn base_link() -> String {
        FrameId::base_link(AgentId::new("car")).to_string()
    }

    #[test]
    fn a_frame_in_a_declared_root_binds() {
        let bound = Pose::bind(
            NoParams {},
            &input(Some(&base_link()), Some(FrameId::global("datum"))),
        );

        assert!(bound.is_ok());
    }

    #[test]
    fn no_instance_and_no_root_are_each_refused() {
        let problems = Pose::bind(NoParams {}, &input(None, None))
            .err()
            .expect("refused");

        assert_eq!(problems.len(), 2, "{problems:?}");
        assert!(problems[0].contains("has no instance"));
        assert!(problems[1].contains("declares no root"));
    }

    #[test]
    fn x_y_and_z_are_the_frames_position_in_its_root() {
        let edge = StampedTransform {
            parent: FrameId::global("datum"),
            child: FrameId::base_link(AgentId::new("car")),
            stamp: MonotonicTime(0.0),
            transform: ErasedTransform::erase(Transform::<Flu, Enu>::from_isometry(
                Isometry3::translation(-2.5, 4.0, 0.25),
            )),
        };
        let mut evaluation = OneInputKind::<Pose>::new()
            .build(
                &toml::Table::new(),
                &[input(Some(&base_link()), Some(FrameId::global("datum")))],
                &[Some(Gap::Seconds(
                    GapWidth::try_from(1.0).expect("positive"),
                ))],
            )
            .unwrap_or_else(|problems| panic!("{problems:?}"));
        evaluation.receive(0, MonotonicTime(0.0), &edge);

        let derived = evaluation.derive(MonotonicTime(1.0), &[0, 1, 2]);

        let read: Vec<String> = derived
            .metrics
            .iter()
            .map(|series| {
                let series = series.as_ref().expect("wanted");
                format!("{:?}", series.samples()[0].value)
            })
            .collect();
        assert_eq!(read, ["Float(-2.5)", "Float(4.0)", "Float(0.25)"]);
        let units: Vec<SiUnit> = evaluation.metrics().iter().map(|m| m.si_unit).collect();
        assert_eq!(units, [SiUnit::Meters; 3]);
    }
}
