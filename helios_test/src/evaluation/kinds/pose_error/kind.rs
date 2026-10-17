//! [`PoseError`]: the kind, its roles and the metrics it offers, and the
//! checks that bind one entry's evaluator.

use super::{config::PoseErrorConfig, evaluator::PoseErrorEvaluator};
use crate::evaluation::{
    kind::{BoundInput, InputRejection},
    typed::{projections::blend_rooted_poses, Component, Input, TypedEvaluator, TypedKind},
};

use helios_core::kernel::si_unit::SiUnit;
use helios_core::spatial::transforms::PoseDifference;

/// The frame's pose as an estimator believes it: the base.
const ESTIMATE: &str = "estimate";

/// The frame's true pose, from a truth source, interpolated to each
/// estimate's stamp.
const TRUTH: &str = "truth";

/// A heading, along or cross of a frame that faces no way.
const FACES_NO_WAY: InputRejection =
    InputRejection("a frame with no forward axis, which has no heading, along or cross");

/// _Pre-step._ The `pose_error` kind. Both inputs are edges of a transform
/// tree with one child, the frame graded; each edge's parent is its tree's
/// root. Its `align` parameter says how the two roots relate: one frame
/// under `declared`, or placed one in the other by the first sample's two
/// poses under `first_pose`, so a constant offset between the roots is not
/// graded, nor the estimate's error at that first sample. That fits an
/// estimator that starts where its body is. One that starts elsewhere and
/// is pulled in by an aiding sensor (an EKF at its origin until the first GPS
/// fix) carries its starting offset into every later sample.
pub struct PoseError;

impl TypedKind for PoseError {
    const NAME: &'static str = "pose_error";
    type Params = PoseErrorConfig;
    type Evaluator = PoseErrorEvaluator;

    fn inputs(&self) -> <PoseErrorEvaluator as TypedEvaluator>::Inputs {
        (
            Input::base(ESTIMATE, PoseErrorEvaluator::keep),
            Input::interpolated(TRUTH, PoseErrorEvaluator::keep, blend_rooted_poses),
        )
    }

    fn menu(&self) -> Option<Vec<Component<PoseDifference>>> {
        Some(components())
    }

    fn check(&self, params: &PoseErrorConfig) -> Result<(), String> {
        match params.align.unbuilt_until() {
            None => Ok(()),
            Some(first_case) => Err(format!(
                "align = \"{}\" is not built yet; its first case will be {first_case}",
                params.align
            )),
        }
    }

    fn bind(
        &self,
        params: PoseErrorConfig,
        inputs: &[BoundInput],
    ) -> Result<PoseErrorEvaluator, Vec<String>> {
        let (Some(estimate), Some(truth)) = (inputs.first(), inputs.get(1)) else {
            return Err(vec![format!(
                "{} needs both of its inputs bound",
                Self::NAME
            )]);
        };
        match (&estimate.instance, &truth.instance) {
            (Some(estimated), Some(true_frame)) if estimated == true_frame => {
                Ok(PoseErrorEvaluator::new(estimated.clone(), params.align))
            }
            (estimated, true_frame) => Err(vec![format!(
                "the {ESTIMATE} is the pose of '{}' and the {TRUTH} of '{}': {} grades one \
                 frame, so select the same instance for both",
                estimated.as_deref().unwrap_or("no instance"),
                true_frame.as_deref().unwrap_or("no instance"),
                Self::NAME,
            )]),
        }
    }
}

/// The estimate minus the truth, read every way a run file may grade it:
/// the distance between the two positions, split about the root's up axis,
/// on each of the root's axes, and along and across the way the true body
/// faces; the angle between the two orientations, and the heading error
/// about up.
pub(super) fn components() -> Vec<Component<PoseDifference>> {
    vec![
        Component::each("position", SiUnit::Meters, |d: &PoseDifference| {
            Ok(d.position())
        }),
        Component::each("horizontal", SiUnit::Meters, |d: &PoseDifference| {
            Ok(d.horizontal())
        }),
        Component::each("vertical", SiUnit::Meters, |d: &PoseDifference| {
            Ok(d.vertical())
        }),
        Component::each("x", SiUnit::Meters, |d: &PoseDifference| {
            Ok(d.translation().x)
        }),
        Component::each("y", SiUnit::Meters, |d: &PoseDifference| {
            Ok(d.translation().y)
        }),
        Component::each("z", SiUnit::Meters, |d: &PoseDifference| {
            Ok(d.translation().z)
        }),
        Component::each("along", SiUnit::Meters, |d: &PoseDifference| {
            d.along().ok_or(FACES_NO_WAY)
        }),
        Component::each("cross", SiUnit::Meters, |d: &PoseDifference| {
            d.cross().ok_or(FACES_NO_WAY)
        }),
        Component::each("rotation", SiUnit::Radians, |d: &PoseDifference| {
            Ok(d.rotation())
        }),
        Component::each("heading", SiUnit::Radians, |d: &PoseDifference| {
            d.heading().ok_or(FACES_NO_WAY)
        }),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::evaluation::{kind::EvaluatorKind, typed::Typed};

    use helios_core::prelude::{AgentId, FrameId};

    const CAR: &str = "car";

    fn input(instance: Option<FrameId>) -> BoundInput {
        BoundInput {
            agent: AgentId::new(CAR),
            instance: instance.map(|frame| frame.to_string()),
        }
    }

    fn params(text: &str) -> toml::Table {
        text.parse().expect("valid TOML")
    }

    #[test]
    fn inputs_on_two_frames_or_without_an_instance_are_refused() {
        let base_link = FrameId::base_link(AgentId::new(CAR));
        let wheel = FrameId::agent(AgentId::new(CAR), "wheel");

        let two_frames = Typed(PoseError).build(
            &toml::Table::new(),
            &[input(Some(base_link.clone())), input(Some(wheel))],
        );
        let no_instance = Typed(PoseError)
            .build(&toml::Table::new(), &[input(Some(base_link)), input(None)])
            .err()
            .expect("no truth instance");

        assert!(two_frames.is_err());
        assert!(
            no_instance[0].contains("select the same instance for both"),
            "{no_instance:?}"
        );
    }

    #[test]
    fn a_frame_policy_not_built_is_refused_naming_its_first_case() {
        let problem = Typed(PoseError)
            .check_params(&params("align = \"best_fit\""))
            .expect_err("best_fit is not built");

        assert!(
            problem.starts_with("pose_error: align = \"best_fit\""),
            "{problem}"
        );
        assert!(
            problem.contains("the first hardware or SLAM run"),
            "{problem}"
        );
        assert!(Typed(PoseError)
            .check_params(&params("align = \"first_pose\""))
            .is_ok());
        assert!(Typed(PoseError)
            .check_params(&params("gain = 2.0"))
            .is_err());
    }

    #[test]
    fn the_menu_reads_the_pose_every_way_with_its_units() {
        let menu: Vec<(&str, SiUnit)> = Typed(PoseError)
            .menu()
            .expect("a fixed menu")
            .iter()
            .map(|metric| (metric.name, metric.si_unit))
            .collect();

        assert_eq!(
            menu,
            vec![
                ("position", SiUnit::Meters),
                ("horizontal", SiUnit::Meters),
                ("vertical", SiUnit::Meters),
                ("x", SiUnit::Meters),
                ("y", SiUnit::Meters),
                ("z", SiUnit::Meters),
                ("along", SiUnit::Meters),
                ("cross", SiUnit::Meters),
                ("rotation", SiUnit::Radians),
                ("heading", SiUnit::Radians),
            ]
        );
    }
}
