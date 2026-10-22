//! The standard projections and blends: what a kind keeps of a common
//! payload, and how a kept value lies between two. Free functions, so a kind
//! in any crate calls them from its own projections; each blend calls core.

use crate::evaluation::kind::InputRejection;

use helios_core::control::{commands::BodyTwist, BodyTwistRef};
use helios_core::interchange::motion::StampedTwist;
use helios_core::prelude::FrameId;
use helios_core::spatial::conventions::Flu;
use helios_core::spatial::transforms::{tf::stamped::StampedTransform, ErasedTransform};

/// _Per-step._ A frame's pose, kept with the root it is in.
#[derive(Debug, Clone)]
pub struct RootedPose {
    pub root: FrameId,
    pub pose: ErasedTransform,
}

/// The pose a transform edge gives its child, in the edge's parent, or
/// refused when it isn't finite.
pub fn rooted_pose(edge: &StampedTransform) -> Result<RootedPose, InputRejection> {
    if !edge.transform.is_finite() {
        return Err(InputRejection("a pose that isn't finite"));
    }
    Ok(RootedPose {
        root: edge.parent.clone(),
        pose: edge.transform,
    })
}

/// The pose `fraction` of the way from `before` to `after`, both in one
/// root, or refused for a frame re-parented or re-tagged between the two,
/// which has no pose between them.
pub fn blend_rooted_poses(
    before: &RootedPose,
    after: &RootedPose,
    fraction: f64,
) -> Result<RootedPose, InputRejection> {
    if before.root != after.root {
        return Err(InputRejection(
            "two poses of a frame in different roots, which have no pose between them",
        ));
    }
    let pose = before
        .pose
        .interpolate(&after.pose, fraction)
        .map_err(|_| {
            InputRejection(
                "two poses of a frame on different axis conventions, which have no pose \
                 between them",
            )
        })?;
    Ok(RootedPose {
        root: before.root.clone(),
        pose,
    })
}

/// The velocity of `body`, expressed in it, or refused when `value` is
/// another frame's velocity or on another frame's axes.
pub fn body_twist(value: &StampedTwist<Flu>, body: &FrameId) -> Result<BodyTwist, InputRejection> {
    if value.moving != *body {
        return Err(InputRejection(
            "a velocity of another frame than the body's base_link",
        ));
    }
    if value.expressed_in != *body {
        return Err(InputRejection(
            "a velocity expressed in another frame than the body's base_link",
        ));
    }
    Ok(BodyTwist::new(value.linear, value.angular))
}

/// The velocity a guidance reference asks of its body, expressed in it by
/// the reference's contract.
pub fn reference_twist(reference: &BodyTwistRef) -> BodyTwist {
    *reference.twist()
}

/// A number, refused when it isn't finite.
pub fn number(value: &f64) -> Result<f64, InputRejection> {
    if value.is_finite() {
        Ok(*value)
    } else {
        Err(InputRejection("a number that isn't finite"))
    }
}

/// The number `fraction` of the way from `before` to `after`.
pub fn blend_numbers(before: &f64, after: &f64, fraction: f64) -> Result<f64, InputRejection> {
    Ok(before + (after - before) * fraction)
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::prelude::{AgentId, MonotonicTime};
    use helios_core::spatial::conventions::Enu;
    use helios_core::spatial::quantities::FreeVector;
    use helios_core::spatial::transforms::{ConventionOf, Transform};

    use nalgebra::Isometry3;

    /// The car's base_link `x` m along `parent`'s x axis, the edge erased
    /// as its producer erases it: a `Transform` from the body to `Root`.
    fn edge<Root: ConventionOf>(parent: FrameId, x: f64) -> StampedTransform {
        StampedTransform {
            parent,
            child: FrameId::base_link(AgentId::new("car")),
            stamp: MonotonicTime(0.0),
            transform: ErasedTransform::erase(Transform::<Flu, Root>::from_isometry(
                Isometry3::translation(x, 0.0, 0.0),
            )),
        }
    }

    fn kept(edge: &StampedTransform) -> RootedPose {
        rooted_pose(edge).unwrap_or_else(|rejection| panic!("{rejection}"))
    }

    #[test]
    fn a_pose_is_blended_in_its_root_and_refused_across_two_roots_or_conventions() {
        let datum = FrameId::global("datum");
        let before = kept(&edge::<Enu>(datum.clone(), 0.0));
        let after = kept(&edge::<Enu>(datum.clone(), 4.0));
        let reparented = kept(&edge::<Enu>(FrameId::global("other"), 4.0));
        let retagged = kept(&edge::<Flu>(datum, 4.0));

        let between = blend_rooted_poses(&before, &after, 0.25).expect("one root");

        assert!((between.pose.isometry().translation.vector.x - 1.0).abs() < 1e-12);
        let reparented = blend_rooted_poses(&before, &reparented, 0.5).err();
        let retagged = blend_rooted_poses(&before, &retagged, 0.5).err();

        assert!(reparented.is_some_and(|why| why.0.contains("different roots")));
        assert!(retagged.is_some_and(|why| why.0.contains("different axis conventions")));
    }

    #[test]
    fn a_pose_that_isnt_finite_is_refused() {
        assert!(rooted_pose(&edge::<Enu>(FrameId::global("datum"), f64::NAN)).is_err());
    }

    #[test]
    fn a_body_twist_is_its_bodys_own_on_its_own_axes() {
        let body = FrameId::base_link(AgentId::new("car"));
        let twist = |moving: FrameId, expressed_in: FrameId| StampedTwist::<Flu> {
            moving,
            relative_to: FrameId::global("datum"),
            expressed_in,
            stamp: MonotonicTime(0.0),
            linear: FreeVector::new(1.0, 0.0, 0.0),
            angular: FreeVector::zeros(),
        };
        let wheel = FrameId::agent(AgentId::new("car"), "wheel");

        assert!(body_twist(&twist(body.clone(), body.clone()), &body).is_ok());
        assert!(body_twist(&twist(wheel.clone(), body.clone()), &body).is_err());
        assert!(body_twist(&twist(body.clone(), wheel), &body).is_err());
    }
}
