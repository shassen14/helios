use crate::spatial::transforms::{
    convention::{Convention, ConventionOf},
    interpolate_pose, Transform,
};

use nalgebra::{Isometry3, Translation3};
use std::{error::Error, fmt::Display};

/// A rigid transform whose frame identities have been **erased** to runtime tags.
///
/// This is what a [`TfProvider`](crate::spatial::tf::TfProvider) lookup returns.
/// The transform graph is keyed by runtime [`FrameId`](crate::spatial::FrameId)
/// (`World` / `Body` / `Sensor`), so a lookup cannot hand back a statically-typed
/// [`Transform<A, B>`](Transform) — the frames aren't known at compile time.
/// Instead it carries the raw [`Isometry3`] plus the two [`Convention`] tags of
/// its endpoints, and [`typed`](Self::typed) is the checked crossing back to a
/// static [`Transform`].
///
/// It is the join between Layer 2 (the runtime `FrameId` graph) and Layer 1 (the
/// compile-time convention markers): the graph produces it keyed by *identity*,
/// the leaves consume it typed by *convention*.
#[derive(Debug, Clone, Copy)]
pub struct ErasedTransform {
    isometry: Isometry3<f64>,
    from: Convention,
    to: Convention,
}

impl ErasedTransform {
    /// Builds the pose of `ends.child` expressed in `ends.parent` from a raw
    /// isometry, each end's [`Convention`] named by its role.
    ///
    /// The one entry point from a raw isometry, for a value whose conventions
    /// are data, not types: a mount edge whose sensor convention comes from
    /// config, a host tree that stores each frame's convention as its owner
    /// declared it, a calibration file. A typed transform crosses with
    /// [`erase`](Self::erase) instead. The isometry already carries the frames'
    /// actual axes; the tags only name them and never re-orient it. Naming each
    /// end is what keeps the polarity from being inferred from argument order.
    pub fn new(isometry: Isometry3<f64>, ends: EdgeEnds) -> Self {
        Self {
            isometry,
            from: ends.child,
            to: ends.parent,
        }
    }

    /// The pose of a frame whose origin is its parent's, each held in the
    /// axes `ends` names: no translation, and the rotation carrying the
    /// child's axes into the parent's through ENU. Two frames in one
    /// convention coincide exactly.
    pub fn coincident(ends: EdgeEnds) -> Self {
        let rotation = ends.parent.to_enu().inverse() * ends.child.to_enu();
        Self::new(
            Isometry3::from_parts(Translation3::identity(), rotation),
            ends,
        )
    }

    /// Erases a statically-typed [`Transform<A, B>`](Transform) to its runtime
    /// form, reading the tags from the frame markers' [`ConventionOf`] impls.
    ///
    /// The inverse of [`typed`](Self::typed): use it when you already hold a typed
    /// transform and must hand it across a
    /// [`TfProvider`](crate::spatial::tf::TfProvider) boundary that speaks the
    /// erased form.
    pub fn erase<A: ConventionOf, B: ConventionOf>(transform: Transform<A, B>) -> ErasedTransform {
        ErasedTransform::new(
            transform.into_inner(),
            EdgeEnds {
                child: A::CONVENTION,
                parent: B::CONVENTION,
            },
        )
    }

    /// Crosses back to a statically-typed [`Transform<A, B>`](Transform),
    /// checking the stored tags against the requested frames.
    ///
    /// Returns [`Ok`] only when `A`'s convention matches the stored `from` tag and
    /// `B`'s matches `to`; otherwise [`Err`] with a [`FrameMismatch`] naming both
    /// the expected and the actual pair. This is the single, auditable point where
    /// the static frame guarantee is re-established from runtime data — a wrong
    /// turbofish, or a mis-tagged edge, surfaces here as an error the caller must
    /// handle, never as a silently mistagged transform.
    ///
    /// The check is **convention-level, not identity-level**: it verifies both
    /// ends carry the expected axis convention, not that you fetched the specific
    /// edge you meant. Two same-convention frames — a body and its FLU sensor — are
    /// indistinguishable to this check; asking for the right
    /// [`FrameId`](crate::spatial::FrameId) at the lookup is what pins identity.
    pub fn typed<A: ConventionOf, B: ConventionOf>(
        &self,
    ) -> Result<Transform<A, B>, FrameMismatch> {
        if A::CONVENTION != self.from || B::CONVENTION != self.to {
            return Err(FrameMismatch {
                expected_from: A::CONVENTION,
                expected_to: B::CONVENTION,
                actual_from: self.from,
                actual_to: self.to,
            });
        }

        Ok(Transform::from_isometry(self.isometry))
    }

    /// The inverse transform, with its endpoint tags swapped.
    ///
    /// An edge tagged `from → to` inverts to one tagged `to → from`, so the
    /// isometry and the two [`Convention`] tags must move together — invert the
    /// isometry alone and [`typed`](Self::typed) would reject the correct
    /// endpoints. This is the *flip* half of the LCA lookup walk: an edge stored
    /// `child → parent` is inverted when the walk needs `parent → child`.
    pub fn inverse(self) -> ErasedTransform {
        ErasedTransform {
            isometry: self.isometry.inverse(),
            from: self.to,
            to: self.from,
        }
    }

    /// Composition: `self` (`from → to`) followed by `other` (`to → next`),
    /// giving the direct transform `from → next`.
    ///
    /// The *chain* half of the LCA lookup walk: adjacent edges fold into one as
    /// the walk climbs from a leaf to the common ancestor and back down. Isometry
    /// multiplication applies its right operand first, so "`self` then `other`"
    /// is `other.isometry * self.isometry`, matching [`Transform::then`].
    ///
    /// The seam tags (`self.to` and `other.from`) must agree, but that agreement
    /// is a tree invariant the buffer's ingest already guarantees — a mismatch
    /// here is a buffer bug, not bad input — so it is a `debug_assert` rather
    /// than a `Result` the whole walk would have to thread.
    pub fn then(self, other: ErasedTransform) -> ErasedTransform {
        debug_assert!(
            self.to == other.from,
            "ErasedTransform compose seam mismatch: self is tagged … → {:?}, \
             other is tagged {:?} → …",
            self.to,
            other.from,
        );

        ErasedTransform {
            isometry: other.isometry * self.isometry,
            from: self.from,
            to: other.to,
        }
    }

    /// The transform `fraction` of the way from `self` to `to`, as
    /// [`interpolate_pose`] blends them, or a [`FrameMismatch`] when the two
    /// are tagged with different conventions: a blend of an ENU pose and a
    /// NED one is neither.
    pub fn interpolate(
        &self,
        to: &ErasedTransform,
        fraction: f64,
    ) -> Result<ErasedTransform, FrameMismatch> {
        self.same_conventions(to)?;
        Ok(ErasedTransform {
            isometry: interpolate_pose(&self.isometry, &to.isometry, fraction),
            ..*self
        })
    }

    pub fn from_convention(&self) -> Convention {
        self.from
    }

    pub fn to_convention(&self) -> Convention {
        self.to
    }

    pub fn isometry(&self) -> Isometry3<f64> {
        self.isometry
    }

    /// Whether every component of its translation and rotation is finite.
    pub fn is_finite(&self) -> bool {
        self.isometry
            .translation
            .vector
            .iter()
            .chain(self.isometry.rotation.coords.iter())
            .all(|value| value.is_finite())
    }

    /// `Ok` when `other` is tagged with the same two conventions, so their
    /// isometries are in the same axes and can be compared or blended.
    /// Otherwise a [`FrameMismatch`] that expected `self`'s tags.
    pub fn same_conventions(&self, other: &ErasedTransform) -> Result<(), FrameMismatch> {
        if self.from == other.from && self.to == other.to {
            return Ok(());
        }
        Err(FrameMismatch {
            expected_from: self.from,
            expected_to: self.to,
            actual_from: other.from,
            actual_to: other.to,
        })
    }
}

/// The two ends of an [`ErasedTransform`], each by its role, with its frame's
/// [`Convention`]: `child`, the frame posed, and `parent`, the frame its pose
/// is expressed in. As tf2 labels a lookup's result, a pose between any two
/// frames has a child and a parent, edge of the tree or not.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdgeEnds {
    pub child: Convention,
    pub parent: Convention,
}

/// The error from [`ErasedTransform::typed`] when the requested frame pair does
/// not match the edge's stored tags.
///
/// Carries both sides so a log line can name the discrepancy: the `expected_*`
/// pair is what the caller asked for (`A` / `B`), the `actual_*` pair is what the
/// edge was tagged with at construction.
#[derive(Debug)]
pub struct FrameMismatch {
    expected_from: Convention,
    expected_to: Convention,
    actual_from: Convention,
    actual_to: Convention,
}

impl Display for FrameMismatch {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "frame convention mismatch: asked for {:?} → {:?}, edge is tagged {:?} → {:?}",
            self.expected_from, self.expected_to, self.actual_from, self.actual_to
        )
    }
}

impl Error for FrameMismatch {}

#[cfg(test)]
mod tests {
    use super::{EdgeEnds, ErasedTransform};
    use crate::spatial::conventions::{Enu, Flu};
    use crate::spatial::transforms::{Convention, Transform};

    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
    use std::f64::consts::FRAC_PI_2;

    /// A non-identity `Enu → Flu` isometry (quarter turn about +Z, +10 along X),
    /// so a passing round-trip proves the crossing preserves the raw data, not
    /// just the tags.
    fn sample_isometry() -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(10.0, 0.0, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2),
        )
    }

    #[test]
    fn typed_recovers_the_transform_when_conventions_match() {
        let erased = ErasedTransform::new(
            sample_isometry(),
            EdgeEnds {
                child: Convention::Enu,
                parent: Convention::Flu,
            },
        );
        let typed = erased
            .typed::<Enu, Flu>()
            .expect("matching tags cross cleanly");
        // The isometry survives the erase→typed hop untouched.
        assert_eq!(typed.into_inner(), sample_isometry());
    }

    #[test]
    fn named_ends_tag_the_child_as_the_frame_posed_and_the_parent_as_its_root() {
        let mount = Isometry3::translation(0.5, 0.0, 0.2);

        let lidar_in_body = ErasedTransform::new(
            mount,
            EdgeEnds {
                child: Convention::Flu,
                parent: Convention::Enu,
            },
        );

        let typed = lidar_in_body
            .typed::<Flu, Enu>()
            .expect("the child is the frame posed");
        assert_eq!(typed.into_inner(), mount);
        assert!(lidar_in_body.typed::<Enu, Flu>().is_err());
    }

    #[test]
    fn typed_rejects_a_swapped_frame_pair() {
        // The edge is Enu → Flu; asking for Flu → Enu must not silently succeed.
        let erased = ErasedTransform::new(
            sample_isometry(),
            EdgeEnds {
                child: Convention::Enu,
                parent: Convention::Flu,
            },
        );
        let err = erased
            .typed::<Flu, Enu>()
            .expect_err("a swapped frame pair must be rejected");
        assert_eq!(err.expected_from, Convention::Flu);
        assert_eq!(err.expected_to, Convention::Enu);
        assert_eq!(err.actual_from, Convention::Enu);
        assert_eq!(err.actual_to, Convention::Flu);
    }

    #[test]
    fn typed_rejects_a_same_role_convention_mismatch() {
        // Both ends are FLU on the edge, but the caller asks for Enu on the `from`
        // side — the cross-convention confusion the tag check exists to catch.
        let erased = ErasedTransform::new(
            Isometry3::identity(),
            EdgeEnds {
                child: Convention::Flu,
                parent: Convention::Flu,
            },
        );
        assert!(erased.typed::<Enu, Flu>().is_err());
    }

    #[test]
    fn erase_then_typed_round_trips() {
        let transform = Transform::<Enu, Flu>::from_isometry(sample_isometry());
        let erased = ErasedTransform::erase(transform);
        let recovered = erased
            .typed::<Enu, Flu>()
            .expect("erase then typed round-trips through matching tags");
        assert_eq!(recovered.into_inner(), sample_isometry());
    }

    /// A second, distinct `Flu → Enu` isometry (quarter turn about +X, +5 along
    /// Y) so a two-hop chain composes two *different* transforms — a chain of one
    /// repeated edge would hide an argument-order bug.
    fn second_isometry() -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(0.0, 5.0, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), FRAC_PI_2),
        )
    }

    /// The sample edge as an [`ErasedTransform`] tagged `Enu → Flu`. A helper
    /// because `then`/`inverse` consume `self` by value, so a test that uses one
    /// edge twice needs two independent values.
    fn sample_edge() -> ErasedTransform {
        ErasedTransform::new(
            sample_isometry(),
            EdgeEnds {
                child: Convention::Enu,
                parent: Convention::Flu,
            },
        )
    }

    /// Two isometries are close when translation and rotation agree within float
    /// tolerance. `angle_to` folds the quaternion double-cover (`q` and `−q` are
    /// the same rotation) into one non-negative angle, so a compose/inverse
    /// round-trip that lands on `−q` still reads as identity.
    fn iso_close(a: Isometry3<f64>, b: Isometry3<f64>) -> bool {
        (a.translation.vector - b.translation.vector).norm() < 1e-9
            && a.rotation.angle_to(&b.rotation) < 1e-9
    }

    #[test]
    fn inverse_swaps_the_tags() {
        // The isometry-and-tags-move-together invariant: an Enu → Flu edge
        // inverts to a Flu → Enu one, or a correct edge would fail `typed`.
        let inverted = sample_edge().inverse();
        assert_eq!(inverted.from, Convention::Flu);
        assert_eq!(inverted.to, Convention::Enu);
        assert!(iso_close(inverted.isometry, sample_isometry().inverse()));
    }

    #[test]
    fn inverse_is_its_own_undo() {
        // Inverting twice is a no-op — data and tags both return to the original.
        let there_and_back = sample_edge().inverse().inverse();
        assert!(iso_close(there_and_back.isometry, sample_isometry()));
        assert_eq!(there_and_back.from, Convention::Enu);
        assert_eq!(there_and_back.to, Convention::Flu);
    }

    #[test]
    fn then_keeps_the_endpoints_and_drops_the_seam() {
        // Enu → Flu then Flu → Enu is tagged Enu → Enu: `from` comes from the
        // first edge, `to` from the second, and the shared Flu seam vanishes. A
        // `to: self.to` bug would leave `to` tagged Flu and fail here.
        let chained = sample_edge().then(ErasedTransform::new(
            second_isometry(),
            EdgeEnds {
                child: Convention::Flu,
                parent: Convention::Enu,
            },
        ));
        assert_eq!(chained.from, Convention::Enu);
        assert_eq!(chained.to, Convention::Enu);
    }

    #[test]
    fn then_composes_right_operand_first() {
        // "self then other" must apply self first, i.e. other.iso * self.iso.
        // The two hops differ, so the reversed product would land elsewhere and
        // this pins the direction, not just that *some* composition happened.
        let chained = sample_edge().then(ErasedTransform::new(
            second_isometry(),
            EdgeEnds {
                child: Convention::Flu,
                parent: Convention::Enu,
            },
        ));
        assert!(iso_close(
            chained.isometry,
            second_isometry() * sample_isometry()
        ));
    }

    #[test]
    fn then_an_edge_with_its_inverse_is_identity() {
        // The capstone: A → B folded with (A → B)⁻¹ = B → A collapses to the
        // identity pose on A → A — the flip-and-chain the LCA walk relies on.
        let round = sample_edge().then(sample_edge().inverse());
        assert!(iso_close(round.isometry, Isometry3::identity()));
        assert_eq!(round.from, Convention::Enu);
        assert_eq!(round.to, Convention::Enu);
    }

    #[test]
    fn interpolate_blends_alike_tagged_transforms_and_refuses_unlike_ones() {
        let start = ErasedTransform::new(
            Isometry3::identity(),
            EdgeEnds {
                child: Convention::Enu,
                parent: Convention::Flu,
            },
        );
        let between = start
            .interpolate(&sample_edge(), 0.5)
            .expect("alike tags blend");
        assert!(iso_close(
            between.isometry,
            Isometry3::from_parts(
                Translation3::new(5.0, 0.0, 0.0),
                UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2 / 2.0),
            )
        ));
        assert_eq!(between.from, Convention::Enu);

        let unlike = ErasedTransform::new(
            Isometry3::identity(),
            EdgeEnds {
                child: Convention::Flu,
                parent: Convention::Flu,
            },
        );
        assert!(unlike.interpolate(&sample_edge(), 0.5).is_err());
    }

    #[test]
    fn a_transform_with_a_non_finite_component_is_not_finite() {
        let nan = ErasedTransform::new(
            Isometry3::translation(f64::NAN, 0.0, 0.0),
            EdgeEnds {
                child: Convention::Enu,
                parent: Convention::Flu,
            },
        );
        assert!(sample_edge().is_finite());
        assert!(!nan.is_finite());
    }

    #[test]
    fn frames_in_one_convention_coincide_exactly() {
        let ends = EdgeEnds {
            child: Convention::Enu,
            parent: Convention::Enu,
        };
        let coincident = ErasedTransform::coincident(ends);

        assert!(iso_close(coincident.isometry, Isometry3::identity()));
        assert_eq!(coincident.from, Convention::Enu);
        assert_eq!(coincident.to, Convention::Enu);
    }

    /// A body frame at the world's origin: its forward is the world's north
    /// and its left the world's west, and the edge back is the inverse.
    #[test]
    fn a_frame_in_other_axes_is_turned_into_its_parents() {
        let flu_on_enu = ErasedTransform::coincident(EdgeEnds {
            child: Convention::Flu,
            parent: Convention::Enu,
        });
        let enu_on_flu = ErasedTransform::coincident(EdgeEnds {
            child: Convention::Enu,
            parent: Convention::Flu,
        });

        let rotation = flu_on_enu.isometry.rotation;
        assert!(flu_on_enu.isometry.translation.vector.norm() < 1e-9);
        assert!((rotation * Vector3::x() - Vector3::y()).norm() < 1e-9);
        assert!((rotation * Vector3::y() + Vector3::x()).norm() < 1e-9);
        assert_eq!(flu_on_enu.from, Convention::Flu);
        assert_eq!(flu_on_enu.to, Convention::Enu);
        assert!(iso_close(
            enu_on_flu.isometry,
            flu_on_enu.isometry.inverse()
        ));
    }
}
