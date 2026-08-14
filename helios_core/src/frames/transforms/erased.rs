use crate::frames::transforms::{
    convention::{Convention, ConventionOf},
    Transform,
};

use nalgebra::Isometry3;
use std::{error::Error, fmt::Display};

/// A rigid transform whose frame identities have been **erased** to runtime tags.
///
/// This is what a [`TfProvider`](crate::data::ports::TfProvider) lookup returns.
/// The transform graph is keyed by runtime [`FrameId`](crate::frames::FrameId)
/// (`World` / `Body` / `Sensor`), so a lookup cannot hand back a statically-typed
/// [`Transform<A, B>`](Transform) — the frames aren't known at compile time.
/// Instead it carries the raw [`Isometry3`] plus the two [`Convention`] tags of
/// its endpoints, and [`typed`](Self::typed) is the checked crossing back to a
/// static [`Transform`].
///
/// It is the join between Layer 2 (the runtime `FrameId` graph) and Layer 1 (the
/// compile-time convention markers): the graph produces it keyed by *identity*,
/// the leaves consume it typed by *convention*.
pub struct ErasedTransform {
    isometry: Isometry3<f64>,
    from: Convention,
    to: Convention,
}

impl ErasedTransform {
    /// Builds an erased transform from a raw isometry and the [`Convention`] of
    /// each endpoint, in the same order the frames were looked up (`from`, then
    /// `to`).
    ///
    /// The entry point for a provider that works in raw isometries — e.g. the sim
    /// `TfTree`, which derives each endpoint's convention from its
    /// [`FrameId`](crate::frames::FrameId) when it stamps the tags. The tag order
    /// must match the lookup's `(from, to)` order, or [`typed`](Self::typed) will
    /// reject a correct edge.
    pub fn from_parts(isometry: Isometry3<f64>, from: Convention, to: Convention) -> Self {
        Self { isometry, from, to }
    }

    /// Erases a statically-typed [`Transform<A, B>`](Transform) to its runtime
    /// form, reading the tags from the frame markers' [`ConventionOf`] impls.
    ///
    /// The inverse of [`typed`](Self::typed): use it when you already hold a typed
    /// transform and must hand it across a
    /// [`TfProvider`](crate::data::ports::TfProvider) boundary that speaks the
    /// erased form.
    pub fn erase<A: ConventionOf, B: ConventionOf>(transform: Transform<A, B>) -> ErasedTransform {
        ErasedTransform::from_parts(transform.into_inner(), A::CONVENTION, B::CONVENTION)
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
    /// [`FrameId`](crate::frames::FrameId) at the lookup is what pins identity.
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
    use super::ErasedTransform;
    use crate::frames::conventions::{Enu, Flu};
    use crate::frames::transforms::{Convention, Transform};

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
        let erased =
            ErasedTransform::from_parts(sample_isometry(), Convention::Enu, Convention::Flu);
        let typed = erased
            .typed::<Enu, Flu>()
            .expect("matching tags cross cleanly");
        // The isometry survives the erase→typed hop untouched.
        assert_eq!(typed.into_inner(), sample_isometry());
    }

    #[test]
    fn typed_rejects_a_swapped_frame_pair() {
        // The edge is Enu → Flu; asking for Flu → Enu must not silently succeed.
        let erased =
            ErasedTransform::from_parts(sample_isometry(), Convention::Enu, Convention::Flu);
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
        let erased =
            ErasedTransform::from_parts(Isometry3::identity(), Convention::Flu, Convention::Flu);
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
}
