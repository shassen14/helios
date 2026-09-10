use crate::frames::{
    conventions::{Enu, Flu, Frame},
    transforms::Rotation,
};

use std::f64::consts::FRAC_PI_2;

use nalgebra::{UnitQuaternion, Vector3};

/// The runtime tag for a Layer-1 axis convention — the erased counterpart of the
/// compile-time [`Frame`] markers.
///
/// A statically-typed quantity carries its convention in a phantom type
/// ([`Enu`] / [`Flu`]); once a transform is erased for the runtime graph
/// ([`ErasedTransform`](super::ErasedTransform)) that type is gone, and this enum
/// records the convention instead so the crossing back to static land can be
/// checked.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Convention {
    Enu,
    Flu,
}

impl std::fmt::Display for Convention {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Convention::Enu => "ENU",
            Convention::Flu => "FLU",
        })
    }
}

/// Maps a compile-time [`Frame`] marker to its runtime [`Convention`].
///
/// A **capability** trait, deliberately kept off [`Frame`] itself: `Frame` is
/// unsealed so downstream crates may add conventions core cannot name (a geodetic
/// ECEF/UTM frame is a parameter-carrying projection, not an axis-relabel, and has
/// no [`Convention`] variant). Such a frame stays a `Frame` and does its own local
/// algebra, but does not implement `ConventionOf`, so it simply cannot cross the
/// erased runtime graph until core learns to tag it. Only frames that participate
/// in transform lookups implement this.
pub trait ConventionOf: Frame {
    const CONVENTION: Convention;
}

impl ConventionOf for Flu {
    const CONVENTION: Convention = Convention::Flu;
}

impl ConventionOf for Enu {
    const CONVENTION: Convention = Convention::Enu;
}

/// The axis geometry behind a [`Convention`]: the rotation carrying this frame's
/// axes into the canonical ENU world frame.
///
/// [`ConventionOf`] records *which* convention a frame is (the discrete tag);
/// this states *how its axes sit* relative to ENU (the geometry). ENU is the
/// crate-canonical world frame — the odom and estimator world frame — so every
/// axis-relabel convention is expressed as its offset from it, and ENU's own
/// basis is the identity.
///
/// It requires [`ConventionOf`] as a supertrait, so it rides the exact line core
/// already draws: an axis-relabel frame gets a basis, while a projection frame
/// (a geodetic ECEF/UTM) has no [`Convention`] and simply does not implement
/// this — the same firewall that keeps it out of the erased graph.
///
/// This is the one fact a downstream frame author must supply to make a new
/// frame convertible: answer "what rotation carries my axes into ENU?" and every
/// crossing built on top of it (rendering relabels, transform composition) is
/// derived. The `Sized` bound is trivially met — every frame marker is a
/// zero-sized unit struct — and lets the return type name `Rotation<Self, Enu>`.
pub trait CanonicalBasis: ConventionOf + Sized {
    /// This frame's axes expressed in ENU: the rotation `Self → Enu`.
    fn to_enu() -> Rotation<Self, Enu>;
}

impl CanonicalBasis for Enu {
    /// ENU is the canonical frame, so its basis is the identity.
    fn to_enu() -> Rotation<Enu, Enu> {
        Rotation::identity()
    }
}

impl CanonicalBasis for Flu {
    /// FLU forward (+X) sits at ENU north (+Y): a +90° turn about the shared up
    /// axis (+Z), which also sends left (+Y) to west (−X) and leaves up fixed.
    fn to_enu() -> Rotation<Flu, Enu> {
        let quat = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        Rotation::from_unit_quaternion(quat)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::quantities::FreeVector;

    /// Floats through a quaternion are not bit-exact; compare within tolerance.
    fn close(a: Vector3<f64>, b: Vector3<f64>) -> bool {
        (a - b).norm() < 1e-9
    }

    #[test]
    fn flu_basis_maps_each_axis_into_enu() {
        // Forward → North, Left → West, Up → Up.
        let fwd = Flu::to_enu().act(FreeVector::<Flu>::new(1.0, 0.0, 0.0));
        let left = Flu::to_enu().act(FreeVector::<Flu>::new(0.0, 1.0, 0.0));
        let up = Flu::to_enu().act(FreeVector::<Flu>::new(0.0, 0.0, 1.0));
        assert!(close(fwd.into_inner(), Vector3::new(0.0, 1.0, 0.0)));
        assert!(close(left.into_inner(), Vector3::new(-1.0, 0.0, 0.0)));
        assert!(close(up.into_inner(), Vector3::new(0.0, 0.0, 1.0)));
    }

    #[test]
    fn enu_basis_is_the_identity() {
        let v = FreeVector::<Enu>::new(1.0, -2.0, 3.0);
        let mapped = Enu::to_enu().act(v);
        assert!(close(mapped.into_inner(), v.into_inner()));
    }
}
