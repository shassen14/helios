use crate::frames::conventions::{Enu, Flu, Frame};

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
