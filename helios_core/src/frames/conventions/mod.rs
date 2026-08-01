//! Compile-time coordinate-frame conventions.
//!
//! A [`Frame`] is a zero-sized marker naming the axis convention a quantity is
//! expressed in. Geometry carriers such as [`FrameVector3`] are generic over
//! it, so mixing frames — adding an [`Enu`] vector to an [`Flu`] one — is a
//! type error rather than a silent runtime bug. Crossing between frames is
//! deliberately *not* an arithmetic operation; it goes through an explicit
//! rotation or transform (added in a later step).
//!
//! The trait is intentionally unsealed: downstream crates may define their own
//! conventions (NED, FRD, ECEF, …) by implementing [`Frame`] on their own
//! marker types.

pub mod vector;

pub use vector::FrameVector3;

/// Marker trait for a coordinate-frame convention.
///
/// Implemented only on zero-sized marker types. Carries no methods — its sole
/// job is to tag a geometry type with the frame its components live in.
#[diagnostic::on_unimplemented(
    message = "`{Self}` is not a coordinate-frame convention",
    note = "`Frame` is implemented only on marker types such as `Enu` or `Flu`"
)]
pub trait Frame: Send + Sync + 'static {}

/// World frame: East = +X, North = +Y, Up = +Z. Right-handed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Enu;

impl Frame for Enu {}

/// Body/sensor frame: Forward = +X, Left = +Y, Up = +Z. Right-handed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Flu;

impl Frame for Flu {}

/// A vector expressed in the world [`Enu`] frame.
pub type EnuVector = FrameVector3<Enu>;

/// A vector expressed in the body [`Flu`] frame.
pub type FluVector = FrameVector3<Flu>;
