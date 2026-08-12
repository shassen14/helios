//! Frame-tagged geometric quantities, split by transformation law.
//!
//! A quantity's Rust type selects how it transforms between frames, so a wrong
//! transform is unspellable rather than a silent runtime bug:
//!
//! - [`Point<F>`] is an **affine position**: under a transform `T = (R, t)` it
//!   maps as `R·p + t`. Points subtract to displacements and translate by them,
//!   but do not add to each other.
//! - [`FreeVector<F>`] is a **free vector** (velocity, force, displacement):
//!   under the same `T` it maps as `R·v`, with **no** translation. Free vectors
//!   form a vector space — they add, negate, and scale.
//!
//! Both are generic over a [`Frame`](super::conventions::Frame) marker, so
//! mixing frames is also a type error. Crossing frames is deliberately not an
//! arithmetic operation; it goes through an explicit rotation or transform
//! (added in a later step).

pub mod free_vector;
pub mod point;

pub use free_vector::FreeVector;
pub use point::Point;

use crate::frames::conventions::{Enu, Flu};

/// A [`FreeVector`] expressed in the world [`Enu`](super::conventions::Enu) frame.
pub type EnuVector = FreeVector<Enu>;

/// A [`FreeVector`] expressed in the body [`Flu`](super::conventions::Flu) frame.
pub type FluVector = FreeVector<Flu>;
