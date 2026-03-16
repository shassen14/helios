//! SLAM super-trait combining state estimation and mapping.
//!
//! [`SlamSystem`] is a marker super-trait for algorithms that implement both
//! `StateEstimator` and `Mapper` in a unified pass. No concrete implementations exist yet.

use crate::estimation::StateEstimator;
use crate::mapping::Mapper;
use std::any::Any;

/// A "super-trait" for a unified SLAM algorithm.
/// It acts as a marker for any struct that implements both StateEstimator and Mapper.
///
/// NOTE: We are defining this now so our Bevy component can exist, but we will not
/// create any concrete implementations of it yet.
pub trait SlamSystem: StateEstimator + Mapper {
    /// Allows for dynamic downcasting to access algorithm-specific methods.
    fn as_any_mut(&mut self) -> &mut dyn Any;
}
