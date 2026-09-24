// Shared test fixtures for helios_runtime integration tests.
#![allow(dead_code)]

use helios_core::spatial::tf::TfProvider;
use helios_core::spatial::primitives::MonotonicTime;
use helios_core::spatial::transforms::{Convention, ErasedTransform};
use helios_core::spatial::FrameId;
use nalgebra::Isometry3;

/// Minimal `TfProvider` returning identity transforms.
pub struct MockRuntime;

impl TfProvider for MockRuntime {
    fn get_transform(&self, _: FrameId, _: FrameId, _: MonotonicTime) -> Option<ErasedTransform> {
        Some(ErasedTransform::from_parts(
            Isometry3::identity(),
            Convention::Flu,
            Convention::Flu,
        ))
    }
}
