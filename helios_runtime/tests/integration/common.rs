// Shared test fixtures for helios_runtime integration tests.
#![allow(dead_code)]

use helios_core::data::ports::TfProvider;
use helios_core::data::primitives::MonotonicTime;
use helios_core::frames::transforms::{Convention, ErasedTransform};
use helios_core::frames::FrameId;
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
