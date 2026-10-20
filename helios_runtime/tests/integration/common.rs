// Shared test fixtures for helios_runtime integration tests.
#![allow(dead_code)]

use helios_core::spatial::conventions::Flu;
use helios_core::spatial::primitives::MonotonicTime;
use helios_core::spatial::tf::TfProvider;
use helios_core::spatial::transforms::{ErasedTransform, Transform};
use helios_core::spatial::FrameId;

/// Minimal `TfProvider` returning identity transforms.
pub struct MockRuntime;

impl TfProvider for MockRuntime {
    fn get_transform(&self, _: FrameId, _: FrameId, _: MonotonicTime) -> Option<ErasedTransform> {
        Some(ErasedTransform::erase(Transform::<Flu, Flu>::identity()))
    }
}
