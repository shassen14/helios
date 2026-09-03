// Shared test fixtures for helios_runtime integration tests.
#![allow(dead_code)]

use helios_core::data::primitives::MonotonicTime;
use helios_core::frames::transforms::{Convention, ErasedTransform};
use helios_core::frames::FrameId;
use helios_runtime::runtime::AgentRuntime;
use nalgebra::Isometry3;

/// Minimal AgentRuntime with identity transforms and a fixed clock.
pub struct MockRuntime;

impl AgentRuntime for MockRuntime {
    fn get_transform(&self, _: FrameId, _: FrameId, _: MonotonicTime) -> Option<ErasedTransform> {
        Some(ErasedTransform::from_parts(
            Isometry3::identity(),
            Convention::Flu,
            Convention::Flu,
        ))
    }
    fn now(&self) -> MonotonicTime {
        MonotonicTime(0.0)
    }
}
