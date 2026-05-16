// Shared test fixtures for helios_runtime integration tests.
#![allow(dead_code)]

use helios_core::data::primitives::{FrameHandle, MonotonicTime};
use helios_runtime::runtime::AgentRuntime;
use nalgebra::Isometry3;

/// Minimal AgentRuntime with identity transforms and a fixed clock.
pub struct MockRuntime;

impl AgentRuntime for MockRuntime {
    fn get_transform(&self, _: FrameHandle, _: FrameHandle) -> Option<Isometry3<f64>> {
        Some(Isometry3::identity())
    }
    fn world_pose(&self, _: FrameHandle) -> Option<Isometry3<f64>> {
        Some(Isometry3::identity())
    }
    fn now(&self) -> MonotonicTime {
        MonotonicTime(0.0)
    }
}
