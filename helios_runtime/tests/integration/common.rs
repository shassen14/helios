// Shared test fixtures for helios_runtime integration tests.
#![allow(dead_code)]

use helios_core::spatial::conventions::Flu;
use helios_core::spatial::primitives::MonotonicTime;
use helios_core::spatial::tf::TfProvider;
use helios_core::spatial::transforms::{Convention, ErasedTransform, Transform};
use helios_core::spatial::FrameId;
use helios_runtime::{BodyChannel, HostDeclaration};

use std::collections::HashMap;

/// How often the hosts these tests stand in for tick.
pub const TICK_RATE_HZ: f64 = 100.0;

/// A host ticking at [`TICK_RATE_HZ`] that names no world frame.
pub fn host() -> HostDeclaration {
    HostDeclaration {
        tick_rate_hz: TICK_RATE_HZ,
        world_frames: HashMap::new(),
    }
}

/// Each frame `channels` are mounted at, in FLU, as a host lists its body's
/// frames.
pub fn mount_frames(channels: &[BodyChannel]) -> HashMap<FrameId, Convention> {
    channels
        .iter()
        .filter_map(BodyChannel::mount)
        .map(|mount| (mount.clone(), Convention::Flu))
        .collect()
}

/// Minimal `TfProvider` returning identity transforms.
pub struct MockRuntime;

impl TfProvider for MockRuntime {
    fn get_transform(&self, _: FrameId, _: FrameId, _: MonotonicTime) -> Option<ErasedTransform> {
        Some(ErasedTransform::erase(Transform::<Flu, Flu>::identity()))
    }
}
