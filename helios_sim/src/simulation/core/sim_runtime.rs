// helios_sim/src/simulation/core/sim_runtime.rs
//
// SimRuntime: implements AgentRuntime over a TfTree snapshot and elapsed time.
// Created fresh each tick by the systems that drive the AutonomyPipeline.

use bevy::prelude::Entity;
use helios_core::prelude::TfProvider;
use helios_core::types::{FrameHandle, MonotonicTime};
use helios_runtime::runtime::AgentRuntime;
use nalgebra::Isometry3;

use crate::simulation::core::transforms::TfTree;

/// Wraps a TfTree reference and the current elapsed simulation time.
pub struct SimRuntime<'a> {
    pub tf: &'a TfTree,
    pub elapsed_secs: f64,
}

impl AgentRuntime for SimRuntime<'_> {
    fn get_transform(&self, from: FrameHandle, to: FrameHandle) -> Option<Isometry3<f64>> {
        self.tf.get_transform(from, to)
    }

    fn world_pose(&self, frame: FrameHandle) -> Option<Isometry3<f64>> {
        self.tf.lookup_by_entity(Entity::from_bits(frame.0))
    }

    fn now(&self) -> MonotonicTime {
        MonotonicTime(self.elapsed_secs)
    }
}
