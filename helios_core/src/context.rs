// helios_core/src/context.rs

use crate::prelude::FrameHandle;
use nalgebra::{Isometry3, Vector3};
use std::collections::HashMap;

// This struct provides a snapshot of all necessary kinematic information.
#[derive(Default)]
pub struct KinematicContext {
    /// The pose of every tracked frame relative to the World frame.
    transforms_to_world: HashMap<FrameHandle, Isometry3<f64>>,
    /// The angular velocity of every tracked frame, expressed in its own local frame.
    angular_velocities: HashMap<FrameHandle, Vector3<f64>>,
    /// The angular acceleration of every tracked frame, expressed in its own local frame.
    angular_accelerations: HashMap<FrameHandle, Vector3<f64>>,
}

impl KinematicContext {
    // This is a generic implementation of the TfProvider trait from before.
    pub fn get_transform(&self, from: FrameHandle, to: FrameHandle) -> Option<Isometry3<f64>> {
        let pose_from_world = self.transforms_to_world.get(&from)?;
        let pose_to_world = self.transforms_to_world.get(&to)?;
        Some(pose_from_world.inverse() * pose_to_world)
    }

    pub fn get_angular_velocity(&self, frame: FrameHandle) -> Option<&Vector3<f64>> {
        self.angular_velocities.get(&frame)
    }

    pub fn get_angular_acceleration(&self, frame: FrameHandle) -> Option<&Vector3<f64>> {
        self.angular_accelerations.get(&frame)
    }

    // A method to populate the context from the Bevy world would exist in the adapter crate.
}
