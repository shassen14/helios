//! Raycasting sensor model trait and concrete sensor implementations.

pub mod lidar_2d;

use crate::data::sensor::{PointCloud2D, PointCloud3D};
use dyn_clone::DynClone;
use nalgebra::Vector3;
use std::fmt::Debug;

/// Represents a single ray to be cast by the simulation engine.
/// All vectors are in the SENSOR's local coordinate frame.
#[derive(Debug, Clone)]
pub struct SensorRay {
    pub id: u32,
    pub direction: Vector3<f64>,
}

/// Represents the result of a single raycast from the physics engine.
#[derive(Debug, Clone)]
pub struct RayHit {
    pub ray_id: u32,
    pub distance: f32,
}

/// Output type for `process_hits`. Typed per sensor family so `MeasurementData` is not required.
#[derive(Debug, Clone)]
pub enum RaycastingOutput {
    PointCloud2D(PointCloud2D),
    PointCloud3D(PointCloud3D),
}

/// The contract for any sensor model that works by casting rays into the environment.
pub trait RaycastingSensorModel: Send + Sync + DynClone + Debug {
    /// Generates the complete set of rays that defines this sensor's scan pattern.
    /// The rays are returned in the sensor's local coordinate frame.
    fn generate_rays(&self) -> Vec<SensorRay>;

    /// Takes the raw results from the physics engine's raycasting and processes
    /// them into a final `RaycastingOutput` packet, applying sensor-specific noise.
    ///
    /// The caller supplies the RNG so this function is deterministic under the project's
    /// seeded-PRNG rule (no `thread_rng` inside algorithm code).
    fn process_hits(&self, hits: &[RayHit], rng: &mut dyn rand::RngCore) -> RaycastingOutput;

    /// Returns the maximum effective range of the sensor in meters.
    fn get_max_range(&self) -> f32;
}

dyn_clone::clone_trait_object!(RaycastingSensorModel);
