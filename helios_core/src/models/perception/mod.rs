// heios_core/src/models/perception/mod.rs

pub mod lidar_2d;
pub mod lidar_3d;

use crate::messages::MeasurementData;
use dyn_clone::DynClone;
use nalgebra::Vector3;
use std::fmt::Debug;

/// Represents a single ray to be cast by the simulation engine.
/// All vectors are in the SENSOR's local coordinate frame.
#[derive(Debug, Clone)]
pub struct SensorRay {
    /// A unique identifier for this ray within its scan pattern.
    pub id: u32,
    /// The direction vector of the ray. Should be a unit vector.
    pub direction: Vector3<f64>,
}

/// Represents the result of a single raycast from the physics engine.
#[derive(Debug, Clone)]
pub struct RayHit {
    /// The ID of the ray that produced this hit.
    pub ray_id: u32,
    /// The measured distance to the hit object.
    pub distance: f32,
    // Future additions could include:
    // pub hit_normal: Vector3<f64>,
    // pub material_reflectivity: f32,
}

/// The contract for any sensor model that works by casting rays into the environment.
///
/// The model is responsible for defining its own scan pattern (`generate_rays`) and
/// for processing the raw physics results (`process_hits`) into a final, clean
/// `MeasurementData` packet, including the application of sensor-specific noise.
pub trait RaycastingSensorModel: Send + Sync + DynClone + Debug {
    /// Generates the complete set of rays that defines this sensor's scan pattern.
    /// The rays are returned in the sensor's local coordinate frame.
    fn generate_rays(&self) -> Vec<SensorRay>;

    /// Takes the raw results from the physics engine's raycasting and processes
    /// them into a final `MeasurementData` packet. This is where sensor-specific
    /// noise (range, angular) and other effects should be applied.
    ///
    /// # Arguments
    /// * `hits`: A slice containing all the rays that successfully hit an object.
    ///
    /// # Returns
    /// The processed `MeasurementData` (e.g., a PointCloud).
    fn process_hits(&self, hits: &[RayHit]) -> MeasurementData;

    /// Returns the maximum effective range of the sensor in meters.
    fn get_max_range(&self) -> f32;
}

// Make the trait object cloneable.
dyn_clone::clone_trait_object!(RaycastingSensorModel);
