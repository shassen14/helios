use crate::{prelude::RangeField, spatial::conventions::Flu};

use std::fmt::Debug;

use dyn_clone::DynClone;
use nalgebra::Vector3;

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

/// What `process_hits` produces, one variant per output type a ray-cast sensor
/// can emit. A lidar emits an organized [`RangeField`] in its FLU frame.
#[derive(Debug, Clone)]
pub enum RaycastingOutput {
    RangeField(RangeField<Flu>),
}

/// The contract for any sensor model that works by casting rays into the environment.
pub trait RaycastingSensorModel: Send + Sync + DynClone + Debug {
    /// Generates the complete set of rays for one scan, in the sensor's local
    /// frame.
    ///
    /// The rays are where the sensor *actually* points this scan, pointing error
    /// included, so the caller supplies the RNG. Casting them is the host's job;
    /// the error model stays here.
    fn generate_rays(&self, rng: &mut dyn rand::RngCore) -> Vec<SensorRay>;

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
