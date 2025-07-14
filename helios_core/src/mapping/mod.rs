use crate::estimation::FilterContext; // Mappers also need context
use crate::messages::ModuleInput;
use nalgebra::{DMatrix, Isometry3};
use std::collections::HashMap;

// --- Map Data Structures ---
// We'll define the MapData enum here, as it's the primary output of this module.
#[derive(Clone, Debug)]
pub enum MapData {
    None, // A default variant for when no map is produced.
    OccupancyGrid2D {
        origin: Isometry3<f64>,
        resolution: f64,
        data: DMatrix<u8>,
    },
    FeatureMap {
        // In the future, this will hold landmark data.
        landmarks: HashMap<u64, Isometry3<f64>>,
    },
}

impl Default for MapData {
    fn default() -> Self {
        MapData::None
    }
}

// --- The Mapper Trait ("Contract") ---
/// The contract for any algorithm that performs the "Mapper" role.
/// Its job is to build a representation of the environment.
pub trait Mapper: Send + Sync {
    /// The method for processing all input data.
    fn process(&mut self, input: &ModuleInput, context: &FilterContext);

    /// The method for retrieving the resulting map.
    fn get_map(&self) -> &MapData;
}

// --- 3. Declare the implementation sub-modules ---
mod none;
// mod occupancy_grid; // When you add this later

// --- 4. Re-export the public structs for a clean API ---
pub use none::NoneMapper;
// pub use occupancy_grid::OccupancyGridMapper; // When you add this later
