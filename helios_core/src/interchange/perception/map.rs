use nalgebra::{DMatrix, Isometry3};
use std::collections::HashMap;

// --- Map Data Structures ---
// The primary output of the mapping domain, consumed by planning (and later
// tracking) — a cross-domain noun, so it lives here rather than in `mapping/`.
//
// Cold-start ("no map yet") is expressed by the absence of a value, not by an
// in-band sentinel — `Mapper::get_map` returns `Option<&MapData>` and the bus
// slot stays empty until a mapper has real data.
#[derive(Clone, Debug)]
pub enum MapData {
    OccupancyGrid2D {
        origin: Isometry3<f64>,
        resolution: f64,
        data: DMatrix<u8>,
        /// Monotonically increasing counter; incremented on every `rebuild_cache` call.
        /// Consumers can compare this against a stored value to skip redundant work when
        /// the map data has not changed since the last frame.
        version: u64,
    },
    FeatureMap {
        // In the future, this will hold landmark data.
        landmarks: HashMap<u64, Isometry3<f64>>,
    },
}
