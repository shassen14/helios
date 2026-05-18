//! Mapper trait and map data structures.
//!
//! Defines the [`Mapper`] trait (integrate scans, retrieve map) and the
//! [`MapData`] enum whose variants represent different map representations
//! (occupancy grid, feature map). Concrete implementations (
//! `OccupancyGridMapper`) live in submodules.
//!
//! ## Trait shape — sensor-pose-in, map-out
//!
//! The trait is intentionally narrow: callers pass the **already-resolved**
//! sensor world pose (and robot world pose for recentering) and the typed
//! payload. The mapper itself does no TF lookups, holds no sensor identity,
//! and does not know whether the pose came from physics ground-truth or a
//! real estimator — that is the pipeline node's responsibility.
//!
//! When a second mapping family lands (3D occupancy, OctoMap, semantic
//! mapping) split the trait the same way `GaussianStateEstimator` was split
//! out.

use crate::data::sensor::PointCloud2D;
use nalgebra::{DMatrix, Isometry3};
use std::collections::HashMap;

// --- Map Data Structures ---
// We'll define the MapData enum here, as it's the primary output of this module.
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

// --- The Mapper Trait ("Contract") ---
/// The contract for any algorithm that builds a representation of the environment.
///
/// The trait is 2D-scan-shaped today because the only implementor is
/// [`OccupancyGridMapper`]. A 3D / RGBD / semantic mapper will motivate a
/// family split — do that when the second implementor lands, not preemptively.
pub trait Mapper: Send + Sync {
    /// Notify the mapper that the robot has moved to `robot_world_pose`.
    ///
    /// Implementations may use this to recenter a rolling-window grid,
    /// rebuild caches, or no-op if the map is world-fixed.
    fn recenter(&mut self, robot_world_pose: &Isometry3<f64>);

    /// Integrate a single 2D scan reading.
    ///
    /// `sensor_world_pose` is the sensor's pose in the world frame at the
    /// reading's timestamp (composed by the caller from the robot's state
    /// and the static sensor→robot transform). `cloud` is in the sensor's
    /// FLU frame; the mapper transforms it into the world frame internally.
    fn integrate_scan_2d(&mut self, sensor_world_pose: &Isometry3<f64>, cloud: &PointCloud2D);

    /// Return the current map, if one has been produced.
    ///
    /// Returns `None` while the mapper is in cold-start (no scans
    /// integrated yet, or otherwise unable to publish a map). Once it
    /// returns `Some`, subsequent calls may return updated `MapData`
    /// but should not regress to `None` for the lifetime of the mapper.
    ///
    /// Takes `&mut self` so implementations can lazily rebuild a cached
    /// representation from internal log-odds / particle / factor state on
    /// demand. Callers should treat the returned reference as read-only
    /// for the rest of the borrow.
    fn get_map(&mut self) -> Option<&MapData>;
}

// --- 3. Declare the implementation sub-modules ---
mod occupancy_grid;

// --- 4. Re-export the public structs for a clean API ---
pub use occupancy_grid::OccupancyGridMapper;
