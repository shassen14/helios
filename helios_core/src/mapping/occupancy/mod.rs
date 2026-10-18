//! The occupancy family: probabilistic occupancy-grid mapping. The 2D grid
//! ([`OccupancyGridMapper`]) is the only member today; a 3D / OctoMap / semantic
//! mapper joins here when it lands.

mod occupancy_grid;

pub use occupancy_grid::OccupancyGridMapper;
