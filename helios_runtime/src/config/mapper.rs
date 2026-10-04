use serde::Deserialize;

#[derive(Debug, Deserialize, Clone, Copy, Default)]
#[serde(rename_all = "snake_case")]
pub enum MapperPoseSourceConfig {
    #[default]
    GroundTruth,
    Estimated,
}

/// The `kind` tag of [`MapLayerConfig::OccupancyGrid2D`], which is also the key its
/// factory is registered under in the mapper family.
pub(crate) const OCCUPANCY_GRID_2D_KIND: &str = "OccupancyGrid2D";

/// The `kind` tag of [`MapLayerConfig::None`], which is also the key its
/// factory is registered under in the mapper family.
pub(crate) const NO_MAPPER_KIND: &str = "None";

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum MapLayerConfig {
    None,
    OccupancyGrid2D {
        rate: f32,
        resolution: f32,
        /// Bus channel the grid integrates `Vec<SensorReading<PointCloud<Flu>>>`
        /// from. Must match the channel the host's scan sensor publishes on.
        scan_channel: String,
        /// Width of the rolling window in meters (East axis).
        width_m: f32,
        /// Height of the rolling window in meters (North axis).
        height_m: f32,
        #[serde(default)]
        pose_source: MapperPoseSourceConfig,
    },
}

impl MapLayerConfig {
    pub(crate) fn get_kind_str(&self) -> &str {
        match self {
            MapLayerConfig::None => NO_MAPPER_KIND,
            MapLayerConfig::OccupancyGrid2D { .. } => OCCUPANCY_GRID_2D_KIND,
        }
    }

    /// Returns the update rate for mappers that need a `ModuleTimer`, `None` otherwise.
    pub fn get_timer_rate(&self) -> Option<f32> {
        match self {
            MapLayerConfig::OccupancyGrid2D { rate, .. } => Some(*rate),
            MapLayerConfig::None => None,
        }
    }
}
