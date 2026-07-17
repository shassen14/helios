use serde::Deserialize;

#[derive(Debug, Deserialize, Clone, Copy, Default)]
#[serde(rename_all = "snake_case")]
pub enum MapperPoseSourceConfig {
    #[default]
    GroundTruth,
    Estimated,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum MapLayerConfig {
    None,
    OccupancyGrid2D {
        rate: f32,
        resolution: f32,
        /// Bus channel the grid integrates `Vec<SensorReading<PointCloud2D>>`
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
            MapLayerConfig::None => "None",
            MapLayerConfig::OccupancyGrid2D { .. } => "OccupancyGrid2D",
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
