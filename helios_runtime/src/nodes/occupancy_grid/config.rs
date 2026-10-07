//! [`OccupancyGridConfig`] — the `[nodes.<name>]` section of an
//! `OccupancyGrid2D` entry.

use serde::{Deserialize, Serialize};

/// A rolling 2D occupancy grid built from one point-cloud channel. The grid
/// publishes its map on a channel named after the node, so its name is the
/// table key and has no field here.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct OccupancyGridConfig {
    /// Update rate in Hz.
    pub(crate) rate: f32,
    /// Cell size in meters.
    pub(crate) resolution: f32,
    /// Sensor channel the grid integrates `Vec<SensorReading<PointCloud<Flu>>>`
    /// from: a host channel, or a channel another node derives.
    pub(crate) scan_channel: String,
    /// Width of the rolling window in meters (East axis).
    pub(crate) width_m: f32,
    /// Height of the rolling window in meters (North axis).
    pub(crate) height_m: f32,
    /// Which pose the grid places scans with. Not read yet: the node always
    /// reads the `FrameAwareState` channel, which the estimator writes.
    #[serde(default)]
    pub(crate) pose_source: MapperPoseSourceConfig,
}

/// Where a mapper takes the robot's pose from.
#[derive(Debug, Deserialize, Serialize, Clone, Copy, Default, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub(crate) enum MapperPoseSourceConfig {
    /// The host's ground-truth pose, from an oracle channel.
    #[default]
    GroundTruth,
    /// The estimator's state.
    Estimated,
}

#[cfg(test)]
mod tests {
    use super::*;

    const SECTION: &str =
        "rate = 5.0\nresolution = 0.5\nscan_channel = \"scan\"\nwidth_m = 100.0\nheight_m = 100.0";

    // The registry relies on `deny_unknown_fields` to turn a misspelled key
    // into an error instead of a silently ignored one.
    #[test]
    fn a_misspelled_key_is_rejected() {
        let section = format!("{SECTION}\nscan_chanel = \"scan\"");
        assert!(toml::from_str::<OccupancyGridConfig>(&section).is_err());
    }

    #[test]
    fn pose_source_defaults_to_ground_truth_and_reads_snake_case() {
        let default: OccupancyGridConfig = toml::from_str(SECTION).expect("a valid section");
        assert_eq!(default.pose_source, MapperPoseSourceConfig::GroundTruth);

        let estimated: OccupancyGridConfig =
            toml::from_str(&format!("{SECTION}\npose_source = \"estimated\""))
                .expect("a valid section");
        assert_eq!(estimated.pose_source, MapperPoseSourceConfig::Estimated);
    }
}
