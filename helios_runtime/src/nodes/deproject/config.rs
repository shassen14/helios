//! [`DeprojectConfig`] — the `[nodes.<name>]` section of a `Deproject` entry.

use serde::{Deserialize, Serialize};

/// Where a deproject node reads its range fields and writes its point clouds.
/// The node's name is its table key, so it has no field here.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct DeprojectConfig {
    /// Host sensor channel carrying `Vec<SensorReading<RangeField<Flu>>>`.
    pub(crate) input: String,
    /// Sensor channel this node writes `Vec<SensorReading<PointCloud<Flu>>>`
    /// to. Consumers such as a mapper's `scan_channel` name it; it must not
    /// reuse a host channel's name.
    pub(crate) output: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    // The registry relies on `deny_unknown_fields` to turn a misspelled key
    // into an error instead of a silently ignored one.
    #[test]
    fn a_misspelled_key_is_rejected() {
        let section = "input = \"sensor.lidar.front\"\noutptu = \"sensor.lidar.front.points\"";
        assert!(toml::from_str::<DeprojectConfig>(section).is_err());
    }

    #[test]
    fn both_channels_are_read() {
        let section = "input = \"sensor.lidar.front\"\noutput = \"sensor.lidar.front.points\"";
        let config: DeprojectConfig = toml::from_str(section).expect("a valid section");
        assert_eq!(config.input, "sensor.lidar.front");
        assert_eq!(config.output, "sensor.lidar.front.points");
    }
}
