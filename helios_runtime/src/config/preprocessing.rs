use serde::Deserialize;

/// The `kind` tag of [`PreprocessingConfig::Deproject`], also used to name the
/// kind in errors.
pub(crate) const DEPROJECT_KIND: &str = "Deproject";

/// A node that turns one measurement channel into another before any algorithm
/// consumes it. One variant per kind of transform.
///
/// Every variant reads one channel and writes one, so the assembler can check
/// an output name against the host's channels through
/// [`output_channel`](Self::output_channel) without knowing the kind. A later
/// kind (motion de-skew, lens undistortion, filtering) adds an arm here rather
/// than a new config section.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields, tag = "kind")]
pub enum PreprocessingConfig {
    /// Flattens an organized range field into a point cloud: every cell that
    /// holds a return becomes one point, and cells without one are dropped.
    Deproject {
        /// Host sensor channel carrying `Vec<SensorReading<RangeField<Flu>>>`.
        input: String,
        /// Sensor channel this node writes
        /// `Vec<SensorReading<PointCloud<Flu, ()>>>` to. Consumers such as a
        /// mapper's `scan_channel` name it; it must not reuse a host channel's
        /// name.
        output: String,
    },
}

impl PreprocessingConfig {
    pub(crate) fn get_kind_str(&self) -> &str {
        match self {
            PreprocessingConfig::Deproject { .. } => DEPROJECT_KIND,
        }
    }

    /// The channel this node writes.
    pub(crate) fn output_channel(&self) -> &str {
        match self {
            PreprocessingConfig::Deproject { output, .. } => output,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn deproject() -> PreprocessingConfig {
        PreprocessingConfig::Deproject {
            input: "sensor.lidar.front".to_string(),
            output: "lidar.front.points".to_string(),
        }
    }

    // The kind string names the node kind in errors, so it must match the
    // serde tag a profile writes.
    #[test]
    fn deproject_reports_its_kind() {
        assert_eq!(deproject().get_kind_str(), DEPROJECT_KIND);
        assert_eq!(DEPROJECT_KIND, "Deproject");
    }

    #[test]
    fn output_channel_returns_the_configured_name() {
        assert_eq!(deproject().output_channel(), "lidar.front.points");
    }
}
