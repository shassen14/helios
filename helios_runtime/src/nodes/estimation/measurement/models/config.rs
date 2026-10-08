//! The measurement kinds' configs: [`AccelerometerConfig`] and
//! [`MagnetometerConfig`]. The `gps_position` and `gyroscope` kinds take no
//! keys.

use crate::nodes::estimation::gravity::default_gravity_enu;

use serde::{Deserialize, Serialize};

/// The `accelerometer` kind's config.
#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(super) struct AccelerometerConfig {
    /// The filter's believed gravity, world ENU `[east, north, up]` (m/s²).
    /// Must match the simulated world's gravity unless the mismatch is the
    /// experiment.
    #[serde(default = "default_gravity_enu")]
    pub(super) gravity_enu: [f64; 3],
}

/// The `magnetometer` kind's config.
#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(super) struct MagnetometerConfig {
    /// The filter's believed local field, world ENU (µT). Required: there is
    /// no sensible default. Must match the simulated world's field unless the
    /// mismatch is the experiment.
    pub(super) magnetic_field_enu: [f64; 3],
}
