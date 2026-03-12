use serde::Deserialize;
use std::path::PathBuf;

/// Configuration for one terrain tile (visual mesh + optional collision mesh).
/// Declared in a scenario TOML under `[[world.terrains]]`.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct TerrainConfig {
    /// Path to the visual GLB, relative to the Bevy asset root.
    pub mesh: PathBuf,
    /// Path to the collision GLB. If absent, no physics collider is created.
    #[serde(default)]
    pub collider: Option<PathBuf>,
    /// Physics medium this terrain represents. Affects agent physics and sensor
    /// behaviour. Recognised values: `"air"`, `"water"`, `"vacuum"`.
    /// Defaults to `"air"`.
    #[serde(default = "default_medium")]
    pub medium: String,
    /// ENU world-frame position offset [x_east, y_north, z_up] in meters.
    /// Defaults to [0, 0, 0].
    #[serde(default)]
    pub position: [f64; 3],
    /// Orientation [roll, pitch, yaw] in **degrees**. Defaults to [0, 0, 0].
    #[serde(default)]
    pub orientation_degrees: [f64; 3],
}

fn default_medium() -> String {
    "air".to_string()
}

/// Atmosphere and lighting configuration for a scenario.
/// Declared in a scenario TOML as `[world.atmosphere]`.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct AtmosphereConfig {
    /// Gravity vector in Bevy world space [x, y, z] m/s². Default = [0, -9.81, 0].
    #[serde(default = "default_gravity")]
    pub gravity: [f32; 3],
    /// Sun elevation above the horizon in **degrees** (0 = horizon, 90 = zenith).
    #[serde(default = "default_sun_elevation")]
    pub sun_elevation: f32,
    /// Sun azimuth in **degrees** measured from North, clockwise
    /// (0 = North, 90 = East, 180 = South, 270 = West).
    #[serde(default = "default_sun_azimuth")]
    pub sun_azimuth: f32,
    /// Directional light illuminance in lux. Default 8000 lx (bright overcast day).
    #[serde(default = "default_ambient_lux")]
    pub ambient_lux: f32,
    /// Exponential fog density (0 = no fog). Default 0.
    #[serde(default)]
    pub fog_density: f32,
}

fn default_gravity() -> [f32; 3]     { [0.0, -9.81, 0.0] }
fn default_sun_elevation() -> f32    { 45.0 }
fn default_sun_azimuth() -> f32      { 180.0 }
fn default_ambient_lux() -> f32      { 8000.0 }

impl Default for AtmosphereConfig {
    fn default() -> Self {
        Self {
            gravity:       default_gravity(),
            sun_elevation: default_sun_elevation(),
            sun_azimuth:   default_sun_azimuth(),
            ambient_lux:   default_ambient_lux(),
            fog_density:   0.0,
        }
    }
}
