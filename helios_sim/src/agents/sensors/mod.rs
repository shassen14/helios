//! Sensor simulation: samples ground-truth world state into the typed readings
//! the pipeline consumes.
//!
//! Each submodule is one sensor's host plugin. The forward models — the actual
//! truth-plus-noise math — live in `helios_core::sensors`; these plugins only
//! supply ground-truth state and a seeded RNG, invoke the model, and publish the
//! payload on the agent's declared channel. State sensors ([`gps`], [`imu`],
//! [`magnetometer`]) need no world query. World sensors ([`raycasting`]) run the
//! two-phase generate-rays / process-hits shape against the physics scene.
//!
//! [`HeliosSensorsPlugin`] adds them all.

pub mod gps;
pub mod imu;
pub mod magnetometer;
pub mod plugin_set;
pub mod raycasting;

pub use plugin_set::HeliosSensorsPlugin;
