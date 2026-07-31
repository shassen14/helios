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
//!
//! ## Sensor entity names
//!
//! Every spawner names its entity `"{agent}/{sensor}"`, where `{sensor}` is the
//! **key** the sensor is mounted under in the agent's `sensors` map — never a
//! `name` field read off the sensor's own config. The IMU extends this with a
//! `/accelerometer` or `/gyroscope` suffix, since it spawns two entities.
//!
//! The map key is the identifier the config format structurally guarantees
//! unique within an agent: duplicate TOML keys are a parse error, whereas two
//! sensors mounted from the same entity file carry identical `name` fields with
//! nothing to catch it. That distinction is load-bearing rather than cosmetic —
//! this name is the `stable_id` each sensor's RNG stream is derived from, so two
//! sensors sharing one would draw byte-identical noise. Correlated noise reads
//! as plausible sensor data and is not caught by anything downstream.
//!
//! The sensor config structs carry no `name` field for the same reason: it would
//! live in the entity file and so be identical across every mount of that file,
//! which is the one property an instance identifier must not have. Two sensors
//! of one type at different mounting points are two leaf entity files, each with
//! its own `transform` and channels, mounted under two map keys.

pub mod gps;
pub mod imu;
pub mod magnetometer;
pub mod plugin_set;
pub mod raycasting;
pub mod state_sensor;

pub use plugin_set::HeliosSensorsPlugin;
