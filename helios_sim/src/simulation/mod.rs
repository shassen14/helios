//! Top-level simulation module grouping all ECS subsystems.
//!
//! Submodules: `core` (TF tree, TopicBus, ground-truth sync), `plugins` (sensor, vehicle,
//! autonomy, control, debugging plugins), `profile` + `profile_plugin` (capability-gated
//! plugin loading), `registry` (AutonomyRegistry factory dispatch), `config`, `utils`.

pub mod config;
pub mod core;
pub mod plugins;
pub mod profile;
pub mod profile_plugin;
pub mod registry;
pub mod utils;
