//! Top-level simulation module grouping all ECS subsystems.
//!
//! Submodules: `core` (TF tree, ground-truth sync), `plugins` (sensor, vehicle,
//! autonomy, control plugins), `registry` (AutonomyRegistry factory dispatch),
//! `config`, `utils`.

pub mod config;
pub mod core;
pub mod plugins;
pub mod registry;
pub mod utils;
