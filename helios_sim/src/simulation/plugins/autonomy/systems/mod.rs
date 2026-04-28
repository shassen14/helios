// helios_sim/src/simulation/plugins/autonomy/systems/mod.rs
//
// Submodule declarations and public re-exports for autonomy systems.

mod estimation;
mod mapping;
mod spawn;
mod spawn_helpers;

pub use estimation::{route_sensor_messages, run_estimation, update_odom_frames};
pub use mapping::run_mapping;
pub use spawn::{spawn_autonomy_pipeline, spawn_odom_frames, spawn_passthrough_pipeline};
