//! Domain plugins that compose the simulation. Each plugin owns one subsystem.
//!
//! Submodules: `autonomy` (EKF/pipeline wiring), `control` (controller dispatch),
//! `sensors` (IMU, GPS, lidar), `vehicles` (Ackermann, quadcopter physics adapters),
//! `planning`, `debugging` (gizmos + keybindings),
//! `isolation` (mock profile plugins), `metrics`, `research`, `world`.

pub mod autonomy;
pub mod control;
pub mod debugging;
pub mod isolation;
pub mod metrics;
pub mod path_following;
pub mod planning;
pub mod research;
pub mod sensors;
pub mod vehicles;
pub mod world;
