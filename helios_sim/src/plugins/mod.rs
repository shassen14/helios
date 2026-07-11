//! Domain plugins that compose the simulation. Each plugin owns one subsystem.
//!
//! Submodules: `autonomy` (EKF/pipeline wiring), `control` (controller dispatch),
//! `sensors` (IMU, GPS, lidar), `vehicles` (Ackermann, quadcopter physics adapters),
//! `planning`, `world`.
//!
//! The former `metrics` and `research` plugin trees were moved to
//! `docs/archive/helios_sim/` pending a decision to revive (control metrics and
//! data logging migrate to the `helios_test` crate) or delete them.

pub mod sensors;
pub mod vehicles;
pub mod world;
