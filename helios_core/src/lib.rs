//! Pure algorithmic library for robotics. Framework-agnostic; runs on real hardware.
//!
//! Contains all algorithm implementations (EKF, UKF, A*), trait definitions
//! (`StateEstimator`, `Controller`, `Planner`), and shared data structures
//! (`MeasurementMessage`, `FrameAwareState`). Has zero dependency on Bevy or Avian3D.
//! See submodules for details.

// pub mod context;
pub mod control;
pub mod estimation;
pub mod frames;
pub mod mapping;
pub mod messages;
pub mod models;
pub mod path_following;
pub mod planning;
pub mod prelude;
pub mod sensor_data;
pub mod sensor_reading;
pub mod slam;
pub mod tracking;
pub mod types;
pub mod utils;
