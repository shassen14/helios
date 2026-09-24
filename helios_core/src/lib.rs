//! Pure algorithmic library for robotics. Framework-agnostic; runs on real hardware.
//!
//! Contains all algorithm implementations (EKF, UKF, A*), trait definitions
//! (`GaussianStateEstimator`, `MeasurementModel`, `SensorPayload`, `Controller`,
//! `Planner`), and shared data structures (`FrameAwareState`). Has zero dependency
//! on Bevy or Avian3D. See submodules for details.

pub mod control;
pub mod estimation;
pub mod interchange;
pub mod kernel;
pub mod mapping;
pub mod path_following;
pub mod planning;
pub mod plant;
pub mod prelude;
pub mod sensors;
pub mod spatial;
