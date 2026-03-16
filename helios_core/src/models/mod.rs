//! Mathematical models for dynamics, estimation, and perception.
//!
//! Organized into three submodules: [`controls`] (ControlDynamics trait),
//! [`estimation`] (EstimationDynamics + Measurement traits and concrete models),
//! and [`perception`] (RaycastingSensorModel trait for lidar/depth sensors).

pub mod controls;
pub mod estimation;
pub mod perception;
