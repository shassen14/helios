// src/lib.rs

// This defines the top-level module of our library.
pub mod simulation;

/// The prelude module provides convenient access to the most common types.
/// Users can `use my_robotics_lib::prelude::*;` to get started quickly.
pub mod prelude {
    // Re-export the entire plugin structs so they can be easily added.
    pub use crate::simulation::plugins::estimation::ekf::EkfPlugin;
    pub use crate::simulation::plugins::sensors::imu::ImuPlugin;
    // ... pub use crate::simulation::plugins::planning::astar::AStarPlugin;

    // Re-export the components that are needed to build a scene.
    // pub use crate::simulation::core::dynamics::{Dynamics, DynamicsModel, SimpleCarDynamics}; // Example
    pub use crate::simulation::core::config::*;
    pub use crate::simulation::core::topics::*;
    pub use crate::simulation::core::topics::{EstimatedPose, GroundTruthState};
    pub use crate::simulation::plugins::estimation::ekf::{EKF, EkfState};
    // pub use crate::simulation::plugins::sensors::imu::Imu;
}
