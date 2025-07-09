// src/lib.rs

// This defines the top-level module of our library.
pub mod simulation;

/// The prelude module provides convenient access to the most common types.
/// Users can `use my_robotics_lib::prelude::*;` to get started quickly.
pub mod prelude {
    use bevy::app::App;
    use bevy::app::Plugin;

    // core
    pub use crate::simulation::core::app_state::AppState;
    pub use crate::simulation::core::config::*;
    use crate::simulation::core::simulation_setup::SimulationSetupPlugin;
    pub use crate::simulation::core::topics::*;

    // Re-export the entire plugin structs so they can be easily added.
    pub use crate::simulation::plugins::estimation::ekf::EkfPlugin;
    pub use crate::simulation::plugins::sensors::imu::ImuPlugin;
    pub use crate::simulation::plugins::vehicles::ackermann::AckermannCarPlugin;
    pub use crate::simulation::plugins::world::spawner::WorldSpawnerPlugin;
    // ... pub use crate::simulation::plugins::planning::astar::AStarPlugin;

    // Re-export the components that are needed to build a scene.
    // pub use crate::simulation::core::dynamics::{Dynamics, DynamicsModel, SimpleCarDynamics}; // Example
    pub use crate::simulation::plugins::estimation::ekf::{EKF, EkfState};
    // pub use crate::simulation::plugins::sensors::imu::Imu;

    pub struct RoboticsSuitePlugin;

    impl Plugin for RoboticsSuitePlugin {
        fn build(&self, app: &mut App) {
            app.add_plugins((
                // Add ALL available module plugins here.
                // Cargo features can be used to conditionally compile these.
                SimulationSetupPlugin,
                WorldSpawnerPlugin,
                AckermannCarPlugin,
                ImuPlugin,
                EkfPlugin,
                // QuadcopterPlugin, // etc.
            ));
        }
    }
}
