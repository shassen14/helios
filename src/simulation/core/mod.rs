pub mod app_state;
pub mod config;
pub mod dynamics;
pub mod prng;
pub mod simulation_setup;
pub mod spawn_requests;
pub mod topics;

// Other way possibly?
// // 1. Declare the modules within this 'core' module.
// // These are now private to the `core` module.
// mod app_state;
// mod config;
// mod dynamics;
// mod prng;
// mod topics;

// // 2. Publicly re-export the types you want to be easily accessible from the outside.
// pub use app_state::AppState;
// pub use config::{SimulationConfig, AgentConfig, Vehicle, ImuConfig /* etc */};
// pub use dynamics::{Dynamics, DynamicsModel, State, Control, StateVariable};
// pub use prng::SimulationRng; // <- Re-exporting our newtype
// pub use topics::{
//     TopicBus,
//     TopicReader,
//     StampedMessage,
//     Topic,
//     ImuData,
//     GroundTruthState,
//     EstimatedPose,
//     /* etc */
// };
