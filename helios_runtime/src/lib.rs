// helios_runtime/src/lib.rs
//
// A thin, Bevy-free layer between helios_core and helios_sim.
// Provides AutonomyPipeline — a typed stage vector that runs identically
// in simulation and on real hardware.

pub mod pipeline;
pub mod prelude;
pub mod runtime;
pub mod stage;
